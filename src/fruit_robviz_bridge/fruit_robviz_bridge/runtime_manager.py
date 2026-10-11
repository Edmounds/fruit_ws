# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Small, configuration-driven manager for on-demand robot processes."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import fcntl
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import threading
import time
from typing import Any, Iterator

import yaml


class RuntimeErrorBase(Exception):
    """Base error with an HTTP-friendly status code."""

    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.message = message
        self.status = status


class RuntimeConflict(RuntimeErrorBase):
    """Raised when a runtime operation cannot be started safely."""

    def __init__(self, message: str) -> None:
        super().__init__(message, 409)


@dataclass(frozen=True)
class Task:
    """One allowlisted process or group of dependent tasks."""

    task_id: str
    command: str | None
    dependencies: tuple[str, ...]
    readiness: dict[str, Any]
    stop_command: str | None
    timeout_sec: float
    log_file: str


@dataclass
class ManagedProcess:
    process: subprocess.Popen[str]
    log_path: Path


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _read_process_cmdlines() -> list[str]:
    result: list[str] = []
    ancestors = {os.getpid()}
    parent_pid = os.getppid()
    while parent_pid > 1 and parent_pid not in ancestors:
        ancestors.add(parent_pid)
        try:
            stat_fields = (Path(f"/proc/{parent_pid}/stat").read_text()).split()
            parent_pid = int(stat_fields[3])
        except (OSError, ValueError, IndexError):
            break
    try:
        entries = Path("/proc").iterdir()
    except OSError:
        return result
    for entry in entries:
        if not entry.name.isdigit() or int(entry.name) in ancestors:
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
        except (OSError, PermissionError):
            continue
        command = raw.replace(b"\x00", b" ").decode("utf-8", "ignore").strip()
        if command:
            result.append(command)
    return result


class RuntimeManager:
    """Manage only commands explicitly registered in a YAML configuration."""

    def __init__(
        self,
        config_path: Path,
        runtime_setup_path: Path = Path(
            "/home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash"
        ),
        workspace_root: Path = Path("/home/rc4/fruit_ws"),
        log_root: Path = Path("/home/rc4/fruit_ws/log/runtime-control"),
        lock_path: Path = Path("/home/rc4/fruit_ws/tmp/runtime-control.lock"),
    ) -> None:
        self.config_path = config_path
        self.runtime_setup_path = runtime_setup_path
        self.workspace_root = workspace_root
        self.log_root = log_root
        self.lock_path = lock_path
        self._processes: dict[str, ManagedProcess] = {}
        self._operation_lock = threading.RLock()
        self.tasks = self._load_tasks(config_path)

    @staticmethod
    def _load_tasks(path: Path) -> dict[str, Task]:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict) or not isinstance(
            document.get("tasks"), dict
        ):
            raise ValueError("runtime 配置必须包含 tasks mapping")

        tasks: dict[str, Task] = {}
        for task_id, raw in document["tasks"].items():
            if not isinstance(task_id, str) or not task_id or not isinstance(raw, dict):
                raise ValueError("runtime task 配置无效")
            command = raw.get("command")
            if command is not None and (
                not isinstance(command, str) or not command.strip()
            ):
                raise ValueError(f"任务 {task_id} 的 command 无效")
            dependencies = raw.get("dependencies", [])
            if not isinstance(dependencies, list) or not all(
                isinstance(item, str) and item for item in dependencies
            ):
                raise ValueError(f"任务 {task_id} 的 dependencies 无效")
            readiness = raw.get("readiness", {})
            if not isinstance(readiness, dict):
                raise ValueError(f"任务 {task_id} 的 readiness 无效")
            timeout_sec = float(raw.get("timeout_sec", 10.0))
            if not 0.1 <= timeout_sec <= 60.0:
                raise ValueError(f"任务 {task_id} 的 timeout_sec 超出范围")
            tasks[task_id] = Task(
                task_id=task_id,
                command=command.strip() if isinstance(command, str) else None,
                dependencies=tuple(dependencies),
                readiness=readiness,
                stop_command=raw.get("stop_command"),
                timeout_sec=timeout_sec,
                log_file=str(raw.get("log_file", f"{task_id}.log")),
            )
        for task in tasks.values():
            for dependency in task.dependencies:
                if dependency not in tasks:
                    raise ValueError(
                        f"任务 {task.task_id} 依赖未知任务 {dependency}"
                    )
        return tasks

    @contextmanager
    def _lock(self) -> Iterator[None]:
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock_path.open("a+", encoding="utf-8") as lock_file:
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeConflict("已有运行控制操作正在执行") from error
            try:
                yield
            finally:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)

    def _task(self, task_id: str) -> Task:
        try:
            return self.tasks[task_id]
        except KeyError as error:
            raise RuntimeErrorBase(f"未知任务: {task_id}", 404) from error

    def _tracked_process(self, task_id: str) -> ManagedProcess | None:
        tracked = self._processes.get(task_id)
        if tracked is not None and tracked.process.poll() is not None:
            self._processes.pop(task_id, None)
            return None
        return tracked

    def _matches_process(self, task: Task) -> bool:
        pattern = task.readiness.get("process_pattern")
        return isinstance(pattern, str) and any(
            pattern in command for command in _read_process_cmdlines()
        )

    def _ros_graph(self) -> tuple[set[str], set[str]]:
        command = (
            f"source {shlex.quote(str(self.runtime_setup_path))} && "
            "ros2 node list && printf '\\n---SERVICES---\\n' && ros2 service list"
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", command],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return set(), set()
        if result.returncode != 0:
            return set(), set()
        nodes_text, _, services_text = result.stdout.partition("---SERVICES---")
        nodes = {line.strip() for line in nodes_text.splitlines() if line.strip()}
        services = {
            line.strip() for line in services_text.splitlines() if line.strip()
        }
        return nodes, services

    def _ready(self, task: Task) -> tuple[bool, str]:
        readiness = task.readiness
        if self._matches_process(task):
            return True, "进程匹配"

        port = readiness.get("tcp_port")
        if port is not None:
            try:
                with socket.create_connection(("127.0.0.1", int(port)), 0.3):
                    pass
            except (OSError, ValueError):
                return False, f"端口 {port} 未就绪"

        nodes_required = readiness.get("ros_nodes", [])
        services_required = readiness.get("ros_services", [])
        if nodes_required or services_required:
            nodes, services = self._ros_graph()
            missing_nodes = [item for item in nodes_required if item not in nodes]
            missing_services = [
                item for item in services_required if item not in services
            ]
            if missing_nodes or missing_services:
                missing = missing_nodes + missing_services
                return False, f"ROS 就绪项缺失: {', '.join(missing)}"

        if port is not None or nodes_required or services_required:
            return True, "就绪检查通过"
        tracked = self._tracked_process(task.task_id)
        if tracked is not None:
            return True, "进程运行中"
        return False, "未运行"

    def _status(self, task_id: str, stack: set[str] | None = None) -> dict[str, Any]:
        task = self._task(task_id)
        stack = set() if stack is None else stack
        if task_id in stack:
            raise RuntimeErrorBase(f"任务依赖存在循环: {task_id}")
        stack.add(task_id)
        dependencies = [self._status(item, stack) for item in task.dependencies]
        stack.remove(task_id)

        ready, detail = self._ready(task)
        tracked = self._tracked_process(task_id)
        if (
            task.command is None
            and dependencies
            and all(item["ready"] for item in dependencies)
        ):
            ready = True
            detail = "依赖任务已就绪"
        if tracked is not None and not ready:
            state = "STARTING"
            pid = tracked.process.pid
        elif ready:
            state = "RUNNING"
            pid = tracked.process.pid if tracked else None
        else:
            state = "STOPPED"
            pid = None
        return {
            "id": task_id,
            "state": state,
            "ready": ready,
            "detail": detail,
            "pid": pid,
            "dependencies": dependencies,
            "checked_at": _now(),
        }

    def status(self, task_id: str | None = None) -> dict[str, Any]:
        with self._operation_lock:
            if task_id is not None:
                return self._status(task_id)
            return {
                "checked_at": _now(),
                "tasks": [self._status(item) for item in self.tasks],
            }

    def _launch(self, task: Task) -> None:
        if task.command is None:
            raise RuntimeErrorBase(f"任务 {task.task_id} 没有启动命令")
        self.log_root.mkdir(parents=True, exist_ok=True)
        log_path = self.log_root / Path(task.log_file).name
        command = (
            f"source {shlex.quote(str(self.runtime_setup_path))} && "
            f"exec {task.command}"
        )
        output = log_path.open("a", encoding="utf-8")
        output.write(f"\n--- start {task.task_id} {_now()} ---\n")
        output.flush()
        try:
            process = subprocess.Popen(
                ["/bin/bash", "-lc", command],
                cwd=str(self.workspace_root),
                stdout=output,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
        except Exception:
            output.close()
            raise
        self._processes[task.task_id] = ManagedProcess(process, log_path)

    def _start(self, task_id: str, stack: set[str]) -> list[str]:
        if task_id in stack:
            raise RuntimeErrorBase(f"任务依赖存在循环: {task_id}")
        stack.add(task_id)
        task = self._task(task_id)
        started: list[str] = []
        for dependency in task.dependencies:
            started.extend(self._start(dependency, stack))
        stack.remove(task_id)

        ready, _ = self._ready(task)
        if ready:
            return started
        if task.command is None:
            if task.dependencies and all(
                self._status(item)["ready"] for item in task.dependencies
            ):
                return started
            raise RuntimeConflict(f"任务 {task_id} 的依赖未就绪")
        if self._tracked_process(task_id) is not None:
            raise RuntimeConflict(f"任务 {task_id} 已启动，正在等待就绪")
        self._launch(task)
        started.append(task_id)
        deadline = time.monotonic() + task.timeout_sec
        while time.monotonic() < deadline:
            ready, _ = self._ready(task)
            if ready:
                return started
            if self._tracked_process(task_id) is None:
                break
            time.sleep(0.2)
        status = self._status(task_id)
        if not status["ready"]:
            raise RuntimeConflict(
                f"任务 {task_id} 启动未就绪: {status['detail']}"
            )
        return started

    def start(self, task_id: str) -> dict[str, Any]:
        with self._operation_lock, self._lock():
            started = self._start(task_id, set())
            return {"started": started, "task": self._status(task_id)}

    def _stop_one(self, task_id: str) -> list[str]:
        task = self._task(task_id)
        stopped: list[str] = []
        tracked = self._tracked_process(task_id)
        if task.stop_command:
            ready, _ = self._ready(task)
            if ready:
                command = (
                    f"source {shlex.quote(str(self.runtime_setup_path))} && "
                    f"{task.stop_command}"
                )
                subprocess.run(
                    ["/bin/bash", "-lc", command],
                    cwd=str(self.workspace_root),
                    capture_output=True,
                    text=True,
                    timeout=5.0,
                    check=False,
                )
        if tracked is not None:
            try:
                os.killpg(tracked.process.pid, signal.SIGINT)
                tracked.process.wait(timeout=5.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if tracked.process.poll() is None:
                    os.killpg(tracked.process.pid, signal.SIGTERM)
            finally:
                self._processes.pop(task_id, None)
            stopped.append(task_id)
        return stopped

    def stop(self, task_id: str) -> dict[str, Any]:
        with self._operation_lock, self._lock():
            task = self._task(task_id)
            stopped: list[str] = []
            for dependency in reversed(task.dependencies):
                stopped.extend(self._stop_one(dependency))
            stopped.extend(self._stop_one(task_id))
            return {"stopped": stopped, "task": self._status(task_id)}
