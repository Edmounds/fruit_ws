#!/usr/bin/env python3
"""Small desktop control panel for the field mission scripts."""

from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk


WORKSPACE = Path(__file__).resolve().parent
NAVIGATION_SCRIPT = WORKSPACE / "start_navigation.sh"
MISSION_SCRIPT = WORKSPACE / "start_fruit_mission.sh"
KILL_SCRIPT = WORKSPACE / "kill_fruit_processes.sh"


class MissionPanel:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("水果采摘任务控制")
        self.root.geometry("620x420")
        self.root.minsize(560, 360)
        self.map_var = tk.StringVar(value="A")
        self.flow_var = tk.StringVar(value="full")
        self.status_var = tk.StringVar(value="就绪")
        self.output_queue: queue.Queue[str] = queue.Queue()
        self.processes: dict[str, subprocess.Popen[str]] = {}
        self._build_widgets()
        self.root.after(200, self._drain_output)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def _build_widgets(self) -> None:
        frame = ttk.Frame(self.root, padding=14)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frame, text="场地地图").grid(row=0, column=0, sticky="w", pady=4)
        for column, map_id in enumerate(("A", "B"), start=1):
            ttk.Radiobutton(
                frame,
                text=map_id,
                value=map_id,
                variable=self.map_var,
            ).grid(row=0, column=column, sticky="w", padx=8)

        ttk.Label(frame, text="任务区域流程").grid(row=1, column=0, sticky="w", pady=4)
        flows = {
            "full": "全程",
            "ac": "AC",
            "a": "A区",
            "b": "B区",
            "c": "C区",
        }
        flow_box = ttk.Combobox(
            frame,
            textvariable=self.flow_var,
            values=list(flows),
            state="readonly",
            width=12,
        )
        flow_box.grid(row=1, column=1, columnspan=2, sticky="w", padx=8)
        flow_box.bind("<<ComboboxSelected>>", lambda _event: self._update_status())

        controls = ttk.Frame(frame)
        controls.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(14, 8))
        ttk.Button(
            controls, text="启动重定位/导航", command=self.start_navigation
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(
            controls, text="启动比赛流程", command=self.start_mission
        ).pack(side=tk.LEFT, padx=8)
        ttk.Button(
            controls, text="停止全部进程", command=self.stop_stack
        ).pack(side=tk.LEFT, padx=8)

        ttk.Label(frame, text="当前状态").grid(row=3, column=0, sticky="nw", pady=4)
        ttk.Label(
            frame,
            textvariable=self.status_var,
            foreground="#0b5d3b",
            wraplength=470,
        ).grid(row=3, column=1, columnspan=3, sticky="w", padx=8)

        self.output = tk.Text(frame, height=16, state=tk.DISABLED, wrap=tk.WORD)
        self.output.grid(row=4, column=0, columnspan=4, sticky="nsew", pady=(8, 0))
        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.output.yview)
        scrollbar.grid(row=4, column=4, sticky="ns", pady=(8, 0))
        self.output.configure(yscrollcommand=scrollbar.set)
        frame.columnconfigure(3, weight=1)
        frame.rowconfigure(4, weight=1)

    def _command_env(self) -> dict[str, str]:
        env = os.environ.copy()
        runtime_dir = f"/run/user/{os.getuid()}"
        env.update(
            {
                "DISPLAY": ":0",
                "XDG_RUNTIME_DIR": runtime_dir,
                "XAUTHORITY": f"{runtime_dir}/gdm/Xauthority",
                "DBUS_SESSION_BUS_ADDRESS": f"unix:path={runtime_dir}/bus",
            }
        )
        return env

    def _start(self, name: str, command: list[str]) -> None:
        process = self.processes.get(name)
        if process is not None and process.poll() is None:
            self._append(f"{name} 已在运行，PID={process.pid}")
            return
        self._append("启动: " + " ".join(command))
        try:
            process = subprocess.Popen(
                command,
                cwd=WORKSPACE,
                env=self._command_env(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as error:
            self.status_var.set(f"启动失败: {error}")
            self._append(str(error))
            return
        self.processes[name] = process
        self.status_var.set(f"{name} 运行中，PID={process.pid}")
        threading.Thread(
            target=self._read_process, args=(name, process), daemon=True
        ).start()

    def _read_process(self, name: str, process: subprocess.Popen[str]) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                self.output_queue.put(f"[{name}] {line.rstrip()}")
        code = process.wait()
        self.output_queue.put(f"[{name}] 已退出，状态码={code}")

    def start_navigation(self) -> None:
        self._start(
            "navigation",
            [str(NAVIGATION_SCRIPT), "--map", self.map_var.get()],
        )

    def start_mission(self) -> None:
        self._start(
            "mission",
            [
                str(MISSION_SCRIPT),
                "--map",
                self.map_var.get(),
                "--flow",
                self.flow_var.get(),
            ],
        )

    def stop_stack(self) -> None:
        try:
            result = subprocess.run(
                [str(KILL_SCRIPT)],
                cwd=WORKSPACE,
                env=self._command_env(),
                capture_output=True,
                text=True,
                check=False,
            )
        except OSError as error:
            self.status_var.set(f"停止失败: {error}")
            return
        self._append(result.stdout.rstrip())
        if result.stderr:
            self._append(result.stderr.rstrip())
        self.status_var.set("已请求停止全部任务进程")

    def _update_status(self) -> None:
        self.status_var.set(
            f"场地地图 {self.map_var.get()}，任务区域流程 {self.flow_var.get()}，等待启动"
        )

    def _append(self, text: str) -> None:
        if not text:
            return
        self.output.configure(state=tk.NORMAL)
        self.output.insert(tk.END, text + "\n")
        self.output.see(tk.END)
        self.output.configure(state=tk.DISABLED)

    def _drain_output(self) -> None:
        while True:
            try:
                line = self.output_queue.get_nowait()
            except queue.Empty:
                break
            self._append(line)
        self.root.after(200, self._drain_output)

    def _close(self) -> None:
        running = [name for name, process in self.processes.items() if process.poll() is None]
        if running and not messagebox.askyesno(
            "退出",
            "任务进程仍在运行，退出 GUI 但不停止它们？",
        ):
            return
        self.root.destroy()


def main() -> int:
    os.environ.setdefault("DISPLAY", ":0")
    os.environ.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    os.environ.setdefault(
        "XAUTHORITY", f"{os.environ['XDG_RUNTIME_DIR']}/gdm/Xauthority"
    )
    os.environ.setdefault(
        "DBUS_SESSION_BUS_ADDRESS",
        f"unix:path={os.environ['XDG_RUNTIME_DIR']}/bus",
    )
    root = tk.Tk()
    MissionPanel(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
