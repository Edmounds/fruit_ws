#!/usr/bin/env python3
# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""HTTP endpoint for the persistent RobViz runtime control service."""

from __future__ import annotations

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import subprocess
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse

from fruit_robviz_bridge.runtime_manager import (
    RuntimeErrorBase,
    RuntimeManager,
)


DEFAULT_CONFIG = Path(
    "/home/rc4/fruit_ws/src/fruit_robviz_bridge/config/runtime_tasks.yaml"
)
COMPETITION_STATUS = Path("/home/rc4/fruit_ws/tmp/fruit_competition/status.json")


class RuntimeHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, host: str, port: int, manager: RuntimeManager) -> None:
        self.manager = manager
        super().__init__((host, port), RuntimeRequestHandler)


class RuntimeRequestHandler(BaseHTTPRequestHandler):
    server: RuntimeHTTPServer

    def _send(self, status: HTTPStatus, payload: Any) -> None:
        body = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self._cors_headers()
        self.end_headers()
        self.wfile.write(body)

    def _allowed_origin(self) -> bool:
        origin = self.headers.get("Origin")
        if not origin:
            return True
        try:
            parsed = urlparse(origin)
            port = parsed.port
            host = urlparse("//" + self.headers.get("Host", "")).hostname
        except ValueError:
            return False
        local = {"localhost", "127.0.0.1", "::1"}
        is_local_pair = (
            parsed is not None and parsed.hostname in local and host in local
        )
        return (
            parsed is not None
            and parsed.scheme in ("http", "https")
            and (parsed.hostname == host or is_local_pair)
            and port in (None, 8000)
        )

    def _cors_headers(self) -> None:
        origin = self.headers.get("Origin")
        if origin and self._allowed_origin():
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
        self.send_header(
            "Access-Control-Allow-Methods", "GET, POST, OPTIONS"
        )
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _task_id(self) -> str | None:
        prefix = "/api/runtime/tasks/"
        path = unquote(urlparse(self.path).path)
        if not path.startswith(prefix):
            return None
        suffix = path[len(prefix):].strip("/")
        if not suffix or "/" in suffix:
            return None
        task_id = suffix
        return task_id or None

    def _operation(self, prefix: str = "/api/runtime/tasks/") -> tuple[str | None, str | None]:
        path = unquote(urlparse(self.path).path)
        if not path.startswith(prefix):
            return None, None
        parts = path[len(prefix):].strip("/").split("/")
        if len(parts) != 2 or not parts[0] or not parts[1]:
            return None, None
        return parts[0], parts[1]

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(HTTPStatus.NO_CONTENT)
        self._cors_headers()
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802
        try:
            path = urlparse(self.path).path
            if path == "/health":
                self._send(HTTPStatus.OK, {"status": "ok"})
                return
            if path == "/api/runtime/scripts":
                self._send(HTTPStatus.OK, self.server.manager.scripts())
                return
            if path == "/api/runtime/scripts/log":
                query = parse_qs(urlparse(self.path).query)
                try:
                    cursor = int(query.get("cursor", ["0"])[0])
                except ValueError as error:
                    raise RuntimeErrorBase("日志 cursor 无效") from error
                script_id = query.get("script_id", [None])[0]
                run_id = query.get("run_id", [""])[0]
                self._send(HTTPStatus.OK, self.server.manager.script_log(
                    cursor, run_id, script_id
                ))
                return
            if path == "/api/runtime/competition/status":
                try:
                    payload = json.loads(COMPETITION_STATUS.read_text(encoding="utf-8"))
                except (FileNotFoundError, json.JSONDecodeError):
                    payload = {"phase": "STOPPED"}
                self._send(HTTPStatus.OK, payload)
                return
            if path == "/api/runtime/tasks":
                self._send(HTTPStatus.OK, self.server.manager.status())
                return
            task_id = self._task_id()
            if task_id is not None:
                self._send(
                    HTTPStatus.OK, self.server.manager.status(task_id)
                )
                return
            self._send(HTTPStatus.NOT_FOUND, {"error": "接口不存在"})
        except RuntimeErrorBase as error:
            self._send(HTTPStatus(error.status), {"error": error.message})
        except Exception:
            logging.exception("Runtime status request failed")
            self._send(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": "服务器内部错误"})

    def do_POST(self) -> None:  # noqa: N802
        task_id, operation = self._operation()
        try:
            if not self._allowed_origin():
                self._send(HTTPStatus.FORBIDDEN, {"error": "不允许的 Origin"})
                return
            path = urlparse(self.path).path
            localize_path = '/api/runtime/competition/localize'
            localize = path == localize_path or path.startswith(localize_path + '/')
            expected_flow = path[len(localize_path) + 1:] if path != localize_path else None
            body = None
            if localize:
                if expected_flow is not None and expected_flow not in (
                        'full', 'ac', 'acb', 'a', 'b', 'c'):
                    raise RuntimeErrorBase('定位流程无效')
                if self.headers.get("Content-Type", "").split(";", 1)[0] != "application/json":
                    raise RuntimeErrorBase("请求必须使用 application/json", 415)
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 <= size <= 16384:
                        raise ValueError("body size")
                    body = json.loads(self.rfile.read(size) or b"{}")
                except (ValueError, UnicodeDecodeError) as error:
                    raise RuntimeErrorBase("请求 JSON 无效或超过大小限制") from error
                if not isinstance(body, dict):
                    raise RuntimeErrorBase("请求必须为 JSON object")
            if path.startswith("/api/runtime/scripts/"):
                if self.headers.get("Content-Type", "").split(";", 1)[0] != "application/json":
                    raise RuntimeErrorBase("请求必须使用 application/json", 415)
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= 16384:
                        raise ValueError("body size")
                    body = json.loads(self.rfile.read(size))
                except (ValueError, UnicodeDecodeError) as error:
                    raise RuntimeErrorBase("请求 JSON 无效或超过大小限制") from error
                if not isinstance(body, dict):
                    raise RuntimeErrorBase("请求必须为 JSON object")
                if path == "/api/runtime/scripts/kill":
                    if body:
                        raise RuntimeErrorBase("Kill 不接受参数")
                    result = self.server.manager.kill_scripts()
                else:
                    script_id, action = self._operation("/api/runtime/scripts/")
                    if script_id is None or action not in ("start", "stop"):
                        self._send(HTTPStatus.NOT_FOUND, {"error": "接口不存在"})
                        return
                    if action == "stop":
                        if body:
                            raise RuntimeErrorBase("Stop 不接受参数")
                        result = self.server.manager.stop_script(script_id)
                    else:
                        if body.keys() - {"parameters"}:
                            raise RuntimeErrorBase("存在未知请求字段")
                        result = self.server.manager.start_script(
                            script_id, body.get("parameters", {})
                        )
                self._send(HTTPStatus.OK, result)
                return
            if localize:
                if body:
                    raise RuntimeErrorBase("定位启动不接受参数")
                try:
                    status = json.loads(COMPETITION_STATUS.read_text(encoding="utf-8"))
                except (FileNotFoundError, json.JSONDecodeError):
                    status = {"phase": "STOPPED"}
                prepared = status.get('parameters')
                if expected_flow is not None and (
                        not isinstance(prepared, dict) or prepared.get('flow') != expected_flow):
                    raise RuntimeErrorBase('指定流程与准备会话不匹配', 409)
                if status.get("phase") in {"LOCALIZING", "RUNNING"}:
                    self._send(HTTPStatus.OK, {"accepted": False, "phase": status["phase"]})
                    return
                if status.get("phase") != "PREPARED":
                    raise RuntimeErrorBase("比赛尚未完成准备，不能启动定位", 409)
                log_path = self.server.manager.log_root / 'competition-localize.log'
                log_path.parent.mkdir(parents=True, exist_ok=True)
                script = self.server.manager.workspace_root / 'start_fruit_mission.sh'
                command = ['/bin/bash', str(script), '--phase', 'localize']
                if expected_flow is not None:
                    command.extend(['--flow', expected_flow])
                process = subprocess.Popen(
                    command,
                    stdout=log_path.open("a", encoding="utf-8"),
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                self._send(HTTPStatus.OK, {"accepted": True, "pid": process.pid})
                return
            if task_id is None or operation not in ("start", "stop"):
                self._send(HTTPStatus.NOT_FOUND, {"error": "接口不存在"})
                return
            if operation == "start":
                result = self.server.manager.start(task_id)
            else:
                result = self.server.manager.stop(task_id)
            self._send(HTTPStatus.OK, result)
        except RuntimeErrorBase as error:
            self._send(HTTPStatus(error.status), {"error": error.message})
        except Exception:
            logging.exception("Runtime operation failed")
            self._send(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": "服务器内部错误"})

    def log_message(self, message: str, *args: Any) -> None:
        logging.info("%s - %s", self.address_string(), message % args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve runtime task control API")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8001)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    try:
        manager = RuntimeManager(args.config)
        server = RuntimeHTTPServer(args.host, args.port, manager)
    except Exception as error:
        logging.error("runtime control service 启动失败: %s", error)
        return 1

    logging.info(
        "Runtime control listening on http://%s:%d", args.host, args.port
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
