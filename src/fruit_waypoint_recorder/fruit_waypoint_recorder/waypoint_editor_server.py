# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Serve the waypoint editor and its allowlisted YAML API."""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import math
import os
from pathlib import Path
import shutil
import stat
import tempfile
from typing import Any, Dict, List, Mapping, Tuple
from urllib.parse import parse_qs, urlparse

import yaml

from fruit_waypoint_recorder.recorder import yaw_from_quaternion


DEFAULT_CONFIG = Path(
    "/home/rc4/fruit_ws/src/fruit_waypoint_recorder/config/"
    "waypoint_editor.yaml"
)
VALID_POINT_TYPES = ("waypoint", "goal")
MAX_POINT_COUNT = 5000


class RequestError(Exception):
    """An HTTP error that is safe to return to the client."""

    def __init__(self, status: HTTPStatus, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.message = message


@dataclass(frozen=True)
class EditorConfig:
    """Runtime configuration loaded from the source workspace."""

    host: str
    port: int
    html_file: Path
    waypoint_file: Path
    template_file: Path
    backup_dir: Path
    max_request_bytes: int


@dataclass(frozen=True)
class EditorFile:
    """One waypoint YAML exposed by the editor API."""

    file_id: str
    profile: str
    mode: str
    frame_id: str
    waypoint_file: Path
    template_file: Path
    reference_file: Path

    @property
    def label(self) -> str:
        return f"里程计 / {self.waypoint_file.name}"


def _required_mapping(document: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(document, Mapping):
        raise ValueError(f"{label} 必须是 YAML mapping")
    return document


def _configured_path(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"缺少配置项 {label}")
    return Path(value).expanduser().resolve()


def load_editor_config(path: Path) -> EditorConfig:
    """Load server configuration from an explicit source path."""
    with path.expanduser().resolve().open(encoding="utf-8") as stream:
        document = _required_mapping(
            yaml.safe_load(stream), "waypoint editor 配置"
        )

    host = str(document.get("host", "0.0.0.0")).strip()
    port = int(document.get("port", 8000))
    max_request_bytes = int(document.get("max_request_bytes", 1048576))
    if not host:
        raise ValueError("host 不能为空")
    if not 1 <= port <= 65535:
        raise ValueError("port 必须在 1..65535")
    if not 1024 <= max_request_bytes <= 16 * 1024 * 1024:
        raise ValueError("max_request_bytes 必须在 1 KiB..16 MiB")

    return EditorConfig(
        host=host,
        port=port,
        html_file=_configured_path(document.get("html_file"), "html_file"),
        waypoint_file=_configured_path(
            document.get("waypoint_file"), "waypoint_file"
        ),
        template_file=_configured_path(
            document.get("template_file"), "template_file"
        ),
        backup_dir=_configured_path(document.get("backup_dir"), "backup_dir"),
        max_request_bytes=max_request_bytes,
    )


def load_file_catalog(config: EditorConfig) -> Dict[str, EditorFile]:
    """Resolve the single odom waypoint file exposed by the editor."""
    file_id = "odom"
    return {
        file_id: EditorFile(
            file_id=file_id,
            profile="odom",
            mode="odometry",
            frame_id="odom",
            waypoint_file=config.waypoint_file,
            template_file=config.template_file,
            reference_file=config.waypoint_file,
        )
    }



def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} 必须是有限数值")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} 必须是有限数值") from error
    if not math.isfinite(parsed):
        raise ValueError(f"{label} 必须是有限数值")
    return parsed


def _point_yaw(item: Mapping[str, Any], name: str) -> float:
    if "yaw" in item:
        return _finite_number(item["yaw"], f"点 {name} 的 yaw")
    orientation = _required_mapping(
        item.get("orientation"), f"点 {name} 的 orientation"
    )
    return yaw_from_quaternion(
        _finite_number(orientation.get("x"), f"点 {name} 的 orientation.x"),
        _finite_number(orientation.get("y"), f"点 {name} 的 orientation.y"),
        _finite_number(orientation.get("z"), f"点 {name} 的 orientation.z"),
        _finite_number(orientation.get("w"), f"点 {name} 的 orientation.w"),
    )


def load_waypoint_document(editor_file: EditorFile) -> Tuple[dict, List[dict]]:
    """Parse one allowlisted waypoint YAML into browser-facing point data."""
    with editor_file.waypoint_file.open(encoding="utf-8") as stream:
        document = _required_mapping(
            yaml.safe_load(stream), f"{editor_file.waypoint_file.name}"
        )
    if document.get("schema_version") != 1:
        raise ValueError("waypoint YAML 的 schema_version 必须为 1")
    if document.get("frame_id") != editor_file.frame_id:
        raise ValueError(
            f"{editor_file.waypoint_file.name} 的 frame_id 必须为 "
            f"{editor_file.frame_id}"
        )
    raw_points = document.get("points")
    if not isinstance(raw_points, list):
        raise ValueError("waypoint YAML 的 points 必须是列表")
    if len(raw_points) > MAX_POINT_COUNT:
        raise ValueError(f"waypoint 数量不能超过 {MAX_POINT_COUNT}")

    points: List[dict] = []
    names = set()
    for index, raw_item in enumerate(raw_points, start=1):
        item = _required_mapping(raw_item, f"第 {index} 个点")
        name = str(item.get("name", "")).strip()
        point_type = str(item.get("type", "")).strip()
        if not name:
            raise ValueError(f"第 {index} 个点的名称不能为空")
        if name in names:
            raise ValueError(f"点名称重复: {name}")
        if point_type not in VALID_POINT_TYPES:
            raise ValueError(f"点 {name} 的 type 必须是 waypoint 或 goal")
        position = _required_mapping(
            item.get("position"), f"点 {name} 的 position"
        )
        yaw = _point_yaw(item, name)
        extra = copy.deepcopy(dict(item))
        for key in ("name", "type", "position", "orientation", "yaw"):
            extra.pop(key, None)
        points.append(
            {
                "name": name,
                "type": point_type,
                "x": _finite_number(position.get("x"), f"点 {name} 的 x"),
                "y": _finite_number(position.get("y"), f"点 {name} 的 y"),
                "z": _finite_number(
                    position.get("z", 0.0), f"点 {name} 的 z"
                ),
                "yaw_deg": math.degrees(yaw),
                "extra": extra,
            }
        )
        names.add(name)
    return dict(document), points


def load_template_geometry(
    editor_file: EditorFile,
) -> List[dict]:
    """Load formal order and geometry relative to A1."""
    with editor_file.template_file.open(encoding="utf-8") as stream:
        template_document = _required_mapping(
            yaml.safe_load(stream), "waypoint template"
        )
    if template_document.get("schema_version") != 1:
        raise ValueError("waypoint template 的 schema_version 必须为 1")
    template_points = template_document.get("points")
    if not isinstance(template_points, list):
        raise ValueError("waypoint template 的 points 必须是列表")

    reference_file = EditorFile(
        file_id="odom:reference",
        profile="odom",
        mode="odometry",
        frame_id="odom",
        waypoint_file=editor_file.reference_file,
        template_file=editor_file.template_file,
        reference_file=editor_file.reference_file,
    )
    _, reference_points = load_waypoint_document(reference_file)
    reference_by_name = {
        point["name"]: point for point in reference_points
    }
    reference_a1 = reference_by_name.get("A1")
    if reference_a1 is None:
        raise ValueError("正式 waypoint YAML 缺少参考点 A1")

    result: List[dict] = []
    seen = set()
    for index, raw_item in enumerate(template_points, start=1):
        item = _required_mapping(raw_item, f"template 第 {index} 个点")
        name = str(item.get("name", "")).strip()
        point_type = str(item.get("type", "")).strip()
        if not name:
            raise ValueError(f"template 第 {index} 个点名称不能为空")
        if name in seen:
            raise ValueError(f"template 点名称重复: {name}")
        if point_type not in VALID_POINT_TYPES:
            raise ValueError(
                f"template 点 {name} 的 type 必须是 waypoint 或 goal"
            )
        reference = reference_by_name.get(name)
        if reference is None:
            raise ValueError(f"正式 waypoint YAML 缺少 template 点 {name}")
        result.append(
            {
                "index": index,
                "name": name,
                "type": point_type,
                "dx": reference["x"] - reference_a1["x"],
                "dy": reference["y"] - reference_a1["y"],
                "dz": reference["z"] - reference_a1["z"],
                "yaw_deg": reference["yaw_deg"],
            }
        )
        seen.add(name)

    extra_reference_names = set(reference_by_name) - seen
    if extra_reference_names:
        names = ", ".join(sorted(extra_reference_names))
        raise ValueError(f"正式 waypoint YAML 存在 template 外名称: {names}")
    return result



def order_actual_points(
    points: List[dict], template: List[dict]
) -> List[dict]:
    """Reject non-template names and return actual points in formal order."""
    order = {point["name"]: index for index, point in enumerate(template)}
    unknown = [point["name"] for point in points if point["name"] not in order]
    if unknown:
        raise ValueError(f"waypoint 名称不在 template 中: {', '.join(unknown)}")
    return sorted(points, key=lambda point: order[point["name"]])


def _normalize_degrees(value: float) -> float:
    normalized = (value + 180.0) % 360.0 - 180.0
    return 0.0 if abs(normalized) < 0.5e-12 else normalized


def _serialize_points(
    raw_points: Any,
    template: List[dict],
) -> List[dict]:
    if not isinstance(raw_points, list):
        raise ValueError("points 必须是列表")
    if len(raw_points) > MAX_POINT_COUNT:
        raise ValueError(f"waypoint 数量不能超过 {MAX_POINT_COUNT}")

    template_order = {
        point["name"]: index for index, point in enumerate(template)
    }
    serialized: List[dict] = []
    names = set()
    for index, raw_item in enumerate(raw_points, start=1):
        item = _required_mapping(raw_item, f"第 {index} 个点")
        name = str(item.get("name", "")).strip()
        point_type = str(item.get("type", "")).strip()
        if not name:
            raise ValueError(f"第 {index} 个点的名称不能为空")
        if len(name) > 200:
            raise ValueError(f"点 {name[:40]} 的名称不能超过 200 个字符")
        if name in names:
            raise ValueError(f"点名称重复: {name}")
        if name not in template_order:
            raise ValueError(f"点名称不在 template 中: {name}")
        if point_type not in VALID_POINT_TYPES:
            raise ValueError(f"点 {name} 的 type 必须是 waypoint 或 goal")

        x = _finite_number(item.get("x"), f"点 {name} 的 x")
        y = _finite_number(item.get("y"), f"点 {name} 的 y")
        z = _finite_number(item.get("z", 0.0), f"点 {name} 的 z")
        yaw_degrees = _normalize_degrees(
            _finite_number(item.get("yaw_deg"), f"点 {name} 的 yaw")
        )
        yaw = round(math.radians(yaw_degrees), 6)
        half_yaw = yaw * 0.5

        raw_extra = item.get("extra", {})
        extra = copy.deepcopy(
            _required_mapping(raw_extra, f"点 {name} 的 extra")
        )
        point = dict(extra)
        for key in ("name", "type", "position", "orientation", "yaw"):
            point.pop(key, None)
        point["name"] = name
        point["position"] = {"x": x, "y": y, "z": z}
        point["orientation"] = {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half_yaw),
            "w": math.cos(half_yaw),
        }
        point["yaw"] = yaw
        point["type"] = point_type
        serialized.append(point)
        names.add(name)
    return sorted(
        serialized,
        key=lambda point: template_order[point["name"]],
    )


def save_waypoint_document(
    editor_file: EditorFile,
    backup_root: Path,
    raw_points: Any,
) -> Path:
    """Validate, back up, and atomically replace one waypoint YAML."""
    document, _ = load_waypoint_document(editor_file)
    template = load_template_geometry(editor_file)
    document["points"] = _serialize_points(raw_points, template)

    profile_backup_dir = backup_root / editor_file.profile
    profile_backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
    backup_path = profile_backup_dir / (
        f"{editor_file.waypoint_file.stem}_{stamp}"
        f"{editor_file.waypoint_file.suffix}"
    )
    shutil.copy2(editor_file.waypoint_file, backup_path)

    target_dir = editor_file.waypoint_file.parent
    file_mode = stat.S_IMODE(editor_file.waypoint_file.stat().st_mode)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{editor_file.waypoint_file.name}.",
        suffix=".tmp",
        dir=target_dir,
        text=True,
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as output:
            yaml.safe_dump(
                document,
                output,
                allow_unicode=True,
                sort_keys=False,
            )
            output.flush()
            os.fsync(output.fileno())
        temporary_path.chmod(file_mode)
        os.replace(temporary_path, editor_file.waypoint_file)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise
    return backup_path


def load_map_info(editor_file: EditorFile) -> dict:
    """Describe the coordinate-grid visualization used for every file."""
    return {
        "available": False,
        "reason": f"{editor_file.frame_id} 坐标网格",
    }


class WaypointEditorHTTPServer(ThreadingHTTPServer):
    """HTTP server carrying immutable editor configuration."""

    daemon_threads = True

    def __init__(self, config: EditorConfig) -> None:
        self.editor_config = config
        super().__init__(
            (config.host, config.port), WaypointEditorRequestHandler
        )


class WaypointEditorRequestHandler(BaseHTTPRequestHandler):
    """Same-origin static page and JSON API handler."""

    server: WaypointEditorHTTPServer

    def _security_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Security-Policy", (
            "default-src 'self'; base-uri 'none'; object-src 'none'; "
            "frame-ancestors 'none'; img-src 'self' data: blob:; "
            "connect-src 'self'; script-src 'self' 'unsafe-inline'; "
            "style-src 'self' 'unsafe-inline'"
        ))
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")

    def _send_bytes(
        self,
        status: HTTPStatus,
        content_type: str,
        body: bytes,
    ) -> None:
        try:
            self.send_response(status)
            self._security_headers()
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            logging.info("Client disconnected before response completed")

    def _send_json(self, status: HTTPStatus, payload: Any) -> None:
        body = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self._send_bytes(status, "application/json; charset=utf-8", body)

    def _send_error(self, error: RequestError) -> None:
        self._send_json(error.status, {"error": error.message})

    def _catalog_file(self, query: Mapping[str, List[str]]) -> EditorFile:
        file_id = query.get("id", [""])[0]
        catalog = load_file_catalog(self.server.editor_config)
        editor_file = catalog.get(file_id)
        if editor_file is None:
            raise RequestError(HTTPStatus.NOT_FOUND, "未知的 waypoint 文件 ID")
        return editor_file

    def _read_json_body(self) -> Mapping[str, Any]:
        content_type = self.headers.get_content_type()
        if content_type != "application/json":
            raise RequestError(
                HTTPStatus.UNSUPPORTED_MEDIA_TYPE,
                "请求必须使用 application/json",
            )
        raw_length = self.headers.get("Content-Length")
        if raw_length is None:
            raise RequestError(HTTPStatus.LENGTH_REQUIRED, "缺少 Content-Length")
        try:
            length = int(raw_length)
        except ValueError as error:
            raise RequestError(
                HTTPStatus.BAD_REQUEST, "Content-Length 无效"
            ) from error
        if length < 0 or length > self.server.editor_config.max_request_bytes:
            raise RequestError(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "请求内容过大")
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise RequestError(
                HTTPStatus.BAD_REQUEST, "JSON 请求格式无效"
            ) from error
        if not isinstance(payload, Mapping):
            raise RequestError(HTTPStatus.BAD_REQUEST, "JSON 根节点必须是对象")
        return payload

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query, keep_blank_values=True)
        try:
            if parsed.path == "/":
                body = self.server.editor_config.html_file.read_bytes()
                self._send_bytes(
                    HTTPStatus.OK, "text/html; charset=utf-8", body
                )
                return
            if parsed.path == "/health":
                self._send_json(HTTPStatus.OK, {"status": "ok"})
                return
            if parsed.path == "/api/files":
                catalog = load_file_catalog(self.server.editor_config)
                files = []
                for editor_file in catalog.values():
                    files.append(
                        {
                            "id": editor_file.file_id,
                            "label": editor_file.label,
                            "profile": editor_file.profile,
                            "mode": editor_file.mode,
                            "frame_id": editor_file.frame_id,
                            "file_name": editor_file.waypoint_file.name,
                            "file_available": (
                                editor_file.waypoint_file.is_file()
                            ),
                            "map": load_map_info(editor_file),
                        }
                    )
                self._send_json(HTTPStatus.OK, {"files": files})
                return
            if parsed.path == "/api/document":
                editor_file = self._catalog_file(query)
                document, points = load_waypoint_document(editor_file)
                template = load_template_geometry(editor_file)
                points = order_actual_points(points, template)
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "id": editor_file.file_id,
                        "label": editor_file.label,
                        "schema_version": document["schema_version"],
                        "frame_id": document["frame_id"],
                        "points": points,
                        "template": template,
                        "map": load_map_info(editor_file),
                    },
                )
                return
            raise RequestError(HTTPStatus.NOT_FOUND, "接口不存在")
        except RequestError as error:
            self._send_error(error)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            self._send_error(
                RequestError(HTTPStatus.UNPROCESSABLE_ENTITY, str(error))
            )
        except Exception:
            logging.exception("Unhandled waypoint editor GET error")
            self._send_error(
                RequestError(
                    HTTPStatus.INTERNAL_SERVER_ERROR, "服务器内部错误"
                )
            )

    def do_PUT(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        try:
            if parsed.path != "/api/document":
                raise RequestError(HTTPStatus.NOT_FOUND, "接口不存在")
            payload = self._read_json_body()
            file_id = payload.get("id")
            if not isinstance(file_id, str):
                raise RequestError(HTTPStatus.BAD_REQUEST, "缺少 waypoint 文件 ID")
            catalog = load_file_catalog(self.server.editor_config)
            editor_file = catalog.get(file_id)
            if editor_file is None:
                raise RequestError(HTTPStatus.NOT_FOUND, "未知的 waypoint 文件 ID")
            backup_path = save_waypoint_document(
                editor_file,
                self.server.editor_config.backup_dir,
                payload.get("points"),
            )
            logging.info(
                "Saved %s with backup %s",
                editor_file.waypoint_file,
                backup_path,
            )
            self._send_json(
                HTTPStatus.OK,
                {
                    "saved": True,
                    "file": str(editor_file.waypoint_file),
                    "backup": str(backup_path),
                },
            )
        except RequestError as error:
            self._send_error(error)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            self._send_error(
                RequestError(HTTPStatus.UNPROCESSABLE_ENTITY, str(error))
            )
        except Exception:
            logging.exception("Unhandled waypoint editor PUT error")
            self._send_error(
                RequestError(
                    HTTPStatus.INTERNAL_SERVER_ERROR, "服务器内部错误"
                )
            )

    def do_POST(self) -> None:  # noqa: N802
        self._send_error(
            RequestError(HTTPStatus.METHOD_NOT_ALLOWED, "不支持该请求方法")
        )

    def do_DELETE(self) -> None:  # noqa: N802
        self._send_error(
            RequestError(HTTPStatus.METHOD_NOT_ALLOWED, "不支持该请求方法")
        )

    def log_message(self, message: str, *args: Any) -> None:
        logging.info("%s - %s", self.address_string(), message % args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve the fruit waypoint web editor."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"Editor configuration YAML (default: {DEFAULT_CONFIG})",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    try:
        config = load_editor_config(args.config)
        if not config.html_file.is_file():
            raise ValueError(f"HTML 文件不存在: {config.html_file}")
        load_file_catalog(config)
        server = WaypointEditorHTTPServer(config)
    except Exception as error:
        logging.error("waypoint editor 启动失败: %s", error)
        return 1

    logging.info(
        "Waypoint editor listening on http://%s:%d",
        config.host,
        config.port,
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
