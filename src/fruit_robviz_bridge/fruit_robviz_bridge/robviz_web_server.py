#!/usr/bin/env python3
# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Serve the RobViz web and its allowlisted YAML API."""

from __future__ import annotations

import argparse
import copy
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import hashlib
import logging
import math
import mimetypes
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile
import threading
import uuid
from typing import Any, Dict, List, Mapping, Tuple
from urllib.parse import parse_qs, unquote, urlparse

import yaml

from fruit_robviz_bridge.recorder import yaw_from_quaternion


DEFAULT_CONFIG = Path(
    "/home/rc4/fruit_ws/src/fruit_robviz_bridge/config/"
    "robviz_web.yaml"
)
VALID_POINT_TYPES = ("waypoint", "goal")
MAX_POINT_COUNT = 5000
DEFAULT_IMAGE_SETTINGS = {
    "version": 1,
    "rgb": {
        "transport": "webrtc",
        "bandwidth_mbps": 25.0,
        "max_fps": 10.0,
        "target_bitrate_mbps": 4.0,
        "quality": 90,
        "min_quality": 70,
        "auto_quality": True,
    },
    "depth": {
        "transport": "webrtc",
        "bandwidth_mbps": 25.0,
        "max_fps": 10.0,
        "target_bitrate_mbps": 1.5,
        "quality": 90,
        "min_quality": 70,
        "auto_quality": True,
    },
}


class RequestError(Exception):
    """An HTTP error that is safe to return to the client."""

    def __init__(self, status: HTTPStatus, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.message = message


class VersionConflict(RequestError):
    """The client edited a document that changed after it was loaded."""

    def __init__(self, message: str) -> None:
        super().__init__(HTTPStatus.CONFLICT, message)


@dataclass(frozen=True)
class EditorConfig:
    """Runtime configuration loaded from the source workspace."""

    host: str
    port: int
    static_dir: Path
    waypoint_file: Path
    debug_waypoint_file: Path
    template_file: Path
    image_settings_file: Path
    backup_dir: Path
    max_request_bytes: int
    arm_config_path: Path | None = None


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
    formal: bool

    @property
    def label(self) -> str:
        if self.mode == "template":
            return f"模板 / {self.waypoint_file.name}"
        return f"里程计 / {self.waypoint_file.name}"


@dataclass
class ImageSettingsStore:
    """Thread-safe in-memory image settings owned by the HTTP server."""

    path: Path
    settings: dict

    def __post_init__(self) -> None:
        self._lock = threading.RLock()
        self.settings["session"] = uuid.uuid4().hex

    def snapshot(self) -> dict:
        with self._lock:
            return copy.deepcopy(self.settings)

    def update(self, payload: Mapping[str, Any]) -> dict:
        validated = validate_image_settings(payload)
        with self._lock:
            current_version = int(self.settings["version"])
            if payload.get("session") != self.settings["session"]:
                raise VersionConflict("图像配置服务已重启，请重新加载")
            expected_version = payload.get("version")
            if (
                expected_version is None
                or isinstance(expected_version, bool)
                or not isinstance(expected_version, int)
                or expected_version != current_version
            ):
                raise VersionConflict("图像配置版本已变化，请重新加载")

            next_settings = copy.deepcopy(validated)
            next_settings["version"] = current_version + 1
            next_settings["session"] = self.settings["session"]
            self.settings = next_settings
            return copy.deepcopy(next_settings)

    def save(
        self, payload: Mapping[str, Any], backup_root: Path
    ) -> tuple[dict, Path | None]:
        validated = validate_image_settings(payload)
        with self._lock:
            current_version = int(self.settings["version"])
            if payload.get("session") != self.settings["session"]:
                raise VersionConflict("图像配置服务已重启，请重新加载")
            expected_version = payload.get("version")
            if (
                expected_version is None
                or isinstance(expected_version, bool)
                or not isinstance(expected_version, int)
                or expected_version != current_version
            ):
                raise VersionConflict("图像配置版本已变化，请重新加载")

            next_settings = copy.deepcopy(validated)
            next_settings["version"] = current_version + 1
            next_settings["session"] = self.settings["session"]
            to_save = {
                key: value
                for key, value in next_settings.items()
                if key != "session"
            }
            backup_path = _atomic_save_json(
                self.path,
                to_save,
                backup_root / "image-settings",
            )
            self.settings = next_settings
            return copy.deepcopy(next_settings), backup_path


def _required_mapping(document: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(document, Mapping):
        raise ValueError(f"{label} 必须是 YAML mapping")
    return document


def _configured_path(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"缺少配置项 {label}")
    return Path(value).expanduser().resolve()


def _optional_configured_path(value: Any, label: str) -> Path | None:
    if value is None or value == "":
        return None
    return _configured_path(value, label)


def load_editor_config(path: Path) -> EditorConfig:
    """Load server configuration from an explicit source path."""
    with path.expanduser().resolve().open(encoding="utf-8") as stream:
        document = _required_mapping(
            yaml.safe_load(stream), "RobViz web 配置"
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
        static_dir=_configured_path(
            document.get("static_dir"), "static_dir"
        ),
        waypoint_file=_configured_path(
            document.get("waypoint_file"), "waypoint_file"
        ),
        debug_waypoint_file=_configured_path(
            document.get("debug_waypoint_file"), "debug_waypoint_file"
        ),
        template_file=_configured_path(
            document.get("template_file"), "template_file"
        ),
        image_settings_file=_configured_path(
            document.get("image_settings_file"), "image_settings_file"
        ),
        backup_dir=_configured_path(document.get("backup_dir"), "backup_dir"),
        max_request_bytes=max_request_bytes,
        arm_config_path=_optional_configured_path(
            document.get("arm_config_path"), "arm_config_path"
        ),
    )


def load_file_catalog(config: EditorConfig) -> Dict[str, EditorFile]:
    """Resolve the formal and free-form waypoint files exposed by the editor."""
    return {
        "odom": EditorFile(
            file_id="odom",
            profile="odom",
            mode="odometry",
            frame_id="odom",
            waypoint_file=config.waypoint_file,
            template_file=config.template_file,
            reference_file=config.waypoint_file,
            formal=True,
        ),
        "debug": EditorFile(
            file_id="debug",
            profile="debug",
            mode="freeform",
            frame_id="odom",
            waypoint_file=config.debug_waypoint_file,
            template_file=config.template_file,
            reference_file=config.debug_waypoint_file,
            formal=False,
        ),
        "template": EditorFile(
            file_id="template",
            profile="template",
            mode="template",
            frame_id="",
            waypoint_file=config.template_file,
            template_file=config.template_file,
            reference_file=config.template_file,
            formal=False,
        ),
    }


def document_version(path: Path) -> str:
    """Return a content version that changes for every external edit."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _default_image_settings() -> dict:
    return copy.deepcopy(DEFAULT_IMAGE_SETTINGS)


def _strict_bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} 必须是布尔值")
    return value


def _strict_int(value: Any, label: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} 必须是整数")
    if not minimum <= value <= maximum:
        raise ValueError(f"{label} 必须在 {minimum}..{maximum}")
    return value


def _strict_positive(value: Any, label: str, maximum: float) -> float:
    parsed = _finite_number(value, label)
    if not 0.0 < parsed <= maximum:
        raise ValueError(f"{label} 必须大于 0 且不超过 {maximum}")
    return parsed


def validate_image_settings(payload: Mapping[str, Any]) -> dict:
    """Validate the stable JSON contract shared by RobViz and ROS."""
    if not isinstance(payload, Mapping):
        raise ValueError("图像配置根节点必须是对象")
    normalized = {"version": 1}
    for stream_name in ("rgb", "depth"):
        stream = payload.get(stream_name)
        if not isinstance(stream, Mapping):
            raise ValueError(f"图像配置缺少 {stream_name} 对象")

        transport = str(stream.get("transport", "webrtc")).strip().lower()
        if transport not in ("webrtc", "jpeg"):
            raise ValueError(f"{stream_name}.transport 必须是 webrtc 或 jpeg")

        bw = _strict_positive(
            stream.get("bandwidth_mbps"),
            f"{stream_name}.bandwidth_mbps",
            1000.0,
        )
        max_fps = _strict_positive(
            stream.get("max_fps"), f"{stream_name}.max_fps", 60.0
        )
        target_bitrate = _strict_positive(
            stream.get("target_bitrate_mbps"),
            f"{stream_name}.target_bitrate_mbps",
            bw,
        )
        quality = _strict_int(
            stream.get("quality"),
            f"{stream_name}.quality",
            30,
            95,
        )
        min_quality = _strict_int(
            stream.get("min_quality", 70),
            f"{stream_name}.min_quality",
            30,
            quality,
        )
        auto_quality = _strict_bool(
            stream.get("auto_quality"),
            f"{stream_name}.auto_quality",
        )

        normalized[stream_name] = {
            "transport": transport,
            "bandwidth_mbps": bw,
            "max_fps": max_fps,
            "target_bitrate_mbps": target_bitrate,
            "quality": quality,
            "min_quality": min_quality,
            "auto_quality": auto_quality,
        }
    return normalized


def load_image_settings(path: Path) -> dict:
    """Load and validate persisted image settings, or use safe defaults."""
    if not path.is_file():
        return _default_image_settings()
    with path.open(encoding="utf-8") as stream:
        payload = json.load(stream, parse_constant=_reject_json_constant)
    settings = validate_image_settings(payload)
    version = payload.get("version", 1)
    settings["version"] = _strict_int(version, "version", 1, 2**31 - 1)
    return settings


def _reject_json_constant(value: str):
    raise ValueError(f"JSON 不允许 {value}")


def _atomic_save_json(path: Path, payload: Mapping[str, Any], backup_root: Path) -> Path | None:
    """Back up and atomically save one JSON document."""
    path.parent.mkdir(parents=True, exist_ok=True)
    backup_path = None
    if path.is_file():
        backup_root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        backup_path = backup_root / f"{path.stem}_{stamp}{path.suffix}"
        shutil.copy2(path, backup_path)
    file_mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o644
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent, text=True
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as output:
            json.dump(payload, output, ensure_ascii=False, indent=2)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        temporary_path.chmod(file_mode)
        os.replace(temporary_path, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise
    return backup_path


@contextmanager
def _document_lock(path: Path):
    """Serialize editor and recorder writes on Linux."""
    import fcntl

    lock_path = path.with_name(f".{path.name}.lock")
    lock_path.touch(exist_ok=True)
    with lock_path.open("r+", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


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


def load_waypoint_document(
    editor_file: EditorFile, content: bytes | None = None
) -> Tuple[dict, List[dict]]:
    """Parse one allowlisted waypoint YAML into browser-facing point data."""
    if editor_file.mode == "template":
        if content is None:
            content = editor_file.waypoint_file.read_bytes()
        document = _required_mapping(
            yaml.safe_load(content), f"{editor_file.waypoint_file.name}"
        )
        if document.get("schema_version") != 1:
            raise ValueError("template YAML 的 schema_version 必须为 1")
        raw_points = document.get("points")
        if not isinstance(raw_points, list):
            raise ValueError("template YAML 的 points 必须是列表")
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
            points.append(
                {
                    "name": name,
                    "type": point_type,
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "yaw_deg": 0.0,
                    "extra": {},
                }
            )
            names.add(name)
        return dict(document), points

    if content is None:
        content = editor_file.waypoint_file.read_bytes()
    document = _required_mapping(
        yaml.safe_load(content), f"{editor_file.waypoint_file.name}"
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


def load_waypoint_snapshot(editor_file: EditorFile) -> Tuple[dict, List[dict], str]:
    """Bind parsed points and their version to the same opened byte snapshot."""
    content = editor_file.waypoint_file.read_bytes()
    document, points = load_waypoint_document(editor_file, content)
    return document, points, hashlib.sha256(content).hexdigest()


def load_template_geometry(
    editor_file: EditorFile,
) -> List[dict]:
    """Load formal order and optional geometry relative to A1."""
    if not editor_file.formal:
        return []
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
        formal=True,
    )
    if editor_file.reference_file.is_file():
        _, reference_points = load_waypoint_document(reference_file)
    else:
        reference_points = []
    reference_by_name = {
        point["name"]: point for point in reference_points
    }
    reference_a1 = reference_by_name.get("A1")

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
        has_geometry = reference is not None and reference_a1 is not None
        result.append(
            {
                "index": index,
                "name": name,
                "type": point_type,
                "dx": (
                    reference["x"] - reference_a1["x"]
                    if has_geometry
                    else None
                ),
                "dy": (
                    reference["y"] - reference_a1["y"]
                    if has_geometry
                    else None
                ),
                "dz": (
                    reference["z"] - reference_a1["z"]
                    if has_geometry
                    else None
                ),
                "yaw_deg": reference["yaw_deg"] if has_geometry else None,
            }
        )
        seen.add(name)

    extra_reference_names = set(reference_by_name) - seen
    if extra_reference_names:
        names = ", ".join(sorted(extra_reference_names))
        raise ValueError(f"正式 waypoint YAML 存在 template 外名称: {names}")
    return result


def order_actual_points(
    points: List[dict], template: List[dict], formal: bool = True
) -> List[dict]:
    """Validate formal names or preserve user order for debug documents."""
    if not formal:
        return list(points)
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
    formal: bool,
) -> List[dict]:
    if not isinstance(raw_points, list):
        raise ValueError("points 必须是列表")
    if len(raw_points) > MAX_POINT_COUNT:
        raise ValueError(f"waypoint 数量不能超过 {MAX_POINT_COUNT}")

    template_order = {
        point["name"]: index for index, point in enumerate(template)
    }
    template_types = {point["name"]: point["type"] for point in template}
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
        if formal and name not in template_order:
            raise ValueError(f"点名称不在 template 中: {name}")
        if point_type not in VALID_POINT_TYPES:
            raise ValueError(f"点 {name} 的 type 必须是 waypoint 或 goal")
        if formal and template_types[name] != point_type:
            raise ValueError(f"点 {name} 的 type 与 template 不一致")

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
    if formal:
        return sorted(
            serialized,
            key=lambda point: template_order[point["name"]],
        )
    return serialized


def save_waypoint_document(
    editor_file: EditorFile,
    backup_root: Path,
    raw_points: Any,
    expected_version: str | None = None,
) -> Tuple[Path | None, str]:
    """Validate, back up, and atomically replace one waypoint YAML."""
    if editor_file.mode == "template":
        with _document_lock(editor_file.waypoint_file):
            document, _, current_version = load_waypoint_snapshot(editor_file)
            if expected_version is not None and expected_version != current_version:
                raise VersionConflict("template 文件已被其他进程修改，请重新加载")
            if not isinstance(raw_points, list):
                raise ValueError("points 必须是列表")
            if len(raw_points) > MAX_POINT_COUNT:
                raise ValueError(f"waypoint 数量不能超过 {MAX_POINT_COUNT}")
            serialized_points: List[dict] = []
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
                    raise ValueError(f"模板点名称重复: {name}")
                if point_type not in VALID_POINT_TYPES:
                    raise ValueError(f"点 {name} 的 type 必须是 waypoint 或 goal")
                serialized_points.append(
                    {
                        "name": name,
                        "type": point_type,
                    }
                )
                names.add(name)
            document["points"] = serialized_points

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
                directory_fd = os.open(target_dir, os.O_RDONLY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            except Exception:
                temporary_path.unlink(missing_ok=True)
                raise
            return backup_path, document_version(editor_file.waypoint_file)

    with _document_lock(editor_file.waypoint_file):
        document, _, current_version = load_waypoint_snapshot(editor_file)
        if expected_version is not None and expected_version != current_version:
            raise VersionConflict("waypoint 文件已被其他进程修改，请重新加载")
        template = load_template_geometry(editor_file)
        document["points"] = _serialize_points(
            raw_points, template, editor_file.formal
        )

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
            directory_fd = os.open(target_dir, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise
        return backup_path, document_version(editor_file.waypoint_file)


def load_map_info(editor_file: EditorFile) -> dict:
    """Describe the coordinate-grid visualization used for every file."""
    if editor_file.mode == "template":
        return {
            "available": False,
            "reason": "模板规则定义，无空间坐标",
        }
    return {
        "available": False,
        "reason": f"{editor_file.frame_id} 坐标网格",
    }


class WaypointEditorHTTPServer(ThreadingHTTPServer):
    """HTTP server carrying immutable editor configuration."""

    daemon_threads = True

    def __init__(self, config: EditorConfig) -> None:
        self.editor_config = config
        self.image_settings = ImageSettingsStore(
            config.image_settings_file,
            load_image_settings(config.image_settings_file),
        )
        super().__init__(
            (config.host, config.port), WaypointEditorRequestHandler
        )


class WaypointEditorRequestHandler(BaseHTTPRequestHandler):
    """Same-origin static page and JSON API handler."""

    server: WaypointEditorHTTPServer

    def _security_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        host = self.headers.get("Host", "")
        host_name = host.split(":")[0].strip()
        remote_ws = ""
        if host_name and re.match(r"^[a-zA-Z0-9.\-_\[\]]+$", host_name):
            remote_ws = (
                f" ws://{host_name}:8765 wss://{host_name}:8765 "
                f"ws://{host_name}:8787 wss://{host_name}:8787 "
                f"ws://{host_name}:8000 wss://{host_name}:8000"
            )
        self.send_header("Content-Security-Policy", (
            "default-src 'self'; base-uri 'none'; object-src 'none'; "
            "frame-ancestors 'none'; img-src 'self' data: blob:; "
            "media-src 'self' blob: mediastream:; "
            f"connect-src 'self' ws://127.0.0.1:* ws://localhost:*{remote_ws}; "
            "script-src 'self' 'unsafe-inline' 'wasm-unsafe-eval'; "
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

    def _send_file(self, path: Path) -> None:
        content_type = mimetypes.guess_type(path.name)[0]
        if content_type is None:
            content_type = "application/octet-stream"
        self._send_bytes(HTTPStatus.OK, content_type, path.read_bytes())

    def _static_file(self, request_path: str) -> Path | None:
        root = self.server.editor_config.static_dir
        if root is None:
            return None
        root = root.resolve()
        relative = unquote(request_path).lstrip("/")
        if "\x00" in relative:
            raise RequestError(HTTPStatus.BAD_REQUEST, "静态路径无效")
        candidate = (root / relative).resolve()
        try:
            candidate.relative_to(root)
        except ValueError as error:
            raise RequestError(HTTPStatus.FORBIDDEN, "禁止访问静态目录之外的文件") from error
        if candidate.is_file():
            return candidate
        return None

    def _check_same_origin(self) -> None:
        origin = self.headers.get("Origin")
        if not origin:
            return
        parsed_origin = urlparse(origin)
        host = self.headers.get("Host", "")
        if parsed_origin.scheme not in ("http", "https") or parsed_origin.netloc != host:
            raise RequestError(HTTPStatus.FORBIDDEN, "请求来源不是当前编辑器")

    def _catalog_file(self, query: Mapping[str, List[str]]) -> EditorFile:
        file_id = query.get("id", [""])[0]
        catalog = load_file_catalog(self.server.editor_config)
        editor_file = catalog.get(file_id)
        if editor_file is None:
            raise RequestError(HTTPStatus.NOT_FOUND, "未知的 waypoint 文件 ID")
        return editor_file

    def _arm_path(self) -> Path:
        path = self.server.editor_config.arm_config_path
        if path is None:
            raise RequestError(HTTPStatus.NOT_FOUND, '机械臂编辑未配置 arm_config_path')
        return path

    def _read_json_body(self, allow_empty: bool = False) -> Mapping[str, Any]:
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
        if length == 0 and allow_empty:
            return {}
        try:
            payload = json.loads(
                self.rfile.read(length).decode("utf-8"),
                parse_constant=_reject_json_constant,
            )
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
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
            if parsed.path == "/health":
                self._send_json(HTTPStatus.OK, {"status": "ok"})
                return
            if parsed.path == '/api/arm/document':
                from fruit_robviz_bridge.arm_editor import document
                self._send_json(HTTPStatus.OK, document(self._arm_path()))
                return
            if parsed.path == "/api/image-settings":
                snapshot = self.server.image_settings.snapshot()
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "settings": snapshot,
                        "version": snapshot["version"],
                    },
                )
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
                            "formal": editor_file.formal,
                            "version": (
                                document_version(editor_file.waypoint_file)
                                if editor_file.waypoint_file.is_file()
                                else None
                            ),
                            "map": load_map_info(editor_file),
                        }
                    )
                self._send_json(HTTPStatus.OK, {"files": files})
                return
            if parsed.path == "/api/document":
                editor_file = self._catalog_file(query)
                document, points, version = load_waypoint_snapshot(editor_file)
                template = load_template_geometry(editor_file)
                points = order_actual_points(
                    points, template, editor_file.formal
                )
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "id": editor_file.file_id,
                        "label": editor_file.label,
                        "schema_version": document["schema_version"],
                        "frame_id": document.get("frame_id", ""),
                        "points": points,
                        "template": template,
                        "formal": editor_file.formal,
                        "version": version,
                        "map": load_map_info(editor_file),
                    },
                )
                return
            static_file = self._static_file(parsed.path)
            if static_file is None and (
                parsed.path == "/" or parsed.path == "/index.html"
            ):
                root = self.server.editor_config.static_dir
                static_index = root / "index.html"
                if static_index.is_file():
                    static_file = static_index.resolve()
            if static_file is None and (
                "text/html" in self.headers.get("Accept", "")
                or "." not in Path(parsed.path).name
            ):
                root = self.server.editor_config.static_dir
                static_index = root / "index.html"
                if static_index.is_file():
                    static_file = static_index.resolve()
            if static_file is not None:
                self._send_file(static_file)
                return
            raise RequestError(HTTPStatus.NOT_FOUND, "接口不存在")
        except RequestError as error:
            self._send_error(error)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            self._send_error(
                RequestError(HTTPStatus.UNPROCESSABLE_ENTITY, str(error))
            )
        except Exception:
            logging.exception("Unhandled RobViz web GET error")
            self._send_error(
                RequestError(
                    HTTPStatus.INTERNAL_SERVER_ERROR, "服务器内部错误"
                )
            )

    def do_PUT(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        try:
            self._check_same_origin()
            if parsed.path == '/api/arm/presets':
                from fruit_robviz_bridge.arm_editor import save_presets
                result = save_presets(
                    self._arm_path(), self.server.editor_config.backup_dir / 'arm',
                    self._read_json_body(), _document_lock, VersionConflict,
                )
                self._send_json(HTTPStatus.OK, result)
                return
            if parsed.path == "/api/image-settings":
                payload = self._read_json_body()
                settings = self.server.image_settings.update(payload)
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "applied": True,
                        "settings": settings,
                        "version": settings["version"],
                    },
                )
                return
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
            version = payload.get("version")
            if not isinstance(version, str) or not version:
                raise RequestError(
                    HTTPStatus.BAD_REQUEST,
                    "缺少 waypoint 文件版本，请重新加载",
                )
            backup_path, new_version = save_waypoint_document(
                editor_file,
                self.server.editor_config.backup_dir,
                payload.get("points"),
                version,
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
                    "version": new_version,
                },
            )
        except RequestError as error:
            self._send_error(error)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            self._send_error(
                RequestError(HTTPStatus.UNPROCESSABLE_ENTITY, str(error))
            )
        except Exception:
            logging.exception("Unhandled RobViz web PUT error")
            self._send_error(
                RequestError(
                    HTTPStatus.INTERNAL_SERVER_ERROR, "服务器内部错误"
                )
            )

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        try:
            self._check_same_origin()
            if parsed.path in ('/api/arm/pose', '/api/arm/preview'):
                from fruit_robviz_bridge.arm_editor import calculate
                self._arm_path()
                result = calculate(self._read_json_body(), parsed.path.endswith('/preview'))
                self._send_json(HTTPStatus.OK, result)
                return
            if parsed.path != "/api/image-settings/defaults":
                raise RequestError(HTTPStatus.NOT_FOUND, "接口不存在")
            payload = self._read_json_body()
            settings, backup_path = self.server.image_settings.save(
                payload, self.server.editor_config.backup_dir
            )
            self._send_json(
                HTTPStatus.OK,
                {
                    "saved": True,
                    "status": "saved_for_next_start",
                    "backup": str(backup_path) if backup_path else None,
                    "settings": settings,
                    "version": settings["version"],
                },
            )
        except RequestError as error:
            self._send_error(error)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            self._send_error(
                RequestError(HTTPStatus.UNPROCESSABLE_ENTITY, str(error))
            )
        except Exception:
            logging.exception("Unhandled RobViz web POST error")
            self._send_error(
                RequestError(
                    HTTPStatus.INTERNAL_SERVER_ERROR, "服务器内部错误"
                )
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
        static_index = config.static_dir / "index.html"
        if not static_index.is_file():
            raise ValueError(
                f"静态目录缺少 index.html: {static_index}"
            )
        load_file_catalog(config)
        server = WaypointEditorHTTPServer(config)
    except Exception as error:
        logging.error("RobViz web 启动失败: %s", error)
        return 1

    logging.info(
        "RobViz web listening on http://%s:%d",
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
