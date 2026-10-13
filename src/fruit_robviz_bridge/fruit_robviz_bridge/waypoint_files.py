"""Waypoint document validation and reliable storage shared by HTTP and ROS."""

from contextlib import contextmanager
import copy
from dataclasses import dataclass
from datetime import datetime
import hashlib
import math
import os
from pathlib import Path
import shutil
import stat
import tempfile
from typing import Any, Mapping

import yaml

from fruit_robviz_bridge.recorder import yaw_from_quaternion


VALID_POINT_TYPES = ("waypoint", "goal")
MAX_POINT_COUNT = 5000


class VersionConflict(ValueError):
    """The document changed after the caller loaded its snapshot."""


@dataclass(frozen=True)
class EditorFile:
    """One allowlisted waypoint document and its associated template."""

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


def document_version(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def _required_mapping(document: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(document, Mapping):
        raise ValueError(f"{label} 必须是 YAML mapping")
    return document


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
    orientation = _required_mapping(item.get("orientation"), f"点 {name} 的 orientation")
    return yaw_from_quaternion(*(
        _finite_number(orientation.get(axis), f"点 {name} 的 orientation.{axis}")
        for axis in ("x", "y", "z", "w")
    ))


def load_waypoint_document(editor_file: EditorFile, content: bytes | None = None):
    """Parse a waypoint or template document without HTTP dependencies."""
    if content is None:
        content = editor_file.waypoint_file.read_bytes()
    document = _required_mapping(yaml.safe_load(content), editor_file.waypoint_file.name)
    is_template = editor_file.mode == "template"
    kind = "template" if is_template else "waypoint"
    if document.get("schema_version") != 1:
        raise ValueError(f"{kind} YAML 的 schema_version 必须为 1")
    if not is_template and document.get("frame_id") != editor_file.frame_id:
        raise ValueError(
            f"{editor_file.waypoint_file.name} 的 frame_id 必须为 {editor_file.frame_id}"
        )
    raw_points = document.get("points")
    if not isinstance(raw_points, list):
        raise ValueError(f"{kind} YAML 的 points 必须是列表")
    if len(raw_points) > MAX_POINT_COUNT:
        raise ValueError(f"waypoint 数量不能超过 {MAX_POINT_COUNT}")
    points = []
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
        point = dict(name=name, type=point_type, x=0.0, y=0.0, z=0.0, yaw_deg=0.0, extra={})
        if not is_template:
            position = _required_mapping(item.get("position"), f"点 {name} 的 position")
            extra = copy.deepcopy(dict(item))
            for key in ("name", "type", "position", "orientation", "yaw"):
                extra.pop(key, None)
            point.update(
                x=_finite_number(position.get("x"), f"点 {name} 的 x"),
                y=_finite_number(position.get("y"), f"点 {name} 的 y"),
                z=_finite_number(position.get("z", 0.0), f"点 {name} 的 z"),
                yaw_deg=math.degrees(_point_yaw(item, name)), extra=extra,
            )
        points.append(point)
        names.add(name)
    return dict(document), points


def load_waypoint_snapshot(editor_file: EditorFile):
    """Bind parsed points and version to the same opened byte snapshot."""
    content = editor_file.waypoint_file.read_bytes()
    document, points = load_waypoint_document(editor_file, content)
    return document, points, hashlib.sha256(content).hexdigest()


def load_template_geometry(editor_file: EditorFile) -> list[dict]:
    if not editor_file.formal:
        return []
    with editor_file.template_file.open(encoding="utf-8") as stream:
        document = _required_mapping(yaml.safe_load(stream), "waypoint template")
    if document.get("schema_version") != 1:
        raise ValueError("waypoint template 的 schema_version 必须为 1")
    points = document.get("points")
    if not isinstance(points, list):
        raise ValueError("waypoint template 的 points 必须是列表")
    reference_file = EditorFile(
        "odom:reference", "odom", "odometry", "odom", editor_file.reference_file,
        editor_file.template_file, editor_file.reference_file, True,
    )
    reference_points = []
    if editor_file.reference_file.is_file():
        _, reference_points = load_waypoint_document(reference_file)
    references = {point["name"]: point for point in reference_points}
    a1 = references.get("A1")
    result = []
    seen = set()
    for index, raw_item in enumerate(points, start=1):
        item = _required_mapping(raw_item, f"template 第 {index} 个点")
        name = str(item.get("name", "")).strip()
        point_type = str(item.get("type", "")).strip()
        if not name:
            raise ValueError(f"template 第 {index} 个点名称不能为空")
        if name in seen:
            raise ValueError(f"template 点名称重复: {name}")
        if point_type not in VALID_POINT_TYPES:
            raise ValueError(f"template 点 {name} 的 type 必须是 waypoint 或 goal")
        reference = references.get(name)
        has_geometry = reference is not None and a1 is not None
        result.append({
            "index": index, "name": name, "type": point_type,
            "dx": reference["x"] - a1["x"] if has_geometry else None,
            "dy": reference["y"] - a1["y"] if has_geometry else None,
            "dz": reference["z"] - a1["z"] if has_geometry else None,
            "yaw_deg": reference["yaw_deg"] if has_geometry else None,
        })
        seen.add(name)
    unknown = set(references) - seen
    if unknown:
        raise ValueError(f"正式 waypoint YAML 存在 template 外名称: {', '.join(sorted(unknown))}")
    return result


def order_actual_points(points: list[dict], template: list[dict], formal: bool = True):
    if not formal:
        return list(points)
    order = {point["name"]: index for index, point in enumerate(template)}
    unknown = [point["name"] for point in points if point["name"] not in order]
    if unknown:
        raise ValueError(f"waypoint 名称不在 template 中: {', '.join(unknown)}")
    return sorted(points, key=lambda point: order[point["name"]])


def _serialize_points(raw_points: Any, template: list[dict], formal: bool, is_template: bool):
    if not isinstance(raw_points, list):
        raise ValueError("points 必须是列表")
    if len(raw_points) > MAX_POINT_COUNT:
        raise ValueError(f"waypoint 数量不能超过 {MAX_POINT_COUNT}")
    order = {point["name"]: index for index, point in enumerate(template)}
    types = {point["name"]: point["type"] for point in template}
    serialized = []
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
            label = "模板点" if is_template else "点"
            raise ValueError(f"{label}名称重复: {name}")
        if formal and name not in order:
            raise ValueError(f"点名称不在 template 中: {name}")
        if point_type not in VALID_POINT_TYPES:
            raise ValueError(f"点 {name} 的 type 必须是 waypoint 或 goal")
        if formal and types[name] != point_type:
            raise ValueError(f"点 {name} 的 type 与 template 不一致")
        point = {"name": name, "type": point_type}
        if not is_template:
            x = _finite_number(item.get("x"), f"点 {name} 的 x")
            y = _finite_number(item.get("y"), f"点 {name} 的 y")
            z = _finite_number(item.get("z", 0.0), f"点 {name} 的 z")
            degrees = (_finite_number(item.get("yaw_deg"), f"点 {name} 的 yaw") + 180) % 360 - 180
            if abs(degrees) < 0.5e-12:
                degrees = 0.0
            yaw = round(math.radians(degrees), 6)
            extra = copy.deepcopy(_required_mapping(item.get("extra", {}), f"点 {name} 的 extra"))
            point = dict(extra)
            for key in ("name", "type", "position", "orientation", "yaw"):
                point.pop(key, None)
            point.update(
                name=name, position={"x": x, "y": y, "z": z},
                orientation={"x": 0.0, "y": 0.0, "z": math.sin(yaw * 0.5),
                             "w": math.cos(yaw * 0.5)}, yaw=yaw, type=point_type,
            )
        serialized.append(point)
        names.add(name)
    if formal:
        serialized.sort(key=lambda point: order[point["name"]])
    return serialized


def save_waypoint_document(
    editor_file: EditorFile, backup_root: Path, raw_points: Any,
    expected_version: str | None = None,
):
    """Validate, check version, back up and atomically replace one document."""
    path = editor_file.waypoint_file
    with _document_lock(path):
        document, _, current_version = load_waypoint_snapshot(editor_file)
        if expected_version is not None and expected_version != current_version:
            kind = "template" if editor_file.mode == "template" else "waypoint"
            raise VersionConflict(f"{kind} 文件已被其他进程修改，请重新加载")
        is_template = editor_file.mode == "template"
        template = [] if is_template else load_template_geometry(editor_file)
        document["points"] = _serialize_points(
            raw_points, template, editor_file.formal and not is_template, is_template,
        )
        backup_dir = backup_root / editor_file.profile
        backup_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        backup_path = backup_dir / f"{path.stem}_{stamp}{path.suffix}"
        shutil.copy2(path, backup_path)
        file_mode = stat.S_IMODE(path.stat().st_mode)
        fd, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=path.parent, text=True,
        )
        temporary_path = Path(temporary_name)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as output:
                yaml.safe_dump(document, output, allow_unicode=True, sort_keys=False)
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
        return backup_path, document_version(path)
