# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Route template generation for the fixed fruit-picking field."""

import json
import math
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Union

import yaml


REQUIRED_ANCHORS = (
    "HOME",
    "A_FIRST",
    "A_LAST",
    "B_QR",
    "B_FIRST",
    "B_LAST",
    "C_QR",
    "C1",
    "C4",
    "C9",
    "C12",
)


@dataclass(frozen=True)
class PoseData:
    """Serializable map-frame pose."""

    name: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    yaw: float

    def renamed(self, name: str) -> "PoseData":
        """Return the same pose with a different route name."""
        return replace(self, name=name)

    def to_dict(self) -> Dict[str, object]:
        """Serialize to the route YAML pose shape."""
        return {
            "name": self.name,
            "position": {"x": self.x, "y": self.y, "z": self.z},
            "orientation": {
                "x": self.qx,
                "y": self.qy,
                "z": self.qz,
                "w": self.qw,
            },
            "yaw": round(self.yaw, 6),
        }

    @classmethod
    def from_pose_stamped(cls, name: str, message) -> "PoseData":
        """Create PoseData from a PoseStamped-like message."""
        pose = message.pose
        orientation = pose.orientation
        return cls(
            name=name,
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            qx=float(orientation.x),
            qy=float(orientation.y),
            qz=float(orientation.z),
            qw=float(orientation.w),
            yaw=yaw_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )


@dataclass(frozen=True)
class PickTask:
    """One C-zone task decoded from the QR plan."""

    sequence: int
    position_id: int
    target: str


def quaternion_from_yaw(yaw: float) -> Dict[str, float]:
    """Return a planar quaternion dictionary."""
    half = yaw * 0.5
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_distance(a: float, b: float) -> float:
    """Return the shortest absolute distance between two angles."""
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def _copy_pose(name: str, x: float, y: float, z: float, source: PoseData) -> PoseData:
    return PoseData(
        name=name,
        x=x,
        y=y,
        z=z,
        qx=source.qx,
        qy=source.qy,
        qz=source.qz,
        qw=source.qw,
        yaw=source.yaw,
    )


def _ensure_yaw_close(poses: Iterable[PoseData], tolerance: float) -> None:
    items = list(poses)
    base = items[0].yaw
    for pose in items[1:]:
        if angle_distance(base, pose.yaw) > tolerance:
            raise ValueError("anchor yaw values are not consistent")


def generate_ab_points(
    prefix: str,
    first: PoseData,
    last: PoseData,
    yaw_tolerance: float = 0.35,
) -> List[PoseData]:
    """Generate four evenly spaced A/B stop points from first and last anchors."""
    _ensure_yaw_close((first, last), yaw_tolerance)
    points = []
    for index in range(4):
        ratio = index / 3.0
        points.append(
            _copy_pose(
                f"{prefix}{index + 1}",
                first.x + (last.x - first.x) * ratio,
                first.y + (last.y - first.y) * ratio,
                first.z + (last.z - first.z) * ratio,
                first,
            )
        )
    return points


def generate_c_candidates(
    c1: PoseData,
    c4: PoseData,
    c9: PoseData,
    c12: PoseData,
    yaw_tolerance: float = 0.35,
) -> Dict[str, PoseData]:
    """Generate C1..C12 from four recorded C corner anchors."""
    _ensure_yaw_close((c1, c4, c9, c12), yaw_tolerance)
    candidates: Dict[str, PoseData] = {}
    for col in range(3):
        u = col / 2.0
        for row in range(4):
            v = row / 3.0
            left_x = c1.x + (c4.x - c1.x) * v
            left_y = c1.y + (c4.y - c1.y) * v
            left_z = c1.z + (c4.z - c1.z) * v
            right_x = c9.x + (c12.x - c9.x) * v
            right_y = c9.y + (c12.y - c9.y) * v
            right_z = c9.z + (c12.z - c9.z) * v
            position_id = col * 4 + row + 1
            candidates[f"C{position_id}"] = _copy_pose(
                f"C{position_id}",
                left_x + (right_x - left_x) * u,
                left_y + (right_y - left_y) * u,
                left_z + (right_z - left_z) * u,
                c1,
            )
    return candidates


def parse_c_pick_plan(plan: Union[str, Mapping[str, object]]) -> List[PickTask]:
    """Validate a C QR plan and return tasks in original QR order."""
    data = json.loads(plan) if isinstance(plan, str) else dict(plan)
    if data.get("zone") != "C":
        raise ValueError("expected a C-zone pick plan")
    raw_tasks = list(data.get("tasks", []))
    if len(raw_tasks) != 8:
        raise ValueError("C pick plan must contain exactly 8 tasks")

    tasks = []
    seen = set()
    for index, item in enumerate(raw_tasks, start=1):
        sequence = int(item["sequence"])
        position_id = int(item["position_id"])
        if sequence != index:
            raise ValueError("C pick plan sequence must be contiguous")
        if position_id < 1 or position_id > 12:
            raise ValueError("C position_id must be between 1 and 12")
        if position_id in seen:
            raise ValueError("C position_id values must be unique")
        seen.add(position_id)
        tasks.append(
            PickTask(
                sequence=sequence,
                position_id=position_id,
                target=str(item.get("target", "")),
            )
        )
    return tasks


def build_c_route(
    c_qr: PoseData,
    candidates: Mapping[str, PoseData],
    tasks: Sequence[PickTask],
) -> List[PoseData]:
    """Build C_QR followed by QR-selected C candidate poses."""
    route = [c_qr]
    for task in tasks:
        route.append(candidates[f"C{task.position_id}"])
    return route


def mission_route(
    anchors: Mapping[str, PoseData],
    generated: Mapping[str, object],
    c_tasks: Sequence[PickTask],
) -> List[PoseData]:
    """Build the default full match route; HOME is only the final target."""
    route: List[PoseData] = []
    route.extend(generated["A"])
    route.append(anchors["B_QR"])
    route.extend(build_c_route(anchors["C_QR"], generated["C"], c_tasks))
    route.extend(generated["B"])
    route.append(anchors["HOME"])
    return route


class AnchorSession:
    """Record the fixed eleven anchor poses and generate the route template."""

    required_anchor_names = REQUIRED_ANCHORS

    def __init__(
        self,
        anchors: Optional[Sequence[PoseData]] = None,
        yaw_tolerance: float = 0.35,
    ) -> None:
        self._anchors = list(anchors or [])
        self._yaw_tolerance = yaw_tolerance

    @property
    def anchors(self) -> List[PoseData]:
        """Return recorded anchors in click order."""
        return list(self._anchors)

    def next_prompt(self) -> str:
        """Return a short prompt for the next required click."""
        if len(self._anchors) >= len(REQUIRED_ANCHORS):
            return "All anchors recorded"
        name = REQUIRED_ANCHORS[len(self._anchors)]
        return f"Click {len(self._anchors) + 1}/11: {name}"

    def append(self, pose: PoseData) -> PoseData:
        """Append the next expected anchor, assigning its canonical name."""
        if len(self._anchors) >= len(REQUIRED_ANCHORS):
            raise ValueError("all anchors are already recorded")
        expected = REQUIRED_ANCHORS[len(self._anchors)]
        recorded = pose.renamed(expected)
        self._anchors.append(recorded)
        return recorded

    def undo(self) -> Optional[PoseData]:
        """Remove the most recently recorded anchor."""
        if not self._anchors:
            return None
        return self._anchors.pop()

    def clear(self) -> None:
        """Remove all recorded anchors."""
        self._anchors.clear()

    def generated(self) -> Dict[str, object]:
        """Generate A/B/C route points from the recorded anchors."""
        anchors = {pose.name: pose for pose in self._anchors}
        missing = [name for name in REQUIRED_ANCHORS if name not in anchors]
        if missing:
            raise ValueError(f"missing anchor: {missing[0]}")
        return {
            "A": generate_ab_points(
                "A",
                anchors["A_FIRST"],
                anchors["A_LAST"],
                self._yaw_tolerance,
            ),
            "B": generate_ab_points(
                "B",
                anchors["B_FIRST"],
                anchors["B_LAST"],
                self._yaw_tolerance,
            ),
            "C": generate_c_candidates(
                anchors["C1"],
                anchors["C4"],
                anchors["C9"],
                anchors["C12"],
                self._yaw_tolerance,
            ),
        }

    def to_document(self) -> Dict[str, object]:
        """Serialize anchors and generated points to the route template YAML."""
        generated = self.generated()
        return {
            "schema_version": 1,
            "frame_id": "map",
            "created_at": datetime.now().astimezone().isoformat(),
            "anchors": {pose.name: pose.to_dict() for pose in self._anchors},
            "generated": {
                "A": [pose.to_dict() for pose in generated["A"]],
                "B": [pose.to_dict() for pose in generated["B"]],
                "C": {
                    name: pose.to_dict()
                    for name, pose in generated["C"].items()
                },
            },
        }

    def save(self, path: Path) -> None:
        """Write the complete route template YAML."""
        document = self.to_document()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as output:
            yaml.safe_dump(document, output, allow_unicode=True, sort_keys=False)


def load_pose(data: Mapping[str, object], fallback_name: str) -> PoseData:
    """Load one PoseData from the saved YAML shape."""
    position = data["position"]
    orientation = data["orientation"]
    yaw = float(data.get("yaw", yaw_from_quaternion(
        orientation["x"],
        orientation["y"],
        orientation["z"],
        orientation["w"],
    )))
    return PoseData(
        name=str(data.get("name", fallback_name)),
        x=float(position["x"]),
        y=float(position["y"]),
        z=float(position["z"]),
        qx=float(orientation["x"]),
        qy=float(orientation["y"]),
        qz=float(orientation["z"]),
        qw=float(orientation["w"]),
        yaw=yaw,
    )


def load_route_template(path: Path) -> Dict[str, object]:
    """Load anchors and generated points from a saved route template."""
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    anchors = {
        name: load_pose(data, name)
        for name, data in raw.get("anchors", {}).items()
    }
    generated = raw.get("generated", {})
    return {
        "frame_id": raw.get("frame_id", "map"),
        "anchors": anchors,
        "generated": {
            "A": [load_pose(item, item.get("name", "A")) for item in generated.get("A", [])],
            "B": [load_pose(item, item.get("name", "B")) for item in generated.get("B", [])],
            "C": {
                name: load_pose(data, name)
                for name, data in generated.get("C", {}).items()
            },
        },
    }
