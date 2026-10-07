# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Waypoint-maker recording helpers."""

import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import yaml

VALID_TEST_POINT_TYPES = ("waypoint", "goal")


@dataclass(frozen=True)
class PoseData:
    """Serializable fixed-frame pose used by the waypoint recorder."""

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
        return PoseData(
            name=name,
            x=self.x,
            y=self.y,
            z=self.z,
            qx=self.qx,
            qy=self.qy,
            qz=self.qz,
            qw=self.qw,
            yaw=self.yaw,
        )

    def to_dict(self) -> dict:
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


def quaternion_from_yaw(yaw: float) -> dict[str, float]:
    """Return a planar quaternion dictionary."""
    half = yaw * 0.5
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class TestPoint:
    """One named test-route point."""

    name: str
    point_type: str
    pose: PoseData

    def to_dict(self) -> dict:
        """Serialize to the test waypoint YAML shape."""
        data = self.pose.renamed(self.name).to_dict()
        data["type"] = self.point_type
        return data


def mean_pose(name: str, poses: Sequence[PoseData]) -> PoseData:
    """Return the arithmetic position and circular yaw mean of sampled poses."""
    if not poses:
        raise ValueError("at least one pose sample is required")
    count = float(len(poses))
    x = sum(pose.x for pose in poses) / count
    y = sum(pose.y for pose in poses) / count
    z = sum(pose.z for pose in poses) / count
    yaw = math.atan2(
        sum(math.sin(pose.yaw) for pose in poses),
        sum(math.cos(pose.yaw) for pose in poses),
    )
    q = quaternion_from_yaw(yaw)
    return PoseData(
        name=name,
        x=x,
        y=y,
        z=z,
        qx=q["x"],
        qy=q["y"],
        qz=q["z"],
        qw=q["w"],
        yaw=yaw,
    )


class TestPointStore:
    """Maintain named test points and the next-goal test cursor."""

    __test__ = False

    def __init__(self, output_path: Path, frame_id: str = "map") -> None:
        self.output_path = output_path
        self.frame_id = str(frame_id).strip()
        if self.frame_id not in ("map", "odom"):
            raise ValueError("waypoint frame_id must be map or odom")
        self._points: List[TestPoint] = []
        self._cursor = 0
        self._undo_snapshots: List[Tuple[List[TestPoint], int]] = []

    @property
    def points(self) -> List[TestPoint]:
        """Return recorded test points in order."""
        return list(self._points)

    @property
    def cursor(self) -> int:
        """Return the next untested point index."""
        return self._cursor

    def append(self, name: str, point_type: str, pose: PoseData) -> TestPoint:
        """Append one named test point."""
        cleaned_name = name.strip()
        cleaned_type = point_type.strip()
        if not cleaned_name:
            raise ValueError("test point name must not be empty")
        if cleaned_type not in VALID_TEST_POINT_TYPES:
            raise ValueError("point_type must be waypoint or goal")
        if any(point.name == cleaned_name for point in self._points):
            raise ValueError(f"test point already exists: {cleaned_name}")
        self._remember()
        point = TestPoint(
            name=cleaned_name,
            point_type=cleaned_type,
            pose=pose.renamed(cleaned_name),
        )
        self._points.append(point)
        return point

    def upsert(
        self,
        name: str,
        point_type: str,
        pose: PoseData,
        overwrite: bool = False,
        order: Optional[Sequence[str]] = None,
    ) -> TestPoint:
        """Append a point or replace one after explicit overwrite confirmation."""
        cleaned_name = name.strip()
        cleaned_type = point_type.strip()
        if not cleaned_name:
            raise ValueError("test point name must not be empty")
        if cleaned_type not in VALID_TEST_POINT_TYPES:
            raise ValueError("point_type must be waypoint or goal")
        rank = {item: index for index, item in enumerate(order or ())}
        if order is not None:
            if cleaned_name not in rank:
                raise ValueError(f"test point name is not in order: {cleaned_name}")
        replacement = TestPoint(
            name=cleaned_name,
            point_type=cleaned_type,
            pose=pose.renamed(cleaned_name),
        )
        for index, current in enumerate(self._points):
            if current.name != cleaned_name:
                continue
            if not overwrite:
                raise ValueError(
                    f"test point already exists: {cleaned_name}; "
                    "confirm_overwrite=true is required"
                )
            self._remember()
            self._points[index] = replacement
            return replacement
        self._remember()
        if order is None:
            self._points.append(replacement)
            return replacement
        insert_at = next(
            (
                index
                for index, current in enumerate(self._points)
                if rank.get(current.name, len(rank)) > rank[cleaned_name]
            ),
            len(self._points),
        )
        self._points.insert(insert_at, replacement)
        return replacement

    def next_auto_name(self, prefix: str) -> str:
        """Return the next name after the highest recorded profile index."""
        cleaned_prefix = prefix.strip()
        if not cleaned_prefix:
            raise ValueError("auto_name_prefix must not be empty")
        highest_index = 0
        for point in self._points:
            suffix = point.name[len(cleaned_prefix):]
            if point.name.startswith(cleaned_prefix) and suffix.isdigit():
                highest_index = max(highest_index, int(suffix))
        return f"{cleaned_prefix}{highest_index + 1}"

    def undo(self) -> Optional[TestPoint]:
        """Undo the latest test point and keep the cursor valid."""
        if self._undo_snapshots:
            previous_points, previous_cursor = self._undo_snapshots.pop()
            current_names = {point.name for point in self._points}
            previous_names = {point.name for point in previous_points}
            changed = next(
                (
                    point
                    for point in self._points
                    if point.name not in previous_names
                ),
                None,
            )
            if changed is None and current_names == previous_names:
                changed = next(
                    (
                        point
                        for point in self._points
                        if next(
                            (
                                old
                                for old in previous_points
                                if old.name == point.name
                            ),
                            None,
                        ) != point
                    ),
                    None,
                )
            if changed is None:
                changed = next(
                    (
                        point
                        for point in previous_points
                        if point.name not in current_names
                    ),
                    None,
                )
            self._points = list(previous_points)
            self._cursor = min(previous_cursor, len(self._points))
            return changed
        if not self._points:
            return None
        removed = self._points.pop()
        self._cursor = min(self._cursor, len(self._points))
        return removed

    def clear(self) -> None:
        """Clear all test points."""
        self._points.clear()
        self._cursor = 0

    def reset_cursor(self) -> None:
        """Restart test execution from the first point."""
        self._cursor = 0

    def advance_cursor(self, cursor: int) -> None:
        """Mark test points before cursor as already run."""
        self._cursor = min(max(cursor, 0), len(self._points))

    def load(self, input_path: Path) -> None:
        """Load test points from a waypoint-maker YAML file."""
        with input_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError("test waypoint YAML must be a mapping")
        if document.get("schema_version") != 1:
            raise ValueError("test waypoint YAML schema_version must be 1")
        if document.get("frame_id") != self.frame_id:
            raise ValueError(
                f"waypoint YAML frame_id must be {self.frame_id}"
            )
        raw_points = document.get("points")
        if not isinstance(raw_points, list):
            raise ValueError("test waypoint YAML points must be a list")

        loaded: List[TestPoint] = []
        seen = set()
        for index, item in enumerate(raw_points, start=1):
            if not isinstance(item, dict):
                raise ValueError(f"test point {index} must be a mapping")
            name = str(item.get("name", "")).strip()
            point_type = str(item.get("type", "")).strip()
            if not name:
                raise ValueError(f"test point {index} name must not be empty")
            if name in seen:
                raise ValueError(f"test point already exists: {name}")
            if point_type not in VALID_TEST_POINT_TYPES:
                raise ValueError("point_type must be waypoint or goal")
            position = item.get("position")
            orientation = item.get("orientation")
            if not isinstance(position, dict) or not isinstance(orientation, dict):
                raise ValueError(f"test point {name} must include pose data")
            if "yaw" in item:
                yaw = float(item["yaw"])
            else:
                yaw = yaw_from_quaternion(
                    float(orientation["x"]),
                    float(orientation["y"]),
                    float(orientation["z"]),
                    float(orientation["w"]),
                )
            pose = PoseData(
                name=name,
                x=float(position["x"]),
                y=float(position["y"]),
                z=float(position.get("z", 0.0)),
                qx=float(orientation["x"]),
                qy=float(orientation["y"]),
                qz=float(orientation["z"]),
                qw=float(orientation["w"]),
                yaw=yaw,
            )
            loaded.append(TestPoint(name=name, point_type=point_type, pose=pose))
            seen.add(name)

        self.output_path = input_path
        self._points = loaded
        self._cursor = 0
        self._undo_snapshots = []

    def next_test_segment(self) -> Tuple[List[PoseData], int]:
        """Return waypoints through the next goal pose and the success cursor."""
        segment = []
        for index in range(self._cursor, len(self._points)):
            segment.append(self._points[index].pose)
            if self._points[index].point_type == "goal":
                return segment, index + 1
        raise ValueError("no next goal point to test")

    def save(self, allow_empty: bool = False) -> None:
        """Write the test waypoint YAML."""
        if not self._points and not allow_empty:
            raise ValueError("no test points to save")
        document = {
            "schema_version": 1,
            "frame_id": self.frame_id,
            "created_at": datetime.now().astimezone().isoformat(),
            "points": [point.to_dict() for point in self._points],
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", encoding="utf-8") as output:
            yaml.safe_dump(document, output, allow_unicode=True, sort_keys=False)

    def _remember(self) -> None:
        self._undo_snapshots.append((list(self._points), self._cursor))
        if len(self._undo_snapshots) > 100:
            self._undo_snapshots.pop(0)
