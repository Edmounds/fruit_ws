# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Route-template and waypoint-maker recording helpers."""

import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import yaml

from fruit_navigation.route_model import (
    AnchorSession,
    PoseData,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


VALID_TEST_POINT_TYPES = ("waypoint", "goal")


def new_session_path(output_directory: Path, now: datetime) -> Path:
    """Return a timestamped route-template path without replacing a file."""
    stem = f"competition_waypoints_{now.strftime('%Y%m%d_%H%M%S')}"
    candidate = output_directory / f"{stem}.yaml"
    suffix = 1
    while candidate.exists():
        candidate = output_directory / f"{stem}_{suffix:02d}.yaml"
        suffix += 1
    return candidate


class WaypointTemplateStore:
    """Maintain one eleven-anchor recording session and save generated points."""

    def __init__(self, output_path: Path, yaw_tolerance: float) -> None:
        self.output_path = output_path
        self._session = AnchorSession(yaw_tolerance=yaw_tolerance)

    @property
    def prompt(self) -> str:
        """Return the next click prompt."""
        return self._session.next_prompt()

    @property
    def anchors(self):
        """Return recorded anchors in click order."""
        return self._session.anchors

    def append(self, pose: PoseData) -> PoseData:
        """Append one RViz-clicked pose as the next canonical anchor."""
        return self._session.append(pose)

    def undo(self) -> Optional[PoseData]:
        """Undo the latest anchor."""
        return self._session.undo()

    def clear(self) -> None:
        """Clear the current session."""
        self._session.clear()

    def save(self) -> None:
        """Validate anchors, generate points, and write YAML."""
        self._session.save(self.output_path)


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

    def __init__(self, output_path: Path) -> None:
        self.output_path = output_path
        self._points: List[TestPoint] = []
        self._cursor = 0

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
        point = TestPoint(
            name=cleaned_name,
            point_type=cleaned_type,
            pose=pose.renamed(cleaned_name),
        )
        self._points.append(point)
        return point

    def undo(self) -> Optional[TestPoint]:
        """Undo the latest test point and keep the cursor valid."""
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
        if document.get("frame_id") != "map":
            raise ValueError("test waypoint YAML frame_id must be map")
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

    def next_test_segment(self) -> Tuple[List[PoseData], int]:
        """Return waypoints through the next goal pose and the success cursor."""
        segment = []
        for index in range(self._cursor, len(self._points)):
            segment.append(self._points[index].pose)
            if self._points[index].point_type == "goal":
                return segment, index + 1
        raise ValueError("no next goal point to test")

    def save(self) -> None:
        """Write the test waypoint YAML."""
        if not self._points:
            raise ValueError("no test points to save")
        document = {
            "schema_version": 1,
            "frame_id": "map",
            "created_at": datetime.now().astimezone().isoformat(),
            "points": [point.to_dict() for point in self._points],
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", encoding="utf-8") as output:
            yaml.safe_dump(document, output, allow_unicode=True, sort_keys=False)
