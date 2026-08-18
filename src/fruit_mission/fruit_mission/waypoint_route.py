"""Formal waypoint YAML loading and C-zone route planning."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

import yaml

from fruit_mission.logic import expected_ripe_type


A_GOAL_PATTERN = re.compile(r"^A(\d+)$")
C_GOAL_PATTERN = re.compile(r"^C_(\d+)_(\d+)$")
C_ENTRANCE = "to_C_entrance"
C_QR = "C_QR"
B_QR = "B_QR"
C_OTHER_ENTRY = "temp_point"
C_PASSWAY_1 = "C_passway_1"
C_PASSWAY_2 = "C_passway_2"

ENTRY_LANE = ("C_8_12", "C_7_11", "C_6_10", "C_5_9")
OTHER_LANE = ("C_1_5", "C_2_6", "C_3_7", "C_4_8")
LANES = {
    "entry": ENTRY_LANE,
    "other": OTHER_LANE,
}
LANE_BRIDGES = {
    ("entry", "other"): (C_PASSWAY_1, C_PASSWAY_2),
    ("other", "entry"): (C_PASSWAY_2, C_PASSWAY_1),
}


@dataclass(frozen=True)
class WaypointPose:
    """One named map-frame waypoint pose."""

    name: str
    point_type: str
    order: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class CStopCandidate:
    """One C-position candidate stop parsed from a composite goal name."""

    position_id: int
    goal_name: str
    position_side: str
    camera_side: str
    lane: str
    lane_index: int
    yaml_order: int

    @property
    def preferred_side(self) -> str:
        """Compatibility name for the camera side used by mission execution."""
        return self.camera_side


@dataclass(frozen=True)
class _RouteCost:
    distance: float
    lane_changes: int
    yaml_order: int

    def key(self) -> tuple[float, int, int]:
        return (self.distance, self.lane_changes, self.yaml_order)


class FormalWaypointRoute:
    """Route helper built from the formal waypoint-maker YAML."""

    def __init__(self, points: Sequence[WaypointPose]) -> None:
        if not points:
            raise ValueError("waypoint file must contain at least one point")
        self._points = list(points)
        self._by_name: Dict[str, WaypointPose] = {}
        for point in self._points:
            if point.name in self._by_name:
                raise ValueError(f"duplicate waypoint name: {point.name}")
            self._by_name[point.name] = point
        self._adjust_c_other_entry_yaw()
        self._c_candidates = self._parse_c_candidates()
        self._validate_required_points()

    @classmethod
    def from_file(cls, path: Path) -> "FormalWaypointRoute":
        """Load the formal waypoint-maker YAML from disk."""
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError("waypoint YAML must be a mapping")
        if document.get("schema_version") != 1:
            raise ValueError("waypoint YAML schema_version must be 1")
        if document.get("frame_id") != "map":
            raise ValueError("waypoint YAML frame_id must be map")
        raw_points = document.get("points")
        if not isinstance(raw_points, list):
            raise ValueError("waypoint YAML points must be a list")
        return cls(_load_points(raw_points))

    def pose(self, name: str) -> WaypointPose:
        """Return one waypoint pose by name."""
        try:
            return self._by_name[name]
        except KeyError as exc:
            raise KeyError(f"unknown waypoint: {name}") from exc

    def ordered_names_to(self, target_name: str, cursor: int) -> tuple[List[str], int]:
        """Return YAML-order waypoint names from cursor through the target."""
        target = self.pose(target_name)
        start = min(max(int(cursor), 0), len(self._points))
        if target.order < start:
            return [target.name], start
        return (
            [point.name for point in self._points[start:target.order + 1]],
            target.order + 1,
        )

    def a_goal_names(self) -> List[str]:
        """Return A-zone goal names sorted by numeric suffix."""
        goals = []
        for point in self._points:
            match = A_GOAL_PATTERN.match(point.name)
            if match and point.point_type == "goal":
                goals.append((int(match.group(1)), point.order, point.name))
        if not goals:
            raise ValueError("waypoint YAML must contain A-zone goals")
        goals.sort()
        return [name for _number, _order, name in goals]

    @property
    def b_qr_name(self) -> str:
        """Return the B QR waypoint name."""
        return B_QR

    @property
    def c_qr_name(self) -> str:
        """Return the C QR waypoint name."""
        return C_QR

    def c_position_candidates(self) -> Dict[int, List[CStopCandidate]]:
        """Return C stop candidates keyed by QR position id."""
        return {
            position_id: list(candidates)
            for position_id, candidates in self._c_candidates.items()
        }

    def build_c_distribution(self, c_plan: Mapping[str, object]) -> Dict[str, object]:
        """Build the executable C task list in QR order with planned movement."""
        tasks = _c_plan_tasks(c_plan)
        route = self._choose_c_route(tasks)
        return {
            "zone": "C",
            "task_count": c_plan.get("task_count", len(route)),
            "tasks": route,
        }

    def _parse_c_candidates(self) -> Dict[int, List[CStopCandidate]]:
        candidates: Dict[int, List[CStopCandidate]] = {
            position_id: [] for position_id in range(1, 13)
        }
        lane_lookup = {
            goal_name: (lane, index)
            for lane, names in LANES.items()
            for index, goal_name in enumerate(names)
        }
        for point in self._points:
            match = C_GOAL_PATTERN.match(point.name)
            if not match or point.point_type != "goal":
                continue
            if point.name not in lane_lookup:
                continue
            lane, lane_index = lane_lookup[point.name]
            left_id = int(match.group(1))
            right_id = int(match.group(2))
            candidates[left_id].append(
                CStopCandidate(
                    left_id,
                    point.name,
                    "left",
                    _camera_side_for_pose("left", point),
                    lane,
                    lane_index,
                    point.order,
                )
            )
            candidates[right_id].append(
                CStopCandidate(
                    right_id,
                    point.name,
                    "right",
                    _camera_side_for_pose("right", point),
                    lane,
                    lane_index,
                    point.order,
                )
            )
        for position_id, options in candidates.items():
            if not options:
                raise ValueError(f"C position {position_id} has no waypoint candidate")
        return candidates

    def _validate_required_points(self) -> None:
        required = [B_QR, C_QR, C_ENTRANCE, C_OTHER_ENTRY, C_PASSWAY_1, C_PASSWAY_2]
        required.extend(name for names in LANES.values() for name in names)
        for name in required:
            self.pose(name)
        for name in (C_PASSWAY_1, C_PASSWAY_2):
            if self.pose(name).point_type != "waypoint":
                raise ValueError(f"{name} must be a waypoint")

    def _choose_c_route(
        self,
        tasks: Sequence[Mapping[str, object]],
    ) -> List[Dict[str, object]]:
        states: Dict[Optional[CStopCandidate], tuple[_RouteCost, List[Dict[str, object]]]]
        states = {None: (_RouteCost(0, 0, 0), [])}
        for task in tasks:
            position_id = int(task["position_id"])
            target_name = str(task["target"])
            next_states: Dict[
                CStopCandidate,
                tuple[_RouteCost, List[Dict[str, object]]],
            ] = {}
            for current, (current_cost, current_tasks) in states.items():
                for candidate in self._c_candidates[position_id]:
                    movement = self._movement_points(current, candidate)
                    step_cost = self._movement_cost(current, candidate, movement)
                    total_cost = _RouteCost(
                        current_cost.distance + step_cost.distance,
                        current_cost.lane_changes + step_cost.lane_changes,
                        candidate.yaml_order,
                    )
                    planned_task = {
                        "sequence": int(task["sequence"]),
                        "position_id": position_id,
                        "target_name": target_name,
                        "expected_ripe_type": expected_ripe_type(target_name),
                        "goal_name": candidate.goal_name,
                        "position_side": candidate.position_side,
                        "camera_side": candidate.camera_side,
                        "preferred_side": candidate.preferred_side,
                        "movement_points": movement,
                        "result": None,
                        "last_observation": None,
                    }
                    existing = next_states.get(candidate)
                    if existing is None or total_cost.key() < existing[0].key():
                        next_states[candidate] = (
                            total_cost,
                            current_tasks + [planned_task],
                        )
            states = next_states
        if not states:
            return []
        return min(states.values(), key=lambda item: item[0].key())[1]

    def _movement_points(
        self,
        current: Optional[CStopCandidate],
        target: CStopCandidate,
    ) -> List[str]:
        if current is None:
            return self._initial_c_movement_points(target)
        if current.lane == target.lane:
            return _lane_points_between(
                current.lane,
                current.lane_index,
                target.lane_index,
                include_start=False,
            )
        return self._lane_change_movement_points(current, target)

    def _initial_c_movement_points(self, target: CStopCandidate) -> List[str]:
        if target.lane == "entry":
            return [C_ENTRANCE] + _lane_points_between(
                "entry",
                0,
                target.lane_index,
                include_start=True,
            )
        return [C_OTHER_ENTRY] + _lane_points_between(
            "other",
            len(OTHER_LANE) - 1,
            target.lane_index,
            include_start=True,
        )

    @staticmethod
    def _lane_change_movement_points(
        current: CStopCandidate,
        target: CStopCandidate,
    ) -> List[str]:
        if current.lane == "entry":
            to_bridge = _lane_points_between(
                "entry",
                current.lane_index,
                len(ENTRY_LANE) - 1,
                include_start=False,
            )
            from_bridge = _lane_points_between(
                "other",
                0,
                target.lane_index,
                include_start=True,
            )
        else:
            to_bridge = _lane_points_between(
                "other",
                current.lane_index,
                0,
                include_start=False,
            )
            from_bridge = _lane_points_between(
                "entry",
                len(ENTRY_LANE) - 1,
                target.lane_index,
                include_start=True,
            )
        return (
            to_bridge
            + list(LANE_BRIDGES[(current.lane, target.lane)])
            + from_bridge
        )

    def _movement_cost(
        self,
        current: Optional[CStopCandidate],
        target: CStopCandidate,
        movement: Sequence[str],
    ) -> _RouteCost:
        start_name = C_QR if current is None else current.goal_name
        lane_changes = int(current is not None and current.lane != target.lane)
        return _RouteCost(
            self._route_distance(start_name, movement),
            lane_changes,
            target.yaml_order,
        )

    def _route_distance(self, start_name: str, movement: Sequence[str]) -> float:
        names = [start_name] + list(movement)
        distance = 0.0
        for first, second in zip(names, names[1:]):
            start = self.pose(first)
            end = self.pose(second)
            distance += math.hypot(end.x - start.x, end.y - start.y)
        return distance

    def _adjust_c_other_entry_yaw(self) -> None:
        qr = self.pose(C_QR)
        entry = self.pose(C_OTHER_ENTRY)
        yaw = math.atan2(entry.y - qr.y, entry.x - qr.x) + math.pi
        adjusted = WaypointPose(
            name=C_OTHER_ENTRY,
            point_type=entry.point_type,
            order=entry.order,
            x=entry.x,
            y=entry.y,
            z=entry.z,
            qx=0.0,
            qy=0.0,
            qz=math.sin(yaw * 0.5),
            qw=math.cos(yaw * 0.5),
        )
        self._by_name[C_OTHER_ENTRY] = adjusted
        for index, point in enumerate(self._points):
            if point.name == C_OTHER_ENTRY:
                self._points[index] = adjusted
                return


def _load_points(raw_points: Iterable[Mapping[str, Any]]) -> List[WaypointPose]:
    points = []
    for order, item in enumerate(raw_points):
        if not isinstance(item, dict):
            raise ValueError(f"waypoint point {order + 1} must be a mapping")
        name = str(item.get("name", "")).strip()
        if not name:
            raise ValueError(f"waypoint point {order + 1} name must not be empty")
        point_type = str(item.get("type", "")).strip()
        if point_type not in ("goal", "waypoint"):
            raise ValueError("waypoint type must be goal or waypoint")
        position = item["position"]
        orientation = item["orientation"]
        points.append(
            WaypointPose(
                name=name,
                point_type=point_type,
                order=order,
                x=float(position["x"]),
                y=float(position["y"]),
                z=float(position["z"]),
                qx=float(orientation["x"]),
                qy=float(orientation["y"]),
                qz=float(orientation["z"]),
                qw=float(orientation["w"]),
            )
        )
    return points


def _camera_side_for_pose(position_side: str, pose: WaypointPose) -> str:
    if _faces_reverse_x(pose):
        return _other_side(position_side)
    return position_side


def _faces_reverse_x(pose: WaypointPose) -> bool:
    yaw = math.atan2(
        2.0 * (pose.qw * pose.qz + pose.qx * pose.qy),
        1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz),
    )
    return math.cos(yaw) < 0.0


def _other_side(side: str) -> str:
    return "right" if side == "left" else "left"


def _lane_points_between(
    lane: str,
    start_index: int,
    target_index: int,
    include_start: bool,
) -> List[str]:
    names = LANES[lane]
    step = 1 if target_index >= start_index else -1
    indexes = list(range(start_index, target_index + step, step))
    if not include_start:
        indexes = indexes[1:]
    return [names[index] for index in indexes]


def _c_plan_tasks(c_plan: Mapping[str, object]) -> List[Mapping[str, object]]:
    if c_plan.get("zone") != "C":
        raise ValueError("expected a C-zone pick plan")
    tasks = list(c_plan.get("tasks", []))
    for index, task in enumerate(tasks, start=1):
        sequence = int(task["sequence"])
        position_id = int(task["position_id"])
        if sequence != index:
            raise ValueError("C pick plan sequence must be contiguous")
        if position_id < 1 or position_id > 12:
            raise ValueError("C position_id must be between 1 and 12")
    return tasks
