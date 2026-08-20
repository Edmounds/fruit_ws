"""Configured waypoint YAML loading and C-zone route planning."""

from __future__ import annotations

import heapq
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

import yaml

from fruit_mission.logic import expected_ripe_type


A_GOAL_PATTERN = re.compile(r"^A(\d+)$")
C_GOAL_PATTERN = re.compile(r"^C_(\d+)_(\d+)$")
B_QR = "B_QR"
C_QR = "C_QR"
C_ENTRANCE_1 = "C_entrance_passway_1"
C_TOP_1 = "C_top_waypoint_1"
C_TOP_2 = "C_top_waypoint_2"
C_ENTRANCE_2 = "C_entrance_waypoint_2"

ENTRY_GOAL_CHAIN = (
    "C_8_12",
    "C_7_11",
    "C_6_10",
    "C_5_9",
)
OTHER_GOAL_CHAIN = (
    "C_1_5",
    "C_2_6",
    "C_3_7",
    "C_4_8",
)
C_GOAL_NAMES = ENTRY_GOAL_CHAIN + OTHER_GOAL_CHAIN


def _c_graph_edges(entrance_1: str) -> tuple[tuple[str, str], ...]:
    """Build the fixed semantic C graph for the resolved entrance name."""
    entry_chain = (entrance_1,) + ENTRY_GOAL_CHAIN + (C_TOP_1,)
    other_chain = (C_TOP_2,) + OTHER_GOAL_CHAIN + (C_ENTRANCE_2,)
    return (
        tuple(zip(entry_chain, entry_chain[1:]))
        + tuple(zip(other_chain, other_chain[1:]))
        + (
            (entrance_1, C_ENTRANCE_2),
            (C_TOP_1, C_TOP_2),
        )
    )


def _c_waypoint_names(entrance_1: str) -> tuple[str, ...]:
    return (entrance_1, C_TOP_1, C_TOP_2, C_ENTRANCE_2)


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
    yaml_order: int


@dataclass(frozen=True)
class CNavigationPlan:
    """One C task route planned from the robot's current map pose."""

    start_x: float
    start_y: float
    position_id: int
    goal_name: str
    position_side: str
    graph_path: tuple[str, ...]
    navigation_names: tuple[str, ...]
    navigation_yaws: tuple[float, ...]
    distance: float
    camera_side: str


class MissionWaypointRoute:
    """Mission route helper built from the configured waypoint YAML."""

    def __init__(
        self,
        points: Sequence[WaypointPose],
        required_flow: str = "full",
    ) -> None:
        if not points:
            raise ValueError("waypoint file must contain at least one point")
        self._points = list(points)
        self._by_name: Dict[str, WaypointPose] = {}
        for point in self._points:
            if point.name in self._by_name:
                raise ValueError(f"duplicate waypoint name: {point.name}")
            self._by_name[point.name] = point
        self._required_flow = str(required_flow).strip().lower() or "full"
        self._c_entrance_1 = C_ENTRANCE_1
        self._validate_required_points()
        if self._requires_c_points:
            self._c_candidates = self._parse_c_candidates()
            self._c_adjacency = self._build_c_adjacency()
        else:
            self._c_candidates = {}
            self._c_adjacency = {}

    @classmethod
    def from_file(
        cls,
        path: Path,
        required_flow: str = "full",
    ) -> "MissionWaypointRoute":
        """Load the configured waypoint YAML from disk."""
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
        return cls(_load_points(raw_points), required_flow)

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

    def b_goal_names(self) -> List[str]:
        """Return B-zone goals in their numeric tree order."""
        goals = []
        for point in self._points:
            match = re.fullmatch(r"B(\d+)", point.name)
            if match and point.point_type == "goal":
                goals.append((int(match.group(1)), point.order, point.name))
        goals.sort()
        expected = list(range(1, len(goals) + 1))
        if [number for number, _order, _name in goals] != expected:
            raise ValueError("waypoint YAML must contain contiguous B1..Bn goals")
        if len(goals) != 8:
            raise ValueError("waypoint YAML must contain B1 through B8 goals")
        return [name for _number, _order, name in goals]

    def transition_names(self, transition: str) -> List[str]:
        """Return an explicit named transition without YAML-order expansion."""
        transitions = {
            "a_to_b_qr": ["to_B_QR_waypoint", B_QR],
            "b_to_c_qr": ["to_C_waypoint_1", "to_C_waypoint_2", "to_C_waypoint_3", C_QR],
            "b_to_first_tree": ["to_C_waypoint_1", "B1"],
            "b_to_c_qr_end": ["to_C_waypoint_2", "to_C_waypoint_3"],
            "c_to_b_first_tree": [
                C_ENTRANCE_2,
                "to_B_waypoint_1",
                "to_B_waypoint_2",
                "toB_waypoint_3",
                "B1",
            ],
        }
        try:
            names = transitions[transition]
        except KeyError as error:
            raise ValueError(f"unknown waypoint transition: {transition}") from error
        for name in names:
            self.pose(name)
        return list(names)

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

    def c_graph_edges(self) -> List[tuple[str, str]]:
        """Return the configured bidirectional C topology edges."""
        return list(_c_graph_edges(self._c_entrance_1))

    def build_c_distribution(self, c_plan: Mapping[str, object]) -> Dict[str, object]:
        """Build the C task table without pre-planning runtime movement."""
        tasks = []
        for task in _c_plan_tasks(c_plan):
            position_id = int(task["position_id"])
            target_name = str(task["target"])
            tasks.append(
                {
                    "sequence": int(task["sequence"]),
                    "position_id": position_id,
                    "target_name": target_name,
                    "expected_ripe_type": expected_ripe_type(target_name),
                    "navigation_status": None,
                    "candidate_goal_names": [
                        candidate.goal_name
                        for candidate in self._c_candidates[position_id]
                    ],
                    "navigation_start": None,
                    "navigation_start_source": None,
                    "goal_name": None,
                    "position_side": None,
                    "graph_path": [],
                    "movement_points": [],
                    "route_distance": None,
                    "camera_side": None,
                    "preferred_side": None,
                    "navigation_failure_reason": None,
                    "navigation_failure_detail": None,
                    "result": None,
                    "last_observation": None,
                }
            )
        return {
            "zone": "C",
            "task_count": c_plan.get("task_count", len(tasks)),
            "tasks": tasks,
        }

    def plan_c_navigation(
        self,
        start_x: float,
        start_y: float,
        start_yaw: float,
        position_id: int,
        max_start_distance: float,
    ) -> CNavigationPlan:
        """Plan the shortest named C route from one live map pose."""
        candidates = self._c_candidates.get(int(position_id), [])
        if not candidates:
            raise ValueError(f"C position {position_id} has no waypoint candidate")

        best: Optional[tuple[tuple[float, int, int, str], CNavigationPlan]] = None
        for candidate in candidates:
            distance, graph_path = self._shortest_c_path(
                float(start_x),
                float(start_y),
                candidate.goal_name,
                float(max_start_distance),
            )
            navigation_names = [
                name
                for name in graph_path
                if self.pose(name).point_type == "waypoint"
            ]
            navigation_names.append(candidate.goal_name)
            navigation_names = _without_consecutive_duplicates(navigation_names)
            navigation_yaws = self._navigation_yaws_from_xy(
                float(start_x),
                float(start_y),
                float(start_yaw),
                navigation_names,
            )
            final_yaw = navigation_yaws[-1] if navigation_yaws else float(start_yaw)
            plan = CNavigationPlan(
                start_x=float(start_x),
                start_y=float(start_y),
                position_id=int(position_id),
                goal_name=candidate.goal_name,
                position_side=candidate.position_side,
                graph_path=tuple(graph_path),
                navigation_names=tuple(navigation_names),
                navigation_yaws=tuple(navigation_yaws),
                distance=distance,
                camera_side=_camera_side_for_yaw(candidate.position_side, final_yaw),
            )
            key = (
                distance,
                sum(
                    self.pose(name).point_type == "waypoint"
                    for name in navigation_names
                ),
                candidate.yaml_order,
                candidate.goal_name,
            )
            if best is None or key < best[0]:
                best = (key, plan)
        if best is None:
            raise ValueError(f"Could not plan a route to C position {position_id}")
        return best[1]

    def closest_c_graph_distance(self, x: float, y: float) -> float:
        """Return the distance from a map point to the nearest C graph edge."""
        return min(
            _point_to_segment_distance(
                float(x),
                float(y),
                self.pose(first).x,
                self.pose(first).y,
                self.pose(second).x,
                self.pose(second).y,
            )
            for first, second in self.c_graph_edges()
        )

    def _validate_required_points(self) -> None:
        flow = self._required_flow
        if flow not in {"full", "ac", "a", "b", "c"}:
            raise ValueError(f"unknown mission flow: {flow}")
        if flow in {"full", "ac", "a"}:
            self.a_goal_names()
            self._require_type("to_B_QR_waypoint", "waypoint")
        if flow in {"full", "ac", "b"}:
            self._require_type(B_QR, "goal")
        if flow in {"full", "ac", "c"}:
            self._require_type(C_QR, "goal")
        if flow in {"full", "ac", "b"}:
            for name in ("to_C_waypoint_1", "to_C_waypoint_2", "to_C_waypoint_3"):
                self._require_type(name, "waypoint")
        if flow in {"full", "ac", "c"}:
            self._requires_c_points = True
            waypoint_names = _c_waypoint_names(self._c_entrance_1)
            for name in C_GOAL_NAMES + waypoint_names:
                self._require_type(name, "goal" if name in C_GOAL_NAMES else "waypoint")
        else:
            self._requires_c_points = False
        if flow == "full":
            for name in (C_ENTRANCE_2, "to_B_waypoint_1", "to_B_waypoint_2", "toB_waypoint_3"):
                self._require_type(name, "waypoint")
            self.b_goal_names()
        if flow == "b":
            self.b_goal_names()

    def _require_type(self, name: str, point_type: str) -> None:
        point = self.pose(name)
        if point.point_type != point_type:
            raise ValueError(f"{name} must be a {point_type}")

    def _parse_c_candidates(self) -> Dict[int, List[CStopCandidate]]:
        candidates: Dict[int, List[CStopCandidate]] = {
            position_id: [] for position_id in range(1, 13)
        }
        for name in C_GOAL_NAMES:
            point = self.pose(name)
            match = C_GOAL_PATTERN.fullmatch(name)
            if match is None:
                raise ValueError(f"invalid C goal name: {name}")
            left_id = int(match.group(1))
            right_id = int(match.group(2))
            candidates[left_id].append(
                CStopCandidate(left_id, name, "left", point.order)
            )
            candidates[right_id].append(
                CStopCandidate(right_id, name, "right", point.order)
            )
        for position_id, options in candidates.items():
            if not options:
                raise ValueError(f"C position {position_id} has no waypoint candidate")
            options.sort(key=lambda candidate: (candidate.yaml_order, candidate.goal_name))
        return candidates

    def _build_c_adjacency(self) -> Dict[str, List[tuple[str, float]]]:
        adjacency: Dict[str, List[tuple[str, float]]] = {
            name: [] for name in C_GOAL_NAMES + _c_waypoint_names(self._c_entrance_1)
        }
        for first, second in self.c_graph_edges():
            distance = _pose_distance(self.pose(first), self.pose(second))
            adjacency[first].append((second, distance))
            adjacency[second].append((first, distance))
        for neighbors in adjacency.values():
            neighbors.sort(key=lambda item: item[0])
        return adjacency

    def _shortest_c_path(
        self,
        start_x: float,
        start_y: float,
        target_name: str,
        max_start_distance: float,
    ) -> tuple[float, List[str]]:
        closest_edge = min(
            self.c_graph_edges(),
            key=lambda edge: _point_to_segment_distance(
                start_x,
                start_y,
                self.pose(edge[0]).x,
                self.pose(edge[0]).y,
                self.pose(edge[1]).x,
                self.pose(edge[1]).y,
            ),
        )
        graph_distance, fraction = _point_to_segment_projection(
            start_x,
            start_y,
            self.pose(closest_edge[0]).x,
            self.pose(closest_edge[0]).y,
            self.pose(closest_edge[1]).x,
            self.pose(closest_edge[1]).y,
        )
        start_neighbors = closest_edge
        snap_distance = graph_distance
        if snap_distance > max_start_distance:
            raise ValueError(
                f"robot is {snap_distance:.3f} m from the C route graph, "
                f"limit is {max_start_distance:.3f} m"
            )

        heap: List[tuple[float, int, tuple[str, ...], str]] = []
        best: Dict[str, tuple[float, int, tuple[str, ...]]] = {}
        edge_length = _pose_distance(
            self.pose(closest_edge[0]),
            self.pose(closest_edge[1]),
        )
        start_distances = (
            graph_distance + fraction * edge_length,
            graph_distance + (1.0 - fraction) * edge_length,
        )
        for name, distance in zip(start_neighbors, start_distances):
            pose = self.pose(name)
            waypoint_count = int(pose.point_type == "waypoint")
            path = (name,)
            key = (distance, waypoint_count, path)
            if name not in best or key < best[name]:
                best[name] = key
                heapq.heappush(heap, (distance, waypoint_count, path, name))

        while heap:
            distance, waypoint_count, path, name = heapq.heappop(heap)
            if best.get(name) != (distance, waypoint_count, path):
                continue
            if name == target_name:
                return distance, list(path)
            for neighbor, edge_distance in self._c_adjacency[name]:
                next_distance = distance + edge_distance
                next_waypoint_count = waypoint_count + int(
                    self.pose(neighbor).point_type == "waypoint"
                )
                next_path = path + (neighbor,)
                next_key = (next_distance, next_waypoint_count, next_path)
                if neighbor not in best or next_key < best[neighbor]:
                    best[neighbor] = next_key
                    heapq.heappush(
                        heap,
                        (next_distance, next_waypoint_count, next_path, neighbor),
                    )
        raise ValueError(f"C route graph cannot reach {target_name}")

    def _navigation_yaws_from_xy(
        self,
        start_x: float,
        start_y: float,
        start_yaw: float,
        target_names: Sequence[str],
    ) -> List[float]:
        current_x = start_x
        current_y = start_y
        current_yaw = start_yaw
        yaws = []
        for name in target_names:
            target = self.pose(name)
            dx = target.x - current_x
            dy = target.y - current_y
            if math.hypot(dx, dy) > 1e-9:
                current_yaw = math.atan2(dy, dx)
            yaws.append(current_yaw)
            current_x = target.x
            current_y = target.y
        return yaws

    def path_aligned_targets(
        self,
        start_name: str,
        target_names: Iterable[str],
        start_yaw: Optional[float] = None,
    ) -> tuple[List[tuple[str, float]], str, float]:
        """Return existing route points with yaw aligned to each XY segment."""
        current = self.pose(start_name)
        current_name = start_name
        current_yaw = _pose_yaw(current) if start_yaw is None else start_yaw
        targets: List[tuple[str, float]] = []
        for name in target_names:
            target = self.pose(str(name))
            dx = target.x - current.x
            dy = target.y - current.y
            current_name = target.name
            current = target
            if math.hypot(dx, dy) <= 1e-9:
                continue
            current_yaw = math.atan2(dy, dx)
            targets.append((target.name, current_yaw))
        return targets, current_name, current_yaw


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


def _pose_distance(first: WaypointPose, second: WaypointPose) -> float:
    return math.hypot(second.x - first.x, second.y - first.y)


def _point_to_segment_distance(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float:
    distance, _fraction = _point_to_segment_projection(
        px,
        py,
        x1,
        y1,
        x2,
        y2,
    )
    return distance


def _point_to_segment_projection(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> tuple[float, float]:
    dx = x2 - x1
    dy = y2 - y1
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return math.hypot(px - x1, py - y1), 0.0
    fraction = ((px - x1) * dx + (py - y1) * dy) / length_squared
    fraction = min(max(fraction, 0.0), 1.0)
    nearest_x = x1 + fraction * dx
    nearest_y = y1 + fraction * dy
    return math.hypot(px - nearest_x, py - nearest_y), fraction


def _without_consecutive_duplicates(names: Sequence[str]) -> List[str]:
    result = []
    for name in names:
        if not result or result[-1] != name:
            result.append(name)
    return result


def _camera_side_for_yaw(position_side: str, yaw: float) -> str:
    if math.cos(yaw) < 0.0:
        return _other_side(position_side)
    return position_side


def _pose_yaw(pose: WaypointPose) -> float:
    return math.atan2(
        2.0 * (pose.qw * pose.qz + pose.qx * pose.qy),
        1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz),
    )


def _other_side(side: str) -> str:
    return "right" if side == "left" else "left"


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
