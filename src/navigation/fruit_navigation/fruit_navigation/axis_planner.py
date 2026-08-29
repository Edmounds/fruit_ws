# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Pure helpers for cardinal-axis waypoint navigation."""

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Mapping


@dataclass(frozen=True)
class AxisPose:
    """Minimal map-frame pose used by the axis planner."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class AxisSegment:
    """One straight X or Y segment after rotating to a cardinal heading."""

    axis: str
    target: float
    lateral_target: float
    heading: float
    distance: float
    final_yaw: float


@dataclass
class ProgressWatchdog:
    """Report stalled motion when an error stops improving."""

    timeout: float
    min_delta: float
    now: float
    best_error: float = field(default=float("inf"), init=False)
    last_progress: float = field(init=False)

    def __post_init__(self) -> None:
        self.last_progress = self.now

    def stalled(self, error: float, now: float) -> bool:
        """Return true when error has not improved before the deadline."""
        error = abs(error)
        if error < self.best_error - self.min_delta:
            self.best_error = error
            self.last_progress = now
        return now - self.last_progress > self.timeout


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    normalized = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if normalized == math.pi:
        return -math.pi
    return normalized


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def validate_waypoints(waypoints: Iterable[Mapping[str, float]]) -> None:
    """Validate the simple map-frame route representation used in tests."""
    items = list(waypoints)
    if not items:
        raise ValueError("route must contain at least one waypoint")
    for waypoint in items:
        if waypoint.get("frame_id") != "map":
            raise ValueError("all waypoints must be in map frame")
        for key in ("x", "y", "yaw"):
            value = float(waypoint[key])
            if not math.isfinite(value):
                raise ValueError(f"waypoint {key} must be finite")


def choose_axis_segments(
    current: AxisPose,
    goal: AxisPose,
    position_tolerance: float,
    axis_segment_min_distance: float = 0.0,
) -> List[AxisSegment]:
    """Split a target pose into X/Y moves, trying the easiest heading first."""
    dx = goal.x - current.x
    dy = goal.y - current.y
    split_x = abs(dx) > position_tolerance
    split_y = abs(dy) > position_tolerance
    if split_x and split_y and axis_segment_min_distance > position_tolerance:
        if abs(dx) < axis_segment_min_distance and abs(dy) < axis_segment_min_distance:
            split_x = abs(dx) >= abs(dy)
            split_y = not split_x
        elif abs(dx) < axis_segment_min_distance:
            split_x = False
        elif abs(dy) < axis_segment_min_distance:
            split_y = False

    candidates = []
    if split_x:
        candidates.append(("x", goal.x, 0.0 if dx > 0.0 else math.pi, abs(dx)))
    if split_y:
        candidates.append(("y", goal.y, math.pi / 2.0 if dy > 0.0 else -math.pi / 2.0, abs(dy)))

    candidates.sort(key=lambda item: abs(normalize_angle(item[2] - current.yaw)))
    segments = []
    working_x = current.x
    working_y = current.y
    for axis, target, heading, distance in candidates:
        if axis == "x":
            lateral_target = working_y if split_y else goal.y
            working_x = goal.x
        else:
            lateral_target = working_x if split_x else goal.x
            working_y = goal.y
        segments.append(
            AxisSegment(
                axis=axis,
                target=target,
                lateral_target=lateral_target,
                heading=normalize_angle(heading),
                distance=distance,
                final_yaw=normalize_angle(goal.yaw),
            )
        )
    return segments


def axis_forward_remaining(
    axis: str,
    target: float,
    current: AxisPose,
    heading: float,
) -> float:
    """Return remaining distance measured along the segment's forward direction."""
    if axis == "x":
        return (target - current.x) * math.cos(heading)
    if axis == "y":
        return (target - current.y) * math.sin(heading)
    raise ValueError(f"unsupported axis: {axis}")


def axis_cross_track_error(
    axis: str,
    current: AxisPose,
    lateral_target: float,
    heading: float,
) -> float:
    """Return signed lateral error, positive on the left of the heading."""
    if axis == "x":
        return math.cos(heading) * (current.y - lateral_target)
    if axis == "y":
        return -math.sin(heading) * (current.x - lateral_target)
    raise ValueError(f"unsupported axis: {axis}")


def pose_from_ros(pose_stamped) -> AxisPose:
    """Convert a PoseStamped-like object to AxisPose."""
    pose = pose_stamped.pose
    orientation = pose.orientation
    return AxisPose(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ),
    )
