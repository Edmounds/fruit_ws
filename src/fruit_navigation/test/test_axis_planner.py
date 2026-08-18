import math

import pytest

from fruit_navigation.axis_planner import (
    AxisPose,
    choose_axis_segments,
    normalize_angle,
    validate_waypoints,
)


def test_normalize_angle_returns_minus_pi_to_pi():
    assert normalize_angle(0.0) == 0.0
    assert math.isclose(normalize_angle(3.0 * math.pi), -math.pi)
    assert math.isclose(normalize_angle(-3.0 * math.pi), -math.pi)


def test_axis_segments_choose_current_heading_first():
    current = AxisPose(x=0.0, y=0.0, yaw=0.0)
    goal = AxisPose(x=1.0, y=1.0, yaw=math.pi / 2.0)

    segments = choose_axis_segments(current, goal, position_tolerance=0.01)

    assert [segment.axis for segment in segments] == ["x", "y"]
    assert segments[0].heading == 0.0
    assert math.isclose(segments[1].heading, math.pi / 2.0)


def test_axis_segments_use_positive_linear_x_by_rotating_to_negative_axis():
    current = AxisPose(x=1.0, y=1.0, yaw=0.0)
    goal = AxisPose(x=-1.0, y=1.0, yaw=math.pi)

    segments = choose_axis_segments(current, goal, position_tolerance=0.01)

    assert len(segments) == 1
    assert segments[0].axis == "x"
    assert math.isclose(abs(segments[0].heading), math.pi)
    assert segments[0].distance > 0.0


def test_axis_segments_skip_axes_inside_tolerance_and_keep_final_yaw():
    current = AxisPose(x=1.0, y=1.0, yaw=0.0)
    goal = AxisPose(x=1.005, y=2.0, yaw=-math.pi / 2.0)

    segments = choose_axis_segments(current, goal, position_tolerance=0.01)

    assert [segment.axis for segment in segments] == ["y"]
    assert math.isclose(segments[0].final_yaw, -math.pi / 2.0)


def test_validate_waypoints_rejects_empty_or_invalid_map_routes():
    with pytest.raises(ValueError):
        validate_waypoints([])
    with pytest.raises(ValueError):
        validate_waypoints([{"frame_id": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0}])
    with pytest.raises(ValueError):
        validate_waypoints([{"frame_id": "map", "x": math.nan, "y": 0.0, "yaw": 0.0}])
