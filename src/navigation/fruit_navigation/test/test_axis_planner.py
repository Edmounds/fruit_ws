import itertools
import math

import pytest

from fruit_navigation.axis_planner import (
    AxisPose,
    OdomProgressWatchdog,
    ProgressWatchdog,
    axis_cross_track_error,
    axis_forward_remaining,
    choose_axis_segments,
    choose_reverse_motion,
    normalize_angle,
    validate_waypoints,
)


def test_normalize_angle_returns_minus_pi_to_pi():
    assert normalize_angle(0.0) == 0.0
    assert math.isclose(normalize_angle(3.0 * math.pi), -math.pi)
    assert math.isclose(normalize_angle(-3.0 * math.pi), -math.pi)


@pytest.mark.parametrize(
    ("start_yaw", "target_yaw", "path_heading", "expected"),
    (
        (0.0, 0.0, 0.0, False),
        (0.0, 0.0, math.pi, True),
        (0.0, -math.pi / 2.0, math.pi, True),
        (0.0, -math.pi / 2.0, 0.0, False),
        (-3.118, 1.569, math.atan2(-0.710, 0.205), True),
        (math.pi / 4.0, math.pi, 0.0, True),
        (0.0, math.pi, 0.0, False),
        (math.pi, 0.0, 0.0, True),
        (math.pi / 2.0, -math.pi / 2.0, 0.0, False),
        (-math.pi / 2.0, math.pi / 2.0, 0.0, False),
    ),
)
def test_direction_minimizes_rotation_and_resolves_ties(
    start_yaw, target_yaw, path_heading, expected,
):
    for offset in (-2.0 * math.pi, -0.3, 0.0, math.pi, 2.0 * math.pi):
        assert choose_reverse_motion(
            start_yaw + offset, target_yaw + offset, path_heading + offset, 0.042,
        ) is expected


@pytest.mark.parametrize(
    ("reverse_saving", "expected"),
    ((-0.2, False), (0.02, False), (0.125, False), (0.126, True)),
)
def test_direction_uses_entry_rotation_inside_the_total_rotation_tie_band(
    reverse_saving, expected,
):
    # Forward needs less entry rotation, but reverse can save total rotation.
    assert choose_reverse_motion(
        0.5, math.pi - 0.5 + reverse_saving / 2.0, 0.0, 0.0625,
    ) is expected


def test_direction_near_opposite_endpoint_yaws_is_stable_under_pose_perturbations():
    for dx, dy, dyaw in itertools.product(
        (-0.01, 0.0, 0.01), (-0.01, 0.0, 0.01), (-0.042, 0.0, 0.042),
    ):
        heading = math.atan2(-1.010 - (-1.944 + dy), 0.048 - (-0.042 + dx))
        assert choose_reverse_motion(-3.126 + dyaw, 0.0, heading, 0.042) is True


@pytest.mark.parametrize("value", (math.nan, math.inf, -math.inf))
@pytest.mark.parametrize("index", range(4))
def test_direction_rejects_nonfinite_inputs(value, index):
    values = [0.0, 0.0, 0.0, 0.042]
    values[index] = value
    with pytest.raises(ValueError):
        choose_reverse_motion(*values)


@pytest.mark.parametrize("yaw_tolerance", (0.0, -0.01))
def test_direction_rejects_nonpositive_yaw_tolerance(yaw_tolerance):
    with pytest.raises(ValueError):
        choose_reverse_motion(0.0, 0.0, 0.0, yaw_tolerance)


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


@pytest.mark.parametrize(
    ("axis", "heading", "current", "target", "expected"),
    (
        ("x", 0.0, AxisPose(0.25, 0.0, 0.0), 1.0, 0.75),
        ("x", math.pi, AxisPose(0.25, 0.0, math.pi), -1.0, 1.25),
        ("y", math.pi / 2.0, AxisPose(0.0, 0.25, math.pi / 2.0), 1.0, 0.75),
        ("y", -math.pi / 2.0, AxisPose(0.0, 0.25, -math.pi / 2.0), -1.0, 1.25),
    ),
)
def test_axis_forward_remaining_is_positive_in_all_cardinal_directions(
    axis, heading, current, target, expected
):
    assert math.isclose(
        axis_forward_remaining(axis, target, current, heading),
        expected,
    )


def test_axis_forward_remaining_is_negative_after_crossing_target():
    current = AxisPose(x=1.1, y=0.0, yaw=0.0)

    assert axis_forward_remaining("x", 1.0, current, 0.0) < 0.0


@pytest.mark.parametrize(
    ("axis", "heading"),
    (
        ("x", 0.0),
        ("x", math.pi),
        ("y", math.pi / 2.0),
        ("y", -math.pi / 2.0),
    ),
)
def test_cross_track_error_has_a_consistent_left_of_heading_sign(axis, heading):
    left_offset = AxisPose(
        x=-math.sin(heading) * 0.1,
        y=math.cos(heading) * 0.1,
        yaw=heading,
    )

    error = axis_cross_track_error(axis, left_offset, 0.0, heading)

    assert error > 0.0


def test_progress_watchdog_times_out_without_improvement():
    watchdog = ProgressWatchdog(timeout=2.0, min_delta=0.01, now=10.0)

    assert watchdog.stalled(1.0, now=10.0) is False
    assert watchdog.stalled(1.0, now=11.9) is False
    assert watchdog.stalled(1.0, now=12.1) is True


def test_progress_watchdog_resets_after_meaningful_improvement():
    watchdog = ProgressWatchdog(timeout=2.0, min_delta=0.01, now=10.0)

    assert watchdog.stalled(1.0, now=10.0) is False
    assert watchdog.stalled(0.8, now=11.5) is False
    assert watchdog.stalled(0.8, now=13.4) is False
    assert watchdog.stalled(0.8, now=13.6) is True


def test_odom_progress_watchdog_accepts_position_or_yaw_motion():
    watchdog = OdomProgressWatchdog(
        stall_timeout=5.0,
        window=1.0,
        position_threshold=0.03,
        yaw_threshold=0.01,
        now=0.0,
    )
    assert watchdog.observe(AxisPose(0.0, 0.0, 0.0), now=0.0)[0] is False
    assert watchdog.observe(AxisPose(0.0, 0.0, 0.02), now=1.0)[0] is True
    assert watchdog.stalled(now=5.9) is False
    assert watchdog.observe(AxisPose(0.0, 0.0, 0.02), now=5.0)[0] is False
    assert watchdog.observe(AxisPose(0.04, 0.0, 0.02), now=6.0)[0] is True


def test_odom_progress_watchdog_stalls_without_window_motion():
    watchdog = OdomProgressWatchdog(
        stall_timeout=5.0,
        window=1.0,
        position_threshold=0.03,
        yaw_threshold=0.01,
        now=0.0,
    )
    for second in range(7):
        assert watchdog.observe(AxisPose(0.0, 0.0, 0.0), now=float(second))[0] is False
    assert watchdog.stalled(now=5.0) is True


def test_odom_progress_watchdog_keeps_a_full_window_at_control_rate():
    watchdog = OdomProgressWatchdog(
        stall_timeout=5.0,
        window=1.0,
        position_threshold=0.03,
        yaw_threshold=0.01,
        now=0.0,
    )

    progressed = False
    for index in range(25):
        now = index * 0.05
        progressed, _, _ = watchdog.observe(
            AxisPose(0.05 * index, 0.0, 0.0), now=now,
        )

    assert progressed is True
    assert watchdog.stalled(now=5.0) is False
