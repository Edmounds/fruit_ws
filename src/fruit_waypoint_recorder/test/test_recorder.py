from datetime import datetime, timezone

import yaml

from fruit_waypoint_recorder.recorder import (
    JoyEdgeDetector,
    WaypointStore,
    new_session_path,
)


def test_joy_edges_trigger_once_until_controls_are_released():
    detector = JoyEdgeDetector(
        save_button=0,
        undo_axis=6,
        undo_direction=1.0,
        axis_threshold=0.5,
    )

    assert detector.observe([0], [0.0] * 7) == (False, False)
    assert detector.observe([1], [0.0] * 7) == (True, False)
    assert detector.observe([1], [0.0] * 7) == (False, False)
    assert detector.observe([0], [0.0] * 7) == (False, False)
    assert detector.observe([0], [0.0] * 6 + [1.0]) == (False, True)
    assert detector.observe([0], [0.0] * 6 + [1.0]) == (False, False)
    assert detector.observe([0], [0.0] * 7) == (False, False)


def test_joy_edges_ignore_missing_indexes():
    detector = JoyEdgeDetector(
        save_button=4,
        undo_axis=7,
        undo_direction=-1.0,
        axis_threshold=0.5,
    )

    assert detector.observe([0], [0.0]) == (False, False)


def test_waypoint_store_creates_file_appends_and_undoes_last_pose(tmp_path):
    output_path = tmp_path / "waypoints_20260817_193000.yaml"
    created_at = datetime(2026, 8, 17, 19, 30, tzinfo=timezone.utc)
    store = WaypointStore(output_path, "map", "base_link", created_at)

    first = store.append(
        translation={"x": 1.0, "y": 2.0, "z": 0.0},
        rotation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        recorded_at=created_at,
    )
    second = store.append(
        translation={"x": 3.0, "y": 4.0, "z": 0.1},
        rotation={"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0},
        recorded_at=created_at,
    )

    assert first["name"] == "waypoint_001"
    assert first["yaw"] == 0.0
    assert second["name"] == "waypoint_002"
    assert second["yaw"] == 3.141593
    assert store.undo()["name"] == "waypoint_002"

    saved = yaml.safe_load(output_path.read_text(encoding="utf-8"))
    assert saved["frame_id"] == "map"
    assert saved["child_frame_id"] == "base_link"
    assert [item["name"] for item in saved["waypoints"]] == ["waypoint_001"]
    assert saved["waypoints"][0]["position"] == {"x": 1.0, "y": 2.0, "z": 0.0}


def test_waypoint_store_reports_empty_undo(tmp_path):
    store = WaypointStore(
        tmp_path / "empty.yaml",
        "map",
        "base_link",
        datetime(2026, 8, 17, tzinfo=timezone.utc),
    )

    assert store.undo() is None


def test_new_session_path_never_overwrites_an_existing_session(tmp_path):
    now = datetime(2026, 8, 17, 19, 30, 45, tzinfo=timezone.utc)

    first = new_session_path(tmp_path, now)
    first.touch()
    second = new_session_path(tmp_path, now)

    assert first.name == "waypoints_20260817_193045.yaml"
    assert second.name == "waypoints_20260817_193045_01.yaml"
