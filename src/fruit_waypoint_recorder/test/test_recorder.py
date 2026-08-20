import math

import pytest
import yaml

from fruit_waypoint_recorder.recorder import (
    PoseData,
    TestPointStore,
    mean_pose,
    quaternion_from_yaw,
)


def pose(name, x, y, yaw=0.0):
    q = quaternion_from_yaw(yaw)
    return PoseData(
        name=name,
        x=x,
        y=y,
        z=0.0,
        qx=q["x"],
        qy=q["y"],
        qz=q["z"],
        qw=q["w"],
        yaw=yaw,
    )


def test_mean_pose_uses_circular_yaw_average_across_pi_boundary():
    averaged = mean_pose(
        "wrapped",
        [
            pose("one", 1.0, 2.0, math.radians(179.0)),
            pose("two", 3.0, 4.0, math.radians(-179.0)),
        ],
    )

    assert averaged.name == "wrapped"
    assert averaged.x == pytest.approx(2.0)
    assert averaged.y == pytest.approx(3.0)
    assert abs(abs(averaged.yaw) - math.pi) < math.radians(2.0)


def test_test_point_store_saves_named_waypoints_and_goals(tmp_path):
    store = TestPointStore(tmp_path / "test_waypoints.yaml")

    store.append("a_001", "waypoint", pose("", 1.0, 2.0, 0.25))
    store.append("a_pick", "goal", pose("", 3.0, 4.0, 0.5))
    store.save()

    saved = yaml.safe_load(store.output_path.read_text(encoding="utf-8"))

    assert saved["schema_version"] == 1
    assert saved["frame_id"] == "map"
    assert [point["name"] for point in saved["points"]] == ["a_001", "a_pick"]
    assert [point["type"] for point in saved["points"]] == ["waypoint", "goal"]
    assert saved["points"][1]["position"] == {"x": 3.0, "y": 4.0, "z": 0.0}


def test_test_point_store_loads_named_waypoints_and_goals(tmp_path):
    input_path = tmp_path / "loaded_waypoints.yaml"
    input_path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "frame_id": "map",
                "created_at": "2026-08-18T00:00:00+08:00",
                "points": [
                    {
                        "name": "w1",
                        "type": "waypoint",
                        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                        "orientation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0,
                            "w": 1.0,
                        },
                        "yaw": 0.0,
                    },
                    {
                        "name": "g1",
                        "type": "goal",
                        "position": {"x": 3.0, "y": 4.0, "z": 0.0},
                        "orientation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0,
                            "w": 1.0,
                        },
                        "yaw": 0.0,
                    },
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    store = TestPointStore(tmp_path / "old.yaml")
    store.append("old", "goal", pose("", 0.0, 0.0))
    store.advance_cursor(1)

    store.load(input_path)

    assert store.output_path == input_path
    assert store.cursor == 0
    assert [point.name for point in store.points] == ["w1", "g1"]
    assert [point.point_type for point in store.points] == ["waypoint", "goal"]


def test_test_point_store_rejects_empty_duplicate_or_invalid_types(tmp_path):
    store = TestPointStore(tmp_path / "test_waypoints.yaml")
    store.append("a_001", "waypoint", pose("", 1.0, 2.0))

    with pytest.raises(ValueError, match="name"):
        store.append(" ", "waypoint", pose("", 2.0, 3.0))
    with pytest.raises(ValueError, match="already exists"):
        store.append("a_001", "goal", pose("", 2.0, 3.0))
    with pytest.raises(ValueError, match="point_type"):
        store.append("a_002", "stop", pose("", 2.0, 3.0))


def test_test_point_store_segments_through_the_next_goal(tmp_path):
    store = TestPointStore(tmp_path / "test_waypoints.yaml")
    store.append("w1", "waypoint", pose("", 1.0, 0.0))
    store.append("g1", "goal", pose("", 2.0, 0.0))
    store.append("w2", "waypoint", pose("", 3.0, 0.0))
    store.append("g2", "goal", pose("", 4.0, 0.0))

    first_segment, first_after = store.next_test_segment()
    store.advance_cursor(first_after)
    second_segment, second_after = store.next_test_segment()

    assert [point.name for point in first_segment] == ["w1", "g1"]
    assert [point.name for point in second_segment] == ["w2", "g2"]
    assert second_after == 4


def test_test_point_store_does_not_advance_without_success(tmp_path):
    store = TestPointStore(tmp_path / "test_waypoints.yaml")
    store.append("w1", "waypoint", pose("", 1.0, 0.0))
    store.append("g1", "goal", pose("", 2.0, 0.0))

    store.next_test_segment()

    assert store.cursor == 0
