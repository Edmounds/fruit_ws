import json

import rclpy
import yaml
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from fruit_navigation.route_model import PoseData
from fruit_waypoint_recorder.waypoint_recorder_node import WaypointRecorderNode


def test_marker_refresh_clears_removed_anchors_before_republishing():
    rclpy.init()
    node = WaypointRecorderNode()
    try:
        node._store.append(pose("", 1.0, 2.0))
        node._test_store.append("a_001", "goal", pose("", 3.0, 4.0))

        populated = node._markers().markers
        assert populated[0].action == Marker.DELETEALL
        assert any(marker.type == Marker.SPHERE for marker in populated[1:])
        assert any(marker.type == Marker.ARROW for marker in populated[1:])
        assert any(marker.type == Marker.TEXT_VIEW_FACING for marker in populated[1:])

        node._store.clear()
        node._test_store.clear()
        cleared = node._markers().markers
        assert len(cleared) == 1
        assert cleared[0].action == Marker.DELETEALL
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_record_command_uses_sampled_pose_for_test_and_formal_modes():
    rclpy.init()
    node = WaypointRecorderNode()
    try:
        node._sample_current_pose = lambda name: pose(name, 1.0, 2.0)

        test_message = String()
        test_message.data = json.dumps(
            {
                "command": "record",
                "mode": "test",
                "name": "a_001",
                "point_type": "goal",
            }
        )
        node._on_command(test_message)

        formal_message = String()
        formal_message.data = json.dumps({"command": "record", "mode": "formal"})
        node._on_command(formal_message)

        assert node._test_store.points[0].name == "a_001"
        assert node._test_store.points[0].point_type == "goal"
        assert node._store.anchors[0].name == "HOME"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_load_test_file_command_accepts_yaml_under_source_root(tmp_path):
    input_path = tmp_path / "src" / "points.yaml"
    input_path.parent.mkdir()
    input_path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "frame_id": "map",
                "points": [
                    pose("w1", 1.0, 2.0).to_dict() | {"type": "waypoint"},
                    pose("g1", 3.0, 4.0).to_dict() | {"type": "goal"},
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    rclpy.init()
    node = WaypointRecorderNode()
    try:
        node._source_root = input_path.parent.resolve()
        message = String()
        message.data = json.dumps(
            {"command": "load_test_file", "path": str(input_path)}
        )

        node._on_command(message)

        assert node._test_store.output_path == input_path.resolve()
        assert [point.name for point in node._test_store.points] == ["w1", "g1"]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_load_test_file_command_rejects_yaml_outside_source_root(tmp_path):
    source_root = tmp_path / "src"
    source_root.mkdir()
    outside = tmp_path / "outside.yaml"
    outside.write_text("schema_version: 1\nframe_id: map\npoints: []\n")

    rclpy.init()
    node = WaypointRecorderNode()
    try:
        node._source_root = source_root.resolve()
        message = String()
        message.data = json.dumps(
            {"command": "load_test_file", "path": str(outside)}
        )

        node._on_command(message)

        assert not node._test_store.points
        assert "must be under" in node._last_message
    finally:
        node.destroy_node()
        rclpy.shutdown()


def pose(name, x, y):
    return PoseData(
        name=name,
        x=x,
        y=y,
        z=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        yaw=0.0,
    )
