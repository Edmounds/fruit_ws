import rclpy
from visualization_msgs.msg import Marker

from fruit_navigation.route_model import PoseData
from fruit_waypoint_recorder.waypoint_recorder_node import WaypointRecorderNode


def test_marker_refresh_clears_removed_anchors_before_republishing():
    rclpy.init()
    node = WaypointRecorderNode()
    try:
        node._store.append(
            PoseData(
                name="",
                x=1.0,
                y=2.0,
                z=0.0,
                qx=0.0,
                qy=0.0,
                qz=0.0,
                qw=1.0,
                yaw=0.0,
            )
        )

        populated = node._markers().markers
        assert populated[0].action == Marker.DELETEALL
        assert any(marker.action == Marker.ADD for marker in populated[1:])

        node._store.clear()
        cleared = node._markers().markers
        assert len(cleared) == 1
        assert cleared[0].action == Marker.DELETEALL
    finally:
        node.destroy_node()
        rclpy.shutdown()
