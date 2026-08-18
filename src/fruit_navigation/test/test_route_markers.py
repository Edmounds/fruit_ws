import rclpy
from visualization_msgs.msg import Marker

from fruit_navigation.route_marker_node import RouteMarkerNode
from fruit_navigation.route_model import PoseData, quaternion_from_yaw


def pose(name, x, y, yaw=0.0):
    quaternion = quaternion_from_yaw(yaw)
    return PoseData(
        name=name,
        x=x,
        y=y,
        z=0.0,
        qx=quaternion["x"],
        qy=quaternion["y"],
        qz=quaternion["z"],
        qw=quaternion["w"],
        yaw=yaw,
    )


def test_final_route_line_includes_an_order_marker_for_every_target():
    rclpy.init()
    node = RouteMarkerNode()
    try:
        markers = []
        route = [pose("A1", 1.0, 1.0), pose("B_QR", 2.0, 1.0)]

        node._append_route_line(markers, 0, route)

        assert markers[0].type == Marker.LINE_STRIP
        order_markers = [marker for marker in markers if marker.ns == "route_order"]
        assert [marker.text for marker in order_markers] == ["1", "2"]
        assert [(marker.pose.position.x, marker.pose.position.y) for marker in order_markers] == [
            (1.0, 1.0),
            (2.0, 1.0),
        ]
    finally:
        node.destroy_node()
        rclpy.shutdown()
