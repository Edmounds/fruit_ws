# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Send a saved route template to /follow_waypoints for chassis testing."""

from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node

from fruit_navigation.route_model import load_route_template


class RouteFileClient(Node):
    """Optional node that sends A/B route points when route_file is provided."""

    def __init__(self) -> None:
        super().__init__("route_file_client")
        self.declare_parameter("route_file", "")
        route_file = str(self.get_parameter("route_file").value).strip()
        self._route_file = Path(route_file) if route_file else None
        self._client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self._timer = self.create_timer(0.5, self._send_once)
        self._sent = False

    def _send_once(self) -> None:
        if self._sent:
            return
        self._sent = True
        if self._route_file is None:
            self.get_logger().info("route_file is empty; waiting for external goals")
            return
        template = load_route_template(self._route_file)
        generated = template["generated"]
        route = list(generated["A"]) + [template["anchors"]["B_QR"]]
        route.extend(generated["B"])
        route.append(template["anchors"]["HOME"])
        goal = FollowWaypoints.Goal()
        goal.poses = [self._pose_stamped(pose) for pose in route]
        self._client.wait_for_server()
        self._client.send_goal_async(goal)
        self.get_logger().info(f"Sent {len(goal.poses)} poses to /follow_waypoints")

    @staticmethod
    def _pose_stamped(pose) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = "map"
        stamped.pose.position.x = pose.x
        stamped.pose.position.y = pose.y
        stamped.pose.position.z = pose.z
        stamped.pose.orientation.x = pose.qx
        stamped.pose.orientation.y = pose.qy
        stamped.pose.orientation.z = pose.qz
        stamped.pose.orientation.w = pose.qw
        return stamped


def main(args=None) -> None:
    """Run the route file client."""
    rclpy.init(args=args)
    node = None
    try:
        node = RouteFileClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
