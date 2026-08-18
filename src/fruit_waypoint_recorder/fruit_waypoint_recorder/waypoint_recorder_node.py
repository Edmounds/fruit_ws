# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""ROS 2 node for recording a fixed eleven-anchor route template."""

from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from fruit_navigation.route_model import PoseData
from fruit_waypoint_recorder.recorder import (
    WaypointTemplateStore,
    new_session_path,
)


DEFAULT_OUTPUT_DIRECTORY = "/home/rc4/fruit_ws/src/fruit_waypoint_recorder/waypoints"


def latched_qos() -> QoSProfile:
    """Return transient local QoS for operator prompts."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class WaypointRecorderNode(Node):
    """Record route anchors from RViz SetGoal clicks."""

    def __init__(self) -> None:
        super().__init__("fruit_waypoint_recorder")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("output_directory", DEFAULT_OUTPUT_DIRECTORY)
        self.declare_parameter("output_file", "competition_waypoints.yaml")
        self.declare_parameter("yaw_tolerance", 0.35)

        self._frame_id = self.get_parameter("frame_id").value
        output_directory = Path(self.get_parameter("output_directory").value)
        output_file = str(self.get_parameter("output_file").value)
        yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        if output_file:
            output_path = output_directory / output_file
        else:
            output_path = new_session_path(output_directory, datetime.now().astimezone())
        self._store = WaypointTemplateStore(output_path, yaw_tolerance)

        self._subscription = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self._on_goal_pose,
            10,
        )
        self._prompt_publisher = self.create_publisher(
            String,
            "~/prompt",
            latched_qos(),
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            "~/markers",
            1,
        )
        self._undo_service = self.create_service(Trigger, "~/undo", self._on_undo)
        self._clear_service = self.create_service(Trigger, "~/clear", self._on_clear)
        self._save_service = self.create_service(Trigger, "~/save", self._on_save)
        self._publish_state()
        self.get_logger().info(f"Recording route template to {output_path}")

    def _on_goal_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id != self._frame_id:
            self.get_logger().error(
                f"Rejected goal in {message.header.frame_id}; expected {self._frame_id}"
            )
            return
        try:
            pose = self._store.append(PoseData.from_pose_stamped("", message))
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self.get_logger().info(f"Recorded {pose.name}; {self._store.prompt}")
        self._publish_state()

    def _on_undo(self, _request, response):
        removed = self._store.undo()
        response.success = removed is not None
        response.message = (
            f"Removed {removed.name}; {self._store.prompt}"
            if removed is not None
            else "No anchor to undo"
        )
        self._publish_state()
        return response

    def _on_clear(self, _request, response):
        self._store.clear()
        response.success = True
        response.message = self._store.prompt
        self._publish_state()
        return response

    def _on_save(self, _request, response):
        try:
            self._store.save()
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response
        response.success = True
        response.message = f"Saved route template to {self._store.output_path}"
        self._publish_state()
        return response

    def _publish_state(self) -> None:
        prompt = String()
        prompt.data = self._store.prompt
        self._prompt_publisher.publish(prompt)
        self._marker_publisher.publish(self._markers())

    def _markers(self) -> MarkerArray:
        markers = []
        for index, pose in enumerate(self._store.anchors):
            marker = Marker()
            marker.header.frame_id = self._frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "fruit_waypoint_recorder"
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = pose.x
            marker.pose.position.y = pose.y
            marker.pose.position.z = pose.z + 0.2
            marker.scale.z = 0.18
            marker.color = ColorRGBA(r=1.0, g=0.75, b=0.0, a=1.0)
            marker.text = f"{index + 1}: {pose.name} yaw={pose.yaw:.2f}"
            markers.append(marker)
        return MarkerArray(markers=markers)


def main(args=None) -> None:
    """Run the waypoint recorder node."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = WaypointRecorderNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
