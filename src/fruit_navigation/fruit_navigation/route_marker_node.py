# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Publish route markers over the PGM map without editing the map file."""

from pathlib import Path
from typing import Iterable, List, Optional, Sequence

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from fruit_navigation.route_model import (
    PickTask,
    PoseData,
    load_route_template,
    mission_route,
    parse_c_pick_plan,
)


class RouteMarkerNode(Node):
    """Show anchors, generated points, C selection, and final route in RViz."""

    def __init__(self) -> None:
        super().__init__("route_marker_node")
        self.declare_parameter(
            "route_file",
            "/home/rc4/fruit_ws/src/fruit_waypoint_recorder/waypoints/"
            "competition_waypoints.yaml",
        )
        self.declare_parameter("publish_period_sec", 1.0)
        self._route_file = Path(self.get_parameter("route_file").value)
        period = float(self.get_parameter("publish_period_sec").value)
        self._template = None
        self._c_tasks: Optional[List[PickTask]] = None
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            "/fruit/navigation_markers",
            1,
        )
        self._path_publisher = self.create_publisher(NavPath, "/fruit/final_route", 1)
        self._c_subscription = self.create_subscription(
            String,
            "/fruit/c_pick_plan",
            self._on_c_pick_plan,
            10,
        )
        self._timer = self.create_timer(period, self._publish)

    def _on_c_pick_plan(self, message: String) -> None:
        try:
            self._c_tasks = parse_c_pick_plan(message.data)
        except ValueError as error:
            self.get_logger().error(f"Rejected C pick plan for markers: {error}")
            return
        self._publish()

    def _load_template(self) -> bool:
        if not self._route_file.exists():
            self.get_logger().warning(
                f"Route template does not exist yet: {self._route_file}",
                throttle_duration_sec=5.0,
            )
            return False
        try:
            self._template = load_route_template(self._route_file)
            return True
        except Exception as error:
            self.get_logger().error(f"Failed to load route template: {error}")
            return False

    def _publish(self) -> None:
        if self._template is None and not self._load_template():
            return
        markers = []
        marker_id = 0
        anchors = self._template["anchors"]
        generated = self._template["generated"]
        selected_c = {task.position_id: task.sequence for task in self._c_tasks or []}

        for pose in anchors.values():
            marker_id = self._append_pose_markers(
                markers,
                marker_id,
                pose,
                self._color(1.0, 0.75, 0.0, 1.0),
                scale=0.18,
                label_suffix=" anchor",
            )
        for pose in generated["A"] + generated["B"]:
            marker_id = self._append_pose_markers(
                markers,
                marker_id,
                pose,
                self._color(0.0, 0.7, 1.0, 1.0),
                scale=0.12,
            )
        for name, pose in generated["C"].items():
            position_id = int(name[1:])
            if position_id in selected_c:
                label_suffix = f" QR{selected_c[position_id]}"
                color = self._color(0.0, 1.0, 0.2, 1.0)
            elif self._c_tasks:
                label_suffix = ""
                color = self._color(0.35, 0.35, 0.35, 0.35)
            else:
                label_suffix = ""
                color = self._color(0.9, 0.1, 0.9, 1.0)
            marker_id = self._append_pose_markers(
                markers,
                marker_id,
                pose,
                color,
                scale=0.12,
                label_suffix=label_suffix,
            )

        final_route = self._final_route()
        if final_route:
            marker_id = self._append_route_line(markers, marker_id, final_route)
            self._path_publisher.publish(self._path(final_route))
        self._marker_publisher.publish(MarkerArray(markers=markers))

    def _final_route(self) -> List[PoseData]:
        anchors = self._template["anchors"]
        generated = self._template["generated"]
        if self._c_tasks:
            return mission_route(anchors, generated, self._c_tasks)
        return (
            list(generated["A"])
            + [anchors["B_QR"]]
            + [anchors["C_QR"]]
            + list(generated["B"])
            + [anchors["HOME"]]
        )

    def _append_pose_markers(
        self,
        markers: List[Marker],
        marker_id: int,
        pose: PoseData,
        color: ColorRGBA,
        scale: float,
        label_suffix: str = "",
    ) -> int:
        point_marker = self._base_marker(marker_id, Marker.SPHERE)
        marker_id += 1
        point_marker.pose.position.x = pose.x
        point_marker.pose.position.y = pose.y
        point_marker.pose.position.z = pose.z
        point_marker.scale.x = scale
        point_marker.scale.y = scale
        point_marker.scale.z = scale
        point_marker.color = color
        markers.append(point_marker)

        text_marker = self._base_marker(marker_id, Marker.TEXT_VIEW_FACING)
        marker_id += 1
        text_marker.pose.position.x = pose.x
        text_marker.pose.position.y = pose.y
        text_marker.pose.position.z = pose.z + 0.25
        text_marker.scale.z = 0.16
        text_marker.color = self._color(1.0, 1.0, 1.0, max(color.a, 0.6))
        text_marker.text = (
            f"{pose.name}{label_suffix} "
            f"({pose.x:.2f},{pose.y:.2f}) yaw={pose.yaw:.2f}"
        )
        markers.append(text_marker)

        arrow_marker = self._base_marker(marker_id, Marker.ARROW)
        marker_id += 1
        arrow_marker.pose.position.x = pose.x
        arrow_marker.pose.position.y = pose.y
        arrow_marker.pose.position.z = pose.z + 0.05
        arrow_marker.pose.orientation.z = pose.qz
        arrow_marker.pose.orientation.w = pose.qw
        arrow_marker.scale.x = 0.28
        arrow_marker.scale.y = 0.04
        arrow_marker.scale.z = 0.04
        arrow_marker.color = color
        markers.append(arrow_marker)
        return marker_id

    def _append_route_line(
        self,
        markers: List[Marker],
        marker_id: int,
        route: Sequence[PoseData],
    ) -> int:
        line = self._base_marker(marker_id, Marker.LINE_STRIP)
        marker_id += 1
        line.scale.x = 0.03
        line.color = self._color(1.0, 1.0, 1.0, 0.85)
        line.points = [self._point(pose) for pose in route]
        markers.append(line)
        return marker_id

    def _path(self, route: Iterable[PoseData]) -> NavPath:
        path = NavPath()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in route:
            stamped = PoseStamped()
            stamped.header = path.header
            stamped.pose.position.x = pose.x
            stamped.pose.position.y = pose.y
            stamped.pose.position.z = pose.z
            stamped.pose.orientation.x = pose.qx
            stamped.pose.orientation.y = pose.qy
            stamped.pose.orientation.z = pose.qz
            stamped.pose.orientation.w = pose.qw
            path.poses.append(stamped)
        return path

    def _base_marker(self, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fruit_navigation"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        return marker

    @staticmethod
    def _point(pose: PoseData) -> Point:
        point = Point()
        point.x = pose.x
        point.y = pose.y
        point.z = pose.z
        return point

    @staticmethod
    def _color(r: float, g: float, b: float, a: float) -> ColorRGBA:
        return ColorRGBA(r=r, g=g, b=b, a=a)


def main(args=None) -> None:
    """Run the route marker node."""
    rclpy.init(args=args)
    node = None
    try:
        node = RouteMarkerNode()
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
