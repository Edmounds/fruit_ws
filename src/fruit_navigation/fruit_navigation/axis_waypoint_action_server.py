# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""ROS action server for KISS cardinal-axis waypoint following."""

import math
import threading
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from fruit_navigation.axis_planner import (
    AxisPose,
    ProgressWatchdog,
    axis_cross_track_error,
    axis_forward_remaining,
    choose_axis_segments,
    normalize_angle,
    pose_from_ros,
)


class AxisWaypointActionServer(Node):
    """Follow map-frame waypoints using only rotate-in-place and forward motion."""

    def __init__(self) -> None:
        super().__init__("axis_waypoint_action_server")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("min_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("position_tolerance", 0.05)
        self.declare_parameter("yaw_tolerance", 0.08)
        self.declare_parameter("linear_gain", 0.8)
        self.declare_parameter("yaw_gain", 1.8)
        self.declare_parameter("lateral_gain", 0.8)
        self.declare_parameter("tf_timeout_sec", 0.3)
        self.declare_parameter("progress_timeout_sec", 5.0)
        self.declare_parameter("progress_min_delta", 0.01)
        self.declare_parameter("diagnostics_topic", "/diagnostics")

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._control_frequency = float(self.get_parameter("control_frequency").value)
        self._max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self._min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self._position_tolerance = float(self.get_parameter("position_tolerance").value)
        self._yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self._linear_gain = float(self.get_parameter("linear_gain").value)
        self._yaw_gain = float(self.get_parameter("yaw_gain").value)
        self._lateral_gain = float(self.get_parameter("lateral_gain").value)
        self._tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self._progress_timeout_sec = float(
            self.get_parameter("progress_timeout_sec").value
        )
        self._progress_min_delta = float(self.get_parameter("progress_min_delta").value)
        self._goal_lock = threading.Lock()
        self._active_goal = False
        self._localization_ready = False
        self._localization_error = False

        self._validate_parameters()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_vel = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self._diagnostics_subscription = self.create_subscription(
            DiagnosticArray,
            self.get_parameter("diagnostics_topic").value,
            self._on_diagnostics,
            10,
        )
        self._action_server = ActionServer(
            self,
            FollowWaypoints,
            "/follow_waypoints",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )

    def _validate_parameters(self) -> None:
        if self._control_frequency <= 0.0:
            raise ValueError("control_frequency must be greater than zero")
        if self._max_linear_speed <= 0.0 or self._max_angular_speed <= 0.0:
            raise ValueError("speed limits must be positive")
        if self._min_linear_speed < 0.0:
            raise ValueError("min_linear_speed must not be negative")
        if self._min_linear_speed > self._max_linear_speed:
            raise ValueError("min_linear_speed must not exceed max_linear_speed")
        if self._position_tolerance <= 0.0 or self._yaw_tolerance <= 0.0:
            raise ValueError("tolerances must be positive")
        if self._tf_timeout_sec <= 0.0 or self._progress_timeout_sec <= 0.0:
            raise ValueError("timeouts must be positive")
        if self._progress_min_delta <= 0.0:
            raise ValueError("progress_min_delta must be positive")

    def _on_goal(self, goal_request) -> GoalResponse:
        try:
            self._validate_goal(goal_request)
        except ValueError as error:
            self.get_logger().error(f"Rejected waypoint goal: {error}")
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._active_goal:
                return GoalResponse.REJECT
            if not self._localization_ready or self._localization_error:
                self.get_logger().error(
                    "Rejected waypoint goal: GICP localization is not ready"
                )
                return GoalResponse.REJECT
            self._active_goal = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self._publish_stop()
        return CancelResponse.ACCEPT

    def _validate_goal(self, goal) -> None:
        if not goal.poses:
            raise ValueError("goal must contain at least one pose")
        for pose in goal.poses:
            if pose.header.frame_id != self._map_frame:
                raise ValueError("all poses must be in map frame")
            values = (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
            if not all(math.isfinite(float(value)) for value in values):
                raise ValueError("pose values must be finite")

    def _execute(self, goal_handle):
        missed_waypoints = []
        result = FollowWaypoints.Result()
        try:
            for index, target in enumerate(goal_handle.request.poses):
                feedback = FollowWaypoints.Feedback()
                feedback.current_waypoint = index
                goal_handle.publish_feedback(feedback)
                if not self._follow_one(goal_handle, pose_from_ros(target)):
                    missed_waypoints.append(index)
                    if goal_handle.is_active:
                        goal_handle.abort()
                    result.missed_waypoints = missed_waypoints
                    self._publish_stop_burst()
                    return result
            goal_handle.succeed()
            result.missed_waypoints = missed_waypoints
            self._publish_stop()
            return result
        finally:
            with self._goal_lock:
                self._active_goal = False

    def _follow_one(self, goal_handle, target: AxisPose) -> bool:
        current = self._lookup_pose()
        if current is None:
            return False
        segments = choose_axis_segments(current, target, self._position_tolerance)
        for segment in segments:
            if not self._rotate_to(goal_handle, segment.heading):
                return False
            if not self._drive_axis(
                goal_handle,
                segment.axis,
                segment.target,
                segment.lateral_target,
                segment.heading,
            ):
                return False
        return self._rotate_to(goal_handle, target.yaw)

    def _rotate_to(self, goal_handle, target_yaw: float) -> bool:
        period = 1.0 / self._control_frequency
        watchdog = ProgressWatchdog(
            self._progress_timeout_sec,
            self._progress_min_delta,
            time.monotonic(),
        )
        while rclpy.ok():
            if self._should_stop(goal_handle):
                return False
            current = self._lookup_pose()
            if current is None:
                return False
            error = normalize_angle(target_yaw - current.yaw)
            if abs(error) <= self._yaw_tolerance:
                self._publish_stop()
                return True
            if watchdog.stalled(error, time.monotonic()):
                self.get_logger().error("Aborting waypoint goal: no rotation progress")
                self._publish_stop_burst()
                return False
            twist = Twist()
            twist.angular.z = self._clamp(
                self._yaw_gain * error,
                -self._max_angular_speed,
                self._max_angular_speed,
            )
            self._cmd_vel.publish(twist)
            time.sleep(period)
        return False

    def _drive_axis(
        self,
        goal_handle,
        axis: str,
        target_value: float,
        lateral_target: float,
        heading: float,
    ) -> bool:
        period = 1.0 / self._control_frequency
        watchdog = ProgressWatchdog(
            self._progress_timeout_sec,
            self._progress_min_delta,
            time.monotonic(),
        )
        while rclpy.ok():
            if self._should_stop(goal_handle):
                return False
            current = self._lookup_pose()
            if current is None:
                return False
            remaining = axis_forward_remaining(axis, target_value, current, heading)
            if remaining <= self._position_tolerance:
                self._publish_stop()
                return True
            if watchdog.stalled(remaining, time.monotonic()):
                self.get_logger().error("Aborting waypoint goal: no progress")
                self._publish_stop_burst()
                return False

            heading_error = normalize_angle(heading - current.yaw)
            lateral_error = axis_cross_track_error(
                axis,
                current,
                lateral_target,
                heading,
            )
            speed = remaining * self._linear_gain
            speed = self._clamp(speed, self._min_linear_speed, self._max_linear_speed)
            twist = Twist()
            twist.linear.x = speed
            correction = self._yaw_gain * heading_error - self._lateral_gain * lateral_error
            twist.angular.z = self._clamp(
                correction,
                -self._max_angular_speed,
                self._max_angular_speed,
            )
            self._cmd_vel.publish(twist)
            time.sleep(period)
        return False

    def _should_stop(self, goal_handle) -> bool:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self._publish_stop_burst()
            return True
        if not self._localization_ready or self._localization_error:
            self.get_logger().error("Aborting waypoint goal: GICP localization ERROR")
            self._publish_stop_burst()
            return True
        return False

    def _lookup_pose(self) -> Optional[AxisPose]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
        except TransformException as error:
            self.get_logger().error(f"Aborting waypoint goal: TF unavailable: {error}")
            self._publish_stop_burst()
            return None
        transform_time = Time.from_msg(transform.header.stamp)
        transform_age = (self.get_clock().now() - transform_time).nanoseconds / 1e9
        if transform_age > self._tf_timeout_sec:
            self.get_logger().error(
                f"Aborting waypoint goal: TF is {transform_age:.3f}s old"
            )
            self._publish_stop_burst()
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return AxisPose(
            x=float(translation.x),
            y=float(translation.y),
            yaw=math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            ),
        )

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "pb_gicp_localization/registration":
                continue
            values = {item.key: item.value for item in status.values}
            if status.level == DiagnosticStatus.ERROR:
                self._localization_error = True
            elif (
                status.level == DiagnosticStatus.OK
                and values.get("accepted") == "1"
                and values.get("has_accepted_once") == "1"
            ):
                self._localization_ready = True
                self._localization_error = False

    def _publish_stop(self) -> None:
        self._cmd_vel.publish(Twist())

    def _publish_stop_burst(self) -> None:
        for _ in range(3):
            self._publish_stop()
            time.sleep(0.02)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)


def main(args=None) -> None:
    """Run the axis waypoint action server."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = AxisWaypointActionServer()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._publish_stop_burst()
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
