# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""ROS action server for KISS line-segment waypoint following."""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from fruit_navigation.axis_planner import (
    AxisPose,
    normalize_angle,
    OdomProgressWatchdog,
    pose_from_ros,
)


class AxisWaypointActionServer(Node):
    """Follow odom-frame waypoints using rotate-in-place and line tracking."""

    def __init__(self) -> None:
        super().__init__("axis_waypoint_action_server")
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")

        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("min_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("min_angular_speed", 0.0)
        self.declare_parameter("max_linear_acceleration", 0.8)
        self.declare_parameter("max_angular_acceleration", 2.0)
        self.declare_parameter("forward_tolerance", 0.03)
        self.declare_parameter("lateral_tolerance", 0.06)
        self.declare_parameter("rotate_yaw_tolerance", 0.06)
        self.declare_parameter("reverse_yaw_tolerance", 0.25)
        self.declare_parameter("linear_gain", 0.8)
        self.declare_parameter("yaw_gain", 1.8)
        self.declare_parameter("lookahead_distance", 0.3)
        self.declare_parameter("tf_timeout_sec", 0.5)
        self.declare_parameter("waypoint_retry_attempts", 0)
        self.declare_parameter("waypoint_pass_tolerance", 0.08)
        self.declare_parameter("progress_window_sec", 1.0)
        self.declare_parameter("progress_position_threshold", 0.03)
        self.declare_parameter("progress_yaw_threshold", 0.01)
        self.declare_parameter("stall_timeout_sec", 5.0)

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._control_frequency = float(self.get_parameter("control_frequency").value)
        self._max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self._min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self._min_angular_speed = float(self.get_parameter("min_angular_speed").value)
        self._max_linear_acceleration = float(
            self.get_parameter("max_linear_acceleration").value
        )
        self._max_angular_acceleration = float(
            self.get_parameter("max_angular_acceleration").value
        )
        self._forward_tolerance = float(self.get_parameter("forward_tolerance").value)
        self._lateral_tolerance = float(self.get_parameter("lateral_tolerance").value)
        self._rotate_yaw_tolerance = float(
            self.get_parameter("rotate_yaw_tolerance").value
        )
        self._reverse_yaw_tolerance = float(
            self.get_parameter("reverse_yaw_tolerance").value
        )
        self._linear_gain = float(self.get_parameter("linear_gain").value)
        self._yaw_gain = float(self.get_parameter("yaw_gain").value)
        self._lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self._tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self._waypoint_retry_attempts = int(
            self.get_parameter("waypoint_retry_attempts").value
        )
        self._waypoint_pass_tolerance = float(
            self.get_parameter("waypoint_pass_tolerance").value
        )
        self._progress_window_sec = float(
            self.get_parameter("progress_window_sec").value
        )
        self._progress_position_threshold = float(
            self.get_parameter("progress_position_threshold").value
        )
        self._progress_yaw_threshold = float(
            self.get_parameter("progress_yaw_threshold").value
        )
        self._stall_timeout_sec = float(
            self.get_parameter("stall_timeout_sec").value
        )
        self._goal_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._last_linear_command = 0.0
        self._last_angular_command = 0.0
        self._active_goal = False
        self._progress_watchdog = None

        self._validate_parameters()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_vel = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
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
        if self._min_angular_speed < 0.0:
            raise ValueError("min_angular_speed must not be negative")
        if self._min_angular_speed > self._max_angular_speed:
            raise ValueError("min_angular_speed must not exceed max_angular_speed")
        if self._max_linear_acceleration <= 0.0:
            raise ValueError("max_linear_acceleration must be positive")
        if self._max_angular_acceleration <= 0.0:
            raise ValueError("max_angular_acceleration must be positive")
        if self._forward_tolerance <= 0.0 or self._lateral_tolerance <= 0.0:
            raise ValueError("tolerances must be positive")
        if self._rotate_yaw_tolerance <= 0.0:
            raise ValueError("rotate_yaw_tolerance must be positive")
        if self._reverse_yaw_tolerance <= 0.0:
            raise ValueError("reverse_yaw_tolerance must be positive")
        if not math.isfinite(self._lookahead_distance) or self._lookahead_distance <= 0.0:
            raise ValueError("lookahead_distance must be finite and positive")
        if self._tf_timeout_sec <= 0.0:
            raise ValueError("timeouts must be positive")
        if self._waypoint_retry_attempts < 0:
            raise ValueError("waypoint_retry_attempts must not be negative")
        if self._waypoint_pass_tolerance <= 0.0:
            raise ValueError("waypoint_pass_tolerance must be positive")
        if self._progress_window_sec <= 0.0:
            raise ValueError("progress_window_sec must be positive")
        if self._progress_position_threshold <= 0.0:
            raise ValueError("progress_position_threshold must be positive")
        if self._progress_yaw_threshold <= 0.0:
            raise ValueError("progress_yaw_threshold must be positive")
        if self._stall_timeout_sec <= 0.0:
            raise ValueError("stall_timeout_sec must be positive")

    def _on_goal(self, goal_request) -> GoalResponse:
        try:
            self._validate_goal(goal_request)
        except ValueError as error:
            self.get_logger().error(f"Rejected waypoint goal: {error}")
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._active_goal:
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
                raise ValueError(f"all poses must be in {self._map_frame} frame")
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
            self._progress_watchdog = OdomProgressWatchdog(
                stall_timeout=self._stall_timeout_sec,
                window=self._progress_window_sec,
                position_threshold=self._progress_position_threshold,
                yaw_threshold=self._progress_yaw_threshold,
                now=time.monotonic(),
            )
            for index, target in enumerate(goal_handle.request.poses):
                feedback = FollowWaypoints.Feedback()
                feedback.current_waypoint = index
                goal_handle.publish_feedback(feedback)
                is_final_goal = index == len(goal_handle.request.poses) - 1
                target_pose = pose_from_ros(target)
                if not self._follow_with_recovery(
                    goal_handle,
                    target_pose,
                    is_final_goal,
                ):
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
            self._progress_watchdog = None
            with self._goal_lock:
                self._active_goal = False

    def _follow_with_recovery(
        self,
        goal_handle,
        target: AxisPose,
        is_final_goal: bool,
    ) -> bool:
        attempts = 1
        if not is_final_goal:
            attempts += self._waypoint_retry_attempts
        for attempt in range(attempts):
            if self._follow_one(goal_handle, target, is_final_goal):
                return True
            if goal_handle.is_cancel_requested:
                return False
            if (
                self._progress_watchdog is not None
                and self._progress_watchdog.stalled(time.monotonic())
            ):
                return False
            if attempt + 1 < attempts:
                self.get_logger().warning(
                    "Waypoint motion failed; retrying "
                    f"attempt={attempt + 2}/{attempts} "
                    f"target=({target.x:.3f},{target.y:.3f})"
                )

        if is_final_goal:
            return False
        current = self._lookup_pose()
        if current is None or goal_handle.is_cancel_requested:
            return False
        distance = math.hypot(target.x - current.x, target.y - current.y)
        if distance <= self._waypoint_pass_tolerance:
            self.get_logger().warning(
                "Waypoint tolerated after failed recovery: "
                f"target=({target.x:.3f},{target.y:.3f}) "
                f"actual=({current.x:.3f},{current.y:.3f}) "
                f"distance={distance:.3f} "
                f"pass_tolerance={self._waypoint_pass_tolerance:.3f}"
            )
            self._publish_stop_burst()
            return True
        return False

    def _follow_one(
        self,
        goal_handle,
        target: AxisPose,
        is_final_goal: bool = False,
    ) -> bool:
        start = self._lookup_pose()
        if start is None:
            return False
        distance = math.hypot(target.x - start.x, target.y - start.y)
        self.get_logger().info(
            "Starting waypoint: "
            f"start=({start.x:.3f},{start.y:.3f},{start.yaw:.3f}) "
            f"target=({target.x:.3f},{target.y:.3f},{target.yaw:.3f}) "
            f"distance={distance:.3f} is_final_goal={is_final_goal}"
        )
        if distance > self._forward_tolerance:
            path_heading = math.atan2(target.y - start.y, target.x - start.x)
            reverse_heading = normalize_angle(path_heading + math.pi)
            use_reverse = (
                abs(normalize_angle(target.yaw - reverse_heading))
                <= self._reverse_yaw_tolerance
            )
            approach_yaw = reverse_heading if use_reverse else path_heading
            if not self._rotate_to(
                goal_handle,
                approach_yaw,
                self._rotate_yaw_tolerance,
            ):
                return False

            start = self._lookup_pose()
            if start is None:
                return False
            distance = math.hypot(target.x - start.x, target.y - start.y)
            path_heading = math.atan2(target.y - start.y, target.x - start.x)
            reverse_heading = normalize_angle(path_heading + math.pi)
            use_reverse = (
                abs(normalize_angle(target.yaw - reverse_heading))
                <= self._reverse_yaw_tolerance
            )
            approach_yaw = reverse_heading if use_reverse else path_heading
            linear_direction = -1.0 if use_reverse else 1.0
            self.get_logger().info(
                "Waypoint line after rotation: "
                f"start=({start.x:.3f},{start.y:.3f},{start.yaw:.3f}) "
                f"distance={distance:.3f} "
                f"path_heading={path_heading:.3f} "
                f"approach_yaw={approach_yaw:.3f} "
                f"motion={'reverse' if use_reverse else 'forward'}"
            )
            if not self._drive_line(
                goal_handle,
                start,
                target,
                path_heading,
                approach_yaw,
                linear_direction,
                distance,
            ):
                return False
        return self._rotate_to(goal_handle, target.yaw, self._rotate_yaw_tolerance)

    def _rotate_to(self, goal_handle, target_yaw: float, yaw_tolerance: float) -> bool:
        period = 1.0 / self._control_frequency
        while rclpy.ok():
            if self._should_stop(goal_handle):
                return False
            current = self._lookup_pose()
            if current is None:
                return False
            self._observe_progress(current)
            error = normalize_angle(target_yaw - current.yaw)
            if abs(error) <= yaw_tolerance:
                command = self._publish_limited_command(Twist())
                if self._command_is_stopped(command):
                    return True
                time.sleep(period)
                continue
            twist = Twist()
            twist.angular.z = self._rotation_command(error)
            self._publish_limited_command(twist)
            time.sleep(period)
        return False

    def _drive_line(
        self,
        goal_handle,
        start: AxisPose,
        target: AxisPose,
        path_heading: float,
        approach_yaw: float,
        linear_direction: float,
        distance: float,
    ) -> bool:
        period = 1.0 / self._control_frequency
        direction_x = math.cos(path_heading)
        direction_y = math.sin(path_heading)
        while rclpy.ok():
            if self._should_stop(goal_handle):
                return False
            current = self._lookup_pose()
            if current is None:
                return False
            self._observe_progress(current)
            relative_x = current.x - start.x
            relative_y = current.y - start.y
            traveled = relative_x * direction_x + relative_y * direction_y
            remaining = distance - traveled
            lateral_error = -relative_x * direction_y + relative_y * direction_x
            position_reached = (
                abs(remaining) <= self._forward_tolerance
                and abs(lateral_error) <= self._lateral_tolerance
            )
            if position_reached:
                command = self._publish_limited_command(Twist())
                if self._command_is_stopped(command):
                    self.get_logger().info(
                        "Reached map waypoint: "
                        f"target=({target.x:.3f},{target.y:.3f}) "
                        f"actual=({current.x:.3f},{current.y:.3f}) "
                        f"longitudinal_error={remaining:.3f} "
                        f"lateral_error={lateral_error:.3f}"
                    )
                    return True
                time.sleep(period)
                continue

            heading_error = normalize_angle(approach_yaw - current.yaw)
            # Extend the steering carrot along the line near the endpoint;
            # shortening it to zero would amplify noise and cause sharp turns.
            lookahead = math.copysign(
                self._lookahead_distance, remaining * linear_direction
            )
            lateral = lateral_error * linear_direction
            carrot_y = (
                lookahead * math.sin(heading_error)
                - lateral * math.cos(heading_error)
            )
            curvature = 2.0 * carrot_y / (lookahead ** 2 + lateral ** 2)

            speed = abs(self._linear_command(remaining * self._linear_gain))
            speed = min(
                speed,
                max(
                    self._min_linear_speed,
                    math.sqrt(2.0 * self._max_linear_acceleration * abs(remaining)),
                ),
            )
            if curvature != 0.0:
                speed = min(
                    speed,
                    max(self._min_linear_speed, self._max_angular_speed / abs(curvature)),
                )
            twist = Twist()
            twist.linear.x = math.copysign(speed, remaining * linear_direction)
            twist.angular.z = self._clamp(
                twist.linear.x * curvature,
                -self._max_angular_speed,
                self._max_angular_speed,
            )
            self._publish_limited_command(twist)
            time.sleep(period)
        return False

    def _should_stop(self, goal_handle) -> bool:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self._publish_stop_burst()
            return True
        if (
            self._progress_watchdog is not None
            and self._progress_watchdog.stalled(time.monotonic())
        ):
            self.get_logger().error(
                "Aborting waypoint goal: no odom progress for "
                f"{self._stall_timeout_sec:.1f} seconds"
            )
            self._publish_stop_burst()
            return True
        return False

    def _observe_progress(self, current: AxisPose) -> None:
        if self._progress_watchdog is None:
            return
        progressed, position_delta, yaw_delta = self._progress_watchdog.observe(
            current,
            time.monotonic(),
        )
        if progressed:
            self.get_logger().info(
                "Odom progress: "
                f"position_delta={position_delta:.3f}m "
                f"yaw_delta={yaw_delta:.3f}rad"
            )

    def _lookup_pose(self) -> Optional[AxisPose]:
        deadline = time.monotonic() + self._tf_timeout_sec
        transform = None
        last_error = None
        last_transform_age = None
        stop_published = False
        while rclpy.ok():
            try:
                candidate = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    self._base_frame,
                    Time(),
                    timeout=Duration(),
                )
            except TransformException as error:
                last_error = error
            else:
                transform_time = Time.from_msg(candidate.header.stamp)
                transform_age = (
                    self.get_clock().now() - transform_time
                ).nanoseconds / 1e9
                if transform_age <= self._tf_timeout_sec:
                    transform = candidate
                    break
                last_transform_age = transform_age

            if not stop_published:
                self._publish_stop()
                stop_published = True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.01, remaining))

        if transform is None:
            if last_transform_age is not None:
                self.get_logger().error(
                    "Aborting waypoint goal: "
                    f"TF is {last_transform_age:.3f}s old after waiting "
                    f"{self._tf_timeout_sec:.3f}s for a fresh transform"
                )
            else:
                detail = f": {last_error}" if last_error is not None else ""
                self.get_logger().error(
                    f"Aborting waypoint goal: TF unavailable{detail}"
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

    def _publish_stop(self) -> bool:
        if not rclpy.ok(context=self.context):
            return False
        with self._command_lock:
            try:
                self._cmd_vel.publish(Twist())
            except RCLError:
                return False
            self._last_linear_command = 0.0
            self._last_angular_command = 0.0
        return True

    def _publish_stop_burst(self) -> None:
        for _ in range(3):
            if not self._publish_stop():
                return
            time.sleep(0.02)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    def _rotation_command(self, yaw_error: float) -> float:
        command = self._clamp(
            self._yaw_gain * yaw_error,
            -self._max_angular_speed,
            self._max_angular_speed,
        )
        if abs(command) < self._min_angular_speed:
            return math.copysign(self._min_angular_speed, yaw_error)
        return command

    def _linear_command(self, requested_speed: float) -> float:
        speed = self._clamp(
            abs(requested_speed),
            self._min_linear_speed,
            self._max_linear_speed,
        )
        return math.copysign(speed, requested_speed)

    def _publish_limited_command(self, requested: Twist) -> Twist:
        period = 1.0 / self._control_frequency
        with self._command_lock:
            command = Twist()
            command.linear.x = self._move_towards(
                self._last_linear_command,
                requested.linear.x,
                self._max_linear_acceleration * period,
            )
            command.angular.z = self._move_towards(
                self._last_angular_command,
                requested.angular.z,
                self._max_angular_acceleration * period,
            )
            self._cmd_vel.publish(command)
            self._last_linear_command = command.linear.x
            self._last_angular_command = command.angular.z
        return command

    @staticmethod
    def _move_towards(
        current: float,
        target: float,
        max_delta: float,
    ) -> float:
        return current + AxisWaypointActionServer._clamp(
            target - current,
            -max_delta,
            max_delta,
        )

    @staticmethod
    def _command_is_stopped(command: Twist) -> bool:
        return command.linear.x == 0.0 and command.angular.z == 0.0


def main(args=None) -> None:
    """Run the axis waypoint action server."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = AxisWaypointActionServer()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
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
