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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from fruit_navigation.axis_planner import (
    AxisPose,
    ProgressWatchdog,
    normalize_angle,
    pose_from_ros,
)


class AxisWaypointActionServer(Node):
    """Follow map-frame waypoints using rotate-in-place and line tracking."""

    def __init__(self) -> None:
        super().__init__("axis_waypoint_action_server")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("require_gicp_localization", True)
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
        self.declare_parameter("blend_yaw_tolerance", 0.12)
        self.declare_parameter("reverse_yaw_tolerance", 0.25)
        self.declare_parameter("linear_gain", 0.8)
        self.declare_parameter("yaw_gain", 1.8)
        self.declare_parameter("lateral_gain", 0.8)
        self.declare_parameter("final_yaw_mode", "blend_on_path")
        self.declare_parameter("final_yaw_blend_distance", 0.6)
        self.declare_parameter("tf_timeout_sec", 0.3)
        self.declare_parameter("progress_timeout_sec", 5.0)
        self.declare_parameter("progress_min_delta", 0.01)
        self.declare_parameter("waypoint_retry_attempts", 1)
        self.declare_parameter("waypoint_pass_tolerance", 0.08)
        self.declare_parameter("diagnostics_topic", "/diagnostics")

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._require_gicp_localization = bool(
            self.get_parameter("require_gicp_localization").value
        )
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
        self._blend_yaw_tolerance = float(
            self.get_parameter("blend_yaw_tolerance").value
        )
        self._reverse_yaw_tolerance = float(
            self.get_parameter("reverse_yaw_tolerance").value
        )
        self._linear_gain = float(self.get_parameter("linear_gain").value)
        self._yaw_gain = float(self.get_parameter("yaw_gain").value)
        self._lateral_gain = float(self.get_parameter("lateral_gain").value)
        self._final_yaw_mode = self.get_parameter("final_yaw_mode").value
        self._final_yaw_blend_distance = float(
            self.get_parameter("final_yaw_blend_distance").value
        )
        self._tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self._progress_timeout_sec = float(
            self.get_parameter("progress_timeout_sec").value
        )
        self._progress_min_delta = float(self.get_parameter("progress_min_delta").value)
        self._waypoint_retry_attempts = int(
            self.get_parameter("waypoint_retry_attempts").value
        )
        self._waypoint_pass_tolerance = float(
            self.get_parameter("waypoint_pass_tolerance").value
        )
        self._goal_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._last_linear_command = 0.0
        self._last_angular_command = 0.0
        self._active_goal = False
        self._localization_ready = not self._require_gicp_localization
        self._localization_error = False

        self._validate_parameters()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_vel = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        diagnostics_qos = QoSProfile(depth=1)
        diagnostics_qos.reliability = ReliabilityPolicy.RELIABLE
        diagnostics_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._diagnostics_subscription = None
        if self._require_gicp_localization:
            self._diagnostics_subscription = self.create_subscription(
                DiagnosticArray,
                self.get_parameter("diagnostics_topic").value,
                self._on_diagnostics,
                diagnostics_qos,
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
        if self._rotate_yaw_tolerance <= 0.0 or self._blend_yaw_tolerance <= 0.0:
            raise ValueError("yaw tolerances must be positive")
        if self._reverse_yaw_tolerance <= 0.0:
            raise ValueError("reverse_yaw_tolerance must be positive")
        if self._final_yaw_mode not in ("blend_on_path", "rotate_at_goal"):
            raise ValueError("final_yaw_mode must be blend_on_path or rotate_at_goal")
        if self._final_yaw_blend_distance <= 0.0:
            raise ValueError("final_yaw_blend_distance must be positive")
        if self._tf_timeout_sec <= 0.0 or self._progress_timeout_sec <= 0.0:
            raise ValueError("timeouts must be positive")
        if self._progress_min_delta <= 0.0:
            raise ValueError("progress_min_delta must be positive")
        if self._waypoint_retry_attempts < 0:
            raise ValueError("waypoint_retry_attempts must not be negative")
        if self._waypoint_pass_tolerance <= 0.0:
            raise ValueError("waypoint_pass_tolerance must be positive")

    def _on_goal(self, goal_request) -> GoalResponse:
        try:
            self._validate_goal(goal_request)
        except ValueError as error:
            self.get_logger().error(f"Rejected waypoint goal: {error}")
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._active_goal:
                return GoalResponse.REJECT
            if self._require_gicp_localization and (
                not self._localization_ready or self._localization_error
            ):
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
            if self._hard_stop_requested(goal_handle):
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
        if current is None or self._hard_stop_requested(goal_handle):
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

    def _hard_stop_requested(self, goal_handle) -> bool:
        return bool(
            goal_handle.is_cancel_requested
            or (
                self._require_gicp_localization
                and (not self._localization_ready or self._localization_error)
            )
        )

    def _follow_one(
        self,
        goal_handle,
        target: AxisPose,
        is_final_goal: bool = False,
    ) -> bool:
        if not self._refresh_final_yaw_mode():
            return False
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
        final_yaw = (
            target.yaw
            if is_final_goal and self._final_yaw_mode == "blend_on_path"
            else None
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
                final_yaw,
            ):
                return False
        if final_yaw is not None:
            return self._rotate_to(
                goal_handle,
                final_yaw,
                self._blend_yaw_tolerance,
            )
        return self._rotate_to(goal_handle, target.yaw, self._rotate_yaw_tolerance)

    def _rotate_to(self, goal_handle, target_yaw: float, yaw_tolerance: float) -> bool:
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
            if abs(error) <= yaw_tolerance:
                command = self._publish_limited_command(Twist())
                if self._command_is_stopped(command):
                    return True
                time.sleep(period)
                continue
            if watchdog.stalled(error, time.monotonic()):
                self.get_logger().error(
                    "Aborting waypoint goal: no rotation progress "
                    f"current_yaw={current.yaw:.3f} target_yaw={target_yaw:.3f} "
                    f"yaw_error={error:.3f} tolerance={yaw_tolerance:.3f}"
                )
                self._publish_stop_burst()
                return False
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
        final_yaw: Optional[float] = None,
    ) -> bool:
        period = 1.0 / self._control_frequency
        watchdog = ProgressWatchdog(
            self._progress_timeout_sec,
            self._progress_min_delta,
            time.monotonic(),
        )
        direction_x = math.cos(path_heading)
        direction_y = math.sin(path_heading)
        while rclpy.ok():
            if self._should_stop(goal_handle):
                return False
            current = self._lookup_pose()
            if current is None:
                return False
            relative_x = current.x - start.x
            relative_y = current.y - start.y
            traveled = relative_x * direction_x + relative_y * direction_y
            remaining = distance - traveled
            lateral_error = -relative_x * direction_y + relative_y * direction_x
            position_reached = (
                abs(remaining) <= self._forward_tolerance
                and abs(lateral_error) <= self._lateral_tolerance
            )
            final_yaw_error = (
                normalize_angle(target.yaw - current.yaw)
                if final_yaw is not None
                else 0.0
            )
            if position_reached and abs(final_yaw_error) <= self._blend_yaw_tolerance:
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
            progress_error = max(abs(remaining), abs(lateral_error))
            if watchdog.stalled(progress_error, time.monotonic()):
                self.get_logger().error(
                    "Aborting waypoint goal: no progress "
                    f"start=({start.x:.3f},{start.y:.3f}) "
                    f"current=({current.x:.3f},{current.y:.3f},{current.yaw:.3f}) "
                    f"target=({target.x:.3f},{target.y:.3f},{target.yaw:.3f}) "
                    f"traveled={traveled:.3f}/{distance:.3f} "
                    f"remaining={remaining:.3f} lateral_error={lateral_error:.3f} "
                    f"progress_error={progress_error:.3f} "
                    f"forward_tolerance={self._forward_tolerance:.3f} "
                    f"lateral_tolerance={self._lateral_tolerance:.3f}"
                )
                self._publish_stop_burst()
                return False

            yaw_target = approach_yaw
            if final_yaw is not None and remaining < self._final_yaw_blend_distance:
                blend_span = max(
                    self._final_yaw_blend_distance - self._forward_tolerance,
                    self._forward_tolerance,
                )
                blend_progress = self._clamp(
                    (self._final_yaw_blend_distance - remaining) / blend_span,
                    0.0,
                    1.0,
                )
                yaw_target = normalize_angle(
                    approach_yaw
                    + normalize_angle(final_yaw - approach_yaw) * blend_progress
                )

            heading_error = normalize_angle(yaw_target - current.yaw)
            twist = Twist()
            if position_reached and final_yaw is not None:
                twist.angular.z = self._rotation_command(final_yaw_error)
                self._publish_limited_command(twist)
                time.sleep(period)
                continue
            if abs(remaining) > self._forward_tolerance:
                speed = remaining * self._linear_gain * linear_direction
                twist.linear.x = self._linear_command(speed)
            elif abs(lateral_error) > self._lateral_tolerance:
                twist.linear.x = self._linear_command(
                    linear_direction * self._min_linear_speed
                )
            correction = (
                self._yaw_gain * heading_error - self._lateral_gain * lateral_error
            )
            twist.angular.z = self._clamp(
                correction,
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
        if self._require_gicp_localization and (
            not self._localization_ready or self._localization_error
        ):
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

    def _refresh_final_yaw_mode(self) -> bool:
        final_yaw_mode = str(self.get_parameter("final_yaw_mode").value)
        if final_yaw_mode not in ("blend_on_path", "rotate_at_goal"):
            self.get_logger().error(
                "Aborting waypoint goal: invalid final_yaw_mode "
                f"{final_yaw_mode}"
            )
            self._publish_stop_burst()
            return False
        self._final_yaw_mode = final_yaw_mode
        return True


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
