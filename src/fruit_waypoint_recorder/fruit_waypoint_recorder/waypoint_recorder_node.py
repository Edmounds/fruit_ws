# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""ROS 2 node backing the waypoint-maker RViz panel."""

import json
import time
from functools import partial
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from fruit_waypoint_recorder.recorder import (
    PoseData,
    TestPointStore,
    VALID_TEST_POINT_TYPES,
    mean_pose,
    yaw_from_quaternion,
)


FINAL_YAW_MODES = ("blend_on_path", "rotate_at_goal")


def latched_qos() -> QoSProfile:
    """Return transient local QoS for operator prompts."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class WaypointRecorderNode(Node):
    """Record and exercise named points in the configured waypoint file."""

    def __init__(self) -> None:
        super().__init__("fruit_waypoint_recorder")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("sample_duration_sec", 1.0)
        self.declare_parameter("sample_period_sec", 0.05)
        self.declare_parameter("command_topic", "/fruit/waypoint_maker/command")
        self.declare_parameter("state_topic", "/fruit/waypoint_maker/state")
        self.declare_parameter("marker_topic", "/fruit/waypoint_maker/markers")
        self.declare_parameter("follow_waypoints_action", "/follow_waypoints")
        self.declare_parameter("axis_waypoint_node", "/axis_waypoint_action_server")
        self.declare_parameter("default_final_yaw_mode", "blend_on_path")
        self.declare_parameter("source_root", "")

        self._frame_id = self.get_parameter("frame_id").value
        self._base_frame = self.get_parameter("base_frame").value
        source_root = str(self.get_parameter("source_root").value).strip()
        if not source_root:
            raise ValueError("source_root must be configured")
        self._source_root = Path(source_root).resolve()
        self._sample_duration_sec = float(
            self.get_parameter("sample_duration_sec").value
        )
        self._sample_period_sec = float(
            self.get_parameter("sample_period_sec").value
        )
        self._current_final_yaw_mode = str(
            self.get_parameter("default_final_yaw_mode").value
        )
        if self._current_final_yaw_mode not in FINAL_YAW_MODES:
            raise ValueError(
                "default_final_yaw_mode must be blend_on_path or rotate_at_goal"
            )
        waypoint_path = self._resolve_src_yaml_path(
            str(self.get_parameter("waypoint_file").value)
        )
        self._test_store = TestPointStore(waypoint_path)
        self._test_store.load(waypoint_path)
        self._sampling = False
        self._active_test_goal = False
        self._pending_test_cursor = None
        self._last_message = f"Loaded waypoint file {waypoint_path}"

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            self.get_parameter("follow_waypoints_action").value,
        )
        axis_waypoint_node = str(
            self.get_parameter("axis_waypoint_node").value
        ).rstrip("/")
        if not axis_waypoint_node:
            axis_waypoint_node = "/axis_waypoint_action_server"
        self._axis_waypoint_parameters = self.create_client(
            SetParameters,
            f"{axis_waypoint_node}/set_parameters",
        )

        self._prompt_publisher = self.create_publisher(
            String,
            "~/prompt",
            latched_qos(),
        )
        self._state_publisher = self.create_publisher(
            String,
            self.get_parameter("state_topic").value,
            latched_qos(),
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            1,
        )
        self._command_subscription = self.create_subscription(
            String,
            self.get_parameter("command_topic").value,
            self._on_command,
            10,
        )
        self._undo_service = self.create_service(Trigger, "~/undo", self._on_undo)
        self._clear_service = self.create_service(Trigger, "~/clear", self._on_clear)
        self._save_service = self.create_service(Trigger, "~/save", self._on_save)
        self._publish_state()
        self.get_logger().info(f"Editing waypoint file {waypoint_path}")

    def _on_command(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as error:
            self._reject_command(f"Invalid waypoint-maker command JSON: {error}")
            return
        if not isinstance(payload, dict):
            self._reject_command("Waypoint-maker command must be a JSON object")
            return

        command = payload.get("command")
        if command == "record":
            self._record_from_current_pose(payload)
        elif command == "undo":
            self._undo_waypoint()
        elif command == "save":
            self._save_waypoints()
        elif command == "run_next_test":
            self._run_next_test_segment(payload)
        elif command == "reset_test_cursor":
            self._test_store.reset_cursor()
            self._publish_state("Reset test cursor")
        elif command == "load_test_file":
            self._load_test_file(payload)
        else:
            self._reject_command(f"Unknown waypoint-maker command: {command}")

    def _on_undo(self, _request, response):
        removed = self._test_store.undo()
        response.success = removed is not None
        response.message = (
            f"Removed waypoint {removed.name}"
            if removed is not None
            else "No waypoint to undo"
        )
        self._publish_state(response.message)
        return response

    def _on_clear(self, _request, response):
        self._test_store.clear()
        response.success = True
        response.message = "Cleared waypoint file in memory"
        self._publish_state(response.message)
        return response

    def _on_save(self, _request, response):
        try:
            self._test_store.save()
        except ValueError as error:
            response.success = False
            response.message = str(error)
        else:
            response.success = True
            response.message = f"Saved waypoints to {self._test_store.output_path}"
        self._publish_state(response.message)
        return response

    def _record_from_current_pose(self, payload) -> None:
        if self._sampling:
            self._reject_command("Already sampling current pose")
            return
        mode = str(payload.get("mode", "test"))
        if mode != "test":
            self._reject_command(f"Unknown record mode: {mode}")
            return
        name = str(payload.get("name", "")).strip()
        point_type = str(payload.get("point_type", "")).strip()
        if not self._can_record_test_point(name, point_type):
            return

        self._sampling = True
        self._publish_state("Sampling current pose")
        try:
            pose = self._sample_current_pose(name)
            point = self._test_store.append(name, point_type, pose)
            message = f"Recorded {point.point_type} {point.name}"
            self.get_logger().info(message)
        except (RuntimeError, ValueError) as error:
            message = str(error)
            self.get_logger().error(message)
        finally:
            self._sampling = False
            self._publish_state(message)

    def _can_record_test_point(self, name: str, point_type: str) -> bool:
        if not name:
            self._reject_command("test point name must not be empty")
            return False
        if point_type not in VALID_TEST_POINT_TYPES:
            self._reject_command("point_type must be waypoint or goal")
            return False
        if any(point.name == name for point in self._test_store.points):
            self._reject_command(f"test point already exists: {name}")
            return False
        return True

    def _sample_current_pose(self, name: str) -> PoseData:
        samples = []
        deadline = time.monotonic() + self._sample_duration_sec
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._frame_id,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                samples.append(self._pose_from_transform(name, transform))
            except TransformException as error:
                last_error = error
            time.sleep(self._sample_period_sec)
        if not samples:
            detail = f": {last_error}" if last_error else ""
            raise RuntimeError(
                f"No valid TF samples for {self._frame_id}->{self._base_frame}{detail}"
            )
        return mean_pose(name, samples)

    @staticmethod
    def _pose_from_transform(name: str, transform) -> PoseData:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return PoseData(
            name=name,
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
            qx=float(rotation.x),
            qy=float(rotation.y),
            qz=float(rotation.z),
            qw=float(rotation.w),
            yaw=yaw_from_quaternion(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
        )

    def _undo_waypoint(self) -> None:
        removed = self._test_store.undo()
        message = (
            f"Removed waypoint {removed.name}"
            if removed is not None
            else "No waypoint to undo"
        )
        self._publish_state(message)

    def _save_waypoints(self) -> None:
        try:
            self._test_store.save()
        except ValueError as error:
            self._publish_state(str(error))
            return
        self._publish_state(f"Saved waypoints to {self._test_store.output_path}")

    def _load_test_file(self, payload) -> None:
        if self._sampling or self._active_test_goal:
            self._reject_command("Cannot load test waypoints while busy")
            return
        try:
            path = self._resolve_src_yaml_path(str(payload.get("path", "")))
            self._test_store.load(path)
        except (OSError, ValueError) as error:
            self._reject_command(str(error))
            return
        self._publish_state(f"Loaded test waypoints from {self._test_store.output_path}")

    def _resolve_src_yaml_path(self, raw_path: str) -> Path:
        path = Path(raw_path).expanduser()
        if not path.is_absolute():
            path = self._source_root / path
        resolved = path.resolve()
        if resolved.suffix not in (".yaml", ".yml"):
            raise ValueError("waypoint file must be a YAML file")
        try:
            resolved.relative_to(self._source_root)
        except ValueError as error:
            raise ValueError(
                f"waypoint file must be under {self._source_root}"
            ) from error
        if not resolved.is_file():
            raise ValueError(f"waypoint file does not exist: {resolved}")
        return resolved

    def _run_next_test_segment(self, payload) -> None:
        if self._active_test_goal:
            self._reject_command("Test segment is already running")
            return
        final_yaw_mode = str(
            payload.get("final_yaw_mode", self._current_final_yaw_mode)
        ).strip()
        if final_yaw_mode not in FINAL_YAW_MODES:
            self._reject_command(
                "final_yaw_mode must be blend_on_path or rotate_at_goal"
            )
            return
        try:
            segment, after_cursor = self._test_store.next_test_segment()
        except ValueError as error:
            self._reject_command(str(error))
            return
        if not self._follow_waypoints_client.wait_for_server(timeout_sec=0.5):
            self._reject_command("FollowWaypoints action server is unavailable")
            return
        if not self._axis_waypoint_parameters.wait_for_service(timeout_sec=0.5):
            self._reject_command("Axis waypoint parameter service is unavailable")
            return

        goal = FollowWaypoints.Goal()
        goal.poses = [self._pose_stamped(pose) for pose in segment]
        self._active_test_goal = True
        self._pending_test_cursor = after_cursor
        self._publish_state(
            f"Running test segment to {segment[-1].name} with {final_yaw_mode}"
        )
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="final_yaw_mode",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=final_yaw_mode,
                ),
            )
        ]
        future = self._axis_waypoint_parameters.call_async(request)
        future.add_done_callback(
            partial(
                self._on_final_yaw_mode_set,
                goal=goal,
                final_yaw_mode=final_yaw_mode,
            )
        )

    def _on_final_yaw_mode_set(self, future, goal, final_yaw_mode: str) -> None:
        try:
            response = future.result()
        except Exception as error:
            self._finish_test_goal(f"Failed to set final_yaw_mode: {error}", False)
            return
        for result in response.results:
            if not result.successful:
                self._finish_test_goal(
                    f"Failed to set final_yaw_mode: {result.reason}",
                    False,
                )
                return
        self._current_final_yaw_mode = final_yaw_mode
        future = self._follow_waypoints_client.send_goal_async(goal)
        future.add_done_callback(self._on_test_goal_response)

    def _on_test_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self._finish_test_goal(f"Failed to send test segment: {error}", False)
            return
        if not goal_handle.accepted:
            self._finish_test_goal("Test segment was rejected", False)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_test_result)

    def _on_test_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as error:
            self._finish_test_goal(f"Test segment failed: {error}", False)
            return
        missed = list(wrapped.result.missed_waypoints)
        success = wrapped.status == GoalStatus.STATUS_SUCCEEDED and not missed
        if success:
            self._test_store.advance_cursor(int(self._pending_test_cursor))
            self._finish_test_goal("Test segment reached next goal", True)
        else:
            self._finish_test_goal(f"Test segment missed waypoints: {missed}", False)

    def _finish_test_goal(self, message: str, _success: bool) -> None:
        self._active_test_goal = False
        self._pending_test_cursor = None
        self._publish_state(message)

    def _reject_command(self, message: str) -> None:
        self.get_logger().error(message)
        self._publish_state(message)

    def _publish_state(self, message: str = None) -> None:
        if message is not None:
            self._last_message = message
        prompt = String()
        prompt.data = self._last_message
        self._prompt_publisher.publish(prompt)
        state = String()
        state.data = json.dumps(self._state(), ensure_ascii=False)
        self._state_publisher.publish(state)
        self._marker_publisher.publish(self._markers())

    def _state(self) -> dict:
        return {
            "status": self._status(),
            "message": self._last_message,
            "test": {
                "count": len(self._test_store.points),
                "cursor": self._test_store.cursor,
                "next_goal": self._next_test_goal_name(),
                "final_yaw_mode": self._current_final_yaw_mode,
                "output_path": str(self._test_store.output_path),
            },
        }

    def _status(self) -> str:
        if self._sampling:
            return "sampling"
        if self._active_test_goal:
            return "running_test"
        return "idle"

    def _next_test_goal_name(self) -> str:
        for point in self._test_store.points[self._test_store.cursor:]:
            if point.point_type == "goal":
                return point.name
        return ""

    def _markers(self) -> MarkerArray:
        clear = Marker()
        clear.header.frame_id = self._frame_id
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers = [clear]
        marker_id = 1
        for index, point in enumerate(self._test_store.points):
            color = (
                ColorRGBA(r=0.0, g=0.8, b=1.0, a=1.0)
                if point.point_type == "waypoint"
                else ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0)
            )
            marker_id = self._append_pose_markers(
                markers,
                marker_id,
                point.pose,
                f"T{index + 1}: {point.name} [{point.point_type}]",
                color,
            )
        return MarkerArray(markers=markers)

    def _append_pose_markers(
        self,
        markers,
        marker_id: int,
        pose: PoseData,
        label: str,
        color: ColorRGBA,
    ) -> int:
        point_marker = self._base_marker(marker_id, Marker.SPHERE)
        marker_id += 1
        point_marker.pose.position.x = pose.x
        point_marker.pose.position.y = pose.y
        point_marker.pose.position.z = pose.z
        point_marker.scale.x = 0.14
        point_marker.scale.y = 0.14
        point_marker.scale.z = 0.14
        point_marker.color = color
        markers.append(point_marker)

        text_marker = self._base_marker(marker_id, Marker.TEXT_VIEW_FACING)
        marker_id += 1
        text_marker.pose.position.x = pose.x
        text_marker.pose.position.y = pose.y
        text_marker.pose.position.z = pose.z + 0.25
        text_marker.scale.z = 0.16
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text_marker.text = f"{label} yaw={pose.yaw:.2f}"
        markers.append(text_marker)

        arrow_marker = self._base_marker(marker_id, Marker.ARROW)
        marker_id += 1
        arrow_marker.pose.position.x = pose.x
        arrow_marker.pose.position.y = pose.y
        arrow_marker.pose.position.z = pose.z + 0.05
        arrow_marker.pose.orientation.x = pose.qx
        arrow_marker.pose.orientation.y = pose.qy
        arrow_marker.pose.orientation.z = pose.qz
        arrow_marker.pose.orientation.w = pose.qw
        arrow_marker.scale.x = 0.28
        arrow_marker.scale.y = 0.04
        arrow_marker.scale.z = 0.04
        arrow_marker.color = color
        markers.append(arrow_marker)
        return marker_id

    def _base_marker(self, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fruit_waypoint_maker"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _pose_stamped(self, pose: PoseData) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = self._frame_id
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.position.x = pose.x
        stamped.pose.position.y = pose.y
        stamped.pose.position.z = pose.z
        stamped.pose.orientation.x = pose.qx
        stamped.pose.orientation.y = pose.qy
        stamped.pose.orientation.z = pose.qz
        stamped.pose.orientation.w = pose.qw
        return stamped


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
