#!/usr/bin/env python3
# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""ROS 2 node backing the RobViz Waypoints Maker plugin."""

import json
import math
import threading
import time
from functools import partial
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowWaypoints
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
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

from fruit_robviz_bridge.recorder import (
    PoseData,
    TestPointStore,
    VALID_TEST_POINT_TYPES,
    mean_pose,
    yaw_from_quaternion,
)
from fruit_robviz_bridge.waypoint_editor_server import (
    EditorFile,
    RequestError,
    document_version,
    load_waypoint_document,
    load_waypoint_snapshot,
    save_waypoint_document,
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
    """Edit and exercise named points in the configured waypoint file."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__("fruit_waypoint_recorder", *args, **kwargs)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("debug_waypoint_file", "")
        self.declare_parameter("template_file", "")
        self.declare_parameter("backup_dir", "")
        self.declare_parameter("sample_duration_sec", 1.0)
        self.declare_parameter("sample_period_sec", 0.05)
        self.declare_parameter("max_tf_age_sec", 1.5)
        self.declare_parameter("odometry_topic", "/odometry")
        self.declare_parameter("odometry_max_age_sec", 0.5)
        self.declare_parameter("standstill_linear_speed_mps", 0.03)
        self.declare_parameter("standstill_angular_speed_rps", 0.08)
        self.declare_parameter("standstill_confirm_duration_sec", 0.3)
        self.declare_parameter("standstill_timeout_sec", 5.0)
        self.declare_parameter("command_topic", "/fruit/waypoint_maker/command")
        self.declare_parameter("state_topic", "/fruit/waypoint_maker/state")
        self.declare_parameter("marker_topic", "/fruit/waypoint_maker/markers")
        self.declare_parameter("follow_waypoints_action", "/follow_waypoints")
        self.declare_parameter("axis_waypoint_node", "/axis_waypoint_action_server")
        self.declare_parameter("default_final_yaw_mode", "blend_on_path")
        self.declare_parameter("auto_name_prefix", "")
        self.declare_parameter("auto_name_file", "")
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
        self._max_tf_age_sec = float(
            self.get_parameter("max_tf_age_sec").value
        )
        self._odometry_max_age_sec = float(
            self.get_parameter("odometry_max_age_sec").value
        )
        self._standstill_linear_speed_mps = float(
            self.get_parameter("standstill_linear_speed_mps").value
        )
        self._standstill_angular_speed_rps = float(
            self.get_parameter("standstill_angular_speed_rps").value
        )
        self._standstill_confirm_duration_sec = float(
            self.get_parameter("standstill_confirm_duration_sec").value
        )
        self._standstill_timeout_sec = float(
            self.get_parameter("standstill_timeout_sec").value
        )
        positive_values = (
            self._sample_duration_sec,
            self._sample_period_sec,
            self._max_tf_age_sec,
            self._odometry_max_age_sec,
            self._standstill_confirm_duration_sec,
            self._standstill_timeout_sec,
        )
        if not all(math.isfinite(value) and value > 0 for value in positive_values):
            raise ValueError("sampling durations and freshness limits must be finite and positive")
        velocity_limits = (
            self._standstill_linear_speed_mps,
            self._standstill_angular_speed_rps,
        )
        if not all(math.isfinite(value) and value >= 0 for value in velocity_limits):
            raise ValueError("standstill velocity limits must be finite and nonnegative")
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
        debug_waypoint_path = self._resolve_src_yaml_path(
            str(self.get_parameter("debug_waypoint_file").value)
        )
        template_path = self._resolve_src_yaml_path(
            str(self.get_parameter("template_file").value)
        )
        backup_dir_value = str(self.get_parameter("backup_dir").value).strip()
        if not backup_dir_value:
            raise ValueError("backup_dir must be configured")
        backup_dir = Path(backup_dir_value).expanduser().resolve()
        self._file_catalog = {
            "odom": EditorFile(
                file_id="odom",
                profile="odom",
                mode="odometry",
                frame_id=self._frame_id,
                waypoint_file=waypoint_path,
                template_file=template_path,
                reference_file=waypoint_path,
                formal=True,
            ),
            "debug": EditorFile(
                file_id="debug",
                profile="debug",
                mode="freeform",
                frame_id=self._frame_id,
                waypoint_file=debug_waypoint_path,
                template_file=template_path,
                reference_file=debug_waypoint_path,
                formal=False,
            ),
        }
        self._backup_dir = backup_dir
        self._template_types = self._load_template_types(template_path)
        self._template_order = list(self._template_types)
        self._test_store, self._document_version = self._load_store_snapshot(
            self._file_catalog["odom"]
        )
        self._file_id = "odom"
        self._formal = True
        self._loaded_fingerprint = self._points_fingerprint()
        self._configured_auto_name_prefix = str(
            self.get_parameter("auto_name_prefix").value
        ).strip()
        auto_name_file = str(
            self.get_parameter("auto_name_file").value
        ).strip()
        self._auto_name_sequence = self._load_auto_name_sequence(auto_name_file)
        self._lock = threading.RLock()
        self._nav_run_id = 0
        self._sampling = False
        self._active_test_goal = False
        self._active_goal_handle = None
        self._cancel_requested = False
        self._cancel_request_time = None
        self._navigation_unknown = False
        self._waiting_standstill = False
        self._standstill_timer = None
        self._standstill_deadline = 0.0
        self._pending_test_cursor = None
        self._last_message = f"Loaded waypoint file {waypoint_path}"
        self._last_odom_velocity = None
        self._last_odom_wall_time = 0.0
        self._last_odom_stamp_sec = 0.0
        self._standstill_start_wall_time = None
        self._last_backup_path = ""

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
            latched_qos(),
        )
        self._command_subscription = self.create_subscription(
            String,
            self.get_parameter("command_topic").value,
            self._on_command,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self._odometry_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._on_odometry,
            10,
        )
        self._undo_service = self.create_service(Trigger, "~/undo", self._on_undo)
        self._clear_service = self.create_service(Trigger, "~/clear", self._on_clear)
        self._save_service = self.create_service(Trigger, "~/save", self._on_save)
        self._cancel_service = self.create_service(
            Trigger, "~/cancel", self._on_cancel
        )
        self._publish_state()
        self.get_logger().info(f"Editing waypoint file {waypoint_path}")

    def _is_busy(self) -> bool:
        return self._sampling or self._active_test_goal

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
        if not isinstance(command, str):
            self._reject_command("command must be a string")
            return

        if command in ("cancel", "cancel_navigation"):
            with self._lock:
                self._request_cancel()
            return

        if command == "record":
            self._record_from_current_pose(payload)
            return

        with self._lock:
            if self._is_busy():
                self._reject_command(f"Cannot execute {command} while busy")
                return

            if command == "undo":
                self._undo_waypoint()
            elif command == "clear":
                self._test_store.clear()
                self._publish_state("Cleared waypoint file in memory")
            elif command == "save":
                try:
                    self._save_waypoints()
                except (OSError, TypeError, ValueError, RequestError) as error:
                    self._reject_command(str(error))
            elif command in ("run_next_test", "run_next_goal"):
                self._run_next_test_segment(payload)
            elif command in ("reset_test_cursor", "reset_run_cursor"):
                self._test_store.reset_cursor()
                self._publish_state("Reset route run cursor")
            elif command in ("load_test_file", "load_waypoint_file"):
                self._load_test_file(payload)
            elif command in ("reload", "reload_file"):
                self._reload_current_file(payload)
            elif command == "set_final_yaw_mode":
                self._set_final_yaw_mode(payload)
            else:
                self._reject_command(f"Unknown waypoint-maker command: {command}")

    def _on_undo(self, _request, response):
        with self._lock:
            if self._is_busy():
                response.success = False
                response.message = "Cannot undo while busy"
                self._publish_state(response.message)
                return response
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
        with self._lock:
            if self._is_busy():
                response.success = False
                response.message = "Cannot clear while busy"
                self._publish_state(response.message)
                return response
            self._test_store.clear()
            response.success = True
            response.message = "Cleared waypoint file in memory"
            self._publish_state(response.message)
            return response

    def _on_save(self, _request, response):
        with self._lock:
            if self._is_busy():
                response.success = False
                response.message = "Cannot save while busy"
                self._publish_state(response.message)
                return response
            try:
                self._save_waypoints()
            except (OSError, TypeError, ValueError, RequestError) as error:
                response.success = False
                response.message = str(error)
            else:
                response.success = True
                response.message = f"Saved waypoints to {self._test_store.output_path}"
            self._publish_state(response.message)
            return response

    def _prepare_record(self, payload):
        if self._sampling or self._active_test_goal:
            self._reject_command("Cannot record while busy")
            return
        mode = payload.get("mode", "waypoints")
        if not isinstance(mode, str):
            self._reject_command("mode must be a string")
            return
        if mode not in ("test", "waypoints"):
            self._reject_command(f"Unknown record mode: {mode}")
            return
        name_value = payload.get("name", "")
        if not isinstance(name_value, str):
            self._reject_command("name must be a string")
            return
        name = name_value.strip()
        auto_name = payload.get("auto_name", False)
        if not isinstance(auto_name, bool):
            self._reject_command("auto_name must be a boolean")
            return
        confirm_overwrite = payload.get("confirm_overwrite", False)
        if not isinstance(confirm_overwrite, bool):
            self._reject_command("confirm_overwrite must be a boolean")
            return
        if not name and auto_name:
            try:
                name = self._next_auto_name()
            except ValueError as error:
                self._reject_command(str(error))
                return
        point_type_value = payload.get("point_type", "")
        if not isinstance(point_type_value, str):
            self._reject_command("point_type must be a string")
            return
        point_type = point_type_value.strip()
        if self._formal and not point_type and name in self._template_types:
            point_type = self._template_types[name]
        if not self._can_record_test_point(
            name, point_type, confirm_overwrite
        ):
            return

        return name, point_type, confirm_overwrite

    def _record_from_current_pose(self, payload) -> None:
        with self._lock:
            prepared = self._prepare_record(payload)
            if prepared is None:
                return
            name, point_type, confirm_overwrite = prepared
            self._sampling = True
            self._publish_state("Sampling current pose")
        message = "Waypoint recording failed"
        try:
            pose = self._sample_current_pose(name)
            with self._lock:
                point = self._test_store.upsert(
                    name,
                    point_type,
                    pose,
                    overwrite=confirm_overwrite,
                    order=self._template_order if self._formal else None,
                )
            message = f"Recorded {point.point_type} {point.name}"
            self.get_logger().info(message)
        except (RuntimeError, ValueError) as error:
            message = str(error)
            self.get_logger().error(message)
        finally:
            with self._lock:
                self._sampling = False
                self._publish_state(message)

    def _can_record_test_point(
        self, name: str, point_type: str, confirm_overwrite: bool
    ) -> bool:
        if not name:
            self._reject_command("waypoint name must not be empty")
            return False
        if point_type not in VALID_TEST_POINT_TYPES:
            self._reject_command("point_type must be waypoint or goal")
            return False
        if self._formal and name not in self._template_types:
            self._reject_command(f"waypoint name is not in template: {name}")
            return False
        if self._formal and self._template_types[name] != point_type:
            self._reject_command(f"point_type does not match template: {name}")
            return False
        if any(point.name == name for point in self._test_store.points):
            if not confirm_overwrite:
                self._reject_command(
                    f"waypoint already exists: {name}; "
                    "confirm_overwrite=true is required"
                )
                return False
        return True

    def _sample_current_pose(self, name: str) -> PoseData:
        samples = []
        deadline = time.monotonic() + self._sample_duration_sec
        last_error = None
        previous = None
        started = False
        interrupted = False
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._frame_id,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                stamp = transform.header.stamp
                stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                if stamp_sec <= 0 or (now_sec > 0 and now_sec - stamp_sec > self._max_tf_age_sec):
                    raise RuntimeError(
                        "TF sample is stale or has no timestamp"
                    )
                if not self._is_standstill():
                    raise RuntimeError("robot is moving; waypoint recording rejected")
                current = self._pose_from_transform(name, transform)
                if previous is not None:
                    moved = math.hypot(
                        current.x - previous.x,
                        current.y - previous.y,
                    )
                    yaw_delta = abs(
                        (current.yaw - previous.yaw + math.pi) % (2 * math.pi)
                        - math.pi
                    )
                    if moved > 0.03 or yaw_delta > math.radians(5.0):
                        raise RuntimeError(
                            "robot moved during waypoint recording"
                        )
                samples.append(current)
                previous = current
                started = True
            except TransformException as error:
                last_error = error
                if started:
                    interrupted = True
                    break
            except RuntimeError as error:
                last_error = error
                interrupted = True
                break
            time.sleep(self._sample_period_sec)
        if not samples:
            detail = f": {last_error}" if last_error else ""
            raise RuntimeError(
                f"No valid TF samples for {self._frame_id}->{self._base_frame}{detail}"
            )
        if interrupted or len(samples) < 2:
            detail = f": {last_error}" if last_error else ""
            raise RuntimeError(f"Continuous pose sampling was not completed{detail}")
        return mean_pose(name, samples)

    @staticmethod
    def _pose_from_transform(name: str, transform) -> PoseData:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise RuntimeError("TF sample contains a non-finite value")
        norm = math.hypot(
            math.hypot(rotation.x, rotation.y),
            math.hypot(rotation.z, rotation.w),
        )
        if norm <= 1e-6:
            raise RuntimeError("TF sample contains a zero-length quaternion")
        qx = rotation.x / norm
        qy = rotation.y / norm
        qz = rotation.z / norm
        qw = rotation.w / norm
        return PoseData(
            name=name,
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
            qx=float(qx),
            qy=float(qy),
            qz=float(qz),
            qw=float(qw),
            yaw=yaw_from_quaternion(
                qx,
                qy,
                qz,
                qw,
            ),
        )

    def _on_odometry(self, message: Odometry) -> None:
        now = time.monotonic()
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        values = (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
        with self._lock:
            stamp_sec = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp_sec
            if (
                message.header.frame_id != self._frame_id
                or message.child_frame_id != self._base_frame
                or stamp_sec <= 0 or age < 0 or age > self._odometry_max_age_sec
                or stamp_sec <= self._last_odom_stamp_sec
                or not all(math.isfinite(float(value)) for value in values)
            ):
                self._last_odom_velocity = None
                self._last_odom_wall_time = now
                self._standstill_start_wall_time = None
                return
            sample_time = now - age
            velocity = (
                math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z),
                math.sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z),
            )
            if (
                self._last_odom_wall_time > 0
                and (sample_time - self._last_odom_wall_time) > self._odometry_max_age_sec
            ):
                self._standstill_start_wall_time = None
            self._last_odom_velocity = velocity
            self._last_odom_wall_time = sample_time
            self._last_odom_stamp_sec = stamp_sec
            if (
                velocity[0] <= self._standstill_linear_speed_mps
                and velocity[1] <= self._standstill_angular_speed_rps
            ):
                if self._standstill_start_wall_time is None:
                    self._standstill_start_wall_time = sample_time
            else:
                self._standstill_start_wall_time = None

    def _is_standstill(self, since_time: float | None = None) -> bool:
        with self._lock:
            now = time.monotonic()
            if self._last_odom_velocity is None:
                return False
            if (now - self._last_odom_wall_time) > self._odometry_max_age_sec:
                return False
            linear, angular = self._last_odom_velocity
            if (
                linear > self._standstill_linear_speed_mps
                or angular > self._standstill_angular_speed_rps
            ):
                return False
            if self._standstill_start_wall_time is None:
                return False
            effective_start = self._standstill_start_wall_time
            if since_time is not None:
                if now < since_time:
                    return False
                effective_start = max(effective_start, since_time)
            return (
                self._last_odom_wall_time - effective_start
                >= self._standstill_confirm_duration_sec
            )

    def _undo_waypoint(self) -> None:
        if self._sampling or self._active_test_goal:
            self._reject_command("Cannot undo while busy")
            return
        removed = self._test_store.undo()
        message = (
            f"Removed waypoint {removed.name}"
            if removed is not None
            else "No waypoint to undo"
        )
        self._publish_state(message)

    def _set_final_yaw_mode(self, payload) -> None:
        if self._active_test_goal:
            self._reject_command("Cannot change final_yaw_mode while navigating")
            return
        value = payload.get("final_yaw_mode")
        if not isinstance(value, str) or value.strip() not in FINAL_YAW_MODES:
            self._reject_command(
                "final_yaw_mode must be blend_on_path or rotate_at_goal"
            )
            return
        self._current_final_yaw_mode = value.strip()
        self._publish_state(f"Set final_yaw_mode to {self._current_final_yaw_mode}")

    def _auto_name_prefix(self) -> str:
        """Return the explicit profile prefix or infer A/B from the file path."""
        if self._configured_auto_name_prefix:
            return self._configured_auto_name_prefix
        parent_name = self._test_store.output_path.parent.name.strip()
        if parent_name in ("A", "B"):
            return parent_name
        return ""

    def _load_auto_name_sequence(self, raw_path: str) -> list[str]:
        """Load the ordered formal names used for automatic recording."""
        if not raw_path:
            return []
        path = self._resolve_src_yaml_path(raw_path)
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        raw_points = document.get("points")
        if not isinstance(raw_points, list):
            raise ValueError(f"auto_name_file points must be a list: {path}")
        names = []
        seen = set()
        for index, item in enumerate(raw_points, start=1):
            if not isinstance(item, dict):
                raise ValueError(f"auto_name_file point {index} must be a mapping")
            name = str(item.get("name", "")).strip()
            if not name:
                raise ValueError(f"auto_name_file point {index} name must not be empty")
            if name in seen:
                raise ValueError(f"auto_name_file contains duplicate name: {name}")
            names.append(name)
            seen.add(name)
        return names

    @staticmethod
    def _load_template_types(path: Path) -> dict[str, str]:
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        if not isinstance(document, dict) or document.get("schema_version") != 1:
            raise ValueError(f"template YAML is invalid: {path}")
        raw_points = document.get("points")
        if not isinstance(raw_points, list):
            raise ValueError(f"template points must be a list: {path}")
        result = {}
        for index, item in enumerate(raw_points, start=1):
            if not isinstance(item, dict):
                raise ValueError(f"template point {index} must be a mapping")
            name = item.get("name")
            point_type = item.get("type")
            if not isinstance(name, str) or not name.strip():
                raise ValueError(f"template point {index} name must not be empty")
            if point_type not in VALID_TEST_POINT_TYPES:
                raise ValueError(f"template point {name} has an invalid type")
            if name.strip() in result:
                raise ValueError(f"template contains duplicate name: {name}")
            result[name.strip()] = point_type
        return result

    def _points_fingerprint(self) -> str:
        return json.dumps(
            [
                {
                    "name": point.name,
                    "type": point.point_type,
                    "pose": point.pose.to_dict(),
                }
                for point in self._test_store.points
            ],
            ensure_ascii=False,
            sort_keys=True,
        )

    def _load_store_snapshot(self, editor_file):
        document, _, version = load_waypoint_snapshot(editor_file)
        candidate = TestPointStore(editor_file.waypoint_file, self._frame_id)
        candidate.load_document(document, editor_file.waypoint_file)
        if editor_file.formal:
            for point in candidate.points:
                if self._template_types.get(point.name) != point.point_type:
                    raise ValueError(f"waypoint name/type does not match template: {point.name}")
        return candidate, version

    def _confirm_discard(self, payload):
        confirm = payload.get("confirm_discard", False)
        force = payload.get("force", False)
        if not isinstance(confirm, bool) or not isinstance(force, bool):
            raise ValueError("confirm_discard and force must be booleans")
        if self._points_fingerprint() != self._loaded_fingerprint and not (confirm or force):
            raise ValueError("Unsaved in-memory changes exist; confirm_discard=true required")

    def _next_auto_name(self) -> str:
        """Return the first formal name that is not already recorded."""
        recorded_names = {point.name for point in self._test_store.points}
        for name in self._auto_name_sequence:
            if name not in recorded_names:
                return name
        if self._auto_name_sequence:
            raise ValueError("all names from auto_name_file have been recorded")
        return self._test_store.next_auto_name(self._auto_name_prefix())

    def _next_auto_name_for_state(self) -> str:
        """Return the next display name, or an empty value after the template ends."""
        if not self._auto_name_sequence and not self._auto_name_prefix():
            return ""
        try:
            return self._next_auto_name()
        except ValueError:
            return ""

    def _save_waypoints(self) -> None:
        editor_file = self._file_catalog[self._file_id]
        _, disk_points = load_waypoint_document(editor_file)
        extras_by_name = {
            point["name"]: point.get("extra", {}) for point in disk_points
        }
        raw_points = []
        for point in self._test_store.points:
            raw_points.append(
                {
                    "name": point.name,
                    "type": point.point_type,
                    "x": point.pose.x,
                    "y": point.pose.y,
                    "z": point.pose.z,
                    "yaw_deg": math.degrees(point.pose.yaw),
                    "extra": extras_by_name.get(point.name, {}),
                }
            )
        backup_path, version = save_waypoint_document(
            editor_file,
            self._backup_dir,
            raw_points,
            self._document_version,
        )
        self._document_version = version
        self._loaded_fingerprint = self._points_fingerprint()
        self._last_backup_path = str(backup_path) if backup_path else ""
        self._publish_state(f"Saved waypoints to {self._test_store.output_path}")

    def _load_test_file(self, payload) -> None:
        if self._sampling or self._active_test_goal:
            self._reject_command("Cannot load waypoints while busy")
            return
        try:
            self._confirm_discard(payload)
            file_id = payload.get("file_id", payload.get("id"))
            if not isinstance(file_id, str) or file_id not in self._file_catalog:
                raise ValueError("file_id must be odom or debug")
            editor_file = self._file_catalog[file_id]
            candidate, version = self._load_store_snapshot(editor_file)
            self._test_store = candidate
            self._file_id = file_id
            self._formal = editor_file.formal
            self._document_version = version
            self._loaded_fingerprint = self._points_fingerprint()
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            self._reject_command(str(error))
            return
        self._publish_state(f"Loaded waypoints from {self._test_store.output_path}")

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

    def _reload_current_file(self, payload: dict) -> None:
        if self._is_busy():
            self._reject_command("Cannot reload waypoints while busy")
            return
        editor_file = self._file_catalog[self._file_id]
        try:
            self._confirm_discard(payload)
            candidate, version = self._load_store_snapshot(editor_file)
            self._test_store = candidate
            self._document_version = version
            self._loaded_fingerprint = self._points_fingerprint()
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            self._reject_command(f"Failed to reload waypoints: {error}")
            return
        self._publish_state(f"Reloaded waypoints from {self._test_store.output_path}")

    def _run_next_test_segment(self, payload) -> None:
        if self._is_busy():
            self._reject_command("A route segment or sampling is already running")
            return
        editor_file = self._file_catalog[self._file_id]
        disk_ver = document_version(editor_file.waypoint_file)
        if disk_ver != self._document_version:
            self._reject_command(
                f"Waypoint file was modified on disk (version {disk_ver[:8]} != "
                f"loaded {self._document_version[:8]}); please reload waypoints first"
            )
            return

        final_yaw_value = payload.get(
            "final_yaw_mode", self._current_final_yaw_mode
        )
        if not isinstance(final_yaw_value, str):
            self._reject_command("final_yaw_mode must be a string")
            return
        final_yaw_mode = final_yaw_value.strip()
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
        self._nav_run_id += 1
        run_id = self._nav_run_id
        self._active_test_goal = True
        self._active_goal_handle = None
        self._cancel_requested = False
        self._cancel_request_time = None
        self._navigation_unknown = False
        self._waiting_standstill = False
        self._pending_test_cursor = after_cursor
        self._publish_state(
            f"Running route segment to {segment[-1].name} with {final_yaw_mode}"
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
                run_id=run_id,
            )
        )

    def _on_final_yaw_mode_set(self, future, goal, final_yaw_mode: str, run_id: int) -> None:
        with self._lock:
            if run_id != self._nav_run_id or not self._active_test_goal:
                return
            if self._cancel_requested:
                self._finish_test_goal("Route segment cancelled before dispatch", False)
                return
            try:
                response = future.result()
                if response is None or len(response.results) != 1:
                    raise ValueError("missing final_yaw_mode parameter result")
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
            try:
                send_goal_future = self._follow_waypoints_client.send_goal_async(goal)
                send_goal_future.add_done_callback(
                    partial(self._on_test_goal_response, run_id=run_id)
                )
            except Exception as error:
                self._navigation_unknown = True
                self._publish_state(f"Goal dispatch unknown; next segment blocked: {error}")

    def _on_test_goal_response(self, future, run_id: int) -> None:
        with self._lock:
            if run_id != self._nav_run_id or not self._active_test_goal:
                try:
                    goal_handle = future.result()
                    if goal_handle and goal_handle.accepted:
                        goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            try:
                goal_handle = future.result()
            except Exception as error:
                self._navigation_unknown = True
                self._publish_state(f"Goal acceptance unknown; next segment blocked: {error}")
                return
            if not goal_handle.accepted:
                self._finish_test_goal("Route segment was rejected", False)
                return
            self._active_goal_handle = goal_handle
            try:
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(
                    partial(self._on_test_result, run_id=run_id)
                )
            except Exception as error:
                self._navigation_unknown = True
                self._publish_state(f"Route result unavailable; next segment blocked: {error}")
            if self._cancel_requested:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    partial(self._on_cancel_response, run_id=run_id)
                )

    def _on_test_result(self, future, run_id: int) -> None:
        with self._lock:
            if run_id != self._nav_run_id or not self._active_test_goal:
                return
            try:
                wrapped = future.result()
                if wrapped.status not in (
                    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                    GoalStatus.STATUS_CANCELED,
                ):
                    raise ValueError(f"non-terminal action status {wrapped.status}")
                missed = list(wrapped.result.missed_waypoints)
                status = wrapped.status
            except Exception as error:
                self._navigation_unknown = True
                self._publish_state(f"Route result unavailable; next segment blocked: {error}")
                return
            if self._cancel_requested or status == GoalStatus.STATUS_CANCELED:
                self._start_standstill_wait(
                    "Route segment stopped by server; confirming standstill"
                )
                return
            if status == GoalStatus.STATUS_SUCCEEDED and not missed:
                self._test_store.advance_cursor(int(self._pending_test_cursor))
                self._finish_test_goal("Route segment reached next goal", True)
            else:
                self._finish_test_goal(
                    f"Route segment finished with status {status}, missed: {missed}",
                    False,
                )

    def _finish_test_goal(self, message: str, _success: bool) -> None:
        if self._standstill_timer is not None:
            self._standstill_timer.cancel()
            self._standstill_timer = None
        self._active_test_goal = False
        self._active_goal_handle = None
        self._cancel_requested = False
        self._cancel_request_time = None
        self._navigation_unknown = False
        self._waiting_standstill = False
        self._pending_test_cursor = None
        self._publish_state(message)

    def _on_cancel(self, _request, response):
        with self._lock:
            if not self._active_test_goal:
                response.success = False
                response.message = "No route segment is running"
                self._publish_state(response.message)
                return response
            self._request_cancel()
            response.success = True
            response.message = "Route cancellation requested"
            return response

    def _request_cancel(self) -> None:
        if not self._active_test_goal:
            self._reject_command("No route segment is running")
            return
        if self._cancel_requested and not self._navigation_unknown:
            self._publish_state("Route cancellation already requested")
            return
        self._cancel_requested = True
        self._cancel_request_time = time.monotonic()
        self._publish_state("Cancelling route segment")
        goal_handle = self._active_goal_handle
        if goal_handle is None:
            self.get_logger().info(
                "Cancel requested before goal handle received; cancellation armed"
            )
            return
        if self._navigation_unknown:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                partial(self._on_test_result, run_id=self._nav_run_id)
            )
        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            partial(self._on_cancel_response, run_id=self._nav_run_id)
        )

    def _on_cancel_response(self, future, run_id: int) -> None:
        with self._lock:
            if run_id != self._nav_run_id or not self._active_test_goal:
                return
            try:
                response = future.result()
                canceling = getattr(response, "goals_canceling", [])
                if not canceling:
                    self.get_logger().warn("Action server reported no goals canceling")
            except Exception as error:
                self.get_logger().error(f"Route cancellation request failed: {error}")
            self._publish_state("Route cancellation sent; awaiting goal completion and standstill")

    def _start_standstill_wait(self, message: str) -> None:
        if not self._active_test_goal or self._waiting_standstill:
            return
        self._cancel_request_time = time.monotonic()
        if self._is_standstill(since_time=self._cancel_request_time):
            self._finish_test_goal("Route cancelled and standstill confirmed", False)
            return
        self._waiting_standstill = True
        self._standstill_deadline = time.monotonic() + self._standstill_timeout_sec
        self._publish_state(message)
        if self._standstill_timer is not None:
            self._standstill_timer.cancel()
        self._standstill_timer = self.create_timer(
            0.1, self._poll_standstill
        )

    def _poll_standstill(self) -> None:
        with self._lock:
            if not self._waiting_standstill:
                return
            if self._is_standstill(since_time=self._cancel_request_time):
                if self._standstill_timer is not None:
                    self._standstill_timer.cancel()
                    self._standstill_timer = None
                self._finish_test_goal("Route cancelled and standstill confirmed", False)
                return
            if time.monotonic() >= self._standstill_deadline:
                self._publish_state(
                    "Route cancelled, but standstill is not confirmed; "
                    "next segment remains blocked"
                )

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
        points = [point.to_dict() for point in self._test_store.points]
        recorded = {point["name"]: point for point in points}
        template = [
            {
                "name": name,
                "type": point_type,
                "recorded": name in recorded,
                "point": recorded.get(name),
            }
            for name, point_type in self._template_types.items()
        ]
        dirty = self._points_fingerprint() != self._loaded_fingerprint
        editor_file = self._file_catalog.get(self._file_id)
        disk_ver = (
            document_version(editor_file.waypoint_file)
            if editor_file and editor_file.waypoint_file.is_file()
            else ""
        )
        disk_synced = (disk_ver == self._document_version)
        waypoint_state = {
            "count": len(self._test_store.points),
            "cursor": self._test_store.cursor,
            "next_goal": self._next_test_goal_name(),
            "final_yaw_mode": self._current_final_yaw_mode,
            "output_path": str(self._test_store.output_path),
            "auto_name_prefix": self._auto_name_prefix(),
            "next_name": self._next_auto_name_for_state(),
            "file_id": self._file_id,
            "formal": self._formal,
            "version": self._document_version,
            "disk_version": disk_ver,
            "disk_synced": disk_synced,
            "dirty": dirty,
            "points": points,
            "template": template if self._formal else [],
            "last_backup_path": self._last_backup_path,
        }
        return {
            "status": self._status(),
            "message": self._last_message,
            "waypoints": waypoint_state,
            "test": waypoint_state,
            "file_id": self._file_id,
            "formal": self._formal,
            "version": self._document_version,
            "disk_version": disk_ver,
            "disk_synced": disk_synced,
            "dirty": dirty,
            "points": points,
            "last_backup_path": self._last_backup_path,
            "navigation": {
                "active": self._active_test_goal,
                "goal_accepted": self._active_goal_handle is not None,
                "cancel_requested": self._cancel_requested,
                "awaiting_standstill": self._waiting_standstill,
                "unknown": self._navigation_unknown,
                "cursor": self._test_store.cursor,
                "next_goal": self._next_test_goal_name(),
            },
        }

    def _status(self) -> str:
        if self._sampling:
            return "sampling"
        if self._waiting_standstill or self._cancel_requested:
            return "cancelling"
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
                f"W{index + 1}: {point.name} [{point.point_type}]",
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
