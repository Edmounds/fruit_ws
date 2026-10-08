#!/usr/bin/env python3

"""Reload RobViz image settings through the local HTTP API on demand."""

from __future__ import annotations

import json
import math
import time
import threading
from collections import deque
from copy import deepcopy
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import rclpy
import yaml
from fruit_arm_interfaces.msg import ArmTelemetry
from fruit_hardware_interfaces.msg import ArmStatus
from fruit_mission.camera_target_transform import (
    load_camera_extrinsic,
    transform_camera_to_end_effector_mm,
    transform_end_to_base_mm,
)
from fruit_perception.msg import FruitDetections
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger


MAX_RESPONSE_BYTES = 64 * 1024
STREAM_FIELDS = (
    "bandwidth_mbps",
    "auto_quality",
    "quality",
    "width",
    "height",
    "max_fps",
)


class RobvizConfigClient(Node):
    """Bridge the explicit reload service to the RobViz-owned JSON settings."""

    def __init__(self) -> None:
        super().__init__("fruit_robviz_config_client")
        self.declare_parameter(
            "config_url", "http://127.0.0.1:8000/api/image-settings"
        )
        self.declare_parameter("image_node", "/fruit_robviz_image_bridge")
        self.declare_parameter("request_timeout_sec", 5.0)
        timeout = float(self.get_parameter("request_timeout_sec").value)
        if not 0.1 <= timeout <= 10.0:
            raise ValueError("request_timeout_sec must be in [0.1, 10]")
        self._config_url = str(self.get_parameter("config_url").value)
        self._timeout = timeout
        self._callback_group = ReentrantCallbackGroup()
        self._reload_lock = threading.Lock()
        self._parameter_client = self.create_client(
            SetParametersAtomically,
            f"{str(self.get_parameter('image_node').value).rstrip('/')}/"
            "set_parameters_atomically",
            callback_group=self._callback_group,
        )
        self._reload_service = self.create_service(
            Trigger,
            "/fruit_robviz_bridge/reload_image_settings",
            self._on_reload,
            callback_group=self._callback_group,
        )
        self._init_base_preview()

    def _read_config(self, parameter: str, default: str, node: str) -> dict:
        path = self.declare_parameter(parameter, default).value
        try:
            with Path(path).open(encoding="utf-8") as stream:
                return yaml.safe_load(stream)[node]["ros__parameters"]
        except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
            raise RuntimeError(f"Invalid preview configuration {path}: {error}") from error

    def _init_base_preview(self) -> None:
        source = "/home/rc4/fruit_ws/src"
        extrinsic_path = self.declare_parameter(
            "camera_extrinsic_config", f"{source}/fruit_mission/config/camera_extrinsic.yaml"
        ).value
        self._rotation, self._translation = load_camera_extrinsic(extrinsic_path)
        planner = self._read_config(
            "arm_presets_config", f"{source}/fruit_arm_planner/config/arm_presets.yaml",
            "arm_planner_node",
        )
        mission = self._read_config(
            "mission_config", f"{source}/fruit_mission/config/fruit_mission.yaml", "fruit_mission"
        )
        navigation = self._read_config(
            "navigation_config", f"{source}/navigation/fruit_nav_bringup/config/"
            "reality/pointlio_params.yaml", "loam_interface"
        )
        recorder = self._read_config(
            "recorder_config", f"{source}/fruit_waypoint_recorder/config/waypoint_recorder.yaml",
            "fruit_waypoint_recorder",
        )
        self._limits = [
            tuple(math.radians(value) for value in planner["joint_mapping"][name]["limits_deg"])
            for name in ("q1", "q2", "q3", "q4")
        ]
        self._max_age = float(planner["planning"]["status_max_age_sec"])
        self._window = float(recorder["standstill_confirm_duration_sec"])
        self._arm_speed = float(mission["stationary"]["joint_speed_rps"])
        self._linear_speed = float(navigation["initial_stability_max_translation_speed"])
        self._angular_speed = float(navigation["initial_stability_max_rotation_speed"])
        values = (self._max_age, self._window, self._arm_speed,
                  self._linear_speed, self._angular_speed)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise RuntimeError("Invalid preview freshness or standstill configuration")
        if not all(len(pair) == 2 and all(math.isfinite(v) for v in pair)
                   and pair[0] < pair[1] for pair in self._limits):
            raise RuntimeError("Invalid preview hardware joint limits")
        self._camera_frame = self.declare_parameter(
            "camera_frame_id", "camera_color_optical_frame"
        ).value
        self._arm_frame = self.declare_parameter("arm_frame_id", "arm_base_link").value
        self._odom_frame = self.declare_parameter("odom_frame", "odom").value
        self._base_frame = self.declare_parameter("base_frame_id", "base_link").value
        self._feedback = {name: deque(maxlen=256) for name in ("arm", "status", "odom")}
        self._base_publisher = self.create_publisher(
            FruitDetections, "/fruit_perception/camera/base_preview_detections",
            qos_profile_sensor_data,
        )
        self._preview_subscriptions = [
            self.create_subscription(
                ArmTelemetry, "/arm_planner_node/joint_states",
                self._on_arm, 1,
            ),
            self.create_subscription(
                ArmStatus, "/fruit_hardware_bridge/arm/status",
                self._on_status, 1,
            ),
            self.create_subscription(
                Odometry, "/odometry", self._on_odom, qos_profile_sensor_data,
            ),
            self.create_subscription(
                FruitDetections, "/fruit_perception/camera/preview_detections",
                self._on_detections, qos_profile_sensor_data,
            ),
        ]

    @staticmethod
    def _stamp(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _remember(self, name: str, stamp: float, valid: bool, message) -> None:
        history = self._feedback[name]
        wall = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        valid = valid and 0 <= ros_now - stamp <= self._max_age and stamp > 0
        start = stamp if valid else None
        if history:
            previous_stamp, previous_wall, previous_start, _ = history[-1]
            if not 0 < stamp - previous_stamp <= self._max_age:
                valid = False
                start = None
            elif (valid and previous_start is not None
                  and wall - previous_wall <= self._max_age):
                start = previous_start
        history.append((stamp, wall, start, message))

    def _valid_status(self, status) -> bool:
        return (status.task_status in (ArmStatus.STATUS_IDLE, ArmStatus.STATUS_COMPLETED)
                and status.error_code == ArmStatus.ERROR_NONE)

    def _on_arm(self, message: ArmTelemetry) -> None:
        valid = self._valid_status(message) and message.header.frame_id == self._arm_frame
        names = [f"joint_{index}" for index in range(1, 5)]
        for joints in (message.hardware_joint_state, message.model_joint_state):
            valid = valid and list(joints.name) == names
            valid = valid and len(joints.position) == 4 and len(joints.velocity) == 4
            valid = valid and all(math.isfinite(v) for v in joints.position)
            valid = valid and all(math.isfinite(v) and abs(v) <= self._arm_speed
                                  for v in joints.velocity)
        valid = valid and all(low <= value <= high for value, (low, high) in zip(
            message.hardware_joint_state.position, self._limits))
        tcp = message.tcp_position
        valid = valid and all(math.isfinite(v) for v in (
            tcp.x, tcp.y, tcp.z, message.tcp_pitch_deg))
        self._remember("arm", self._stamp(message.header.stamp), valid, message)

    def _on_status(self, message: ArmStatus) -> None:
        positions = (message.joint_1, message.joint_2, message.joint_3, message.joint_4)
        velocities = (message.joint_1_velocity, message.joint_2_velocity,
                      message.joint_3_velocity, message.joint_4_velocity)
        valid = self._valid_status(message)
        valid = valid and all(math.isfinite(v) and low <= v <= high
                              for v, (low, high) in zip(positions, self._limits))
        valid = valid and all(math.isfinite(v) and abs(v) <= self._arm_speed
                              for v in velocities)
        self._remember("status", self.get_clock().now().nanoseconds * 1e-9, valid, message)

    def _on_odom(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        speeds = (math.hypot(linear.x, linear.y, linear.z),
                  math.hypot(angular.x, angular.y, angular.z))
        valid = (message.header.frame_id == self._odom_frame
                 and message.child_frame_id == self._base_frame
                 and all(math.isfinite(v) for v in speeds)
                 and speeds[0] <= self._linear_speed and speeds[1] <= self._angular_speed)
        self._remember("odom", self._stamp(message.header.stamp), valid, message)

    def _sample_for_image(self, name: str, image_stamp: float):
        history = self._feedback[name]
        if not history or history[-1][2] is None:
            return None
        if time.monotonic() - history[-1][1] > self._max_age:
            return None
        for stamp, _, start, message in reversed(history):
            if stamp > image_stamp:
                continue
            if (start is not None and image_stamp - stamp <= self._max_age
                    and stamp - start >= self._window):
                return message
            return None
        return None

    def _on_detections(self, message: FruitDetections) -> None:
        if self._base_publisher.get_subscription_count() == 0:
            return
        output = deepcopy(message)
        output.header.frame_id = self._arm_frame
        stamp = self._stamp(message.header.stamp)
        arm = self._sample_for_image("arm", stamp)
        status = self._sample_for_image("status", stamp)
        odom = self._sample_for_image("odom", stamp)
        topics = (
            "/arm_planner_node/joint_states", "/fruit_hardware_bridge/arm/status", "/odometry"
        )
        valid = (message.header.frame_id == self._camera_frame
                 and 0 <= self.get_clock().now().nanoseconds * 1e-9 - stamp <= 2.0
                 and arm is not None and status is not None and odom is not None
                 and all(self.count_publishers(topic) == 1 for topic in topics))
        if valid:
            valid = (arm.trajectory_id == status.trajectory_id
                     and arm.task_status == status.task_status)
        for detection in output.detections:
            camera = (detection.point.x, detection.point.y, detection.point.z)
            detection.has_point = (valid and detection.has_point and camera[2] > 0
                                   and all(math.isfinite(v) for v in camera))
            if not detection.has_point:
                continue
            try:
                end = transform_camera_to_end_effector_mm(
                    tuple(v * 1000 for v in camera), self._rotation, self._translation
                )
                tcp = arm.tcp_position
                base = transform_end_to_base_mm(
                    end, (tcp.x * 1000, tcp.y * 1000, tcp.z * 1000),
                    arm.model_joint_state.position[0], math.radians(arm.tcp_pitch_deg),
                )
                detection.point.x, detection.point.y, detection.point.z = (
                    v * 0.001 for v in base
                )
            except ValueError:
                detection.has_point = False
        self._base_publisher.publish(output)

    def _fetch_settings(self) -> dict:
        request = Request(
            self._config_url,
            headers={"Accept": "application/json"},
            method="GET",
        )
        try:
            with urlopen(request, timeout=self._timeout) as response:
                if response.headers.get_content_type() != "application/json":
                    raise RuntimeError("RobViz settings API returned non-JSON data")
                content_length = response.headers.get("Content-Length")
                if content_length is not None and int(content_length) > MAX_RESPONSE_BYTES:
                    raise RuntimeError("RobViz settings response is too large")
                raw = response.read(MAX_RESPONSE_BYTES + 1)
        except (HTTPError, URLError, OSError) as error:
            raise RuntimeError(f"RobViz settings request failed: {error}") from error
        if len(raw) > MAX_RESPONSE_BYTES:
            raise RuntimeError("RobViz settings response is too large")
        try:
            payload = json.loads(raw.decode("utf-8"), parse_constant=self._reject_constant)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            raise RuntimeError(f"RobViz settings JSON is invalid: {error}") from error
        if not isinstance(payload, dict) or not isinstance(payload.get("settings"), dict):
            raise RuntimeError("RobViz settings response has an invalid shape")
        return payload

    @staticmethod
    def _reject_constant(value: str):
        raise ValueError(f"JSON constant {value} is not allowed")

    @staticmethod
    def _parameter_value(value) -> ParameterValue:
        parameter = ParameterValue()
        if isinstance(value, bool):
            parameter.type = ParameterType.PARAMETER_BOOL
            parameter.bool_value = value
        elif isinstance(value, int):
            parameter.type = ParameterType.PARAMETER_INTEGER
            parameter.integer_value = value
        elif isinstance(value, float):
            parameter.type = ParameterType.PARAMETER_DOUBLE
            parameter.double_value = value
        elif isinstance(value, str):
            parameter.type = ParameterType.PARAMETER_STRING
            parameter.string_value = value
        else:
            raise ValueError("image setting values must be numeric or boolean")
        return parameter

    def _to_request(
        self, settings: dict, version: int, session: str
    ) -> SetParametersAtomically.Request:
        parameters = [
            Parameter(
                name="settings_version",
                value=self._parameter_value(version),
            ),
            Parameter(name="settings_session", value=self._parameter_value(session)),
        ]
        for stream_name in ("rgb", "depth"):
            stream = settings.get(stream_name)
            if not isinstance(stream, dict):
                raise ValueError(f"missing {stream_name} settings")
            for field in STREAM_FIELDS:
                if field not in stream:
                    raise ValueError(f"missing {stream_name}.{field}")
                parameters.append(
                    Parameter(
                        name=f"{stream_name}.{field}",
                        value=self._parameter_value(stream[field]),
                    )
                )
        request = SetParametersAtomically.Request()
        request.parameters = parameters
        return request

    def _on_reload(self, _request: Trigger.Request, response: Trigger.Response):
        if not self._reload_lock.acquire(blocking=False):
            response.success = False
            response.message = "image settings reload is already in progress"
            return response
        parameter_future = None
        try:
            payload = self._fetch_settings()
            version = payload.get("version", payload["settings"].get("version"))
            if not isinstance(version, int) or isinstance(version, bool) or version < 1:
                raise ValueError("settings version is invalid")
            session = payload["settings"].get("session")
            if not isinstance(session, str) or len(session) != 32:
                raise ValueError("settings session is invalid")
            if not self._parameter_client.wait_for_service(timeout_sec=self._timeout):
                raise RuntimeError("image bridge parameter service is unavailable")
            future = self._parameter_client.call_async(
                self._to_request(payload["settings"], version, session)
            )
            parameter_future = future
            done = threading.Event()
            future.add_done_callback(lambda _future: done.set())
            if not done.wait(self._timeout):
                raise RuntimeError("image bridge parameter update timed out")
            result = future.result()
            if result is None or not result.result.successful:
                reason = result.result.reason if result is not None else "no response"
                raise RuntimeError(f"image bridge rejected settings: {reason}")
            response.success = True
            response.message = json.dumps({"version": version, "session": session})
        except Exception as error:
            self.get_logger().error(f"RobViz image settings reload failed: {error}")
            response.success = False
            response.message = str(error)
        finally:
            if parameter_future is not None and not parameter_future.done():
                parameter_future.add_done_callback(lambda _future: self._reload_lock.release())
            else:
                self._reload_lock.release()
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobvizConfigClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
