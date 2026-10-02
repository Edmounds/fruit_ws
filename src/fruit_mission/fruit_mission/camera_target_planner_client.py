"""Mission-side camera -> gripper -> arm-base conversion and planner client."""

import math
from pathlib import Path
import time

from fruit_arm_interfaces.srv import ExecuteArmPreset, MoveArmCartesian, MoveArmJoints
from fruit_hardware_interfaces.msg import ArmStatus
from fruit_arm_planner.joint_transformer import JointTransformer
from fruit_arm_planner.kinematics import RobotKinematics
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import yaml

from fruit_mission.camera_target_transform import (
    load_camera_extrinsic,
    transform_camera_to_end_effector_mm,
    transform_end_to_base_mm,
)


DEFAULT_EXTRINSIC_CONFIG = (
    '/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml'
)


class CameraTargetPlannerClient:
    """Shared by the mission flow and standalone camera client, not by planner.

    DETECT and the interval before submission must keep the same stationary
    observation pose. Use fresh measured joints, not hardcoded observation angles.
    """

    def __init__(self, node, config_path=DEFAULT_EXTRINSIC_CONFIG):
        self.node = node
        with Path(config_path).open(encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        self.rotation, self.translation_mm = load_camera_extrinsic(config_path)
        with Path(self.config['arm_config_path']).open(encoding='utf-8') as stream:
            arm = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']
        if not all(math.isfinite(v) and v > 0 for v in arm['geometry'].values()):
            raise ValueError('arm geometry must be finite and positive')
        self.kinematics = RobotKinematics.from_geometry_dict(arm['geometry'])
        self.transformer = JointTransformer.from_config_dict(arm['joint_mapping'])
        self.status_max_age_sec = float(arm['planning']['status_max_age_sec'])
        self.gripper_dwell_ms = arm['planning']['default_dwell_ms']
        if type(self.gripper_dwell_ms) is not int or not 0 < self.gripper_dwell_ms <= 0xFFFFFFFF:
            raise ValueError('grasp default_dwell_ms must be a positive uint32')
        if not math.isfinite(self.status_max_age_sec) or self.status_max_age_sec <= 0:
            raise ValueError('status_max_age_sec must be finite and positive')
        for name in ('endpoint_timeout_sec', 'service_timeout_sec'):
            value = self.config[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'{name} must be numeric')
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('camera_frame', 'joint_state_topic',
                     'move_cartesian_service', 'stop_service'):
            if not isinstance(self.config[name], str) or not self.config[name].strip():
                raise ValueError(f'{name} must be a nonempty string')
        motion = self.config['motion']
        duration = motion['duration_ms']
        if type(duration) is not int or not 0 <= duration <= 0xFFFFFFFF:
            raise ValueError('duration_ms must be uint32')
        if type(motion['linear_path']) is not bool:
            raise ValueError('linear_path must be boolean')
        pitch = motion['pitch_deg']
        if pitch is not None and (isinstance(pitch, bool)
                                  or not isinstance(pitch, (int, float))
                                  or not math.isfinite(pitch)):
            raise ValueError('pitch_deg must be null or a finite number')
        self._joints = None
        self._latest_received_at = 0.0
        self._status = None
        self._status_received_at = 0.0
        self._motion_future = None
        self._motion_deadline = 0.0
        self._joint_subscription = node.create_subscription(
            JointState, self.config['joint_state_topic'], self._on_joint_state, 10)
        self._move_client = node.create_client(
            MoveArmCartesian, self.config['move_cartesian_service'])
        self._preset_client = node.create_client(
            ExecuteArmPreset, '/arm_planner_node/execute_preset')
        self._joints_client = node.create_client(MoveArmJoints, '/arm_planner_node/move_joints')
        self._stop_client = node.create_client(Trigger, self.config['stop_service'])
        self._status_subscription = node.create_subscription(
            ArmStatus, '/fruit_hardware_bridge/arm/status', self._on_status, 10)
        self._motion_timer = node.create_timer(0.1, self._check_motion_timeout)
        offset = arm['planning'].get('cartesian_offset_m', [0.0, 0.0, 0.0])
        node.get_logger().info(
            f'Mission camera transform ready; planner adds cartesian_offset_m={offset}')

    def _on_joint_state(self, message):
        self._latest_received_at = time.monotonic()
        self._joints = None
        names = [f'joint_{i}' for i in range(1, 5)]
        if len(message.name) != 4 or set(message.name) != set(names):
            return
        if len(message.position) != 4:
            return
        joints = tuple(message.position[message.name.index(name)] for name in names)
        if not all(math.isfinite(q) for q in joints):
            return
        if not self.transformer.check_hardware_limits(joints)[0]:
            return
        self._joints = joints

    @staticmethod
    def _stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _fresh_joints(self):
        if time.monotonic() - self._latest_received_at > self.status_max_age_sec:
            return None
        return self._joints

    def _on_status(self, message):
        self._status = message
        self._status_received_at = time.monotonic()

    def is_stationary(self, max_joint_speed, received_after=0.0, require_idle=False):
        """Use finite raw feedback, not JointState's substituted velocities."""
        status = self._status
        if (status is None
                or time.monotonic() - self._status_received_at > self.status_max_age_sec
                or self._status_received_at <= received_after
                or status.task_status not in (ArmStatus.STATUS_IDLE, ArmStatus.STATUS_COMPLETED)
                or (require_idle and status.task_status != ArmStatus.STATUS_IDLE)
                or status.error_code != ArmStatus.ERROR_NONE):
            return False
        joints = (status.joint_1, status.joint_2, status.joint_3, status.joint_4)
        speeds = (status.joint_1_velocity, status.joint_2_velocity,
                  status.joint_3_velocity, status.joint_4_velocity)
        return (all(math.isfinite(q) for q in joints)
                and self.transformer.check_hardware_limits(joints)[0]
                and all(math.isfinite(v) and abs(v) <= max_joint_speed for v in speeds))

    def target_request(self, target):
        """Freeze an optical target into base coordinates before any arm motion."""
        stamp_ns = self._stamp_ns(target)
        point_m = (target.point.x, target.point.y, target.point.z)
        if (target.header.frame_id != self.config['camera_frame'] or stamp_ns <= 0
                or not all(math.isfinite(v) for v in point_m) or point_m[2] <= 0):
            raise ValueError('Invalid camera target frame, stamp or depth')
        hardware_joints = self._fresh_joints()
        if hardware_joints is None:
            raise ValueError('No fresh valid measured joint feedback')
        algo = self.transformer.hardware_to_algo(hardware_joints)
        end_x, end_y, end_z, pitch = self.kinematics.forward_kinematics(
            *algo)['end_effector']
        try:
            end_mm = transform_camera_to_end_effector_mm(
                tuple(v * 1000.0 for v in point_m), self.rotation, self.translation_mm)
            base_mm = transform_end_to_base_mm(
                end_mm, (end_x * 1000.0, end_y * 1000.0, end_z * 1000.0), algo[0], pitch)
            request = MoveArmCartesian.Request()
            request.x, request.y, request.z = (v / 1000.0 for v in base_mm)
            desired_pitch = self.config['motion']['pitch_deg']
            request.pitch_deg = float(
                math.degrees(pitch) if desired_pitch is None else desired_pitch)
            request.duration_ms = self.config['motion']['duration_ms']
            request.gripper_action = 2  # CLOSE at the endpoint.
            request.linear_path = self.config['motion']['linear_path']
        except (ValueError, OverflowError, AssertionError) as error:
            raise ValueError(f'Invalid transformed target: {error}') from error
        self.node.get_logger().info(
            f'Camera(m)={point_m} -> end(mm)={end_mm} -> arm base(mm)={base_mm}; '
            f'move_cartesian pitch={request.pitch_deg:.3f}deg, gripper=CLOSE')
        return request

    def move_to_base_target(self, request, wait_until, max_joint_speed):
        """Execute a frozen base-frame request (also used for B's second fruit)."""
        if not self._move_client.wait_for_service(
                timeout_sec=self.config['endpoint_timeout_sec']):
            return False, 'arm_endpoints_not_ready'
        completed, reason = self._call_motion(self._move_client, request, wait_until)
        if not completed:
            return completed, reason
        # Cartesian API has no dwell field. Reuse planner PTP at measured raw
        # joints with HOLD instead of generating a trajectory or sleeping here.
        if not wait_until(
                lambda: self.is_stationary(max_joint_speed), self.config['endpoint_timeout_sec']):
            return False, 'arm_pose_unavailable'
        if not self._joints_client.wait_for_service(
                timeout_sec=self.config['endpoint_timeout_sec']):
            return False, 'arm_endpoints_not_ready'
        status = self._status
        dwell = MoveArmJoints.Request()
        dwell.joints_deg = [math.degrees(q) for q in (
            status.joint_1, status.joint_2, status.joint_3, status.joint_4)]
        dwell.duration_ms = self.gripper_dwell_ms
        dwell.gripper_action = 0
        return self._call_motion(self._joints_client, dwell, wait_until)

    def start_preset(self, name):
        """Submit once; the caller can navigate while the executor receives its result."""
        if not isinstance(name, str) or not name.strip():
            return None
        if not self._preset_client.wait_for_service(
                timeout_sec=self.config['endpoint_timeout_sec']):
            return None
        request = ExecuteArmPreset.Request()
        request.preset_name = name
        self.node.get_logger().info(f'Executing arm preset: {name}')
        return self._start_motion(self._preset_client, request)

    def execute_preset(self, name, wait_until):
        """Execute a named planner preset; only hardware completion is success."""
        return self.finish_motion(self.start_preset(name), wait_until)

    def _start_motion(self, client, request):
        if (self._motion_future is not None
                and not (self._motion_future.done() or self._motion_future.cancelled())):
            self.node.get_logger().warning('An arm request is still pending')
            return None
        try:
            future = client.call_async(request)
        except Exception as error:
            self.node.get_logger().warning(f'Cannot submit arm motion: {error}')
            return None
        self._motion_future = future
        self._motion_deadline = time.monotonic() + self.config['service_timeout_sec']
        return future

    def _check_motion_timeout(self):
        future = self._motion_future
        if (future is not None and not future.done()
                and time.monotonic() >= self._motion_deadline):
            future.cancel()
            self._motion_future = None
            self.request_stop()

    def request_stop(self):
        """Nonblocking STOP also works from the timeout timer; it is not standstill."""
        if self._motion_future is not None and not self._motion_future.done():
            self._motion_future.cancel()
            self._motion_future = None
        if self._status is not None and self._status.task_status == ArmStatus.STATUS_EMERGENCY:
            self.node.get_logger().warning('Hardware emergency remains locked')
            return None
        if not self._stop_client.service_is_ready():
            self.node.get_logger().warning('STOP service unavailable; stop is unconfirmed')
            return None
        try:
            return self._stop_client.call_async(Trigger.Request())
        except Exception as error:
            self.node.get_logger().warning(f'Cannot submit STOP: {error}')
            return None

    def _call_motion(self, client, request, wait_until):
        """Share execution-result validation and STOP on unknown results."""
        return self.finish_motion(self._start_motion(client, request), wait_until)

    def finish_motion(self, future, wait_until):
        if future is None:
            return False, 'arm_endpoints_not_ready'
        try:
            if not wait_until(lambda: future.done() or future.cancelled(),
                              self.config['service_timeout_sec']):
                future.cancel()
                self.request_stop()
                return False, 'arm_callback_timeout'
            if future.cancelled():
                return False, 'arm_callback_timeout'
            response = future.result()
            if response is None:
                raise RuntimeError('planner returned no response')
        except Exception as error:
            self.node.get_logger().warning(f'Planner request failed: {error}')
            self.request_stop()
            return False, 'arm_callback_failed'
        self.node.get_logger().info(
            f'Planner trajectory_id={response.trajectory_id}: {response.message}')
        return bool(response.success), 'completed' if response.success else 'arm_callback_failed'
