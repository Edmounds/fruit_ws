"""Mission-side camera -> gripper -> arm-base conversion and planner client."""

import math
from pathlib import Path
import time

from fruit_arm_interfaces.srv import ExecuteArmPreset, MoveArmCartesian
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
        self._joint_subscription = node.create_subscription(
            JointState, self.config['joint_state_topic'], self._on_joint_state, 10)
        self._move_client = node.create_client(
            MoveArmCartesian, self.config['move_cartesian_service'])
        self._preset_client = node.create_client(
            ExecuteArmPreset, '/arm_planner_node/execute_preset')
        self._stop_client = node.create_client(Trigger, self.config['stop_service'])
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

    def move_to_target(self, target, wait_until):
        """Convert a stamped optical point (m); return (completed, reason_code)."""
        stamp_ns = self._stamp_ns(target)
        point_m = (target.point.x, target.point.y, target.point.z)
        if (target.header.frame_id != self.config['camera_frame'] or stamp_ns <= 0
                or not all(math.isfinite(v) for v in point_m) or point_m[2] <= 0):
            self.node.get_logger().error('Invalid camera target frame, stamp or depth')
            return False, 'camera_target_invalid'
        if not self._move_client.wait_for_service(
                timeout_sec=self.config['endpoint_timeout_sec']):
            return False, 'arm_endpoints_not_ready'
        if not wait_until(lambda: self._fresh_joints() is not None,
                          self.config['endpoint_timeout_sec']):
            self.node.get_logger().error('No fresh valid measured joint feedback')
            return False, 'arm_pose_unavailable'
        hardware_joints = self._fresh_joints()
        if hardware_joints is None:
            return False, 'arm_pose_unavailable'
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
            request.gripper_action = 0  # Go to the point; do not invent a grasp sequence.
            request.linear_path = self.config['motion']['linear_path']
        except (ValueError, OverflowError, AssertionError) as error:
            self.node.get_logger().error(f'Invalid transformed target: {error}')
            return False, 'camera_target_invalid'
        self.node.get_logger().info(
            f'Camera(m)={point_m} -> end(mm)={end_mm} -> arm base(mm)={base_mm}; '
            f'move_cartesian pitch={request.pitch_deg:.3f}deg, gripper=HOLD')
        return self._call_motion(self._move_client, request, wait_until)

    def execute_preset(self, name, wait_until):
        """Execute a named planner preset; only hardware completion is success."""
        if not isinstance(name, str) or not name.strip():
            return False, 'arm_preset_invalid'
        if not self._preset_client.wait_for_service(
                timeout_sec=self.config['endpoint_timeout_sec']):
            return False, 'arm_endpoints_not_ready'
        request = ExecuteArmPreset.Request()
        request.preset_name = name
        self.node.get_logger().info(f'Executing arm preset: {name}')
        return self._call_motion(self._preset_client, request, wait_until)

    def _call_motion(self, client, request, wait_until):
        """Share execution-result validation and STOP on unknown results."""
        try:
            future = client.call_async(request)
            if not wait_until(future.done, self.config['service_timeout_sec']):
                self._stop_after_unknown_result(wait_until)
                return False, 'arm_callback_timeout'
            response = future.result()
            if response is None:
                raise RuntimeError('planner returned no response')
        except Exception as error:
            self.node.get_logger().error(f'Planner request failed: {error}')
            self._stop_after_unknown_result(wait_until)
            return False, 'arm_callback_failed'
        self.node.get_logger().info(
            f'Planner trajectory_id={response.trajectory_id}: {response.message}')
        return bool(response.success), 'completed' if response.success else 'arm_callback_failed'

    def _stop_after_unknown_result(self, wait_until):
        """A client timeout/transport error does not prove the arm is stopped."""
        self.node.get_logger().error('Planner result unknown; requesting STOP')
        try:
            if not self._stop_client.wait_for_service(
                    timeout_sec=self.config['endpoint_timeout_sec']):
                raise RuntimeError('STOP service unavailable')
            future = self._stop_client.call_async(Trigger.Request())
            if not wait_until(future.done, self.config['endpoint_timeout_sec']):
                raise RuntimeError('STOP response timeout')
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError('STOP request failed')
            self.node.get_logger().warning('STOP published; physical stop is not confirmed')
        except Exception as error:
            self.node.get_logger().error(f'Cannot confirm STOP delivery: {error}')
