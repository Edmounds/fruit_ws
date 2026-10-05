"""Mission-side camera -> gripper -> arm-base conversion and planner client."""

import math
from pathlib import Path
import re
import time

from fruit_arm_interfaces.srv import ExecuteArmPreset, MoveArmCartesian, MoveArmJoints
from fruit_arm_interfaces.msg import ArmTelemetry
from fruit_hardware_interfaces.msg import ArmStatus
from fruit_arm_planner.joint_transformer import JointTransformer
from std_srvs.srv import Trigger
import yaml

from fruit_mission.camera_target_transform import (
    apply_target_offset_mm,
    load_camera_extrinsic,
    transform_camera_to_end_effector_mm,
    transform_end_to_base_mm,
)
from fruit_mission.config import validate_target_offset


DEFAULT_EXTRINSIC_CONFIG = (
    '/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml'
)

HARDWARE_FAILURE_RE = re.compile(
    r'Hardware reported failure:\s*task_status=(\d+),\s*error_code=(\d+)'
)


def format_task_status_cn(task_status: int) -> str:
    status_map = {
        ArmStatus.STATUS_IDLE: '空闲 (STATUS_IDLE: 0)',
        ArmStatus.STATUS_RUNNING: '正在运行 (STATUS_RUNNING: 1)',
        ArmStatus.STATUS_COMPLETED: '执行成功 (STATUS_COMPLETED: 2)',
        ArmStatus.STATUS_FAILED: '执行失败 (STATUS_FAILED: 3)',
        ArmStatus.STATUS_EMERGENCY: '急停锁定 (STATUS_EMERGENCY: 4)',
    }
    return status_map.get(task_status, f'未知状态 ({task_status})')


def format_error_code_cn(error_code: int) -> str:
    code_map = {
        ArmStatus.ERROR_NONE: '正常无故障 (ERROR_NONE: 0)',
        ArmStatus.ERROR_TIMEOUT: '任务超时 (ERROR_TIMEOUT: 1001) [机械臂未在时限内到位或路点未完整装载]',
        ArmStatus.ERROR_LIMIT_EXCEEDED: '关节超限 (ERROR_LIMIT_EXCEEDED: 1002) [超出关节软限位]',
        ArmStatus.ERROR_ESTOP_TRIGGERED: '急停触发 (ERROR_ESTOP_TRIGGERED: 1003) [硬件急停保护启动]',
    }
    return code_map.get(error_code, f'固件错误码 ({error_code})')


def translate_planner_message(msg: str) -> str:
    if not msg:
        return '未知错误'
    m = HARDWARE_FAILURE_RE.search(msg)
    if m:
        ts = int(m.group(1))
        ec = int(m.group(2))
        return f'下位机报错：状态={format_task_status_cn(ts)}，故障={format_error_code_cn(ec)}'

    translations = {
        'arm_endpoints_not_ready': '机械臂通信端点未就绪 (arm_endpoints_not_ready)',
        'arm_pose_unavailable': '机械臂未停稳或位姿不可用 (arm_pose_unavailable)',
        'planner returned no response': '规划节点无响应 (planner returned no response)',
        'Target XYZ is out of reachable workspace': '目标位置超出机械臂可达工作空间 (workspace exceeded)',
        'IK failed to find a valid solution': '逆运动学求解无有效解 (IK failed)',
        'Joint limits exceeded': '目标关节角度超出软限位 (Joint limits exceeded)',
        'Arm is busy executing another motion': '机械臂正忙于执行其他动作 (arm busy)',
        'Arm status is stale or missing': '未收到有效四轴状态反馈 (feedback missing)',
        'Motion interrupted by STOP': '运动被停止指令中断 (STOP)',
        'Motion interrupted by ESTOP': '运动被急停指令中断 (ESTOP)',
        'Invalid Cartesian target': '笛卡尔目标参数无效 (invalid cartesian target)',
    }
    for eng, cn in translations.items():
        if eng in msg:
            return cn
    return msg


class CameraTargetPlannerClient:
    """Shared by the mission flow and standalone camera client, not by planner.

    Observation window and the interval before submission must keep the same stationary
    observation pose. Use fresh measured TCP telemetry, not hardcoded observation angles.
    """

    def __init__(
        self,
        node,
        config_path=DEFAULT_EXTRINSIC_CONFIG,
        target_offset_ac_m=(0.0, 0.0, 0.0),
        target_offset_b_m=(0.0, 0.0, 0.0),
    ):
        self.node = node
        self.target_offset_ac_m = validate_target_offset(
            target_offset_ac_m, 'target_offset_ac_m')
        self.target_offset_b_m = validate_target_offset(
            target_offset_b_m, 'target_offset_b_m')
        with Path(config_path).open(encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        self.rotation, self.translation_mm = load_camera_extrinsic(config_path)
        with Path(self.config['arm_config_path']).open(encoding='utf-8') as stream:
            arm = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']
        self.transformer = JointTransformer.from_config_dict(arm['joint_mapping'])
        self.status_max_age_sec = float(arm['planning']['status_max_age_sec'])
        self.gripper_dwell_ms = arm['planning']['default_dwell_ms']
        if type(self.gripper_dwell_ms) is not int or not 0 < self.gripper_dwell_ms <= 0xFFFFFFFF:
            raise ValueError('grasp default_dwell_ms must be a positive uint32')
        if not math.isfinite(self.status_max_age_sec) or self.status_max_age_sec <= 0:
            raise ValueError('status_max_age_sec must be finite and positive')
        for name in ('endpoint_timeout_sec',):
            value = self.config[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'{name} must be numeric')
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            self.config[name] = min(10.0, float(value))
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
        self._pose = None
        self._latest_received_at = 0.0
        self._status = None
        self._status_received_at = 0.0
        self._motion_future = None
        self._joint_subscription = node.create_subscription(
            ArmTelemetry, self.config['joint_state_topic'], self._on_joint_state, 10)
        self._move_client = node.create_client(
            MoveArmCartesian, self.config['move_cartesian_service'])
        self._preset_client = node.create_client(
            ExecuteArmPreset, '/arm_planner_node/execute_preset')
        self._joints_client = node.create_client(MoveArmJoints, '/arm_planner_node/move_joints')
        self._stop_client = node.create_client(Trigger, self.config['stop_service'])
        self._status_subscription = node.create_subscription(
            ArmStatus, '/fruit_hardware_bridge/arm/status', self._on_status, 10)
        node.get_logger().debug(
            'Mission camera transform ready; using planner mapped joints and TCP telemetry')

    def _on_joint_state(self, message):
        self._latest_received_at = time.monotonic()
        self._pose = None
        if message.header.frame_id != 'arm_base_link':
            return
        names = [f'joint_{i}' for i in range(1, 5)]
        hardware = message.hardware_joint_state
        model = message.model_joint_state
        for state in (hardware, model):
            if (len(state.name) != 4 or set(state.name) != set(names)
                    or len(state.position) != 4
                    or not all(math.isfinite(q) for q in state.position)):
                return
        joints = tuple(hardware.position[hardware.name.index(name)] for name in names)
        tcp = message.tcp_position
        pose = (tcp.x, tcp.y, tcp.z, message.tcp_pitch_deg,
                model.position[model.name.index('joint_1')])
        if (not self.transformer.check_hardware_limits(joints)[0]
                or not all(math.isfinite(v) for v in pose)):
            return
        self._pose = pose

    @staticmethod
    def _stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _fresh_pose(self):
        if time.monotonic() - self._latest_received_at > self.status_max_age_sec:
            return None
        return self._pose

    def _on_status(self, message):
        self._status = message
        self._status_received_at = time.monotonic()

    def is_stationary(self, max_joint_speed, received_after=0.0, require_idle=False):
        """Use fresh finite raw hardware feedback for measured standstill."""
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

    def target_request(self, target, area):
        """Freeze an optical target into base coordinates before any arm motion."""
        if area not in ('A', 'B', 'C'):
            raise ValueError("area must be 'A', 'B', or 'C'")
        offset_m = self.target_offset_b_m if area == 'B' else self.target_offset_ac_m
        config_key = 'target_offset_b_m' if area == 'B' else 'target_offset_ac_m'

        stamp_ns = self._stamp_ns(target)
        point_m = (target.point.x, target.point.y, target.point.z)
        if (target.header.frame_id != self.config['camera_frame'] or stamp_ns <= 0
                or not all(math.isfinite(v) for v in point_m) or point_m[2] <= 0):
            raise ValueError('Invalid camera target frame, stamp or depth')
        pose = self._fresh_pose()
        if pose is None:
            raise ValueError('No fresh valid measured TCP telemetry')
        end_x, end_y, end_z, pitch_deg, yaw = pose
        pitch = math.radians(pitch_deg)
        try:
            end_mm = transform_camera_to_end_effector_mm(
                tuple(v * 1000.0 for v in point_m), self.rotation, self.translation_mm)
            base_mm = transform_end_to_base_mm(
                end_mm, (end_x * 1000.0, end_y * 1000.0, end_z * 1000.0), yaw, pitch)

            desired_pitch = self.config['motion']['pitch_deg']
            request_pitch_deg = float(
                pitch_deg if desired_pitch is None else desired_pitch)
            request_pitch_rad = math.radians(request_pitch_deg)

            adjusted_base_mm = apply_target_offset_mm(base_mm, offset_m, request_pitch_rad)
            request = MoveArmCartesian.Request()
            request.x, request.y, request.z = (v / 1000.0 for v in adjusted_base_mm)
            request.pitch_deg = request_pitch_deg
            request.duration_ms = self.config['motion']['duration_ms']
            request.gripper_action = 2  # CLOSE at the endpoint.
            request.linear_path = self.config['motion']['linear_path']
        except (ValueError, OverflowError, AssertionError) as error:
            raise ValueError(f'Invalid transformed target: {error}') from error
        yaw_offset_deg = math.degrees(math.atan2(base_mm[1], base_mm[0]))
        req_xyz = [round(request.x, 4), round(request.y, 4), round(request.z, 4)]
        self.node.get_logger().debug(
            f'area={area} key={config_key} offset(m)={offset_m} '
            f'ref_yaw={yaw_offset_deg:.2f}deg ref_pitch={request.pitch_deg:.2f}deg; '
            f'Camera(m)={point_m} -> end(mm)={end_mm} -> arm base(mm)={base_mm} '
            f'-> req(m)={req_xyz}; '
            f'tcp(m)={[round(end_x, 4), round(end_y, 4), round(end_z, 4)]}, '
            f'obs_pitch={pitch_deg:.2f}deg, obs_yaw={math.degrees(yaw):.2f}deg, '
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
        return self._gripper_at_measured_pose(0, wait_until, max_joint_speed)

    def open_gripper(self, wait_until, max_joint_speed):
        """OPEN at measured raw joints, then HOLD for the configured gripper dwell."""
        completed, reason = self._gripper_at_measured_pose(1, wait_until, max_joint_speed)
        if not completed:
            return completed, reason
        return self._gripper_at_measured_pose(0, wait_until, max_joint_speed)

    def _gripper_at_measured_pose(self, action, wait_until, max_joint_speed):
        if not self._joints_client.wait_for_service(
                timeout_sec=self.config['endpoint_timeout_sec']):
            return False, 'arm_endpoints_not_ready'
        if not wait_until(
                lambda: self.is_stationary(max_joint_speed), self.config['endpoint_timeout_sec']):
            return False, 'arm_pose_unavailable'
        status = self._status
        dwell = MoveArmJoints.Request()
        dwell.joints_deg = [math.degrees(q) for q in (
            status.joint_1, status.joint_2, status.joint_3, status.joint_4)]
        dwell.duration_ms = self.gripper_dwell_ms
        dwell.gripper_action = action
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
        self.node.get_logger().debug(f'Executing arm preset: {name}')
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
        return future

    def request_stop(self):
        """Cancel the pending request and submit STOP; it is not standstill."""
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
            return False, '机械臂通信端点未就绪 (arm_endpoints_not_ready)'
        try:
            if not wait_until(lambda: future.done() or future.cancelled()):
                future.cancel()
                self.request_stop()
                return False, '等待规划结果超时，已下发停止 (timeout)'
            if future.cancelled():
                return False, '规划请求已取消 (cancelled)'
            response = future.result()
            if response is None:
                raise RuntimeError('planner returned no response')
        except Exception as error:
            self.node.get_logger().warning(f'Planner request failed: {error}')
            self.request_stop()
            return False, f'规划服务调用异常 ({error})'
        if response.success:
            self.node.get_logger().info(
                f'Planner trajectory_id={response.trajectory_id}: {response.message}')
            return True, 'completed'
        else:
            translated = translate_planner_message(response.message)
            self.node.get_logger().warning(
                f'Planner trajectory_id={response.trajectory_id} failed: {translated}')
            return False, translated
