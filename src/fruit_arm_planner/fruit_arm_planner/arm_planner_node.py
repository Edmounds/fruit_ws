"""Fruit Arm Planner ROS 2 Node.

Encapsulates 4-DOF kinematics, joint mapping/offsets, dual-mode trajectory
interpolation (PTP / LIN), multi-waypoint presets, and hardware communication.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from fruit_hardware_interfaces.msg import (
    ArmControlMode,
    ArmStatus,
    TrajectoryHeader,
    TrajectoryPoint as HardwareTrajectoryPoint,
)
from fruit_arm_interfaces.srv import (
    ExecuteArmPreset,
    MoveArmCartesian,
    MoveArmJoints,
)
from fruit_arm_interfaces.msg import ArmTelemetry

from fruit_arm_planner.joint_transformer import JointTransformer
from fruit_arm_planner.kinematics import RobotKinematics
from fruit_arm_planner.trajectory_planner import (
    GripperAction,
    TrajectoryPlanner,
    TrajectoryPoint as PlannerTrajectoryPoint,
)


def validate_presets(presets: dict) -> None:
    """Validate a complete preset replacement before installing or saving it."""
    if not isinstance(presets, dict):
        raise ValueError('presets must be a mapping')
    for name, preset in presets.items():
        if (not isinstance(name, str) or not name.strip() or not isinstance(preset, dict)
                or preset.get('coordinate_frame') not in ('joints', 'cartesian', 'link_angles')
                or not isinstance(preset.get('waypoints'), list) or not preset['waypoints']):
            raise ValueError(f'Invalid preset {name}')
        for wp in preset['waypoints']:
            if not isinstance(wp, dict):
                raise ValueError(f'Invalid waypoint in preset {name}')
            target = wp.get('target')
            if (not isinstance(target, list) or len(target) < 4
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                               and math.isfinite(v) for v in target)
                    or not isinstance(wp.get('duration_ms'), int)
                    or isinstance(wp['duration_ms'], bool)
                    or not 0 < wp['duration_ms'] <= 0xFFFFFFFF
                    or not isinstance(wp.get('dwell_ms', 0), int)
                    or wp.get('dwell_ms', 0) < 0
                    or wp.get('gripper', 0) not in (0, 1, 2)
                    or not isinstance(wp.get('linear', False), bool)):
                raise ValueError(f'Invalid waypoint in preset {name}')


class ArmPlannerNode(Node):
    """ROS 2 Node for 4-DOF manipulator trajectory planning and hardware interfacing."""

    def __init__(self) -> None:
        super().__init__('arm_planner_node')

        self.cb_group = ReentrantCallbackGroup()

        # --------------------------------------------------------------------
        # 1. 参数声明
        # --------------------------------------------------------------------
        self.declare_parameter(
            'config_path',
            '/home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml',
        )

        # --------------------------------------------------------------------
        # 2. 算法与核心模块初始化 (由 YAML 配置参数驱动)
        # --------------------------------------------------------------------
        self.presets: Dict[str, dict] = {}

        self._load_config_from_yaml()

        self._state_lock = threading.Lock()
        self._motion_active = False
        self._interrupt_reason: Optional[str] = None
        self._require_idle = False
        self.latest_status: Optional[ArmStatus] = None
        self._status_received_at = 0.0
        self.current_joints: Optional[Tuple[float, float, float, float]] = None

        # --------------------------------------------------------------------
        # 3. 对接底层硬件通信 (fruit_hardware_bridge / auto_serial_bridge)
        # --------------------------------------------------------------------
        hardware_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 轨迹下发三部曲发布者
        self._header_pub = self.create_publisher(
            TrajectoryHeader,
            '/fruit_hardware_bridge/arm/trajectory_header',
            hardware_qos,
        )
        self._point_pub = self.create_publisher(
            HardwareTrajectoryPoint,
            '/fruit_hardware_bridge/arm/trajectory_point',
            hardware_qos,
        )
        self._mode_pub = self.create_publisher(
            ArmControlMode,
            '/fruit_hardware_bridge/arm/control_mode',
            hardware_qos,
        )

        # 监听下位机 20Hz 综合状态
        self._status_sub = self.create_subscription(
            ArmStatus,
            '/fruit_hardware_bridge/arm/status',
            self._on_arm_status,
            QoSProfile(depth=10),
            callback_group=self.cb_group,
        )

        # --------------------------------------------------------------------
        # 4. 对外提供的核心服务接口 (Services - 业务控制一等公民)
        # --------------------------------------------------------------------
        self._srv_move_cartesian = self.create_service(
            MoveArmCartesian,
            '~/move_cartesian',
            self._srv_move_cartesian_cb,
            callback_group=self.cb_group,
        )
        self._srv_move_joints = self.create_service(
            MoveArmJoints,
            '~/move_joints',
            self._srv_move_joints_cb,
            callback_group=self.cb_group,
        )
        self._srv_execute_preset = self.create_service(
            ExecuteArmPreset,
            '~/execute_preset',
            self._srv_execute_preset_cb,
            callback_group=self.cb_group,
        )
        self._srv_reload_presets = self.create_service(
            Trigger, '~/reload_presets', self._srv_reload_presets_cb,
            callback_group=self.cb_group,
        )

        # 快捷服务接口
        self._srv_home = self.create_service(
            Trigger, '~/home', self._srv_home_cb, callback_group=self.cb_group
        )
        self._srv_stop = self.create_service(
            Trigger, '~/stop', self._srv_stop_cb, callback_group=self.cb_group
        )
        self._srv_estop = self.create_service(
            Trigger, '~/emergency_stop', self._srv_estop_cb, callback_group=self.cb_group
        )

        # --------------------------------------------------------------------
        # 5. 辅助话题与遥测接口
        # --------------------------------------------------------------------
        self._cmd_cartesian_sub = self.create_subscription(
            PointStamped, '~/cmd_cartesian', self._on_cmd_cartesian, 10
        )
        self._cmd_joints_sub = self.create_subscription(
            JointState, '~/cmd_joints', self._on_cmd_joints, 10
        )
        self._cmd_preset_sub = self.create_subscription(
            String, '~/cmd_preset', self._on_cmd_preset, 10
        )
        self._joint_state_pub = self.create_publisher(
            ArmTelemetry, '~/joint_states', 10
        )

        self.get_logger().info('Fruit Arm Planner Node fully ready with Service API.')

    # ========================================================================
    # 配置加载
    # ========================================================================

    def _load_config_from_yaml(self) -> None:
        """从配置文件加载几何尺寸、关节偏置映射、规划参数与预设姿态."""
        config_path = self.get_parameter('config_path').value
        try:
            import yaml
            with open(config_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            params = data['arm_planner_node']['ros__parameters']
            for key in ('geometry', 'joint_mapping', 'planning', 'presets'):
                if key not in params:
                    raise ValueError(f'Missing configuration section: {key}')
            for key in ('q1', 'q2', 'q3', 'q4'):
                cfg = params['joint_mapping'][key]
                limits = cfg['limits_deg']
                if (len(limits) != 2 or not all(math.isfinite(float(v)) for v in limits)
                        or float(limits[0]) >= float(limits[1])
                        or not math.isfinite(float(cfg['offset_deg']))
                        or float(cfg['direction']) not in (-1.0, 1.0)):
                    raise ValueError(f'Invalid joint_mapping.{key}')

            # 1. 连杆几何参数
            self.kinematics = RobotKinematics.from_geometry_dict(params['geometry'])
            self.planner = TrajectoryPlanner(self.kinematics)
            self.get_logger().info(
                f"Loaded arm geometry: d1={self.kinematics.d1}m, "
                f"a2={self.kinematics.a2}m, a3={self.kinematics.a3}m, a4={self.kinematics.a4}m, "
                f"tool_z_offset_m={self.kinematics.tool_z_offset_m}m"
            )

            # 2. 关节坐标系映射与零位偏置
            self.transformer = JointTransformer.from_config_dict(params['joint_mapping'])
            for ch in self.transformer.channels:
                self.get_logger().info(
                    f"Joint [{ch.name}] mapping: offset={math.degrees(ch.offset_rad):.1f}°, "
                    f"dir={ch.direction:+.1f}, "
                    f"limits=[{math.degrees(ch.min_limit_rad):.1f}°, {math.degrees(ch.max_limit_rad):.1f}°]"
                )

            # 3. 规划参数
            planning_cfg = params['planning']
            self.default_duration_ms = int(planning_cfg['default_duration_ms'])
            self.interpolation_step_ms = int(planning_cfg['interpolation_step_ms'])
            self.default_dwell_ms = int(planning_cfg['default_dwell_ms'])
            self.hover_height_m = float(planning_cfg['hover_height_m'])
            self.status_max_age_sec = float(planning_cfg['status_max_age_sec'])
            self.mode_start_delay_ms = int(planning_cfg['mode_start_delay_ms'])
            if (self.default_duration_ms <= 0 or self.interpolation_step_ms <= 0
                    or self.default_dwell_ms < 0
                    or not 0 < self.status_max_age_sec < 10
                    or not 0 <= self.mode_start_delay_ms <= 10000
                    or not math.isfinite(self.hover_height_m)):
                raise ValueError('Invalid planning parameters')

            # 4. 预设工序与固定轨迹
            presets_data = params['presets']
            validate_presets(presets_data)
            self.presets = presets_data
            self.get_logger().info(f"Loaded {len(self.presets)} arm presets: {list(self.presets.keys())}")

        except Exception as e:
            raise RuntimeError(f'Failed to load configuration from {config_path}: {e}') from e

    def _srv_reload_presets_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """Reload only presets; a failed or busy reload leaves the old mapping intact."""
        try:
            import yaml
            with self._state_lock:
                if self._motion_active:
                    raise ValueError('Arm planner is busy; reload after motion completes')
            with open(self.get_parameter('config_path').value, encoding='utf-8') as stream:
                presets = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']['presets']
            validate_presets(presets)
            with self._state_lock:
                if self._motion_active:
                    raise ValueError('Arm planner is busy; reload after motion completes')
                self.presets = presets
            res.success = True
            res.message = f'Reloaded {len(presets)} presets: {list(presets)}'
        except Exception as exc:
            res.success = False
            res.message = f'Presets unchanged: {exc}'
        return res

    # ========================================================================
    # 状态回调
    # ========================================================================

    def _on_arm_status(self, msg: ArmStatus) -> None:
        """处理来自下位机的 20Hz 状态遥测."""
        joints = (msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4)
        valid = all(math.isfinite(q) for q in joints) and self.transformer.check_hardware_limits(joints)[0]
        with self._state_lock:
            self.latest_status = msg
            self._status_received_at = time.monotonic()
            self.current_joints = joints if valid else None
            if valid and msg.task_status == ArmStatus.STATUS_IDLE:
                self._require_idle = False
        if not valid:
            self.get_logger().warn('Ignoring arm status with invalid joint feedback')
            return

        # 同一实测快照：原始硬件角、逆映射模型角和真实 TCP；不伪造无效速度为零。
        velocities = (msg.joint_1_velocity, msg.joint_2_velocity,
                      msg.joint_3_velocity, msg.joint_4_velocity)
        model_joints = self.transformer.hardware_to_algo(joints)
        x, y, z, pitch = self.kinematics.forward_kinematics(*model_joints)['end_effector']
        telemetry = ArmTelemetry()
        telemetry.header.stamp = self.get_clock().now().to_msg()
        telemetry.header.frame_id = 'arm_base_link'
        hardware = telemetry.hardware_joint_state
        hardware.header = telemetry.header
        hardware.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        hardware.position = list(joints)
        hardware.velocity = list(velocities)
        model = telemetry.model_joint_state
        model.header = telemetry.header
        model.name = list(hardware.name)
        model.position = list(model_joints)
        model.velocity = list(self.transformer.hardware_velocity_to_algo(velocities))
        telemetry.tcp_position.x = x
        telemetry.tcp_position.y = y
        telemetry.tcp_position.z = z
        telemetry.tcp_pitch_deg = math.degrees(pitch)
        telemetry.trajectory_id = msg.trajectory_id
        telemetry.task_status = msg.task_status
        telemetry.progress = msg.progress
        telemetry.gripper_state = msg.gripper_state
        telemetry.error_code = msg.error_code
        self._joint_state_pub.publish(telemetry)

    # ========================================================================
    # 核心下发与硬件结果等待
    # ========================================================================

    def _run_motion(
        self, build: Callable[[Tuple[float, float, float, float]], Tuple[bool, List[PlannerTrajectoryPoint], str]],
        description: str,
    ) -> Tuple[bool, int, str]:
        with self._state_lock:
            if self._motion_active:
                return False, 0, 'Arm planner is busy'
            self._motion_active = True
            self._interrupt_reason = None
            status = self.latest_status
            age = time.monotonic() - self._status_received_at
            start = self.current_joints
            require_idle = self._require_idle
        try:
            if status is None or start is None or age > self.status_max_age_sec:
                return False, 0, 'Fresh valid four-joint feedback is required'
            if require_idle or status.task_status not in (ArmStatus.STATUS_IDLE, ArmStatus.STATUS_COMPLETED):
                return False, 0, f'Arm is not idle (status={status.task_status})'
            ok, points, message = build(start)
            if not ok:
                return False, 0, message
            return self.execute_trajectory_sync(points, start, description)
        except (ValueError, TypeError, OverflowError) as exc:
            return False, 0, f'Invalid motion request: {exc}'
        finally:
            with self._state_lock:
                self._motion_active = False
                self._interrupt_reason = None

    def _validate_trajectory(
        self, points: List[PlannerTrajectoryPoint], start: Tuple[float, float, float, float]
    ) -> Optional[str]:
        if not 2 <= len(points) <= 0xFFFF:
            return 'Trajectory point count must be 2..65535'
        if points[0].joints != start or points[0].gripper_action != GripperAction.HOLD:
            return 'Trajectory first point must be measured joints with HOLD'
        previous_time = -1
        for index, point in enumerate(points):
            if point.point_id != index or not previous_time < point.time_from_start <= 0xFFFFFFFF:
                return f'Invalid point ID or timestamp at point {index}'
            if not all(math.isfinite(q) for q in point.joints):
                return f'Non-finite joint angle at point {index}'
            ok, message = self.transformer.check_hardware_limits(point.joints)
            if not ok:
                return f'Point {index}: {message}'
            if point.gripper_action not in (0, 1, 2):
                return f'Invalid gripper action at point {index}'
            previous_time = point.time_from_start
        if points[0].time_from_start != 0:
            return 'Trajectory must start at 0 ms'
        return None

    def execute_trajectory_sync(
        self,
        points: List[PlannerTrajectoryPoint],
        start: Tuple[float, float, float, float],
        description: str = 'trajectory',
    ) -> Tuple[bool, int, str]:
        """下发轨迹并等待下位机完成/报错、主动 STOP 或节点退出，无执行期限."""
        error = self._validate_trajectory(points, start)
        if error:
            return False, 0, error

        with self._state_lock:
            if self._interrupt_reason:
                return False, 0, self._interrupt_reason

        traj_id = self.planner.get_new_trajectory_id()
        point_num = len(points)
        actual_traj_duration_ms = points[-1].time_from_start

        self.get_logger().info(
            f"Dispatching {description}: traj_id={traj_id}, points={point_num}, "
            f"duration={actual_traj_duration_ms}ms"
        )

        # 1. 头部声明
        header = TrajectoryHeader()
        header.trajectory_id = traj_id
        header.point_num = point_num
        with self._state_lock:
            if self._interrupt_reason:
                return False, 0, self._interrupt_reason
            self._header_pub.publish(header)

        # 2. 逐点传输
        for pt in points:
            hw_pt = HardwareTrajectoryPoint()
            hw_pt.trajectory_id = traj_id
            hw_pt.point_id = pt.point_id
            hw_pt.time_from_start = pt.time_from_start
            hw_pt.joint_1 = float(pt.joint_1)
            hw_pt.joint_2 = float(pt.joint_2)
            hw_pt.joint_3 = float(pt.joint_3)
            hw_pt.joint_4 = float(pt.joint_4)
            hw_pt.gripper_action = int(pt.gripper_action)
            with self._state_lock:
                if self._interrupt_reason:
                    return False, traj_id, self._interrupt_reason
                self._point_pub.publish(hw_pt)

        # 3. 后台自动点火启动 (Mode=0)
        mode = ArmControlMode()
        mode.control_mode = ArmControlMode.MODE_TRAJECTORY
        if self.mode_start_delay_ms:
            self.get_logger().info(
                f'Waiting {self.mode_start_delay_ms}ms before publishing TRAJECTORY mode'
            )
            time.sleep(self.mode_start_delay_ms / 1000.0)
        with self._state_lock:
            if self._interrupt_reason:
                return False, traj_id, self._interrupt_reason
            self._mode_pub.publish(mode)
            dispatched_at = time.monotonic()

        # 4. 同步监视硬件状态，等待执行完成或下位机报错
        poll_interval = 0.05  # 50ms (20Hz)

        while rclpy.ok():
            time.sleep(poll_interval)
            with self._state_lock:
                reason = self._interrupt_reason
                status = self.latest_status
                received_at = self._status_received_at
            if reason:
                return False, traj_id, reason
            if status is None or received_at <= dispatched_at:
                continue

            # 匹配当前任务
            if status.trajectory_id == traj_id:
                if status.task_status == ArmStatus.STATUS_COMPLETED:
                    elapsed = time.monotonic() - dispatched_at
                    self.get_logger().info(
                        f"Trajectory {traj_id} executed successfully in {elapsed:.2f}s"
                    )
                    return True, traj_id, f"Execution completed in {elapsed:.2f}s"

                if status.task_status in (ArmStatus.STATUS_FAILED, ArmStatus.STATUS_EMERGENCY):
                    err_msg = (
                        f"Hardware reported failure: task_status={status.task_status}, "
                        f"error_code={status.error_code}"
                    )
                    self.get_logger().error(err_msg)
                    return False, traj_id, err_msg

        return False, traj_id, "Execution interrupted by node shutdown"

    # ========================================================================
    # 服务端点回调函数
    # ========================================================================

    def _srv_move_cartesian_cb(
        self, req: MoveArmCartesian.Request, res: MoveArmCartesian.Response
    ) -> MoveArmCartesian.Response:
        """笛卡尔空间控制服务 (支持动态选择 linear_path 直线 vs PTP 弧线)."""
        def build(start):
            duration_ms = int(req.duration_ms) if req.duration_ms > 0 else self.default_duration_ms
            pitch_rad = math.radians(float(req.pitch_deg))
            target_xyz = (float(req.x), float(req.y), float(req.z))
            gripper = int(req.gripper_action)
            if (duration_ms <= 0 or duration_ms > 0xFFFFFFFF
                    or not all(math.isfinite(v) for v in (*target_xyz, pitch_rad))
                    or gripper not in (0, 1, 2)):
                return False, [], 'Invalid Cartesian target, duration or gripper action'
            if req.linear_path:
                fk_current = self.kinematics.forward_kinematics(
                    *self.transformer.hardware_to_algo(start)
                )
                return self.planner.plan_cartesian_linear_path(
                    start_xyz=fk_current['end_effector'][:3],
                    target_xyz=target_xyz,
                    start_pitch=fk_current['end_effector'][3],
                    target_pitch=pitch_rad,
                    duration_ms=duration_ms,
                    step_ms=self.interpolation_step_ms,
                    transformer=self.transformer,
                    start_q_hw=start,
                    gripper_action=gripper,
                )
            ok, q_algo, ik_msg = self.kinematics.inverse_kinematics(
                target_xyz[0], target_xyz[1], target_xyz[2], pitch=pitch_rad, force_elbow_up=True
            )
            if not ok:
                return False, [], f'PTP IK failed: {ik_msg}'

            target_q_hw = self.transformer.algo_to_hardware(q_algo)
            lim_ok, lim_msg = self.transformer.check_hardware_limits(target_q_hw)
            if not lim_ok:
                return False, [], f'PTP limit exceeded: {lim_msg}'
            num_pts = max(2, min(duration_ms + 1, math.ceil(duration_ms / self.interpolation_step_ms) + 1))
            points = self.planner.plan_joint_point_to_point(
                start_q=start,
                end_q=target_q_hw,
                duration_ms=duration_ms,
                num_points=num_pts,
                gripper_action=gripper,
            )
            return True, points, ''

        res.success, res.trajectory_id, res.message = self._run_motion(build, 'move_cartesian')
        return res

    def _srv_move_joints_cb(
        self, req: MoveArmJoints.Request, res: MoveArmJoints.Response
    ) -> MoveArmJoints.Response:
        """关节角直接控制服务."""
        def build(start):
            duration_ms = int(req.duration_ms) if req.duration_ms > 0 else self.default_duration_ms
            target_q_hw = tuple(math.radians(float(d)) for d in req.joints_deg)
            gripper = int(req.gripper_action)
            if (len(target_q_hw) != 4 or not all(math.isfinite(q) for q in target_q_hw)
                    or duration_ms <= 0 or duration_ms > 0xFFFFFFFF or gripper not in (0, 1, 2)):
                return False, [], 'Invalid joint target, duration or gripper action'
            lim_ok, lim_msg = self.transformer.check_hardware_limits(target_q_hw)
            if not lim_ok:
                return False, [], f'Joint limit exceeded: {lim_msg}'
            num_pts = max(2, min(duration_ms + 1, math.ceil(duration_ms / self.interpolation_step_ms) + 1))
            points = self.planner.plan_joint_point_to_point(
                start_q=start, end_q=target_q_hw, duration_ms=duration_ms,
                num_points=num_pts, gripper_action=gripper,
            )
            return True, points, ''

        res.success, res.trajectory_id, res.message = self._run_motion(build, 'move_joints')
        return res

    def _srv_execute_preset_cb(
        self, req: ExecuteArmPreset.Request, res: ExecuteArmPreset.Response
    ) -> ExecuteArmPreset.Response:
        """预设工序序列调用服务 (支持笛卡尔坐标多点序列与双点驻留法夹取)."""
        def build(start):
            preset_name = req.preset_name.strip()
            if preset_name not in self.presets:
                return False, [], f"Unknown preset '{preset_name}'. Available: {list(self.presets.keys())}"
            preset_data = self.presets[preset_name]
            return self.planner.plan_multi_waypoint_sequence(
                current_q_hw=start,
                waypoints=preset_data['waypoints'],
                transformer=self.transformer,
                coordinate_frame=preset_data['coordinate_frame'],
                step_ms=self.interpolation_step_ms,
                default_dwell_ms=self.default_dwell_ms,
            )

        res.success, res.trajectory_id, res.message = self._run_motion(build, f'preset({req.preset_name})')
        return res

    def _srv_home_cb(self, _req, res: Trigger.Response) -> Trigger.Response:
        req = ExecuteArmPreset.Request()
        req.preset_name = 'home'
        preset_res = ExecuteArmPreset.Response()
        preset_res = self._srv_execute_preset_cb(req, preset_res)
        res.success = preset_res.success
        res.message = preset_res.message
        return res

    def _srv_stop_cb(self, _req, res: Trigger.Response) -> Trigger.Response:
        self.stop()
        res.success = True
        res.message = 'Smooth STOP command dispatched'
        return res

    def _srv_estop_cb(self, _req, res: Trigger.Response) -> Trigger.Response:
        self.emergency_stop()
        res.success = True
        res.message = 'Emergency ESTOP command dispatched'
        return res

    def stop(self) -> None:
        """平滑减速停止 (mode=1)."""
        mode = ArmControlMode()
        mode.control_mode = ArmControlMode.MODE_STOP
        with self._state_lock:
            self._interrupt_reason = 'Motion interrupted by STOP'
            self._require_idle = True
            self._mode_pub.publish(mode)
        self.get_logger().info('Dispatched STOP (mode=1)')

    def emergency_stop(self) -> None:
        """硬件紧急停止 (mode=2)."""
        mode = ArmControlMode()
        mode.control_mode = ArmControlMode.MODE_ESTOP
        with self._state_lock:
            self._interrupt_reason = 'Motion interrupted by ESTOP'
            self._require_idle = True
            self._mode_pub.publish(mode)
        self.get_logger().warn('Dispatched ESTOP (mode=2)')

    # ========================================================================
    # 辅助话题回调
    # ========================================================================

    def _on_cmd_cartesian(self, msg: PointStamped) -> None:
        req = MoveArmCartesian.Request()
        req.x = msg.point.x
        req.y = msg.point.y
        req.z = msg.point.z
        req.pitch_deg = 0.0
        req.duration_ms = self.default_duration_ms
        req.gripper_action = 0
        req.linear_path = False
        res = MoveArmCartesian.Response()
        self._srv_move_cartesian_cb(req, res)

    def _on_cmd_joints(self, msg: JointState) -> None:
        if len(msg.position) >= 4:
            req = MoveArmJoints.Request()
            req.joints_deg = [float(math.degrees(p)) for p in msg.position[:4]]
            req.duration_ms = int(msg.effort[0]) if (msg.effort and math.isfinite(msg.effort[0]) and msg.effort[0] > 0) else self.default_duration_ms
            req.gripper_action = 0
            res = MoveArmJoints.Response()
            self._srv_move_joints_cb(req, res)

    def _on_cmd_preset(self, msg: String) -> None:
        req = ExecuteArmPreset.Request()
        req.preset_name = msg.data.strip()
        res = ExecuteArmPreset.Response()
        self._srv_execute_preset_cb(req, res)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmPlannerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
