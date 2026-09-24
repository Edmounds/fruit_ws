"""Fruit Arm Planner ROS 2 Node.

Encapsulates 4-DOF kinematics, joint mapping/offsets, dual-mode trajectory
interpolation (PTP / LIN), multi-waypoint presets, and hardware communication.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Tuple

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

from fruit_arm_planner.joint_transformer import JointTransformer
from fruit_arm_planner.kinematics import RobotKinematics
from fruit_arm_planner.trajectory_planner import (
    GripperAction,
    TrajectoryPlanner,
    TrajectoryPoint as PlannerTrajectoryPoint,
)


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
        self.kinematics = RobotKinematics()
        self.transformer = JointTransformer()
        self.planner = TrajectoryPlanner(self.kinematics)

        self.default_duration_ms = 3000
        self.interpolation_step_ms = 50
        self.hover_height_m = 0.06
        self.presets: Dict[str, dict] = {}

        self._load_config_from_yaml()

        # 下位机硬件实时状态缓存 (20Hz，默认物理零位为大臂竖直朝天、小臂水平朝前)
        self.latest_status: Optional[ArmStatus] = None
        self.current_joints = (0.0, 0.0, 0.0, 0.0)
        self.current_gripper_state = 0
        self.latest_trajectory_id = 0

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

        # 快捷服务接口
        self._srv_home = self.create_service(
            Trigger, '~/home', self._srv_home_cb, callback_group=self.cb_group
        )
        self._srv_retract = self.create_service(
            Trigger, '~/retract', self._srv_retract_cb, callback_group=self.cb_group
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
            JointState, '~/joint_states', 10
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
            params = data.get('arm_planner_node', {}).get('ros__parameters', {})

            # 1. 连杆几何参数
            if 'geometry' in params:
                self.kinematics = RobotKinematics.from_geometry_dict(params['geometry'])
                self.planner = TrajectoryPlanner(self.kinematics)
                self.get_logger().info(
                    f"Loaded arm geometry: d1={self.kinematics.d1}m, "
                    f"a2={self.kinematics.a2}m, a3={self.kinematics.a3}m, a4={self.kinematics.a4}m"
                )

            # 2. 关节坐标系映射与零位偏置
            if 'joint_mapping' in params:
                self.transformer = JointTransformer.from_config_dict(params['joint_mapping'])
                for ch in self.transformer.channels:
                    self.get_logger().info(
                        f"Joint [{ch.name}] mapping: offset={math.degrees(ch.offset_rad):.1f}°, "
                        f"dir={ch.direction:+.1f}, "
                        f"limits=[{math.degrees(ch.min_limit_rad):.1f}°, {math.degrees(ch.max_limit_rad):.1f}°]"
                    )

            # 3. 规划参数
            planning_cfg = params.get('planning', {})
            self.default_duration_ms = int(planning_cfg.get('default_duration_ms', 3000))
            self.interpolation_step_ms = int(planning_cfg.get('interpolation_step_ms', 50))
            self.hover_height_m = float(planning_cfg.get('hover_height_m', 0.06))

            # 4. 预设工序与固定轨迹
            presets_data = params.get('presets', {})
            self.presets = presets_data
            self.get_logger().info(f"Loaded {len(self.presets)} arm presets: {list(self.presets.keys())}")

        except Exception as e:
            self.get_logger().error(f"Failed to load configuration from {config_path}: {e}")

    # ========================================================================
    # 状态回调
    # ========================================================================

    def _on_arm_status(self, msg: ArmStatus) -> None:
        """处理来自下位机的 20Hz 状态遥测."""
        self.latest_status = msg
        self.latest_trajectory_id = msg.trajectory_id

        # 保护：开环舵机或无回传引脚可能返回 NaN，过滤保护避免污染插值
        j1 = msg.joint_1 if not math.isnan(msg.joint_1) else 0.0
        j2 = msg.joint_2 if not math.isnan(msg.joint_2) else 0.0
        j3 = msg.joint_3 if not math.isnan(msg.joint_3) else 0.0
        j4 = msg.joint_4 if not math.isnan(msg.joint_4) else self.current_joints[3]
        self.current_joints = (j1, j2, j3, j4)
        self.current_gripper_state = msg.gripper_state

        # 发布标准 JointState
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        js_msg.position = list(self.current_joints)
        js_msg.velocity = [
            msg.joint_1_velocity if not math.isnan(msg.joint_1_velocity) else 0.0,
            msg.joint_2_velocity if not math.isnan(msg.joint_2_velocity) else 0.0,
            msg.joint_3_velocity if not math.isnan(msg.joint_3_velocity) else 0.0,
            msg.joint_4_velocity if not math.isnan(msg.joint_4_velocity) else 0.0,
        ]
        self._joint_state_pub.publish(js_msg)

    # ========================================================================
    # 核心下发与看门狗执行
    # ========================================================================

    def execute_trajectory_sync(
        self,
        points: List[PlannerTrajectoryPoint],
        expected_duration_ms: int,
        description: str = 'trajectory',
        timeout_buffer_sec: float = 2.0,
    ) -> Tuple[bool, int, str]:
        """将轨迹下发至单片机并同步阻塞等待 COMPLETED (带自动超时 STOP 保护)."""
        if not points:
            return False, 0, 'Empty trajectory points'

        traj_id = self.planner.get_new_trajectory_id()
        point_num = len(points)
        actual_traj_duration_ms = points[-1].time_from_start
        timeout_sec = (actual_traj_duration_ms / 1000.0) + timeout_buffer_sec

        self.get_logger().info(
            f"Dispatching {description}: traj_id={traj_id}, points={point_num}, "
            f"duration={actual_traj_duration_ms}ms, timeout={timeout_sec:.1f}s"
        )

        # 1. 头部声明
        header = TrajectoryHeader()
        header.trajectory_id = traj_id
        header.point_num = point_num
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
            self._point_pub.publish(hw_pt)

        # 3. 后台自动点火启动 (Mode=0)
        mode = ArmControlMode()
        mode.control_mode = ArmControlMode.MODE_TRAJECTORY
        self._mode_pub.publish(mode)

        # 4. 同步监视看门狗等待
        deadline = time.time() + timeout_sec
        start_wait = time.time()
        poll_interval = 0.05  # 50ms (20Hz)

        while time.time() < deadline:
            time.sleep(poll_interval)
            status = self.latest_status
            if status is None:
                continue

            # 匹配当前任务
            if status.trajectory_id == traj_id:
                if status.task_status == ArmStatus.STATUS_COMPLETED:
                    elapsed = time.time() - start_wait
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

        # 超时未报完成：触发安全停机看门狗
        self.stop()
        timeout_msg = (
            f"Execution timeout for trajectory {traj_id} after {timeout_sec:.1f}s. "
            f"Safety STOP sent to hardware."
        )
        self.get_logger().error(timeout_msg)
        return False, traj_id, timeout_msg

    # ========================================================================
    # 服务端点回调函数
    # ========================================================================

    def _srv_move_cartesian_cb(
        self, req: MoveArmCartesian.Request, res: MoveArmCartesian.Response
    ) -> MoveArmCartesian.Response:
        """笛卡尔空间控制服务 (支持动态选择 linear_path 直线 vs PTP 弧线)."""
        duration_ms = int(req.duration_ms) if req.duration_ms > 0 else self.default_duration_ms
        pitch_rad = math.radians(float(req.pitch_deg))
        target_xyz = (float(req.x), float(req.y), float(req.z))
        gripper = int(req.gripper_action)

        if req.linear_path:
            # 模式 B：严格笛卡尔直线插补 (LIN，沿线切片逐点逆解)
            q_algo_current = self.transformer.hardware_to_algo(self.current_joints)
            fk_current = self.kinematics.forward_kinematics(*q_algo_current)
            start_xyz = fk_current["points"][4]
            start_pitch = fk_current["end_effector"][3]

            ok, points, msg = self.planner.plan_cartesian_linear_path(
                start_xyz=start_xyz,
                target_xyz=target_xyz,
                start_pitch=start_pitch,
                target_pitch=pitch_rad,
                duration_ms=duration_ms,
                step_ms=self.interpolation_step_ms,
                transformer=self.transformer,
                gripper_action=gripper,
            )
            if not ok:
                res.success = False
                res.message = f"LIN planning failed: {msg}"
                res.trajectory_id = 0
                return res

            success, traj_id, exec_msg = self.execute_trajectory_sync(
                points, duration_ms, f"LIN_move_cartesian({target_xyz})"
            )
            res.success = success
            res.message = exec_msg
            res.trajectory_id = traj_id
            return res

        else:
            # 模式 A：关节空间平滑过渡 (PTP，仅终点逆解，快速平滑弧线)
            ok, q_algo, ik_msg = self.kinematics.inverse_kinematics(
                target_xyz[0], target_xyz[1], target_xyz[2], pitch=pitch_rad, force_elbow_up=True
            )
            if not ok:
                res.success = False
                res.message = f"PTP IK failed for {target_xyz}: {ik_msg}"
                res.trajectory_id = 0
                return res

            target_q_hw = self.transformer.algo_to_hardware(q_algo)
            lim_ok, lim_msg = self.transformer.check_hardware_limits(target_q_hw)
            if not lim_ok:
                res.success = False
                res.message = f"PTP limit exceeded: {lim_msg}"
                res.trajectory_id = 0
                return res

            num_pts = max(2, duration_ms // self.interpolation_step_ms)
            points = self.planner.plan_joint_point_to_point(
                start_q=self.current_joints,
                end_q=target_q_hw,
                duration_ms=duration_ms,
                num_points=num_pts,
                gripper_action=gripper,
            )

            success, traj_id, exec_msg = self.execute_trajectory_sync(
                points, duration_ms, f"PTP_move_cartesian({target_xyz})"
            )
            res.success = success
            res.message = exec_msg
            res.trajectory_id = traj_id
            return res

    def _srv_move_joints_cb(
        self, req: MoveArmJoints.Request, res: MoveArmJoints.Response
    ) -> MoveArmJoints.Response:
        """关节角直接控制服务."""
        duration_ms = int(req.duration_ms) if req.duration_ms > 0 else self.default_duration_ms
        target_q_hw = tuple(math.radians(float(d)) for d in req.joints_deg)
        gripper = int(req.gripper_action)

        lim_ok, lim_msg = self.transformer.check_hardware_limits(target_q_hw)
        if not lim_ok:
            res.success = False
            res.message = f"Joint limit exceeded: {lim_msg}"
            res.trajectory_id = 0
            return res

        num_pts = max(2, duration_ms // self.interpolation_step_ms)
        points = self.planner.plan_joint_point_to_point(
            start_q=self.current_joints,
            end_q=target_q_hw,
            duration_ms=duration_ms,
            num_points=num_pts,
            gripper_action=gripper,
        )

        success, traj_id, exec_msg = self.execute_trajectory_sync(
            points, duration_ms, f"move_joints({req.joints_deg}°)"
        )
        res.success = success
        res.message = exec_msg
        res.trajectory_id = traj_id
        return res

    def _srv_execute_preset_cb(
        self, req: ExecuteArmPreset.Request, res: ExecuteArmPreset.Response
    ) -> ExecuteArmPreset.Response:
        """预设工序序列调用服务 (支持笛卡尔坐标多点序列与双点驻留法夹取)."""
        preset_name = req.preset_name.strip()
        if preset_name not in self.presets:
            res.success = False
            res.message = f"Unknown preset '{preset_name}'. Available: {list(self.presets.keys())}"
            res.trajectory_id = 0
            return res

        preset_data = self.presets[preset_name]
        coord_frame = preset_data.get("coordinate_frame", "joints")
        waypoints = preset_data.get("waypoints", [])

        if not waypoints:
            res.success = False
            res.message = f"Preset '{preset_name}' has empty waypoints"
            res.trajectory_id = 0
            return res

        ok, points, plan_msg = self.planner.plan_multi_waypoint_sequence(
            current_q_hw=self.current_joints,
            waypoints=waypoints,
            transformer=self.transformer,
            coordinate_frame=coord_frame,
            step_ms=self.interpolation_step_ms,
        )
        if not ok:
            res.success = False
            res.message = f"Preset planning failed: {plan_msg}"
            res.trajectory_id = 0
            return res

        total_ms = points[-1].time_from_start
        success, traj_id, exec_msg = self.execute_trajectory_sync(
            points, total_ms, f"preset({preset_name})"
        )
        res.success = success
        res.message = exec_msg
        res.trajectory_id = traj_id
        return res

    def _srv_home_cb(self, _req, res: Trigger.Response) -> Trigger.Response:
        req = ExecuteArmPreset.Request()
        req.preset_name = 'home'
        preset_res = ExecuteArmPreset.Response()
        preset_res = self._srv_execute_preset_cb(req, preset_res)
        res.success = preset_res.success
        res.message = preset_res.message
        return res

    def _srv_retract_cb(self, _req, res: Trigger.Response) -> Trigger.Response:
        req = ExecuteArmPreset.Request()
        req.preset_name = 'retract'
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
        self._mode_pub.publish(mode)
        self.get_logger().info('Dispatched STOP (mode=1)')

    def emergency_stop(self) -> None:
        """硬件紧急停止 (mode=2)."""
        mode = ArmControlMode()
        mode.control_mode = ArmControlMode.MODE_ESTOP
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
            req.duration_ms = int(msg.effort[0]) if (msg.effort and msg.effort[0] > 0) else self.default_duration_ms
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
