"""Trajectory Planner with S-Curve interpolation and multi-waypoint stitching."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Dict, List, Optional, Tuple

from fruit_arm_planner.kinematics import RobotKinematics


class GripperAction(IntEnum):
    """夹爪动作/状态 (0=保持 HOLD, 1=张开 OPEN, 2=闭合 CLOSE)."""
    HOLD = 0
    OPEN = 1
    CLOSE = 2


@dataclass
class TrajectoryPoint:
    """单个轨迹航路点."""
    point_id: int = 0
    joint_1: float = 0.0  # rad
    joint_2: float = 0.0  # rad
    joint_3: float = 0.0  # rad
    joint_4: float = 0.0  # rad
    time_from_start: int = 0  # ms
    gripper_action: int = GripperAction.HOLD

    @property
    def joints(self) -> Tuple[float, float, float, float]:
        return (self.joint_1, self.joint_2, self.joint_3, self.joint_4)


class TrajectoryPlanner:
    """机械臂轨迹规划器 (Trajectory Planner).

    负责：
    1. 关节空间五次多项式点到点平滑规划 (PTP)
    2. 笛卡尔空间严格三维直线插补 (LIN)
    3. 多航路点工序时间片拼接与双点驻留法夹取 (Dwell Point)
    """

    def __init__(self, kinematics: Optional[RobotKinematics] = None):
        self.kinematics = kinematics or RobotKinematics()
        self._next_trajectory_id = 1000

    def get_new_trajectory_id(self) -> int:
        self._next_trajectory_id = (self._next_trajectory_id % 0xFFFFFFFE) + 1
        return self._next_trajectory_id

    @staticmethod
    def quintic_smooth(s: float) -> float:
        """归一化五次多项式因子: s in [0, 1] -> 10*s^3 - 15*s^4 + 6*s^5."""
        s = max(0.0, min(1.0, s))
        return 10.0 * (s ** 3) - 15.0 * (s ** 4) + 6.0 * (s ** 5)

    def plan_joint_point_to_point(
        self,
        start_q: Tuple[float, float, float, float],
        end_q: Tuple[float, float, float, float],
        duration_ms: int = 3000,
        num_points: int = 20,
        start_time_offset_ms: int = 0,
        start_point_id: int = 0,
        gripper_action: int = GripperAction.HOLD,
    ) -> List[TrajectoryPoint]:
        """关节空间点到点平滑规划 (PTP，五次多项式 S-Curve)."""
        points: List[TrajectoryPoint] = []
        if num_points < 2:
            num_points = 2

        for i in range(num_points):
            ratio = i / (num_points - 1)
            smooth_s = self.quintic_smooth(ratio)

            q1 = start_q[0] + (end_q[0] - start_q[0]) * smooth_s
            q2 = start_q[1] + (end_q[1] - start_q[1]) * smooth_s
            q3 = start_q[2] + (end_q[2] - start_q[2]) * smooth_s
            q4 = start_q[3] + (end_q[3] - start_q[3]) * smooth_s

            t = start_time_offset_ms + int(duration_ms * ratio)
            action = gripper_action if i == num_points - 1 else GripperAction.HOLD

            points.append(
                TrajectoryPoint(
                    point_id=start_point_id + i,
                    joint_1=round(q1, 4),
                    joint_2=round(q2, 4),
                    joint_3=round(q3, 4),
                    joint_4=round(q4, 4),
                    time_from_start=t,
                    gripper_action=action,
                )
            )
        return points

    def plan_cartesian_linear_path(
        self,
        start_xyz: Tuple[float, float, float],
        target_xyz: Tuple[float, float, float],
        start_pitch: float = 0.0,
        target_pitch: float = 0.0,
        duration_ms: int = 2500,
        step_ms: int = 50,
        transformer: Optional[Any] = None,
        start_time_offset_ms: int = 0,
        start_point_id: int = 0,
        gripper_action: int = GripperAction.HOLD,
    ) -> Tuple[bool, List[TrajectoryPoint], str]:
        """笛卡尔空间严格三维直线轨迹规划 (LIN).

        在 3D 空间以 step_ms 时间片切分，沿线逐点逆解并由 transformer 校验物理限位。
        """
        num_points = max(2, duration_ms // step_ms)
        points: List[TrajectoryPoint] = []

        for i in range(num_points):
            ratio = i / (num_points - 1)
            smooth_s = self.quintic_smooth(ratio)

            x = start_xyz[0] + (target_xyz[0] - start_xyz[0]) * smooth_s
            y = start_xyz[1] + (target_xyz[1] - start_xyz[1]) * smooth_s
            z = start_xyz[2] + (target_xyz[2] - start_xyz[2]) * smooth_s
            pitch = start_pitch + (target_pitch - start_pitch) * smooth_s

            ok, q_algo, msg = self.kinematics.inverse_kinematics(
                x, y, z, pitch=pitch, force_elbow_up=True
            )
            if not ok:
                return False, [], f"LIN failed at step {i} (x={x:.3f}, y={y:.3f}, z={z:.3f}): {msg}"

            if transformer:
                q_hw = transformer.algo_to_hardware(q_algo)
                lim_ok, lim_msg = transformer.check_hardware_limits(q_hw)
                if not lim_ok:
                    return False, [], f"LIN limit exceeded at step {i}: {lim_msg}"
                q_final = q_hw
            else:
                q_final = q_algo

            t = start_time_offset_ms + int(duration_ms * ratio)
            action = gripper_action if i == num_points - 1 else GripperAction.HOLD

            points.append(
                TrajectoryPoint(
                    point_id=start_point_id + i,
                    joint_1=round(q_final[0], 4),
                    joint_2=round(q_final[1], 4),
                    joint_3=round(q_final[2], 4),
                    joint_4=round(q_final[3], 4),
                    time_from_start=t,
                    gripper_action=action,
                )
            )

        return True, points, "Cartesian linear path planned successfully"

    def plan_multi_waypoint_sequence(
        self,
        current_q_hw: Tuple[float, float, float, float],
        waypoints: List[Dict[str, Any]],
        transformer: Any,
        coordinate_frame: str = "cartesian",
        step_ms: int = 50,
        default_dwell_ms: int = 500,
    ) -> Tuple[bool, List[TrajectoryPoint], str]:
        """多航路点工序时间片无缝拼接 (支持双点驻留法 Dwell Point 实现到位后夹取).

        :param current_q_hw: 实车当前物理硬件角度 (弧度)
        :param waypoints: 预设航路点列表
        :param transformer: JointTransformer 实例
        :param coordinate_frame: "cartesian" (末端空间坐标) 或 "joints" (硬件角度)
        :param step_ms: 时间片步长 (默认 50ms)
        :param default_dwell_ms: 夹取到位后的原地驻留时间 (默认 500ms)
        """
        if not waypoints:
            return False, [], "Empty waypoints sequence"

        all_points: List[TrajectoryPoint] = []
        cur_q_hw = current_q_hw
        cur_time_ms = 0
        point_id = 0

        for idx, wp in enumerate(waypoints):
            duration_ms = int(wp.get("duration_ms", 2000))
            gripper = int(wp.get("gripper", GripperAction.HOLD))
            linear = bool(wp.get("linear", False))
            dwell_ms = int(wp.get("dwell_ms", default_dwell_ms if gripper != GripperAction.HOLD else 0))

            target = wp.get("target", [])
            if coordinate_frame == "cartesian":
                if len(target) < 3:
                    return False, [], f"Waypoint {idx} invalid cartesian target: {target}"
                x, y, z = float(target[0]), float(target[1]), float(target[2])
                pitch_deg = float(target[3]) if len(target) > 3 else 0.0
                pitch_rad = math.radians(pitch_deg)

                ok, q_algo, ik_msg = self.kinematics.inverse_kinematics(
                    x, y, z, pitch=pitch_rad, force_elbow_up=True
                )
                if not ok:
                    return False, [], f"Waypoint {idx} IK failed for {target}: {ik_msg}"

                target_q_hw = transformer.algo_to_hardware(q_algo)
                lim_ok, lim_msg = transformer.check_hardware_limits(target_q_hw)
                if not lim_ok:
                    return False, [], f"Waypoint {idx} limit exceeded: {lim_msg}"

            else:
                # joints 模式
                if len(target) < 4:
                    return False, [], f"Waypoint {idx} invalid joint target: {target}"
                target_q_hw = tuple(math.radians(float(d)) for d in target[:4])
                lim_ok, lim_msg = transformer.check_hardware_limits(target_q_hw)
                if not lim_ok:
                    return False, [], f"Waypoint {idx} joint limit exceeded: {lim_msg}"

            # --- 阶段 1: 移动段 (强制 gripper=HOLD，到达前绝不开合) ---
            num_pts = max(2, duration_ms // step_ms)
            move_pts = self.plan_joint_point_to_point(
                start_q=cur_q_hw,
                end_q=target_q_hw,
                duration_ms=duration_ms,
                num_points=num_pts,
                start_time_offset_ms=cur_time_ms,
                start_point_id=point_id,
                gripper_action=GripperAction.HOLD,
            )
            all_points.extend(move_pts)
            cur_time_ms += duration_ms
            point_id += len(move_pts)
            cur_q_hw = target_q_hw

            # --- 阶段 2: 夹取驻留段 (Dwell Point: 原地静止并执行夹紧/张开) ---
            if dwell_ms > 0 and gripper != GripperAction.HOLD:
                num_dwell_pts = max(1, dwell_ms // step_ms)
                for d_i in range(1, num_dwell_pts + 1):
                    d_t = cur_time_ms + int(dwell_ms * (d_i / num_dwell_pts))
                    all_points.append(
                        TrajectoryPoint(
                            point_id=point_id,
                            joint_1=round(cur_q_hw[0], 4),
                            joint_2=round(cur_q_hw[1], 4),
                            joint_3=round(cur_q_hw[2], 4),
                            joint_4=round(cur_q_hw[3], 4),
                            time_from_start=d_t,
                            gripper_action=gripper,
                        )
                    )
                    point_id += 1
                cur_time_ms += dwell_ms

        return True, all_points, "Multi-waypoint sequence planned successfully"
