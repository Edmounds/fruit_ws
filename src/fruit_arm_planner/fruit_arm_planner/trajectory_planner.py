"""Trajectory Planner with S-Curve interpolation and multi-waypoint stitching."""

from __future__ import annotations

import math
import secrets
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
    is_via_point: int = 0  # 0=exact arrival, 1=via point

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

    def __init__(self, kinematics: RobotKinematics):
        self.kinematics = kinematics
        self._next_trajectory_id = secrets.randbelow(0xFFFFFFFE) + 1

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
        duration_ms: int,
        num_points: int,
        start_time_offset_ms: int = 0,
        start_point_id: int = 0,
        gripper_action: int = GripperAction.HOLD,
        target_is_via_point: int = 0,
    ) -> List[TrajectoryPoint]:
        """关节空间点到点平滑规划 (PTP，五次多项式 S-Curve)."""
        if duration_ms <= 0 or duration_ms > 0xFFFFFFFF or num_points > duration_ms + 1:
            raise ValueError('PTP duration must allow a distinct millisecond timestamp per point')
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
            is_via_point = (
                0 if i == 0 else
                target_is_via_point if i == num_points - 1 else 1
            )

            points.append(
                TrajectoryPoint(
                    point_id=start_point_id + i,
                    joint_1=q1,
                    joint_2=q2,
                    joint_3=q3,
                    joint_4=q4,
                    time_from_start=t,
                    gripper_action=action,
                    is_via_point=is_via_point,
                )
            )
        return points

    def plan_cartesian_linear_path(
        self,
        start_xyz: Tuple[float, float, float],
        target_xyz: Tuple[float, float, float],
        start_pitch: float,
        target_pitch: float,
        duration_ms: int,
        step_ms: int,
        transformer: Any,
        start_q_hw: Tuple[float, float, float, float],
        start_time_offset_ms: int = 0,
        start_point_id: int = 0,
        gripper_action: int = GripperAction.HOLD,
        target_is_via_point: int = 0,
    ) -> Tuple[bool, List[TrajectoryPoint], str]:
        """笛卡尔空间严格三维直线轨迹规划 (LIN).

        在 3D 空间以 step_ms 时间片切分，沿线逐点逆解并由 transformer 校验物理限位。
        """
        if duration_ms <= 0 or step_ms <= 0 or duration_ms > 0xFFFFFFFF:
            return False, [], 'LIN duration and step must be positive and fit uint32 milliseconds'
        num_points = max(2, min(duration_ms + 1, math.ceil(duration_ms / step_ms) + 1))
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

            if i == 0 and start_q_hw is not None:
                q_final = start_q_hw
            elif transformer:
                q_hw = transformer.algo_to_hardware(q_algo)
                lim_ok, lim_msg = transformer.check_hardware_limits(q_hw)
                if not lim_ok:
                    return False, [], f"LIN limit exceeded at step {i}: {lim_msg}"
                q_final = q_hw
            else:
                q_final = q_algo

            t = start_time_offset_ms + int(duration_ms * ratio)
            action = gripper_action if i == num_points - 1 else GripperAction.HOLD
            is_via_point = (
                0 if i == 0 else
                target_is_via_point if i == num_points - 1 else 1
            )

            points.append(
                TrajectoryPoint(
                    point_id=start_point_id + i,
                    joint_1=q_final[0],
                    joint_2=q_final[1],
                    joint_3=q_final[2],
                    joint_4=q_final[3],
                    time_from_start=t,
                    gripper_action=action,
                    is_via_point=is_via_point,
                )
            )

        return True, points, "Cartesian linear path planned successfully"

    def plan_multi_waypoint_sequence(
        self,
        current_q_hw: Tuple[float, float, float, float],
        waypoints: List[Dict[str, Any]],
        transformer: Any,
        coordinate_frame: str,
        step_ms: int,
        default_dwell_ms: int,
    ) -> Tuple[bool, List[TrajectoryPoint], str]:
        """多航路点工序时间片无缝拼接 (支持双点驻留法 Dwell Point 实现到位后夹取).

        :param current_q_hw: 实车当前物理硬件角度 (弧度)
        :param waypoints: 预设航路点列表
        :param transformer: JointTransformer 实例
        :param coordinate_frame: "cartesian" (末端空间坐标)、"link_angles" (连杆角) 或 "joints" (硬件角)
        :param step_ms: YAML 中配置的时间片步长
        :param default_dwell_ms: YAML 中配置的夹取到位驻留时间
        """
        if not waypoints:
            return False, [], "Empty waypoints sequence"
        if coordinate_frame not in ('cartesian', 'joints', 'link_angles') or step_ms <= 0:
            return False, [], 'Invalid coordinate frame or interpolation step'

        all_points: List[TrajectoryPoint] = []
        cur_q_hw = current_q_hw
        cur_time_ms = 0
        point_id = 0

        for idx, wp in enumerate(waypoints):
            duration_ms = int(wp["duration_ms"])
            gripper = int(wp.get("gripper", GripperAction.HOLD))
            linear = bool(wp.get("linear", False))
            dwell_ms = int(wp.get("dwell_ms", default_dwell_ms if gripper != GripperAction.HOLD else 0))
            is_via_point = int(bool(wp.get('is_via_point', False)))
            if duration_ms <= 0 or dwell_ms < 0 or gripper not in (0, 1, 2):
                return False, [], f'Waypoint {idx} has invalid duration, dwell or gripper action'
            if cur_time_ms + duration_ms + dwell_ms > 0xFFFFFFFF:
                return False, [], f'Waypoint {idx} exceeds uint32 trajectory time'

            target = wp.get("target", [])
            if coordinate_frame == "cartesian":
                if len(target) < 4:
                    return False, [], f"Waypoint {idx} invalid cartesian target: {target}"
                x, y, z = float(target[0]), float(target[1]), float(target[2])
                pitch_deg = float(target[3])
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
                # joints 为硬件角；link_angles 为竖直起算的大臂、绝对小臂及绝对末端角
                if len(target) < 4:
                    return False, [], f"Waypoint {idx} invalid joint target: {target}"
                target_q_hw = (transformer.link_angles_to_hardware(target[:4])
                               if coordinate_frame == 'link_angles'
                               else tuple(math.radians(float(d)) for d in target[:4]))
                lim_ok, lim_msg = transformer.check_hardware_limits(target_q_hw)
                if not lim_ok:
                    return False, [], f"Waypoint {idx} joint limit exceeded: {lim_msg}"

            if linear:
                if coordinate_frame != "cartesian":
                    return False, [], f"Waypoint {idx} linear path requires cartesian coordinates"
                q_algo_start = transformer.hardware_to_algo(cur_q_hw)
                fk_start = self.kinematics.forward_kinematics(*q_algo_start)
                ok, move_pts, msg = self.plan_cartesian_linear_path(
                    start_xyz=fk_start["end_effector"][:3],
                    target_xyz=(x, y, z),
                    start_pitch=fk_start["end_effector"][3],
                    target_pitch=pitch_rad,
                    duration_ms=duration_ms,
                    step_ms=step_ms,
                    transformer=transformer,
                    start_q_hw=cur_q_hw,
                    start_time_offset_ms=cur_time_ms,
                    gripper_action=GripperAction.HOLD,
                    target_is_via_point=is_via_point,
                )
                if not ok:
                    return False, [], f"Waypoint {idx}: {msg}"
            else:
                num_pts = max(2, min(duration_ms + 1, math.ceil(duration_ms / step_ms) + 1))
                move_pts = self.plan_joint_point_to_point(
                    start_q=cur_q_hw,
                    end_q=target_q_hw,
                    duration_ms=duration_ms,
                    num_points=num_pts,
                    start_time_offset_ms=cur_time_ms,
                    gripper_action=GripperAction.HOLD if dwell_ms > 0 else gripper,
                    target_is_via_point=is_via_point,
                )
            if dwell_ms == 0 and gripper != GripperAction.HOLD:
                move_pts[-1].gripper_action = gripper
            if all_points:
                move_pts = move_pts[1:]
            for move_pt in move_pts:
                move_pt.point_id = point_id
                point_id += 1
            all_points.extend(move_pts)
            cur_time_ms += duration_ms
            cur_q_hw = target_q_hw

            # --- 阶段 2: 夹取驻留段 (Dwell Point: 原地静止并执行夹紧/张开) ---
            if dwell_ms > 0 and gripper != GripperAction.HOLD:
                num_dwell_pts = max(1, math.ceil(dwell_ms / step_ms))
                for d_i in range(1, num_dwell_pts + 1):
                    d_t = cur_time_ms + int(dwell_ms * (d_i / num_dwell_pts))
                    all_points.append(
                        TrajectoryPoint(
                            point_id=point_id,
                            joint_1=cur_q_hw[0],
                            joint_2=cur_q_hw[1],
                            joint_3=cur_q_hw[2],
                            joint_4=cur_q_hw[3],
                            time_from_start=d_t,
                            gripper_action=gripper if d_i == 1 else GripperAction.HOLD,
                            is_via_point=is_via_point,
                        )
                    )
                    point_id += 1
                cur_time_ms += dwell_ms

        return True, all_points, "Multi-waypoint sequence planned successfully"
