"""4-DOF manipulator forward and inverse kinematics."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple


class RobotKinematics:
    """
    4自由度机械臂纯几何运动学求解器 (4-DOF Robot Arm Kinematics)
    构型：偏航基座(Yaw) + 肩部俯仰(Pitch) + 肘部俯仰(Pitch) + 腕部俯仰(Pitch)
    基准状态：各关节为 0 时，机械臂各连杆沿 X 轴水平伸直。
    """

    def __init__(
        self,
        d1: float = 0.14,  # 基座高度 (m)
        a2: float = 0.16,  # 大臂长度 (m)
        a3: float = 0.16,  # 小臂长度 (m)
        a4: float = 0.08,  # 末端/夹爪长度 (m)
        joint_limits: Optional[Dict[str, Tuple[float, float]]] = None,
    ):
        self.d1 = d1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4
        self.joint_limits = joint_limits

    @classmethod
    def from_geometry_dict(cls, geo_dict: Dict[str, Any]) -> RobotKinematics:
        """从几何参数字典创建运动学求解器."""
        return cls(
            d1=float(geo_dict.get("d1", 0.14)),
            a2=float(geo_dict.get("a2", 0.16)),
            a3=float(geo_dict.get("a3", 0.16)),
            a4=float(geo_dict.get("a4", 0.08)),
        )

    def forward_kinematics(
        self, q1: float, q2: float, q3: float, q4: float
    ) -> Dict[str, Any]:
        """正运动学 (FK)：给定算法理论关节角计算各关节空间坐标及末端位姿 (m, rad)."""
        p0 = (0.0, 0.0, 0.0)
        p1 = (0.0, 0.0, self.d1)

        # 2D 旋转平面内坐标 (r, z)
        theta2 = q2
        r2 = self.a2 * math.cos(theta2)
        z2 = self.d1 + self.a2 * math.sin(theta2)
        p2 = (r2 * math.cos(q1), r2 * math.sin(q1), z2)

        theta23 = q2 + q3
        r3 = r2 + self.a3 * math.cos(theta23)
        z3 = z2 + self.a3 * math.sin(theta23)
        p3 = (r3 * math.cos(q1), r3 * math.sin(q1), z3)

        pitch = q2 + q3 + q4
        r4 = r3 + self.a4 * math.cos(pitch)
        z4 = z3 + self.a4 * math.sin(pitch)
        p4 = (r4 * math.cos(q1), r4 * math.sin(q1), z4)

        return {
            "end_effector": (p4[0], p4[1], p4[2], pitch),
            "points": [p0, p1, p2, p3, p4],
        }

    def inverse_kinematics(
        self,
        x: float,
        y: float,
        z: float,
        pitch: Optional[float] = None,
        pitch_deg: Optional[float] = None,
        elbow_up: bool = True,
        force_elbow_up: bool = True,
    ) -> Tuple[bool, Tuple[float, float, float, float], str]:
        """逆运动学 (IK)：根据末端坐标 (x, y, z) 与期望末端俯仰角求解理论关节角 (q1, q2, q3, q4).

        :param x, y, z: 末端目标坐标 (单位: m)
        :param pitch: 期望末端俯仰角 (弧度，水平为0，向下为负，向上为正)
        :param pitch_deg: 期望末端俯仰角 (角度制，若指定则优先使用)
        :param elbow_up: 是否为肘关节高位构型 (Elbow-Up, 默认 True)
        :param force_elbow_up: 是否强制只使用高位构型 (默认 True)
        :return: (成功标志, (q1, q2, q3, q4), 消息描述)
        """
        if pitch_deg is not None:
            pitch = math.radians(pitch_deg)

        # 1. 偏航角 q1 (基座水平偏转)
        q1 = math.atan2(y, x)
        r = math.sqrt(x * x + y * y)

        if r < 1e-4:
            q1 = 0.0

        is_specified_pitch = pitch is not None
        candidate_pitches = [pitch] if is_specified_pitch else [
            0.0, -math.pi / 12, -math.pi / 6, -math.pi / 4, -math.pi / 3, -math.pi / 2, math.pi / 12, math.pi / 6
        ]

        elbow_modes = [True] if force_elbow_up else ([elbow_up] if elbow_up is not None else [True, False])

        for target_pitch in candidate_pitches:
            # 2. 腕关节在 (r, z) 平面内的位置 (rw, zw)
            rw = r - self.a4 * math.cos(target_pitch)
            zw = (z - self.d1) - self.a4 * math.sin(target_pitch)

            # 肩-腕直线距离 d
            d_sq = rw * rw + zw * zw
            d = math.sqrt(d_sq)

            # 三角形几何可构成性校验 (工作空间范围检验)
            if d > (self.a2 + self.a3) or d < abs(self.a2 - self.a3) or d < 1e-4:
                continue

            # 肩-腕连线仰角 alpha
            alpha = math.atan2(zw, rw)

            # 余弦定理计算大臂小臂内角
            cos_beta = (self.a2 * self.a2 + d_sq - self.a3 * self.a3) / (2 * self.a2 * d)
            cos_beta = max(-1.0, min(1.0, cos_beta))
            beta = math.acos(cos_beta)

            cos_gamma = (self.a2 * self.a2 + self.a3 * self.a3 - d_sq) / (2 * self.a2 * self.a3)
            cos_gamma = max(-1.0, min(1.0, cos_gamma))
            gamma = math.acos(cos_gamma)

            for mode in elbow_modes:
                if mode:
                    # Elbow-Up (高位构型)
                    q2 = alpha + beta
                    q3 = -(math.pi - gamma)
                else:
                    # Elbow-Down (低位构型)
                    q2 = alpha - beta
                    q3 = (math.pi - gamma)

                # 腕关节俯仰角
                q4 = target_pitch - (q2 + q3)

                # 规范化到 [-pi, pi]
                q1_norm = (q1 + math.pi) % (2 * math.pi) - math.pi
                q2_norm = (q2 + math.pi) % (2 * math.pi) - math.pi
                q3_norm = (q3 + math.pi) % (2 * math.pi) - math.pi
                q4_norm = (q4 + math.pi) % (2 * math.pi) - math.pi

                # 若指定了理论限位则校验；若无则通过，交由 JointTransformer 校验硬件限位
                if self.joint_limits:
                    if not (
                        self.joint_limits["q1"][0] <= q1_norm <= self.joint_limits["q1"][1] and
                        self.joint_limits["q2"][0] <= q2_norm <= self.joint_limits["q2"][1] and
                        self.joint_limits["q3"][0] <= q3_norm <= self.joint_limits["q3"][1] and
                        self.joint_limits["q4"][0] <= q4_norm <= self.joint_limits["q4"][1]
                    ):
                        continue

                pitch_str = f"pitch={math.degrees(target_pitch):.1f}°"
                return True, (q1_norm, q2_norm, q3_norm, q4_norm), f"求解成功 ({pitch_str})"

        return False, (0.0, 0.0, 0.0, 0.0), "目标超出工作空间可达范围"
