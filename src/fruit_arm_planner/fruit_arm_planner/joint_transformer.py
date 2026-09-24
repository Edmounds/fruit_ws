"""Joint coordinate transformer between algorithm space and hardware space."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class JointChannelConfig:
    """单个关节的映射配置参数."""
    name: str = "q"
    offset_rad: float = 0.0          # 零位偏置 (rad)
    direction: float = 1.0           # 方向系数 (+1.0 或 -1.0)
    min_limit_rad: float = -math.pi  # 硬件最小限位 (rad)
    max_limit_rad: float = math.pi   # 硬件最大限位 (rad)

    def to_hardware(self, algo_val: float) -> float:
        """算法理论角 -> 下位机物理硬件角."""
        return self.direction * algo_val + self.offset_rad

    def to_algo(self, hw_val: float) -> float:
        """下位机物理硬件角 -> 算法理论角."""
        return (hw_val - self.offset_rad) / self.direction

    def is_within_limits(self, hw_val: float, tolerance_rad: float = 1e-4) -> bool:
        """校验硬件角是否在安全限位范围内."""
        return (self.min_limit_rad - tolerance_rad) <= hw_val <= (self.max_limit_rad + tolerance_rad)


class JointTransformer:
    """
    关节坐标系转换器 (Joint Coordinate Transformer)
    负责算法理论空间 (全部伸直为0) 与下位机物理硬件空间 (舵机/电机零点) 之间的双向映射与限位校验。
    Q3 硬件角对应小臂绝对俯仰角，映射时需包含算法 Q2。
    """

    def __init__(self, channels: Optional[List[JointChannelConfig]] = None):
        if channels is None or len(channels) != 4:
            # 默认直通配置
            self.channels = [
                JointChannelConfig(name="q1", offset_rad=0.0, direction=1.0, min_limit_rad=-math.pi, max_limit_rad=math.pi),
                JointChannelConfig(name="q2", offset_rad=0.0, direction=1.0, min_limit_rad=math.radians(3.0), max_limit_rad=math.pi),
                JointChannelConfig(name="q3", offset_rad=0.0, direction=1.0, min_limit_rad=math.radians(-190.0), max_limit_rad=math.radians(190.0)),
                JointChannelConfig(name="q4", offset_rad=0.0, direction=1.0, min_limit_rad=math.radians(-90.0), max_limit_rad=math.radians(92.4)),
            ]
        else:
            self.channels = channels

    @classmethod
    def from_config_dict(cls, mapping_dict: Dict[str, dict]) -> JointTransformer:
        """从 YAML 字典结构加载关节映射配置 (输入单位为 deg, 内部自动转为 rad)."""
        joint_keys = ["q1", "q2", "q3", "q4"]
        default_limits = [
            (-180.0, 180.0),
            (3.0, 180.0),
            (-190.0, 190.0),
            (-90.0, 92.4),
        ]

        channels = []
        for i, key in enumerate(joint_keys):
            cfg = mapping_dict.get(key, {})
            offset_deg = float(cfg.get("offset_deg", 0.0))
            direction = 1.0 if float(cfg.get("direction", 1.0)) >= 0 else -1.0

            lim_deg = cfg.get("limits_deg", default_limits[i])
            min_deg = float(lim_deg[0])
            max_deg = float(lim_deg[1])

            channel = JointChannelConfig(
                name=key,
                offset_rad=math.radians(offset_deg),
                direction=direction,
                min_limit_rad=math.radians(min_deg),
                max_limit_rad=math.radians(max_deg),
            )
            channels.append(channel)

        return cls(channels)

    def algo_to_hardware(
        self, q_algo: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float, float]:
        """将 4 关节算法理论角转为下位机物理硬件角 (rad)."""
        return (
            self.channels[0].to_hardware(q_algo[0]),
            self.channels[1].to_hardware(q_algo[1]),
            self.channels[2].to_hardware(q_algo[1] + q_algo[2]),
            self.channels[3].to_hardware(q_algo[3]),
        )

    def hardware_to_algo(
        self, q_hw: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float, float]:
        """将 4 关节下位机物理硬件角反推为算法理论角 (rad)."""
        q2_algo = self.channels[1].to_algo(q_hw[1])
        return (
            self.channels[0].to_algo(q_hw[0]),
            q2_algo,
            self.channels[2].to_algo(q_hw[2]) - q2_algo,
            self.channels[3].to_algo(q_hw[3]),
        )

    def check_hardware_limits(
        self, q_hw: Tuple[float, float, float, float]
    ) -> Tuple[bool, str]:
        """检查 4 关节物理硬件角是否在安全软限位范围内."""
        for i, channel in enumerate(self.channels):
            val = q_hw[i]
            if not channel.is_within_limits(val):
                val_deg = math.degrees(val)
                min_deg = math.degrees(channel.min_limit_rad)
                max_deg = math.degrees(channel.max_limit_rad)
                msg = (
                    f"Joint {channel.name} out of limits: {val_deg:.1f}° "
                    f"not in [{min_deg:.1f}°, {max_deg:.1f}°]"
                )
                return False, msg
        return True, "All joints within safe limits"
