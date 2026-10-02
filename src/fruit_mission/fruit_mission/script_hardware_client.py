"""QoS shared by operational ROS clients; no legacy task/callback transport."""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def reliable_qos(depth=10):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)


def latched_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
