"""Shared QoS and perception parameter calls for operational ROS clients."""

from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def set_perception_zone(node, client, zone, wait_until, endpoint_timeout=1.0,
                        response_timeout=1.0):
    """Set the authoritative zone and require a successful parameter response."""
    if zone not in ('A', 'B', 'C', 'D'):
        raise ValueError('Zone must be A, B, C or D')
    if not client.wait_for_service(timeout_sec=endpoint_timeout):
        node.get_logger().warning('Perception zone parameter service unavailable')
        return False
    request = SetParametersAtomically.Request()
    request.parameters = [Parameter('zone', value=zone).to_parameter_msg()]
    try:
        future = client.call_async(request)
        if not wait_until(future.done, response_timeout):
            future.cancel()
            node.get_logger().warning('Perception zone parameter response timed out')
            return False
        response = future.result()
        if response is None or not response.result.successful:
            reason = 'no response' if response is None else response.result.reason
            node.get_logger().warning(f'Perception zone parameter rejected: {reason}')
            return False
    except Exception as error:
        node.get_logger().warning(f'Perception zone parameter failed: {error}')
        return False
    return True


def reliable_qos(depth=10):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)


def latched_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
