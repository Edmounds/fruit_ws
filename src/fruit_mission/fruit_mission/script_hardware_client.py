"""Typed ROS helpers for one-shot operational script commands."""

from __future__ import annotations

from enum import IntEnum
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool

from fruit_hardware_interfaces.msg import ExecutionCallback


class CommandResult(IntEnum):
    """Result of one lower-controller business command."""

    COMPLETED = 0
    FAILED = 3
    CALLBACK_TIMEOUT = 4
    BRIDGE_NOT_READY = 5
    ENDPOINTS_NOT_READY = 6


def reliable_qos(depth: int = 10) -> QoSProfile:
    """Return the reliable volatile QoS used by hardware business topics."""
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)


def latched_qos() -> QoSProfile:
    """Return the reliable transient-local QoS used by state topics."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class HardwareCommandClient(Node):
    """Wait for the serial bridge and correlate typed execution callbacks."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.declare_parameter(
            "bridge_ready_topic", "/auto_serial_bridge/ready"
        )
        self.declare_parameter(
            "callback_topic", "/fruit_hardware_bridge/execution/callback"
        )
        self.declare_parameter("bridge_ready_timeout_sec", 30.0)
        self.declare_parameter("endpoint_timeout_sec", 10.0)
        self.declare_parameter("callback_timeout_sec", 30.0)

        self.bridge_ready_topic = str(
            self.get_parameter("bridge_ready_topic").value
        )
        self.callback_topic = str(self.get_parameter("callback_topic").value)
        self.bridge_ready_timeout_sec = float(
            self.get_parameter("bridge_ready_timeout_sec").value
        )
        self.endpoint_timeout_sec = float(
            self.get_parameter("endpoint_timeout_sec").value
        )
        self.callback_timeout_sec = float(
            self.get_parameter("callback_timeout_sec").value
        )

        self._bridge_ready = False
        self._pending_callback_id: int | None = None
        self._callback_status: int | None = None
        self._ready_subscription = self.create_subscription(
            Bool,
            self.bridge_ready_topic,
            self._on_bridge_ready,
            latched_qos(),
        )
        self._callback_subscription = self.create_subscription(
            ExecutionCallback,
            self.callback_topic,
            self._on_callback,
            reliable_qos(),
        )

    def bridge_publisher_exists(self) -> bool:
        """Return whether a bridge-ready publisher exists in the ROS graph."""
        return self.count_publishers(self.bridge_ready_topic) > 0

    def wait_for_bridge_ready(self, timeout_sec: float | None = None) -> bool:
        """Wait for the bridge's typed transient-local ready state."""
        timeout = (
            self.bridge_ready_timeout_sec
            if timeout_sec is None
            else timeout_sec
        )
        return self.wait_until(lambda: self._bridge_ready, timeout)

    def wait_for_command_endpoints(
        self,
        outbound_topic: str,
        timeout_sec: float | None = None,
    ) -> bool:
        """Wait until both sides of a command/callback exchange are visible."""
        timeout = (
            self.endpoint_timeout_sec if timeout_sec is None else timeout_sec
        )
        return self.wait_until(
            lambda: (
                self.count_subscribers(outbound_topic) > 0
                and self.count_publishers(self.callback_topic) > 0
            ),
            timeout,
        )

    def publish_and_wait(
        self,
        publisher: Publisher,
        outbound_topic: str,
        message: object,
        callback_id: int,
        description: str,
    ) -> CommandResult:
        """Publish once and wait for the matching terminal callback."""
        if not self.wait_for_bridge_ready():
            self.get_logger().error(
                "Serial bridge did not report ready on "
                f"{self.bridge_ready_topic}"
            )
            return CommandResult.BRIDGE_NOT_READY
        if not self.wait_for_command_endpoints(outbound_topic):
            self.get_logger().error(
                "Hardware ROS endpoints are not ready: "
                f"{outbound_topic} subscribers="
                f"{self.count_subscribers(outbound_topic)}, "
                f"{self.callback_topic} publishers="
                f"{self.count_publishers(self.callback_topic)}"
            )
            return CommandResult.ENDPOINTS_NOT_READY

        self._pending_callback_id = callback_id
        self._callback_status = None
        publisher.publish(message)
        matched = self.wait_until(
            lambda: self._callback_status is not None,
            self.callback_timeout_sec,
        )
        self._pending_callback_id = None
        if not matched:
            self.get_logger().error(
                f"Timed out waiting for {description} callback"
            )
            return CommandResult.CALLBACK_TIMEOUT
        if self._callback_status == ExecutionCallback.STATUS_COMPLETED:
            return CommandResult.COMPLETED

        self.get_logger().error(
            f"Lower controller reported failed {description} callback"
        )
        return CommandResult.FAILED

    def wait_until(
        self, predicate: Callable[[], bool], timeout_sec: float
    ) -> bool:
        """Spin until a predicate succeeds or a monotonic timeout expires."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok():
            if predicate():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return predicate()

    def _on_bridge_ready(self, message: Bool) -> None:
        self._bridge_ready = bool(message.data)

    def _on_callback(self, message: ExecutionCallback) -> None:
        if message.callback_id != self._pending_callback_id:
            return
        if message.callback_status not in (
            ExecutionCallback.STATUS_COMPLETED,
            ExecutionCallback.STATUS_FAILED,
        ):
            return
        self._callback_status = int(message.callback_status)
