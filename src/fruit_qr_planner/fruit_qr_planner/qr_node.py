"""ROS 2 image subscriber that decodes B/C QR codes into JSON pick plans."""

import json
from typing import Any, Dict, Iterable, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from fruit_qr_planner.planner import PlanSequence


def select_unique_payload(payloads: Iterable[str]) -> Optional[str]:
    candidates = {payload.strip() for payload in payloads if payload and payload.strip()}
    if len(candidates) != 1:
        return None
    return candidates.pop()


def encode_plan_json(plan: Dict[str, Any]) -> str:
    return json.dumps(plan, ensure_ascii=False, separators=(",", ":"))


def plan_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def image_to_bgr(message: Image, cv2_module: Any) -> Any:
    if message.encoding not in ("rgb8", "bgr8"):
        raise ValueError("QR input must use rgb8 or bgr8 encoding")
    row_bytes = message.width * 3
    expected_size = message.height * message.step
    if message.height <= 0 or message.width <= 0 or message.step < row_bytes:
        raise ValueError("QR input has invalid image dimensions or row step")

    buffer = np.frombuffer(message.data, dtype=np.uint8)
    if buffer.size < expected_size:
        raise ValueError("QR input data is shorter than height * step")
    rows = buffer[:expected_size].reshape(message.height, message.step)
    image = np.ascontiguousarray(rows[:, :row_bytes].reshape(message.height, message.width, 3))
    if message.encoding == "rgb8":
        return cv2_module.cvtColor(image, cv2_module.COLOR_RGB2BGR)
    return image


class FruitQrPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("fruit_qr_planner")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("confirm_frames", 3)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        confirm_frames = self.get_parameter("confirm_frames").get_parameter_value().integer_value
        if confirm_frames <= 0:
            raise ValueError("confirm_frames must be greater than zero")

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV Python bindings are required to process QR images") from exc

        try:
            from pyzbar.pyzbar import decode as decode_barcodes
        except ImportError as exc:
            raise RuntimeError(
                "python3-pyzbar and libzbar0 are required to decode QR images"
            ) from exc

        self._cv2 = cv2
        self._decode_barcodes = decode_barcodes
        self._sequence = PlanSequence(confirm_frames=confirm_frames)
        self._b_publisher = self.create_publisher(String, "/fruit/b_pick_plan", plan_qos())
        self._c_publisher = self.create_publisher(String, "/fruit/c_pick_plan", plan_qos())
        self._subscription = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Waiting for B-zone QR code on {image_topic}; confirm_frames={confirm_frames}"
        )

    def _on_image(self, message: Image) -> None:
        if self._sequence.state == PlanSequence.COMPLETED:
            return

        try:
            image = image_to_bgr(message, self._cv2)
            payload = self._decode(image)
            plan, parse_error = self._sequence.observe_with_error(payload or "")
        except Exception as exc:
            self.get_logger().warning(
                f"QR image processing failed: {exc}", throttle_duration_sec=2.0
            )
            return

        if payload and parse_error:
            self.get_logger().warning(
                f"Rejected {self._sequence.state} QR payload: {parse_error}",
                throttle_duration_sec=2.0,
            )
        if plan is None:
            return

        output = String()
        output.data = encode_plan_json(plan)
        if plan["zone"] == "B":
            self._b_publisher.publish(output)
            self.get_logger().info("Published B-zone pick plan with 8 tasks; now waiting for C")
        else:
            self._c_publisher.publish(output)
            self.get_logger().info(
                "Published C-zone pick plan with 8 tasks; QR sequence completed"
            )
            self._stop_scanning()

    def _stop_scanning(self) -> None:
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None

    def _decode(self, image: Any) -> Optional[str]:
        payloads = []
        for barcode in self._decode_barcodes(image):
            if barcode.type != "QRCODE":
                continue
            try:
                payloads.append(barcode.data.decode("utf-8"))
            except UnicodeDecodeError as exc:
                raise ValueError("QR payload is not valid UTF-8") from exc
        return select_unique_payload(payloads)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FruitQrPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
