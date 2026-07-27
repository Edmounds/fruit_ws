"""ROS 2 image subscriber that decodes B/C QR codes into JSON pick plans."""

import json
from typing import Any, Dict, Iterable, Optional

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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


class FruitQrPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("fruit_qr_planner")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("confirm_frames", 3)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        confirm_frames = self.get_parameter("confirm_frames").get_parameter_value().integer_value
        if confirm_frames <= 0:
            raise ValueError("confirm_frames must be greater than zero")

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV Python bindings are required to decode QR images") from exc

        self._bridge = CvBridge()
        self._detector = cv2.QRCodeDetector()
        self._sequence = PlanSequence(confirm_frames=confirm_frames)
        self._b_publisher = self.create_publisher(String, "/fruit/b_pick_plan", 1)
        self._c_publisher = self.create_publisher(String, "/fruit/c_pick_plan", 1)
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
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            payload = self._decode(image)
            plan, parse_error = self._sequence.observe_with_error(payload or "")
        except Exception as exc:
            self.get_logger().warning(f"QR image processing failed: {exc}", throttle_duration_sec=2.0)
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
            self.get_logger().info("Published C-zone pick plan with 8 tasks; QR sequence completed")

    def _decode(self, image: Any) -> Optional[str]:
        try:
            detected, decoded_info, _points, _straight = self._detector.detectAndDecodeMulti(image)
        except (AttributeError, TypeError):
            detected = False
            decoded_info = []

        if detected:
            payload = select_unique_payload(decoded_info)
            if payload is not None or sum(bool(item and item.strip()) for item in decoded_info) > 1:
                return payload

        decoded, _points, _straight = self._detector.detectAndDecode(image)
        return decoded.strip() if decoded and decoded.strip() else None


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
