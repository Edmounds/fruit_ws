"""ROS 2 image subscriber that decodes B/C QR codes into JSON pick plans."""

import json
from typing import Any, Dict, Iterable, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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
    for payload in payloads:
        if payload and payload.strip():
            return payload.strip()
    return None


def encode_plan_json(plan: Dict[str, Any]) -> str:
    return json.dumps(plan, ensure_ascii=False, separators=(",", ":"))


def plan_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def observation_window_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _parse_observation_window_command(data: str) -> Optional[Dict[str, Any]]:
    if not data or not data.strip():
        return None
    try:
        parsed = json.loads(data)
    except Exception:
        return None
    if not isinstance(parsed, dict):
        return None
    window_id = parsed.get("window_id")
    valid = parsed.get("valid")
    start_sec = parsed.get("start_sec")
    start_nanosec = parsed.get("start_nanosec")
    area = parsed.get("area")
    purpose = parsed.get("purpose")
    expected_type = parsed.get("expected_type", "")

    if not isinstance(window_id, str) or not window_id.strip():
        return None
    if isinstance(valid, bool) or not isinstance(valid, int) or valid not in (0, 1):
        return None
    if isinstance(start_sec, bool) or not isinstance(start_sec, int) or start_sec < 0:
        return None
    if (
        isinstance(start_nanosec, bool)
        or not isinstance(start_nanosec, int)
        or start_nanosec < 0
        or start_nanosec >= 1_000_000_000
    ):
        return None
    if not isinstance(area, str) or area not in ("A", "B", "C"):
        return None
    if not isinstance(purpose, str) or purpose not in ("harvest", "qr"):
        return None
    if not isinstance(expected_type, str):
        return None

    return {
        "window_id": window_id.strip(),
        "valid": valid,
        "start_sec": start_sec,
        "start_nanosec": start_nanosec,
        "area": area,
        "purpose": purpose,
        "expected_type": expected_type,
    }


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
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "current_area_topic",
            "/fruit_mission/current_area",
        )
        self.declare_parameter(
            "observation_window_topic",
            "/fruit_mission/observation_window",
        )
        self.declare_parameter("confirm_frames", 1)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        current_area_topic = (
            self.get_parameter("current_area_topic")
            .get_parameter_value()
            .string_value
        )
        observation_window_topic = (
            self.get_parameter("observation_window_topic")
            .get_parameter_value()
            .string_value
        )
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
        self._current_window: Optional[Dict[str, Any]] = None
        self._window_completed = False
        self._b_publisher = self.create_publisher(String, "/fruit/b_pick_plan", plan_qos())
        self._c_publisher = self.create_publisher(String, "/fruit/c_pick_plan", plan_qos())
        self._subscription = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._area_subscription = self.create_subscription(
            String,
            current_area_topic,
            self._on_current_area,
            plan_qos(),
        )
        self._window_subscription = self.create_subscription(
            String,
            observation_window_topic,
            self._on_observation_window,
            observation_window_qos(),
        )
        self.get_logger().info(
            f"Waiting for B-zone QR code on {image_topic}; confirm_frames={confirm_frames}"
        )

    def _on_observation_window(self, message: String) -> None:
        cmd = _parse_observation_window_command(message.data)
        if cmd is None:
            self.get_logger().warning(
                f"Ignoring invalid observation window payload: {message.data}"
            )
            return

        window_id = cmd["window_id"]
        valid = cmd["valid"]
        area = cmd["area"]
        purpose = cmd["purpose"]

        if valid == 1:
            if purpose == "qr" and area in ("B", "C"):
                self._current_window = cmd
                self._window_completed = False
                if area == "B":
                    self._sequence.state = PlanSequence.WAITING_FOR_B
                    self._sequence._reset_candidate()
                else:
                    self._sequence.state = PlanSequence.WAITING_FOR_C
                    self._sequence._reset_candidate()
                self.get_logger().info(
                    f"Opened QR observation window '{window_id}' for area {area}"
                )
            else:
                if self._current_window is not None:
                    self._current_window = None
                    self._sequence._reset_candidate()
        else:
            if (
                self._current_window is not None
                and self._current_window["window_id"] == window_id
            ):
                self._current_window = None
                self._sequence._reset_candidate()
                self.get_logger().info(
                    f"Closed QR observation window '{window_id}'"
                )

    def _on_current_area(self, message: String) -> None:
        entered_c = message.data.strip().upper() == "C"
        if entered_c and self._sequence.expect_c():
            self.get_logger().info(
                "Mission entered C before a B QR plan was decoded; "
                "now waiting for C"
            )

    def _on_image(self, message: Image) -> None:
        if self._current_window is None or self._window_completed:
            return

        window = self._current_window
        msg_stamp = message.header.stamp
        if (msg_stamp.sec, msg_stamp.nanosec) < (window["start_sec"], window["start_nanosec"]):
            return

        target_area = window["area"]
        if target_area == "B" and self._sequence.state != PlanSequence.WAITING_FOR_B:
            self._sequence.state = PlanSequence.WAITING_FOR_B
            self._sequence._reset_candidate()
        elif target_area == "C" and self._sequence.state != PlanSequence.WAITING_FOR_C:
            self._sequence.state = PlanSequence.WAITING_FOR_C
            self._sequence._reset_candidate()

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

        plan["observation_window_id"] = window["window_id"]
        plan["image_stamp_sec"] = msg_stamp.sec
        plan["image_stamp_nanosec"] = msg_stamp.nanosec

        output = String()
        output.data = encode_plan_json(plan)
        if plan["zone"] == "B":
            self._b_publisher.publish(output)
            self._window_completed = True
            self.get_logger().info(
                f"Published B-zone pick plan with {plan['task_count']} tasks "
                f"(window={window['window_id']}); waiting for next window"
            )
        else:
            self._c_publisher.publish(output)
            self._window_completed = True
            self.get_logger().info(
                f"Published C-zone pick plan with {plan['task_count']} tasks "
                f"(window={window['window_id']})"
            )

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
