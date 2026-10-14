"""Continuous camera preview with independent, strictly same-frame overlays."""

import json
import math

import cv2
from fruit_perception.msg import FruitDetections
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from fruit_robviz_bridge.video_frames import validate_and_extract_color


def stamp_key(header):
    return header.stamp.sec, header.stamp.nanosec


def matches_image(header, image_header, frame_id=None):
    """Unknown timestamps never establish a valid overlay association."""
    return (
        bool(image_header.frame_id)
        and header.stamp.sec >= 0
        and stamp_key(header) != (0, 0)
        and stamp_key(header) == stamp_key(image_header)
        and header.frame_id == (frame_id or image_header.frame_id)
    )


def clipped_roi(roi, width, height):
    """Clip integer pixel rectangles without permitting negative-size boxes."""
    if len(roi) != 4 or any(type(v) is not int for v in roi):
        raise ValueError("ROI must contain four integers")
    x, y, w, h = roi
    if w <= 0 or h <= 0:
        raise ValueError("ROI dimensions must be positive")
    left, top = max(0, x), max(0, y)
    right, bottom = min(width, x + w), min(height, y + h)
    if right <= left or bottom <= top:
        raise ValueError("ROI does not intersect the image")
    return left, top, right - 1, bottom - 1


def parse_qr_result(data):
    """Validate the QR preview protocol independently of task-plan parsing."""
    if len(data) > 512 * 1024:
        raise ValueError("QR preview exceeds size limit")
    result = json.loads(data)
    if not isinstance(result, dict):
        raise ValueError("QR preview must be an object")
    sec, nsec = result.get("stamp_sec"), result.get("stamp_nanosec")
    if type(sec) is not int or sec < 0 or type(nsec) is not int or not 0 <= nsec < 10**9:
        raise ValueError("Invalid QR image timestamp")
    if not isinstance(result.get("frame_id"), str) or not result["frame_id"]:
        raise ValueError("Missing QR image frame_id")
    detections = result.get("detections")
    if not isinstance(detections, list) or len(detections) > 64:
        raise ValueError("Invalid QR detection list")
    for item in detections:
        if not isinstance(item, dict) or not isinstance(item.get("text"), str):
            raise ValueError("Invalid QR detection")
        if len(item["text"]) > 4096:
            raise ValueError("QR text exceeds size limit")
        roi = item.get("roi")
        if not isinstance(roi, list) or len(roi) != 4 or any(type(v) is not int for v in roi):
            raise ValueError("Invalid QR rectangle")
        if roi[2] <= 0 or roi[3] <= 0:
            raise ValueError("Invalid QR rectangle dimensions")
    return result


def _roi(detection):
    roi = detection.roi
    return roi.x_offset, roi.y_offset, roi.width, roi.height


def _point_text(label, detection):
    point = detection.point
    values = (point.x, point.y, point.z)
    if (not detection.has_point or not all(math.isfinite(v) for v in values)
            or (label == "camera" and point.z <= 0.0)):
        return f"{label}: unavailable"
    return f"{label}: ({point.x:.3f}, {point.y:.3f}, {point.z:.3f}) m"


def draw_fruit(image, detections, base_detections):
    """Missing base coordinates do not suppress camera detections."""
    height, width = image.shape[:2]
    base = {(item.type, _roi(item)): item for item in base_detections}
    for item in detections:
        if not math.isfinite(item.confidence) or not 0 <= item.confidence <= 1:
            continue
        try:
            x1, y1, x2, y2 = clipped_roi(_roi(item), width, height)
        except ValueError:
            continue
        color = (0, 220, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        labels = [f"{item.type[:80]} {item.confidence:.2f}", _point_text("camera", item)]
        arm = base.get((item.type, _roi(item)))
        labels.append(_point_text("arm_base_link", arm) if arm else "arm_base_link: unavailable")
        for index, label in enumerate(labels):
            cv2.putText(image, label, (x1, min(height - 1, max(16, y1) + index * 17)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)


def draw_qr(image, detections, font=None):
    height, width = image.shape[:2]
    for item in detections:
        try:
            x1, y1, x2, y2 = clipped_roi(item["roi"], width, height)
        except ValueError:
            continue
        color = (0, 255, 80)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        text = f"QR: {item['text'].replace(chr(10), ' | ')[:80] or 'detected'}"
        position = (x1, max(16, y1 - 6))
        if font is not None:
            font.putText(image, text, position, 16, color, -1, cv2.LINE_AA, True)
        else:
            cv2.putText(image, text.encode("ascii", "replace").decode(), position,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


class ImageFusionNode(Node):
    """Publish every incoming camera frame without waiting for results."""

    def __init__(self, image_topic, output_topic, arm_frame_id, use_sim_time=False):
        # The gateway's launch __node remap must not rename this second node.
        super().__init__(
            "fruit_robviz_image_fusion", use_global_arguments=False,
            parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
        )
        self._fruit = None
        self._base = None
        self._qr = None
        self._arm_frame = arm_frame_id
        self._qr_font = None
        try:
            font = cv2.freetype.createFreeType2()
            font.loadFontData("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 0)
            self._qr_font = font
        except (AttributeError, cv2.error) as error:
            self.get_logger().warning(f"Chinese QR font unavailable: {error}")
        self._publisher = self.create_publisher(Image, output_topic, qos_profile_sensor_data)
        self._subscriptions = [
            self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data),
            self.create_subscription(
                FruitDetections, "/fruit_perception/camera/preview_detections",
                lambda msg: setattr(self, "_fruit", msg), qos_profile_sensor_data,
            ),
            self.create_subscription(
                FruitDetections, "/fruit_perception/camera/base_preview_detections",
                lambda msg: setattr(self, "_base", msg), qos_profile_sensor_data,
            ),
            self.create_subscription(
                String, "/fruit_qr_planner/camera/preview_detections",
                self._on_qr, qos_profile_sensor_data,
            ),
        ]
        self.get_logger().info(f"Continuous fusion preview: {image_topic} -> {output_topic}")

    def _on_qr(self, message):
        try:
            self._qr = parse_qr_result(message.data)
        except (ValueError, TypeError) as error:
            self._qr = None
            self.get_logger().warning(f"QR preview rejected: {error}", throttle_duration_sec=2.0)

    def _on_image(self, message):
        fruit = self._fruit
        if fruit and not matches_image(fruit.header, message.header):
            fruit = None
        qr = self._qr
        if qr and (
            stamp_key(message.header) == (0, 0)
            or (qr["stamp_sec"], qr["stamp_nanosec"]) != stamp_key(message.header)
            or qr["frame_id"] != message.header.frame_id
        ):
            qr = None
        if not fruit and not qr:
            self._publisher.publish(message)
            return
        try:
            _, bgr, _ = validate_and_extract_color(message)
            image = bgr.copy()
        except ValueError as error:
            self.get_logger().warning(
                f"Overlay input rejected: {error}", throttle_duration_sec=2.0
            )
            self._publisher.publish(message)
            return

        # Each overlay can fail independently. Never gate video on result completeness.
        if fruit:
            base = self._base
            base_items = base.detections if base and matches_image(
                base.header, message.header, self._arm_frame
            ) else []
            try:
                draw_fruit(image, fruit.detections, base_items)
            except Exception as error:
                self.get_logger().warning(
                    f"Fruit overlay rejected: {error}", throttle_duration_sec=2.0
                )
        if qr:
            try:
                draw_qr(image, qr["detections"], self._qr_font)
            except Exception as error:
                self.get_logger().warning(
                    f"QR overlay rejected: {error}", throttle_duration_sec=2.0
                )
        encoding = message.encoding.lower()
        if encoding != "bgr8":
            conversion = {
                "rgb8": cv2.COLOR_BGR2RGB,
                "rgba8": cv2.COLOR_BGR2RGBA,
                "bgra8": cv2.COLOR_BGR2BGRA,
            }[encoding]
            image = cv2.cvtColor(image, conversion)
        output = Image()
        output.header = message.header
        output.width, output.height = message.width, message.height
        # Keep the input color format so intermittent overlays do not renegotiate WebRTC.
        output.encoding = encoding
        output.step = message.width * image.shape[2]
        output.data = image.tobytes()
        self._publisher.publish(output)
