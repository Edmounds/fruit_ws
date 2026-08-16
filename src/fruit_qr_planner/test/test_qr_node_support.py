import json
import numpy as np
import sys
import types
import unittest


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.qos = types.ModuleType("rclpy.qos")

    class Node:
        pass

    rclpy.node.Node = Node
    rclpy.qos.qos_profile_sensor_data = object()

    class DurabilityPolicy:
        TRANSIENT_LOCAL = "transient_local"

    class HistoryPolicy:
        KEEP_LAST = "keep_last"

    class QoSProfile:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class ReliabilityPolicy:
        RELIABLE = "reliable"

    rclpy.qos.DurabilityPolicy = DurabilityPolicy
    rclpy.qos.HistoryPolicy = HistoryPolicy
    rclpy.qos.QoSProfile = QoSProfile
    rclpy.qos.ReliabilityPolicy = ReliabilityPolicy
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy.node)
    sys.modules.setdefault("rclpy.qos", rclpy.qos)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")

    class Image:
        pass

    sensor_msgs.msg.Image = Image
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs.msg)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self):
            self.data = ""

    std_msgs.msg.String = String
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs.msg)


_install_ros_stubs()

from fruit_qr_planner.qr_node import (  # noqa: E402
    encode_plan_json,
    image_to_bgr,
    plan_qos,
    select_unique_payload,
)
from fruit_qr_planner.qr_node import FruitQrPlannerNode  # noqa: E402


def _decoded_barcode(payload, barcode_type="QRCODE"):
    return types.SimpleNamespace(type=barcode_type, data=payload.encode("utf-8"))


class QrNodeSupportTest(unittest.TestCase):
    def test_selects_one_nonempty_payload(self):
        self.assertEqual(select_unique_payload(["", " 苹果 ", ""]), "苹果")

    def test_rejects_ambiguous_payloads(self):
        self.assertIsNone(select_unique_payload(["苹果", "梨子"]))

    def test_duplicate_payloads_are_not_ambiguous(self):
        self.assertEqual(select_unique_payload(["苹果", "苹果"]), "苹果")

    def test_json_encoding_keeps_chinese_text(self):
        encoded = encode_plan_json({"zone": "B", "target": "苹果"})
        self.assertEqual(json.loads(encoded)["target"], "苹果")
        self.assertIn("苹果", encoded)

    def test_pick_plans_are_latched_for_late_subscribers(self):
        qos = plan_qos()

        self.assertEqual(qos.depth, 1)
        self.assertEqual(qos.reliability, "reliable")
        self.assertEqual(qos.durability, "transient_local")

    def test_completion_destroys_image_subscription(self):
        node = FruitQrPlannerNode.__new__(FruitQrPlannerNode)
        subscription = object()
        node._subscription = subscription
        destroyed = []
        node.destroy_subscription = destroyed.append

        node._stop_scanning()

        self.assertEqual(destroyed, [subscription])
        self.assertIsNone(node._subscription)

    def test_converts_padded_rgb8_image_without_cv_bridge(self):
        message = types.SimpleNamespace(
            encoding="rgb8",
            height=1,
            width=2,
            step=8,
            data=bytes([1, 2, 3, 4, 5, 6, 99, 99]),
        )

        class Cv2:
            COLOR_RGB2BGR = 1

            @staticmethod
            def cvtColor(image, _conversion):
                return image[:, :, ::-1].copy()

        image = image_to_bgr(message, Cv2)

        np.testing.assert_array_equal(
            image,
            np.array([[[3, 2, 1], [6, 5, 4]]], dtype=np.uint8),
        )

    def test_rejects_unexpected_image_encoding(self):
        message = types.SimpleNamespace(
            encoding="mono8", height=1, width=1, step=1, data=b"\x00"
        )

        with self.assertRaisesRegex(ValueError, "rgb8 or bgr8"):
            image_to_bgr(message, object())

    def test_decode_supports_single_qr_decoder(self):
        node = FruitQrPlannerNode.__new__(FruitQrPlannerNode)

        class Decoder:
            def __call__(self, _image):
                return [_decoded_barcode(" 苹果 ")]

        node._decode_barcodes = Decoder()

        self.assertEqual(node._decode(object()), "苹果")

    def test_decode_rejects_multiple_different_codes(self):
        node = FruitQrPlannerNode.__new__(FruitQrPlannerNode)

        class Decoder:
            def __call__(self, _image):
                return [_decoded_barcode("苹果"), _decoded_barcode("梨子")]

        node._decode_barcodes = Decoder()

        self.assertIsNone(node._decode(object()))

    def test_decode_ignores_non_qr_barcodes(self):
        node = FruitQrPlannerNode.__new__(FruitQrPlannerNode)

        class Decoder:
            def __call__(self, _image):
                return [_decoded_barcode("123456", "EAN13"), _decoded_barcode("苹果")]

        node._decode_barcodes = Decoder()

        self.assertEqual(node._decode(object()), "苹果")


if __name__ == "__main__":
    unittest.main()
