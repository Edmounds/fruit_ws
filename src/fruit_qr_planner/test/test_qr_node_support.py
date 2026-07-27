import json
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
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy.node)
    sys.modules.setdefault("rclpy.qos", rclpy.qos)

    cv_bridge = types.ModuleType("cv_bridge")

    class CvBridge:
        pass

    cv_bridge.CvBridge = CvBridge
    sys.modules.setdefault("cv_bridge", cv_bridge)

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

from fruit_qr_planner.qr_node import encode_plan_json, select_unique_payload  # noqa: E402
from fruit_qr_planner.qr_node import FruitQrPlannerNode  # noqa: E402


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

    def test_decode_supports_single_qr_detector(self):
        node = FruitQrPlannerNode.__new__(FruitQrPlannerNode)

        class Detector:
            def detectAndDecodeMulti(self, _image):
                return False, (), None, ()

            def detectAndDecode(self, _image):
                return " 苹果 ", None, None

        node._detector = Detector()

        self.assertEqual(node._decode(object()), "苹果")

    def test_decode_rejects_multiple_different_codes(self):
        node = FruitQrPlannerNode.__new__(FruitQrPlannerNode)

        class Detector:
            def detectAndDecodeMulti(self, _image):
                return True, ("苹果", "梨子"), None, ()

        node._detector = Detector()

        self.assertIsNone(node._decode(object()))

    def test_decode_falls_back_when_multi_detect_has_no_decoded_text(self):
        node = FruitQrPlannerNode.__new__(FruitQrPlannerNode)

        class Detector:
            def detectAndDecodeMulti(self, _image):
                return True, ("",), None, ()

            def detectAndDecode(self, _image):
                return "苹果", None, None

        node._detector = Detector()

        self.assertEqual(node._decode(object()), "苹果")


if __name__ == "__main__":
    unittest.main()
