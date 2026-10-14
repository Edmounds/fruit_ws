"""Image-header association, QR wire validation, and pixel rectangle math."""

import json
import time

import numpy as np
from pyzbar.pyzbar import decode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from fruit_qr_planner.qr_node import decode_qr_results, qr_result_json
from fruit_robviz_bridge.image_fusion import (
    clipped_roi,
    draw_fruit,
    draw_qr,
    ImageFusionNode,
    matches_image,
    parse_qr_result,
)
from fruit_perception.msg import FruitDetection, FruitDetections


def header(sec=1, nsec=2, frame="camera_color_optical_frame"):
    result = Header(frame_id=frame)
    result.stamp.sec, result.stamp.nanosec = sec, nsec
    return result


def test_exact_image_association_and_coordinate_frames():
    image = header()
    assert matches_image(header(), image)
    assert not matches_image(header(nsec=3), image)
    assert not matches_image(header(sec=0, nsec=0), header(sec=0, nsec=0))
    assert not matches_image(header(frame="other_camera"), image)
    assert matches_image(header(frame="arm_base_link"), image, "arm_base_link")
    assert not matches_image(header(frame="base_link"), image, "arm_base_link")


def test_rectangle_clipping():
    assert clipped_roi((-2, -3, 7, 8), 10, 10) == (0, 0, 4, 4)
    assert clipped_roi((8, 9, 5, 5), 10, 10) == (8, 9, 9, 9)


@pytest.mark.parametrize("roi", [(1, 1, 0, 2), (1, 1, -1, 2), (10, 0, 1, 1), (True, 0, 1, 1)])
def test_invalid_rectangles(roi):
    with pytest.raises(ValueError):
        clipped_roi(roi, 10, 10)


def test_qr_result_round_trip_preserves_image_stamp():
    image = Image(header=header())
    detections = [{"text": "苹果\n梨子", "roi": [10, 20, 30, 40]}]
    result = parse_qr_result(qr_result_json(image, detections))
    assert (result["stamp_sec"], result["stamp_nanosec"]) == (1, 2)
    assert result["detections"] == detections


@pytest.mark.parametrize("patch", [
    {"stamp_sec": True}, {"stamp_sec": -1}, {"stamp_nanosec": 10**9},
    {"frame_id": ""}, {"detections": None},
    {"detections": [{"text": "QR", "roi": [0, 0, -1, 1]}]},
    {"detections": [{"text": 123, "roi": [0, 0, 1, 1]}]},
])
def test_qr_wire_rejects_invalid_fields(patch):
    result = {"stamp_sec": 1, "stamp_nanosec": 2, "frame_id": "camera", "detections": []}
    result.update(patch)
    with pytest.raises(ValueError):
        parse_qr_result(json.dumps(result))


def test_blank_frame_has_an_independent_empty_qr_result():
    image = np.full((100, 100, 3), 255, dtype=np.uint8)
    payload, detections = decode_qr_results(image, decode)
    assert payload is None
    assert detections == []


def test_independent_rectangle_rendering_without_base_coordinates():
    fruit = FruitDetection(type="apple_ripe", confidence=0.9)
    fruit.roi.x_offset, fruit.roi.y_offset = 40, 40
    fruit.roi.width, fruit.roi.height = 20, 20
    qr = [{"text": "QR", "roi": [100, 40, 20, 20]}]
    for fruit_items, qr_items in [([fruit], []), ([], qr), ([fruit], qr), ([], [])]:
        image = np.zeros((100, 160, 3), dtype=np.uint8)
        draw_fruit(image, fruit_items, [])
        draw_qr(image, qr_items)
        assert bool(image[59, 50].any()) == bool(fruit_items)
        assert bool(image[59, 110].any()) == bool(qr_items)


def test_ros_image_protocol_keeps_sources_independent_and_color_format_stable():
    """Use public ROS communication only, without hardware or private-method mocks."""
    rclpy.init(args=[], domain_id=196)
    source = Node("fusion_protocol_source")
    fusion = ImageFusionNode("/test/camera", "/test/fused", "arm_base_link")
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(fusion)
    received = []
    source.create_subscription(Image, "/test/fused", received.append, qos_profile_sensor_data)
    camera = source.create_publisher(Image, "/test/camera", qos_profile_sensor_data)
    fruit_pub = source.create_publisher(
        FruitDetections, "/fruit_perception/camera/preview_detections", qos_profile_sensor_data
    )
    qr_pub = source.create_publisher(
        String, "/fruit_qr_planner/camera/preview_detections", qos_profile_sensor_data
    )

    def spin_until(predicate):
        deadline = time.monotonic() + 5.0
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), "ROS protocol check timed out"

    try:
        publishers = (camera, fruit_pub, qr_pub)
        spin_until(lambda: all(pub.get_subscription_count() for pub in publishers))
        spin_until(lambda: source.count_publishers("/test/fused") > 0)
        cases = [(False, False), (True, False), (False, True), (True, True)]
        for index, (has_fruit, has_qr) in enumerate(cases):
            image = Image(header=header(sec=index + 10), width=160, height=100,
                          encoding="rgb8", step=480, data=bytes(48000))
            fruit = FruitDetection(type="apple_ripe", confidence=0.9)
            fruit.roi.x_offset, fruit.roi.y_offset = 40, 40
            fruit.roi.width, fruit.roi.height = 20, 20
            fruit_pub.publish(FruitDetections(
                header=image.header, detections=[fruit] if has_fruit else []
            ))
            qr = [{"text": "QR", "roi": [100, 40, 20, 20]}] if has_qr else []
            qr_pub.publish(String(data=qr_result_json(image, qr)))
            # Deliver results before their matching frame. The renderer itself never waits.
            deadline = time.monotonic() + 0.1
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            received.clear()
            camera.publish(image)
            spin_until(lambda: bool(received))
            output = received[-1]
            assert output.header == image.header
            assert output.encoding == "rgb8"
            pixels = np.frombuffer(output.data, dtype=np.uint8).reshape(100, 160, 3)
            assert bool(pixels[59, 50].any()) == has_fruit
            assert bool(pixels[59, 110].any()) == has_qr

        # Cached boxes belong to the previous frame and cannot decorate this new frame.
        received.clear()
        image.header.stamp.sec += 1
        camera.publish(image)
        spin_until(lambda: bool(received))
        assert bytes(received[-1].data) == bytes(image.data)
    finally:
        executor.shutdown()
        fusion.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
