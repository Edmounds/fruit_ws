#!/usr/bin/env python3

"""Check real compressed preview topics without sending any motion command."""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from fruit_robviz_bridge.video_frames import (
    convert_depth_to_jet,
    encode_jpeg,
    validate_and_extract_color,
)


class PreviewCheck(Node):
    def __init__(self) -> None:
        super().__init__("fruit_robviz_preview_check")
        self.started = time.monotonic()
        self.frames = {"rgb": 0, "depth": 0}
        self.bytes = {"rgb": 0, "depth": 0}
        self.first_decode = {"rgb": False, "depth": False}
        self.subscriptions = [
            self.create_subscription(
                CompressedImage,
                "/fruit_perception/camera/annotated_image/compressed",
                lambda message: self._on_image("rgb", message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CompressedImage,
                "/fruit_perception/camera/native_depth/compressed",
                lambda message: self._on_image("depth", message),
                qos_profile_sensor_data,
            ),
        ]

    def _on_image(self, name: str, message: CompressedImage) -> None:
        if message.format.lower() not in ("jpeg", "jpg"):
            raise RuntimeError(f"{name} format is not jpeg: {message.format}")
        self.frames[name] += 1
        self.bytes[name] += len(message.data)
        if not self.first_decode[name]:
            image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.size == 0:
                raise RuntimeError(f"{name} first JPEG cannot be decoded")
            self.first_decode[name] = True


def run_self_check() -> int:
    """Run offline algorithm self-checks on frame extraction, JET colormap, and encoding."""
    # 1. RGB with row padding: width=100, height=50, step=320 (row_bytes=300)
    rgb_msg = Image()
    rgb_msg.width = 100
    rgb_msg.height = 50
    rgb_msg.encoding = "rgb8"
    rgb_msg.step = 320
    rgb_msg.data = b"\xaa" * (320 * 50)
    gst_bytes, bgr_mat, spec = validate_and_extract_color(rgb_msg)
    if spec != (100, 50, "RGB"):
        raise AssertionError(f"Unexpected RGB spec: {spec}")
    if bgr_mat.shape != (50, 100, 3):
        raise AssertionError(f"Unexpected BGR shape: {bgr_mat.shape}")
    if len(gst_bytes) != 300 * 50:  # 300 is 4-byte aligned (300 & ~3 == 300)
        raise AssertionError(f"Unexpected gst_bytes length: {len(gst_bytes)}")

    # 2. Unsupported encoding rejection
    bad_msg = Image()
    bad_msg.width = 64
    bad_msg.height = 64
    bad_msg.encoding = "mono8"
    bad_msg.step = 64
    bad_msg.data = b"\x00" * (64 * 64)
    try:
        validate_and_extract_color(bad_msg)
        raise AssertionError("Expected ValueError for unsupported encoding mono8")
    except ValueError:
        pass

    # 3. Native depth 16UC1 normal frame (848x480)
    depth_msg = Image()
    depth_msg.width = 848
    depth_msg.height = 480
    depth_msg.encoding = "16UC1"
    depth_msg.step = 848 * 2
    depth_data = np.zeros((480, 848), dtype=np.uint16)
    depth_data[100:200, 100:200] = 1500  # 1.5m
    depth_data[300:400, 300:400] = 3000  # 3.0m
    depth_msg.data = depth_data.tobytes()
    gst_rgb, depth_bgr, d_spec = convert_depth_to_jet(depth_msg)
    if d_spec != (848, 480):
        raise AssertionError(f"Unexpected depth spec: {d_spec}")
    if depth_bgr.shape != (480, 848, 3):
        raise AssertionError(f"Unexpected depth BGR shape: {depth_bgr.shape}")
    if len(gst_rgb) != 848 * 480 * 3:
        raise AssertionError(f"Unexpected gst_rgb length: {len(gst_rgb)}")

    # Check that invalid area remains black (0, 0, 0)
    if not np.all(depth_bgr[50, 50] == [0, 0, 0]):
        raise AssertionError("Invalid depth pixels must be black (0, 0, 0)")
    # Check that valid area is not black
    if np.all(depth_bgr[150, 150] == [0, 0, 0]):
        raise AssertionError("Valid depth pixels must be colored")

    # 4. Native depth with 0 valid pixels
    empty_depth = Image()
    empty_depth.width = 848
    empty_depth.height = 480
    empty_depth.encoding = "16UC1"
    empty_depth.step = 848 * 2
    empty_depth.data = np.zeros((480, 848), dtype=np.uint16).tobytes()
    _, empty_bgr, _ = convert_depth_to_jet(empty_depth)
    # Area below banner (y > 22) must be all black
    if not np.all(empty_bgr[25:, :] == 0):
        raise AssertionError("Empty depth below banner must be all zeros")

    # 5. Native depth with constant non-zero value
    const_depth = Image()
    const_depth.width = 100
    const_depth.height = 100
    const_depth.encoding = "16UC1"
    const_depth.step = 200
    const_depth.data = np.full((100, 100), 2000, dtype=np.uint16).tobytes()
    _, const_bgr, _ = convert_depth_to_jet(const_depth)
    if const_bgr.shape != (100, 100, 3):
        raise AssertionError(f"Unexpected const depth shape: {const_bgr.shape}")

    # 6. JPEG encoding check
    jpeg_bytes = encode_jpeg(depth_bgr, 90)
    decoded = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if decoded is None or decoded.shape != (480, 848, 3):
        raise AssertionError("JPEG decoded image does not match native dimensions")

    print("[self-check] All frame, depth JET, and JPEG checks passed successfully.")
    return 0


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument(
        "--self-check",
        action="store_true",
        help="Run offline conversion and encoding self-check without ROS network",
    )
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.self_check:
        return run_self_check()
    if not 1.0 <= parsed.duration_sec <= 3600.0:
        parser.error("--duration-sec must be in [1, 3600]")
    rclpy.init(args=ros_args)
    node = PreviewCheck()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.monotonic() + parsed.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.25)
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    elapsed = max(time.monotonic() - node.started, 1e-6)
    for name in ("rgb", "depth"):
        node.get_logger().info(
            f"{name} frames={node.frames[name]} fps={node.frames[name] / elapsed:.2f} "
            f"jpeg_bytes={node.bytes[name]} decoded={node.first_decode[name]}",
        )
    return 0 if all(node.first_decode.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
