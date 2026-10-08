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
from sensor_msgs.msg import CompressedImage


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


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parsed, ros_args = parser.parse_known_args(args)
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
