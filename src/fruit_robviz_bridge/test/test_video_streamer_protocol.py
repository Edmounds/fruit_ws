"""Communication protocol test for StreamChannel demand lifecycle and frame caching."""

import asyncio
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

from fruit_robviz_bridge.video_streamer import StreamChannel


def make_test_image(sec: int = 1, width: int = 64, height: int = 48) -> Image:
    msg = Image()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = "camera_color_optical_frame"
    msg.width = width
    msg.height = height
    msg.encoding = "bgr8"
    msg.step = width * 3
    msg.data = bytes(width * height * 3)
    return msg


def test_stream_channel_frame_caching_and_demand_lifecycle():
    """Verify subscription remains persistent and captures frames across viewer lifecycles."""
    rclpy.init(args=[], domain_id=197)
    loop = asyncio.new_event_loop()
    node = Node("test_video_streamer_node")
    source_node = Node("test_video_source_node")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(source_node)

    topic = "/test/stream_channel/image_raw"
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )
    publisher = source_node.create_publisher(Image, topic, qos)

    initial_params = {
        "bandwidth_mbps": 25.0,
        "max_fps": 10.0,
        "target_bitrate_mbps": 4.0,
    }
    channel = StreamChannel("test_stream", topic, node, loop, initial_params)
    channel.settings_applied = True

    def spin_for(seconds: float):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    try:
        # 1. No viewers: raw frames are cached, video demand is False
        assert not channel.has_video_demand()
        assert channel.subscription is not None

        publisher.publish(make_test_image(sec=10))
        spin_for(0.2)
        frame, recv_time = channel.get_latest_raw_frame()
        assert frame is not None
        assert frame.header.stamp.sec == 10
        assert time.monotonic() - recv_time < 1.0

        # 2. Add viewer: video demand becomes True
        sent_messages = []
        channel.add_viewer("v1", lambda d: sent_messages.append(d), requested_fps=10.0)
        assert channel.has_video_demand()

        publisher.publish(make_test_image(sec=20))
        spin_for(0.2)
        frame2, _ = channel.get_latest_raw_frame()
        assert frame2 is not None
        assert frame2.header.stamp.sec == 20

        # 3. Remove viewer: video demand becomes False, subscription remains intact
        channel.remove_viewer("v1")
        assert not channel.has_video_demand()
        assert channel.subscription is not None

        publisher.publish(make_test_image(sec=30))
        spin_for(0.2)
        frame3, _ = channel.get_latest_raw_frame()
        assert frame3 is not None
        assert frame3.header.stamp.sec == 30
    finally:
        channel.destroy()
        pending = asyncio.all_tasks(loop)
        for task in pending:
            task.cancel()
        if pending:
            loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        source_node.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        loop.close()
