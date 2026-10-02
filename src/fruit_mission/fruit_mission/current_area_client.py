"""Publish upper-computer area state; no lower-controller business command."""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fruit_mission.script_hardware_client import latched_qos


def main(args=None):
    parser = argparse.ArgumentParser()
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--probe-bridge', action='store_true')
    action.add_argument('--area', choices=('A', 'B', 'C'))
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Node('current_area_client')
    node.declare_parameter('current_area_topic', '/fruit_mission/current_area')
    node.declare_parameter('bridge_ready_topic', '/auto_serial_bridge/ready')
    publisher = node.create_publisher(
        String, node.get_parameter('current_area_topic').value, latched_qos())
    status = 0
    try:
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if options.probe_bridge:
            status = 0 if node.count_publishers(
                node.get_parameter('bridge_ready_topic').value) else 10
        else:
            publisher.publish(String(data=options.area))
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        status = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(status)
