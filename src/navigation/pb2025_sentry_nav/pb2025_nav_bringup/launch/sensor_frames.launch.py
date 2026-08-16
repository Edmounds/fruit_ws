# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ========================================================================
    # 1. Global configuration
    # ========================================================================
    base_to_lidar = [
        "--x",
        "0.144629969",
        "--y",
        "0.0",
        "--z",
        "0.0",
        "--qx",
        "-0.183012702",
        "--qy",
        "0.183012702",
        "--qz",
        "-0.683012702",
        "--qw",
        "0.683012702",
        "--frame-id",
        "base_link",
        "--child-frame-id",
        "front_mid360",
    ]
    lidar_to_imu = [
        "--x",
        "0.011",
        "--y",
        "0.02329",
        "--z",
        "-0.04412",
        "--qx",
        "0.0",
        "--qy",
        "0.0",
        "--qz",
        "0.0",
        "--qw",
        "1.0",
        "--frame-id",
        "front_mid360",
        "--child-frame-id",
        "livox_frame",
    ]

    # ========================================================================
    # 2. Node list
    # ========================================================================
    nodes_to_launch = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_front_mid360",
            arguments=base_to_lidar,
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="front_mid360_to_livox_imu",
            arguments=lidar_to_imu,
            output="screen",
        ),
    ]

    # ========================================================================
    # 3. Launch arguments
    # ========================================================================
    launch_actions = []

    # ========================================================================
    # 4. Build launch actions
    # ========================================================================
    launch_actions.extend(nodes_to_launch)
    return LaunchDescription(launch_actions)
