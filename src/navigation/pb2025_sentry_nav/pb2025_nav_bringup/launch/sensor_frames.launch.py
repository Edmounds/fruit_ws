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

from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _transform_arguments(transform):
    translation = transform["translation"]
    rotation = transform["rotation"]
    if len(translation) != 3 or len(rotation) != 4:
        raise RuntimeError("static transform must contain translation[3] and rotation[4]")
    return [
        "--x", str(translation[0]),
        "--y", str(translation[1]),
        "--z", str(translation[2]),
        "--qx", str(rotation[0]),
        "--qy", str(rotation[1]),
        "--qz", str(rotation[2]),
        "--qw", str(rotation[3]),
        "--frame-id", transform["parent_frame"],
        "--child-frame-id", transform["child_frame"],
    ]


def _launch_nodes(context):
    # ========================================================================
    # 1. Global configuration
    # ========================================================================
    config_path = Path(LaunchConfiguration("config_file").perform(context))
    if not config_path.is_file():
        raise RuntimeError(f"sensor frame config does not exist: {config_path}")
    transforms = yaml.safe_load(config_path.read_text())

    # ========================================================================
    # 2. Node list
    # ========================================================================
    nodes_to_launch = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_front_mid360",
            arguments=_transform_arguments(transforms["base_to_front_mid360"]),
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="front_mid360_to_livox_imu",
            arguments=_transform_arguments(transforms["front_mid360_to_livox_imu"]),
            output="screen",
        ),
    ]

    # ========================================================================
    # 3. Launch arguments
    # ========================================================================
    return nodes_to_launch


def generate_launch_description():
    # ========================================================================
    # 3. Launch arguments
    # ========================================================================
    config_file = (
        "/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/"
        "pb2025_nav_bringup/config/reality/sensor_frames.yaml"
    )

    # ========================================================================
    # 4. Build launch actions
    # ========================================================================
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=config_file),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
