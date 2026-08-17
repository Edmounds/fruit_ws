# Copyright 2026 Lihan Chen
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SOURCE_SHARE = (
    "/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup"
)
GLIM_MAPPING_LAUNCH = (
    "/home/rc4/fruit_ws/src/navigation/glim_ros2/launch/mapping.launch.py"
)


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    dump_path = LaunchConfiguration("dump_path")
    use_sim_time = LaunchConfiguration("use_sim_time")

    common = {
        "parameters": [params_file, {"use_sim_time": use_sim_time}],
        "output": "screen",
        "emulate_tty": True,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    SOURCE_SHARE, "config", "reality", "nav2_params.yaml"
                ),
                description="Absolute path to the source navigation parameters",
            ),
            DeclareLaunchArgument(
                "dump_path",
                default_value="/home/rc4/fruit_ws/tmp/glim_map",
                description="Directory used to save this GLIM mapping session",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(GLIM_MAPPING_LAUNCH),
                launch_arguments={
                    "config_path": "/home/rc4/fruit_ws/src/navigation/glim/config",
                    "dump_path": dump_path,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                remappings=[
                    ("cloud_in", "/glim_ros/aligned_points_odom"),
                    ("scan", "/obstacle_scan"),
                ],
                **common,
            ),
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                **common,
            ),
        ]
    )
