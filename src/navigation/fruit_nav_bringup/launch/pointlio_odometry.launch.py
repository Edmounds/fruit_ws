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
    "/home/rc4/fruit_ws/src/navigation/fruit_nav_bringup"
)
PCL_PRELOAD = "/lib/x86_64-linux-gnu/libusb-1.0.so.0"


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    lidar_config = LaunchConfiguration("lidar_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    SOURCE_SHARE, "config", "reality", "pointlio_params.yaml"
                ),
                description="Absolute path to the source Point-LIO parameters",
            ),
            DeclareLaunchArgument(
                "lidar_config",
                default_value=os.path.join(
                    SOURCE_SHARE, "config", "reality", "lidar_user_config.json"
                ),
                description="Absolute path to the MID-360 driver configuration",
            ),

            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(SOURCE_SHARE, "launch", "sensor_frames.launch.py")
                )
            ),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_ros_driver2",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                    {"user_config_path": lidar_config},
                    {"xfer_format": 1},
                ],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="point_lio",
                executable="pointlio_mapping",
                name="laserMapping",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                    {"prior_pcd.enable": False},
                    {"prior_pcd.prior_pcd_map_path": ""},
                    {"prior_pcd.init_pose": [0.0, 0.0, 0.0]},
                    {"pcd_save.pcd_save_en": False},
                    {"publish.tf_send_en": False},
                    {"publish.path_en": False},
                ],
                output="screen",
                emulate_tty=True,
                additional_env={"LD_PRELOAD": PCL_PRELOAD},
            ),
            Node(
                package="loam_interface",
                executable="loam_interface_node",
                name="loam_interface",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "stabilize_initial_odom": True,
                    },
                ],
                output="screen",
                emulate_tty=True,
                additional_env={"LD_PRELOAD": PCL_PRELOAD},
            ),
        ]
    )
