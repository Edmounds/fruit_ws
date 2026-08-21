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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SOURCE_SHARE = (
    "/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup"
)
POINTLIO_ODOMETRY_LAUNCH = os.path.join(
    SOURCE_SHARE, "launch", "pointlio_odometry.launch.py"
)


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    axis_params_file = LaunchConfiguration("axis_params_file")
    mid360_config = LaunchConfiguration("mid360_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_teleop = LaunchConfiguration("use_teleop")
    use_rviz = LaunchConfiguration("use_rviz")

    joy_config = "/home/rc4/fruit_ws/src/joy_linux/config/joy_config.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    SOURCE_SHARE, "config", "reality", "localization_params.yaml"
                ),
                description="Absolute path to the source Point-LIO parameters",
            ),
            DeclareLaunchArgument(
                "axis_params_file",
                default_value=(
                    "/home/rc4/fruit_ws/src/fruit_navigation/config/"
                    "axis_navigation.yaml"
                ),
                description="Absolute path to the axis navigation parameters",
            ),
            DeclareLaunchArgument(
                "mid360_config",
                default_value=os.path.join(
                    SOURCE_SHARE, "config", "reality", "mid360_user_config.json"
                ),
                description="Absolute path to the MID-360 driver configuration",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_teleop", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(SOURCE_SHARE, "rviz", "navigation.rviz"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(POINTLIO_ODOMETRY_LAUNCH),
                launch_arguments={
                    "params_file": params_file,
                    "mid360_config": mid360_config,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            Node(
                package="fruit_navigation",
                executable="axis_waypoint_action_server",
                name="axis_waypoint_action_server",
                parameters=[
                    axis_params_file,
                    {
                        "map_frame": "odom",
                        "require_gicp_localization": False,
                        "use_sim_time": use_sim_time,
                    },
                ],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel_out", "cmd_vel")],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="joy_linux",
                executable="joy_linux_node",
                name="joy_linux_node",
                condition=IfCondition(use_teleop),
                parameters=[joy_config, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="joy_linux",
                executable="joy_convert_node",
                name="joy_convert_node",
                condition=IfCondition(use_teleop),
                parameters=[joy_config, {"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel", "cmd_vel_teleop")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(use_rviz),
                arguments=[
                    "-d",
                    LaunchConfiguration("rviz_config"),
                    "-f",
                    "odom",
                ],
            ),
        ]
    )
