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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SOURCE_SHARE = (
    "/home/rc4/fruit_ws/src/navigation/fruit_nav_bringup"
)
POINTLIO_ODOMETRY_LAUNCH = os.path.join(
    SOURCE_SHARE, "launch", "pointlio_odometry.launch.py"
)


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    axis_params_file = LaunchConfiguration("axis_params_file")
    lidar_config = LaunchConfiguration("lidar_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_teleop = LaunchConfiguration("use_teleop")

    joy_config = "/home/rc4/fruit_ws/src/joy_linux/config/joy_config.yaml"

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
                "axis_params_file",
                default_value=(
                    "/home/rc4/fruit_ws/src/navigation/fruit_navigation/config/"
                    "axis_navigation.yaml"
                ),
                description="Absolute path to the axis navigation parameters",
            ),
            DeclareLaunchArgument(
                "axis_log_level",
                default_value="info",
                choices=["debug", "info", "warn", "error", "fatal"],
                description="Log level for axis_waypoint_action_server",
            ),

            DeclareLaunchArgument(
                "lidar_config",
                default_value=os.path.join(
                    SOURCE_SHARE, "config", "reality", "lidar_user_config.json"
                ),
                description="Absolute path to the MID-360 driver configuration",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_teleop", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(POINTLIO_ODOMETRY_LAUNCH),
                launch_arguments={
                    "params_file": params_file,
                    "lidar_config": lidar_config,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            GroupAction(actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    "/home/rc4/fruit_ws/src/navigation/fruit_navigation/launch/"
                    "axis_navigation.launch.py"
                ),
                launch_arguments={
                    "params_file": axis_params_file,
                    "log_level": LaunchConfiguration("axis_log_level"),
                    "use_sim_time": use_sim_time,
                }.items(),
            )]),
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
        ]
    )
