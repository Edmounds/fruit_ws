# Copyright 2026 fruit_ws team
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")

    nodes_to_launch = [
        {
            "package": "rviz2",
            "executable": "rviz2",
            "name": "rviz2",
            "arguments": ["-d", rviz_config],
        },
    ]

    launch_actions = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=(
                "/home/rc4/fruit_ws/src/navigation/fruit_nav_bringup/"
                "rviz/waypoints_maker.rviz"
            ),
        ),

    ]
    for node_config in nodes_to_launch:
        launch_actions.append(
            Node(
                package=node_config["package"],
                executable=node_config["executable"],
                name=node_config["name"],
                output="screen",
                emulate_tty=True,
                arguments=node_config.get("arguments", []),
            )
        )
    return LaunchDescription(launch_actions)
