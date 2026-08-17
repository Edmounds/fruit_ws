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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


SOURCE_SHARE = (
    "/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup"
)


def _launch_nodes(context):
    map_file = LaunchConfiguration("map").perform(context)
    localization_map = LaunchConfiguration("localization_map").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    initial_pose_file = LaunchConfiguration("initial_pose_file").perform(context)
    for label, path in (
        ("map", map_file),
        ("localization_map", localization_map),
        ("params_file", params_file),
    ):
        if not os.path.isfile(path):
            raise RuntimeError(f"{label} does not exist: {path}")
    if initial_pose_file and not os.path.isfile(initial_pose_file):
        raise RuntimeError(f"initial_pose_file does not exist: {initial_pose_file}")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"yaml_filename": map_file},
        convert_types=True,
    )
    common = {"parameters": [configured_params], "output": "screen"}
    system_libusb = "/lib/x86_64-linux-gnu/libusb-1.0.so.0"
    pcl_env = {
        "LD_PRELOAD": ":".join(
            value for value in (system_libusb, os.environ.get("LD_PRELOAD")) if value
        )
    }
    pcl_common = {**common, "additional_env": pcl_env}
    behavior_tree_dir = os.path.join(SOURCE_SHARE, "behavior_trees")
    mid360_config = os.path.join(
        SOURCE_SHARE,
        "config",
        "reality",
        "mid360_user_config.json",
    )
    joy_config = "/home/rc4/fruit_ws/src/joy_linux/config/joy_config.yaml"
    localization_parameters = [configured_params]
    if initial_pose_file:
        localization_parameters.append(initial_pose_file)
    localization_parameters.append({"localization_map": localization_map})

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                "/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/"
                "pb2025_nav_bringup/launch/sensor_frames.launch.py"
            )
        ),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_ros_driver2",
            parameters=[configured_params, {"user_config_path": mid360_config}],
            output="screen",
        ),
        Node(
            package="point_lio",
            executable="pointlio_mapping",
            name="laserMapping",
            **pcl_common,
        ),
        Node(
            package="loam_interface",
            executable="loam_interface_node",
            name="loam_interface",
            **pcl_common,
        ),
        Node(
            package="sensor_scan_generation",
            executable="sensor_scan_generation_node",
            name="sensor_scan_generation",
            **pcl_common,
        ),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            name="terrainAnalysis",
            **pcl_common,
        ),
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            name="terrainAnalysisExt",
            **pcl_common,
        ),
        Node(
            package="pb_gicp_localization",
            executable="pb_gicp_localization_node",
            name="pb_gicp_localization",
            parameters=localization_parameters,
            output="screen",
            additional_env=pcl_env,
        ),
        Node(package="nav2_map_server", executable="map_server", name="map_server", **common),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            remappings=[("cmd_vel", "cmd_vel_nav_raw")],
            **common,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            **common,
        ),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", **common),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            **common,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=[
                configured_params,
                {
                    "default_nav_to_pose_bt_xml": os.path.join(
                        behavior_tree_dir,
                        "navigate_to_pose_w_replanning_and_recovery.xml",
                    ),
                    "default_nav_through_poses_bt_xml": os.path.join(
                        behavior_tree_dir,
                        "navigate_through_poses_w_replanning_and_recovery.xml",
                    ),
                },
            ],
            output="screen",
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            **common,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            remappings=[
                ("cmd_vel", "cmd_vel_nav_raw"),
                ("cmd_vel_smoothed", "cmd_vel_nav"),
            ],
            **common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[
                {"autostart": True},
                {
                    "node_names": [
                        "map_server",
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ]
                },
            ],
            output="screen",
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            parameters=[configured_params],
            remappings=[("cmd_vel_out", "cmd_vel")],
            output="screen",
        ),
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_linux_node",
            condition=IfCondition(LaunchConfiguration("use_teleop")),
            parameters=[joy_config],
        ),
        Node(
            package="joy_linux",
            executable="joy_convert_node",
            name="joy_convert_node",
            condition=IfCondition(LaunchConfiguration("use_teleop")),
            parameters=[joy_config],
            remappings=[("cmd_vel", "cmd_vel_teleop")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Absolute path to the Nav2 map YAML"),
            DeclareLaunchArgument(
                "localization_map", description="Absolute path to the GLIM-derived PCD"
            ),
            DeclareLaunchArgument("initial_pose_file", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    SOURCE_SHARE, "config", "reality", "nav2_params.yaml"
                ),
            ),
            DeclareLaunchArgument("use_teleop", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(SOURCE_SHARE, "rviz", "nav2_default_view.rviz"),
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
