import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MISSION_CONFIG = "/home/rc4/fruit_ws/src/fruit_mission/config/fruit_mission.yaml"


def _validated_node(context):
    with Path(MISSION_CONFIG).open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    params = document["fruit_mission"]["ros__parameters"]
    override = LaunchConfiguration("waypoint_file").perform(context).strip()
    waypoint_file = override or str(params.get("waypoint_file", "")).strip()
    waypoint_path = Path(waypoint_file)
    if not waypoint_file:
        raise RuntimeError("waypoint_file must not be empty")
    if waypoint_path.suffix not in (".yaml", ".yml"):
        raise RuntimeError("waypoint_file must be a YAML file")
    if not waypoint_path.is_file():
        raise RuntimeError(f"waypoint_file does not exist: {waypoint_file}")
    if not os.access(waypoint_path, os.R_OK):
        raise RuntimeError(f"waypoint_file is not readable: {waypoint_file}")

    return [
        Node(
            package="fruit_mission",
            executable="fruit_mission_node",
            name="fruit_mission",
            output="screen",
            emulate_tty=True,
            parameters=[
                MISSION_CONFIG,
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "waypoint_file": waypoint_file,
                    "mission_flow": LaunchConfiguration("mission_flow"),
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value="",
                description="Configured waypoint YAML file",
            ),
            DeclareLaunchArgument(
                "mission_flow",
                default_value="full",
                description="Mission flow: full, ac, a, b, or c",
            ),
            OpaqueFunction(function=_validated_node),
        ]
    )
