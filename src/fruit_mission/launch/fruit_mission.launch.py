"""Compose the owned competition stack and gate only the mission entry."""

import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORKSPACE = Path("/home/rc4/fruit_ws")
MISSION_CONFIG = str(WORKSPACE / "src/fruit_mission/config/fruit_mission.yaml")
CLIENT_CONFIG = str(WORKSPACE / "src/fruit_mission/config/script_ros_clients.yaml")


def _validate_waypoints(context):
    with Path(MISSION_CONFIG).open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    params = document["fruit_mission"]["ros__parameters"]
    override = LaunchConfiguration("waypoint_file").perform(context).strip()
    waypoint_file = override or str(params.get("waypoint_file", "")).strip()
    map_frame = LaunchConfiguration("map_frame").perform(context).strip()
    waypoint_path = Path(waypoint_file)
    if not waypoint_file:
        raise RuntimeError("waypoint_file must not be empty")
    if waypoint_path.suffix not in (".yaml", ".yml"):
        raise RuntimeError("waypoint_file must be a YAML file")
    if not waypoint_path.is_file():
        raise RuntimeError(f"waypoint_file does not exist: {waypoint_file}")
    if not os.access(waypoint_path, os.R_OK):
        raise RuntimeError(f"waypoint_file is not readable: {waypoint_file}")
    with waypoint_path.open(encoding="utf-8") as stream:
        waypoint_document = yaml.safe_load(stream) or {}
    if waypoint_document.get("frame_id") != map_frame:
        raise RuntimeError(f"waypoint YAML frame_id must be {map_frame}")
    return [SetLaunchConfiguration("waypoint_file", waypoint_file)]


def generate_launch_description():
    # 1. 全局配置区
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 2. 节点清单
    modules = (
        ("src/auto_serial_bridge/launch/auto_serial_bridge.launch.py", {
            "log_level": LaunchConfiguration("serial_log_level"),
        }, "start_serial_bridge"),
        ("src/fruit_arm_planner/launch/fruit_arm_planner.launch.py", {
            "log_level": LaunchConfiguration("arm_log_level"),
        }, None),
        ("src/navigation/fruit_nav_bringup/launch/fruit_nav.launch.py", {}, None),
        ("src/fruit_perception/launch/fruit_perception.launch.py", {}, None),
        ("src/fruit_robviz_bridge/launch/fruit_robviz_bridge.launch.py", {},
         "start_robviz_bridge"),
    )
    readiness = Node(
        package="fruit_mission", executable="stack_readiness_client",
        name="stack_readiness_client", output="screen", emulate_tty=True,
        arguments=["stack"], parameters=[CLIENT_CONFIG, {"use_sim_time": use_sim_time}],
    )
    mission = Node(
        package="fruit_mission", executable="fruit_mission_node", name="fruit_mission",
        output="screen", emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[MISSION_CONFIG, {
            "use_sim_time": use_sim_time,
            "waypoint_file": LaunchConfiguration("waypoint_file"),
            "mission_flow": LaunchConfiguration("mission_flow"),
            "map_frame": LaunchConfiguration("map_frame"),
            "open_loop": ParameterValue(LaunchConfiguration("open_loop"), value_type=str),
        }],
    )

    # 3. 声明 Launch 参数
    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("waypoint_file", default_value=""),
        DeclareLaunchArgument("mission_flow", default_value="full",
                              choices=["full", "ac", "a", "b", "c"]),
        DeclareLaunchArgument("map_frame", default_value="odom"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("arm_log_level", default_value="info"),
        DeclareLaunchArgument("serial_log_level", default_value="info"),
        DeclareLaunchArgument("open_loop", default_value="A",
                              choices=["A", "C", "AC", "none"]),
        DeclareLaunchArgument("start_serial_bridge", default_value="true"),
        DeclareLaunchArgument("start_robviz_bridge", default_value="true"),
    ]

    # 4. 自动生成动作列表
    def start_mission(event, launch_context):
        if launch_context.is_shutdown:
            return []
        message = "Mission startup readiness checks finished"
        if event.returncode != 0:
            message += f" (status={event.returncode}); unavailable operations will be skipped"
        return [LogInfo(msg=message), mission]

    actions = [
        *arguments,
        OpaqueFunction(function=_validate_waypoints),
        RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=start_mission)),
        RegisterEventHandler(OnProcessExit(
            target_action=mission,
            on_exit=[EmitEvent(event=Shutdown(reason="Mission process exited"))],
        )),
    ]
    for relative_path, module_arguments, condition in modules:
        module_arguments = {"use_sim_time": use_sim_time, **module_arguments}
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(WORKSPACE / relative_path)),
            launch_arguments=module_arguments.items(),
            condition=IfCondition(LaunchConfiguration(condition)) if condition else None,
        )]))
    return LaunchDescription([*actions, readiness])
