import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "fruit_waypoint_recorder"
    config_file = (
        "/home/rc4/fruit_ws/src/fruit_waypoint_recorder/config/"
        "waypoint_recorder.yaml"
    )
    with open(config_file, encoding="utf-8") as stream:
        default_waypoint_file = yaml.safe_load(stream)["fruit_waypoint_recorder"][
            "ros__parameters"
        ]["waypoint_file"]

    nodes_to_launch = [
        {
            "package": package_name,
            "executable": "waypoint_recorder_node",
            "name": "fruit_waypoint_recorder",
            "parameters": [config_file],
        },
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    waypoint_file = LaunchConfiguration("waypoint_file")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )
    declare_waypoint_file = DeclareLaunchArgument(
        "waypoint_file",
        default_value=default_waypoint_file,
        description="Profile-specific waypoint YAML file",
    )
    launch_actions = [declare_use_sim_time, declare_waypoint_file]
    for node_config in nodes_to_launch:
        parameters = list(node_config.get("parameters", []))
        parameters.append(
            {"use_sim_time": use_sim_time, "waypoint_file": waypoint_file}
        )
        launch_actions.append(
            Node(
                package=node_config["package"],
                executable=node_config["executable"],
                name=node_config["name"],
                output="screen",
                emulate_tty=True,
                parameters=parameters,
                remappings=node_config.get("remappings", []),
            )
        )

    return LaunchDescription(launch_actions)
