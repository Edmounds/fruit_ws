from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = (
        "/home/rc4/fruit_ws/src/fruit_navigation/config/"
        "axis_navigation.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    route_file = LaunchConfiguration("route_file")
    start_route_client = LaunchConfiguration("start_route_client")

    nodes_to_launch = [
        {
            "package": "fruit_navigation",
            "executable": "axis_waypoint_action_server",
            "name": "axis_waypoint_action_server",
            "parameters": [config_file],
        },
        {
            "package": "fruit_navigation",
            "executable": "route_marker_node",
            "name": "route_marker_node",
            "parameters": [config_file, {"route_file": route_file}],
        },
        {
            "package": "fruit_navigation",
            "executable": "route_file_client",
            "name": "route_file_client",
            "parameters": [config_file, {"route_file": route_file}],
            "condition": IfCondition(start_route_client),
        },
    ]

    launch_actions = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("route_file", default_value=""),
        DeclareLaunchArgument("start_route_client", default_value="false"),
    ]
    for node_config in nodes_to_launch:
        parameters = [{"use_sim_time": use_sim_time}]
        parameters.extend(node_config.get("parameters", []))
        launch_actions.append(
            Node(
                package=node_config["package"],
                executable=node_config["executable"],
                name=node_config["name"],
                output="screen",
                emulate_tty=True,
                parameters=parameters,
                condition=node_config.get("condition"),
            )
        )
    return LaunchDescription(launch_actions)
