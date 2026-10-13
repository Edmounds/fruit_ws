from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 全局配置区
    config_file = (
        "/home/rc4/fruit_ws/src/navigation/fruit_navigation/config/"
        "axis_navigation.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    # 2. 节点清单
    nodes_to_launch = [
        {
            "package": "fruit_navigation",
            "executable": "axis_waypoint_action_server",
            "name": "axis_waypoint_action_server",
            "parameters": [params_file],
        },
    ]

    # 3. 声明 Launch 参数
    launch_actions = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=config_file),
        DeclareLaunchArgument("log_level", default_value="info",
                              choices=["debug", "info", "warn", "error", "fatal"]),
    ]
    # 4. 自动生成动作列表
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
                arguments=["--ros-args", "--log-level", log_level],
            )
        )
    return LaunchDescription(launch_actions)
