from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========================================================================
    # 1. Global configuration
    # ========================================================================
    mission_config = (
        "/home/rc4/fruit_ws/src/fruit_mission/config/"
        "fruit_mission.yaml"
    )

    # ========================================================================
    # 2. Node list
    # ========================================================================
    nodes_to_launch = [
        {
            "package": "fruit_mission",
            "executable": "fruit_mission_node",
            "name": "fruit_mission",
            "parameters": [mission_config],
        },
    ]

    # ========================================================================
    # 3. Launch arguments
    # ========================================================================
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    # ========================================================================
    # 4. Generate actions
    # ========================================================================
    launch_actions = [declare_use_sim_time]
    for node_cfg in nodes_to_launch:
        parameters = [{"use_sim_time": use_sim_time}]
        parameters.extend(node_cfg.get("parameters", []))
        launch_actions.append(
            Node(
                package=node_cfg["package"],
                executable=node_cfg["executable"],
                name=node_cfg["name"],
                output="screen",
                emulate_tty=True,
                parameters=parameters,
                remappings=node_cfg.get("remappings", []),
            )
        )

    return LaunchDescription(launch_actions)
