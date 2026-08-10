from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========================================================================
    # 1. Global configuration
    # ========================================================================
    config_path = LaunchConfiguration("config_path")
    dump_path = LaunchConfiguration("dump_path")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ========================================================================
    # 2. Node list
    # ========================================================================
    nodes_to_launch = [
        {
            "package": "glim_ros",
            "executable": "glim_rosnode",
            "name": "glim_ros",
            "parameters": [
                {
                    "config_path": config_path,
                    "dump_path": dump_path,
                }
            ],
        }
    ]

    # ========================================================================
    # 3. Launch arguments
    # ========================================================================
    launch_actions = [
        DeclareLaunchArgument(
            "config_path",
            default_value="/home/rc4/fruit_ws/src/navigation/glim/config",
            description="Absolute path to the GLIM source configuration",
        ),
        DeclareLaunchArgument(
            "dump_path",
            default_value="/tmp/glim_dump",
            description="Directory used to save the GLIM map on shutdown",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
    ]

    # ========================================================================
    # 4. Build launch actions
    # ========================================================================
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
                remappings=node_config.get("remappings", []),
            )
        )

    return LaunchDescription(launch_actions)
