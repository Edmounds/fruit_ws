from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                "/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/"
                "pb2025_nav_bringup/launch/sensor_frames.launch.py"
            )
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value="/home/rc4/fruit_ws/src/navigation/glim/config",
            description="Absolute path to the GLIM source configuration",
        ),
        DeclareLaunchArgument(
            "dump_path",
            default_value="/home/rc4/fruit_ws/tmp/glim_dump",
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
                arguments=node_config.get("arguments", []),
                remappings=node_config.get("remappings", []),
            )
        )

    return LaunchDescription(launch_actions)
