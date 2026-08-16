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
            "package": "tf2_ros",
            "executable": "static_transform_publisher",
            "name": "base_to_front_mid360",
            "arguments": [
                "--x",
                "0.144629969",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--qx",
                "0.183012702",
                "--qy",
                "-0.183012702",
                "--qz",
                "-0.683012702",
                "--qw",
                "0.683012702",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "front_mid360",
            ],
        },
        {
            "package": "tf2_ros",
            "executable": "static_transform_publisher",
            "name": "front_mid360_to_livox_imu",
            "arguments": [
                "--x",
                "0.011",
                "--y",
                "0.02329",
                "--z",
                "-0.04412",
                "--qx",
                "0.0",
                "--qy",
                "0.0",
                "--qz",
                "0.0",
                "--qw",
                "1.0",
                "--frame-id",
                "front_mid360",
                "--child-frame-id",
                "livox_frame",
            ],
        },
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
