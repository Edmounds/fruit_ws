from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    package_name = "fruit_robviz_bridge"
    config_file = (
        "/home/rc4/fruit_ws/src/fruit_robviz_bridge/config/"
        "waypoint_editor.yaml"
    )

    config = LaunchConfiguration("config")
    declare_config = DeclareLaunchArgument(
        "config",
        default_value=config_file,
        description="Source-workspace waypoint editor configuration",
    )
    executable = PathJoinSubstitution(
        [
            FindPackagePrefix(package_name),
            "lib",
            package_name,
            "waypoint_editor_server",
        ]
    )
    server = ExecuteProcess(
        cmd=[executable, "--config", config],
        output="screen",
        emulate_tty=True,
    )
    return LaunchDescription([declare_config, server])
