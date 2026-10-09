import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SOURCE_SHARE = "/home/rc4/fruit_ws/src/fruit_robviz_bridge"


def generate_launch_description():
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = os.path.join(SOURCE_SHARE, "config", "robviz_bridge.yaml")

    image_bridge = Node(
        package="fruit_robviz_bridge",
        executable="fruit_robviz_image_bridge",
        name="fruit_robviz_image_bridge",
        parameters=[config, {"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )
    config_client = Node(
        package="fruit_robviz_bridge",
        executable="robviz_config_client.py",
        name="fruit_robviz_config_client",
        parameters=[config, {"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=config_path,
            description="Source-workspace RobViz bridge configuration",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        image_bridge,
        config_client,
    ])
