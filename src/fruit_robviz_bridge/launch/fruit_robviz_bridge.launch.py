import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


SOURCE_SHARE = "/home/rc4/fruit_ws/src/fruit_robviz_bridge"


def generate_launch_description():
    target_container = LaunchConfiguration("target_container")
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = os.path.join(SOURCE_SHARE, "config", "robviz_bridge.yaml")

    image_component = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[
            ComposableNode(
                package="fruit_robviz_bridge",
                plugin="fruit_robviz_bridge::ImageBridgeComponent",
                name="fruit_robviz_image_bridge",
                parameters=[config, {"use_sim_time": use_sim_time}],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
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
            "target_container",
            default_value="/fruit_perception_container",
            description="Existing perception component container",
        ),
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
        image_component,
        config_client,
    ])
