from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    confirm_frames = LaunchConfiguration("confirm_frames")

    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("confirm_frames", default_value="3"),
            Node(
                package="fruit_qr_planner",
                executable="fruit_qr_planner_node",
                name="fruit_qr_planner",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "confirm_frames": confirm_frames,
                    }
                ],
            ),
        ]
    )

