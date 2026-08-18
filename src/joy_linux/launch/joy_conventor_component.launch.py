from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

JOY_CONFIG = '/home/rc4/fruit_ws/src/joy_linux/config/joy_config.yaml'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        ComposableNodeContainer(
            name='joy_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Joy Linux Component
                ComposableNode(
                    package='joy_linux',
                    plugin='joy_linux::JoyLinux',
                    name='joy_linux_node',
                    parameters=[JOY_CONFIG, {'use_sim_time': use_sim_time}],
                ),
                # Joy Convert Component
                ComposableNode(
                    package='joy_linux',
                    plugin='joy_conventor::JoyConvertNode',
                    name='joy_convert_node',
                    parameters=[JOY_CONFIG, {'use_sim_time': use_sim_time}],
                )
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
