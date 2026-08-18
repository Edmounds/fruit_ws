from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


JOY_CONFIG = '/home/rc4/fruit_ws/src/joy_linux/config/joy_config.yaml'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nodes_to_launch = [
        {
            'package': 'joy_linux',
            'executable': 'joy_linux_node',
            'name': 'joy_linux_node',
        },
        {
            'package': 'joy_linux',
            'executable': 'joy_convert_node',
            'name': 'joy_convert_node',
        },
    ]
    launch_actions = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
    ]

    for node_config in nodes_to_launch:
        launch_actions.append(
            Node(
                package=node_config['package'],
                executable=node_config['executable'],
                name=node_config['name'],
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': use_sim_time}, JOY_CONFIG],
            )
        )

    return LaunchDescription(launch_actions)
