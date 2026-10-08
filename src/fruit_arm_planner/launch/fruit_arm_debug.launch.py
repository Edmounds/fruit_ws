"""Start the serial bridge and arm planner for manual hardware debugging."""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_config = '/home/rc4/fruit_ws/src/auto_serial_bridge/config/protocol.yaml'
    with open(serial_config, encoding='utf-8') as config_file:
        default_port = yaml.safe_load(config_file)['serial_controller']['ros__parameters']['port']
    bridge_launch = (
        '/home/rc4/fruit_ws/src/auto_serial_bridge/launch/'
        'auto_serial_bridge.launch.py'
    )
    planner_launch = (
        '/home/rc4/fruit_ws/src/fruit_arm_planner/launch/'
        'fruit_arm_planner.launch.py'
    )
    port = LaunchConfiguration('port')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=str(default_port),
                              description='Lower-controller serial device'),
        DeclareLaunchArgument('log_level', default_value='info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={'port': port, 'log_level': log_level}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planner_launch),
            launch_arguments={'log_level': log_level}.items(),
        ),
    ])
