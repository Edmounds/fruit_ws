from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


JOY_LAUNCH = '/home/rc4/fruit_ws/src/joy_linux/launch/joy_conventor.launch.py'
SERIAL_LAUNCH = (
    '/home/rc4/fruit_ws/src/auto_serial_bridge/launch/'
    'serial_bridge_by_component.launch.py'
)


def generate_launch_description():
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(JOY_LAUNCH)
    )
    serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SERIAL_LAUNCH),
        condition=IfCondition(LaunchConfiguration('start_serial_bridge')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_serial_bridge', default_value='true',
            description='Start the serial bridge; false when reusing an existing bridge',
        ),
        joy_launch,
        serial_launch,
    ])
