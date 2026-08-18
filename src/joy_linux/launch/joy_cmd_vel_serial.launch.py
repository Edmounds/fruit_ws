from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
        PythonLaunchDescriptionSource(SERIAL_LAUNCH)
    )

    return LaunchDescription([joy_launch, serial_launch])
