"""Independent ACB entry using the shared competition stack lifecycle."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


WORKSPACE = Path("/home/rc4/fruit_ws")


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(__file__).with_name("fruit_mission.launch.py"))),
        launch_arguments={
            "mission_flow": "acb",
            "open_loop": "none",
            "waypoint_file": str(WORKSPACE / (
                "src/navigation/fruit_nav_bringup/waypoints/odom_waypoints_acb.yaml")),
        }.items(),
    )])
