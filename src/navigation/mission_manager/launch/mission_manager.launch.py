from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")
    start_rviz = LaunchConfiguration("start_rviz")

    config_file = PathJoinSubstitution(
        [FindPackageShare("mission_manager"), "config", "mission_manager.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mission_manager"), "rviz", "mission_manager.rviz"]
    )
    default_map_yaml = PathJoinSubstitution(
        [FindPackageShare("mission_manager"), "maps", "sample_map.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock.",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value=default_map_yaml,
            description="Initial 2D map YAML for mission editing.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz Mission Manager.",
        ),
        LifecycleNode(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            namespace="",
            output="screen",
            parameters=[
                {"yaml_filename": map_yaml},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": ["map_server"],
                }
            ],
        ),
        Node(
            package="mission_manager",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
            parameters=[
                config_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="mission_manager",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(start_rviz),
        ),
    ])
