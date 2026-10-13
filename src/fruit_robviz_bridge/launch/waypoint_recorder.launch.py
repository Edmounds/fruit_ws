import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "fruit_robviz_bridge"
    config_file = (
        "/home/rc4/fruit_ws/src/fruit_robviz_bridge/config/"
        "waypoint_recorder.yaml"
    )
    default_auto_name_file = (
        "/home/rc4/fruit_ws/src/navigation/fruit_nav_bringup/waypoints/template.yaml"
    )
    with open(config_file, encoding="utf-8") as stream:
        default_parameters = yaml.safe_load(stream)["fruit_waypoint_recorder"][
            "ros__parameters"
        ]

    nodes_to_launch = [
        {
            "package": package_name,
            "executable": "waypoint_recorder_node",
            "name": "fruit_waypoint_recorder",
            "parameters": [config_file],
        },
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    waypoint_file = LaunchConfiguration("waypoint_file")
    template_file = LaunchConfiguration("template_file")
    source_root = LaunchConfiguration("source_root")
    debug_waypoint_file = LaunchConfiguration("debug_waypoint_file")
    frame_id = LaunchConfiguration("frame_id")
    auto_name_prefix = LaunchConfiguration("auto_name_prefix")
    auto_name_file = LaunchConfiguration("auto_name_file")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )
    declare_waypoint_file = DeclareLaunchArgument(
        "waypoint_file",
        default_value=default_parameters["waypoint_file"],
        description="Profile-specific waypoint YAML file",
    )
    declare_template_file = DeclareLaunchArgument(
        "template_file",
        default_value=default_parameters["template_file"],
        description="Template matching the formal waypoint YAML file",
    )
    declare_source_root = DeclareLaunchArgument(
        "source_root",
        default_value=default_parameters["source_root"],
        description="Allowed source directory for recorder YAML files",
    )
    declare_frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="odom",
        description="Coordinate frame stored in the waypoint YAML",
    )
    declare_debug_waypoint_file = DeclareLaunchArgument(
        "debug_waypoint_file",
        default_value=(
            "/home/rc4/fruit_ws/src/navigation/fruit_nav_bringup/waypoints/"
            "debug_waypoints.yaml"
        ),
        description="Free-form debug waypoint YAML file",
    )

    declare_auto_name_prefix = DeclareLaunchArgument(
        "auto_name_prefix",
        default_value="",
        description="Prefix for sequential waypoint names, such as A or B",
    )
    declare_auto_name_file = DeclareLaunchArgument(
        "auto_name_file",
        default_value=default_auto_name_file,
        description="YAML file containing formal waypoint names in recording order",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="Logging level for waypoint recorder node",
    )
    launch_actions = [
        declare_use_sim_time,
        declare_waypoint_file,
        declare_template_file,
        declare_source_root,
        declare_frame_id,
        declare_debug_waypoint_file,
        declare_auto_name_prefix,
        declare_auto_name_file,
        declare_log_level,
    ]
    for node_config in nodes_to_launch:
        parameters = list(node_config.get("parameters", []))
        parameters.append(
            {
                "use_sim_time": use_sim_time,
                "waypoint_file": waypoint_file,
                "template_file": template_file,
                "source_root": source_root,
                "debug_waypoint_file": debug_waypoint_file,
                "frame_id": frame_id,
                "auto_name_prefix": auto_name_prefix,
                "auto_name_file": auto_name_file,
            }
        )
        launch_actions.append(
            Node(
                package=node_config["package"],
                executable=node_config["executable"],
                name=node_config["name"],
                output="screen",
                emulate_tty=True,
                parameters=parameters,
                remappings=node_config.get("remappings", []),
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            )
        )

    return LaunchDescription(launch_actions)
