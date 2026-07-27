import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _launch_nodes(context):
    map_file = LaunchConfiguration("map").perform(context)
    localization_map = LaunchConfiguration("localization_map").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    for label, path in (
        ("map", map_file),
        ("localization_map", localization_map),
        ("params_file", params_file),
    ):
        if not os.path.isfile(path):
            raise RuntimeError(f"{label} does not exist: {path}")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"yaml_filename": map_file},
        convert_types=True,
    )
    common = {"parameters": [configured_params], "output": "screen"}
    behavior_tree_dir = os.path.join(
        get_package_share_directory("pb2025_nav_bringup"), "behavior_trees"
    )
    mid360_config = os.path.join(
        get_package_share_directory("pb2025_nav_bringup"),
        "config",
        "reality",
        "mid360_user_config.json",
    )

    nodes = [
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_ros_driver2",
            parameters=[configured_params, {"user_config_path": mid360_config}],
            output="screen",
        ),
        Node(package="point_lio", executable="pointlio_mapping", name="laserMapping", **common),
        Node(
            package="loam_interface",
            executable="loam_interface_node",
            name="loam_interface",
            **common,
        ),
        Node(
            package="sensor_scan_generation",
            executable="sensor_scan_generation_node",
            name="sensor_scan_generation",
            **common,
        ),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            name="terrainAnalysis",
            **common,
        ),
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            name="terrainAnalysisExt",
            **common,
        ),
        Node(
            package="pb_gicp_localization",
            executable="pb_gicp_localization_node",
            name="pb_gicp_localization",
            parameters=[configured_params, {"localization_map": localization_map}],
            output="screen",
        ),
        Node(package="nav2_map_server", executable="map_server", name="map_server", **common),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            remappings=[("cmd_vel", "cmd_vel_nav_raw")],
            **common,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            **common,
        ),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", **common),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            **common,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=[
                configured_params,
                {
                    "default_nav_to_pose_bt_xml": os.path.join(
                        behavior_tree_dir,
                        "navigate_to_pose_w_replanning_and_recovery.xml",
                    ),
                    "default_nav_through_poses_bt_xml": os.path.join(
                        behavior_tree_dir,
                        "navigate_through_poses_w_replanning_and_recovery.xml",
                    ),
                },
            ],
            output="screen",
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            **common,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            remappings=[
                ("cmd_vel", "cmd_vel_nav_raw"),
                ("cmd_vel_smoothed", "cmd_vel_nav"),
            ],
            **common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[
                {"autostart": True},
                {
                    "node_names": [
                        "map_server",
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ]
                },
            ],
            output="screen",
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            parameters=[configured_params],
            remappings=[("cmd_vel_out", "cmd_vel")],
            output="screen",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            condition=IfCondition(LaunchConfiguration("use_teleop")),
            parameters=[{"deadzone": 0.3, "autorepeat_rate": 20.0}],
        ),
        Node(
            package="pb_diff_teleop_twist_joy",
            executable="pb_diff_teleop_twist_joy_node",
            name="pb_diff_teleop_twist_joy",
            condition=IfCondition(LaunchConfiguration("use_teleop")),
            parameters=[configured_params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]
    return nodes


def generate_launch_description():
    share = get_package_share_directory("pb2025_nav_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Absolute path to the Nav2 map YAML"),
            DeclareLaunchArgument(
                "localization_map", description="Absolute path to the GLIM-derived PCD"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share, "config", "reality", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument("use_teleop", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(share, "rviz", "nav2_default_view.rviz"),
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
