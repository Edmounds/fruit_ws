# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT.parents[2]
PARAMS = ROOT / "pb2025_nav_bringup/config/reality/localization_params.yaml"
LAUNCH = ROOT / "pb2025_nav_bringup/launch/localization_navigation_launch.py"


def test_competition_launch_removes_nav2_planning_runtime():
    launch = LAUNCH.read_text()
    package = (ROOT / "pb2025_nav_bringup/package.xml").read_text()
    params = PARAMS.read_text()

    forbidden = (
        "nav2_controller",
        "controller_server",
        "nav2_smoother",
        "smoother_server",
        "nav2_planner",
        "planner_server",
        "nav2_behaviors",
        "behavior_server",
        "nav2_bt_navigator",
        "bt_navigator",
        "nav2_waypoint_follower",
        "waypoint_follower",
        "nav2_velocity_smoother",
        "velocity_smoother",
        "nav2_collision_monitor",
        "collision_monitor",
        "local_costmap",
        "global_costmap",
        "pb_diff_pid_pursuit_controller",
        "pb_nav2_plugins",
    )
    for text in (launch, package, params):
        for word in forbidden:
            assert word not in text

    assert not (ROOT / "pb2025_nav_bringup/behavior_trees").exists()
    assert not (ROOT / "pb_diff_pid_pursuit_controller").exists()
    assert not (ROOT / "pb_nav2_plugins").exists()


def test_competition_launch_keeps_localization_map_server_and_axis_control():
    launch = LAUNCH.read_text()
    package = (ROOT / "pb2025_nav_bringup/package.xml").read_text()

    for expected in (
        'package="livox_ros_driver2"',
        'package="point_lio"',
        'package="loam_interface"',
        'package="sensor_scan_generation"',
        'package="pb_gicp_localization"',
        'package="nav2_map_server"',
        'package="nav2_lifecycle_manager"',
        'package="fruit_navigation"',
        'executable="axis_waypoint_action_server"',
        'executable="route_marker_node"',
        'package="twist_mux"',
        '("cmd_vel_out", "cmd_vel")',
    ):
        assert expected in launch

    assert "pointcloud_to_laserscan" not in launch
    assert "/obstacle_scan" not in launch
    assert "localization_params.yaml" in launch
    assert "axis_navigation.yaml" in launch
    assert "/home/rc4/fruit_ws/src/joy_linux/config/joy_config.yaml" in launch
    assert "get_package_share_directory" not in launch

    for dependency in (
        "nav2_map_server",
        "nav2_lifecycle_manager",
        "fruit_navigation",
        "pb_gicp_localization",
        "loam_interface",
        "sensor_scan_generation",
        "twist_mux",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package


def test_localization_parameter_schema_keeps_mapping_only_scan_projection():
    params = yaml.safe_load(PARAMS.read_text())

    for key in (
        "livox_ros_driver2",
        "laserMapping",
        "loam_interface",
        "sensor_scan_generation",
        "pb_gicp_localization",
        "map_server",
        "twist_mux",
        "pointcloud_to_laserscan",
        "slam_toolbox",
    ):
        assert key in params

    cloud_to_scan = params["pointcloud_to_laserscan"]["ros__parameters"]
    assert cloud_to_scan["target_frame"] == "base_link"
    assert cloud_to_scan["min_height"] < cloud_to_scan["max_height"]
    assert 0.0 <= cloud_to_scan["range_min"] < cloud_to_scan["range_max"]

    slam = params["slam_toolbox"]["ros__parameters"]
    assert slam["scan_topic"] == "/obstacle_scan"
    assert slam["transform_publish_period"] == 0.0
    assert slam["use_scan_matching"] is False
    assert slam["do_loop_closing"] is False

    localization = params["pb_gicp_localization"]["ros__parameters"]
    assert len(localization["initial_map_to_odom"]) == 6
    assert all(
        math.isfinite(value)
        for value in localization["initial_map_to_odom"]
    )
    assert localization["max_translation_jump_initial"] >= localization[
        "max_translation_jump"
    ]


def test_mapping_launch_keeps_obstacle_scan_only_for_map_generation():
    launch = (ROOT / "pb2025_nav_bringup/launch/glim_mapping.launch.py").read_text()

    assert "localization_params.yaml" in launch
    assert 'package="pointcloud_to_laserscan"' in launch
    assert 'package="slam_toolbox"' in launch
    assert '("scan", "/obstacle_scan")' in launch


def test_navigation_start_script_uses_source_maps_axis_nav_and_desktop_rviz():
    script_path = WORKSPACE / "start_navigation.sh"
    rviz_path = ROOT / "pb2025_nav_bringup/rviz/navigation.rviz"
    script = script_path.read_text()
    rviz = rviz_path.read_text()

    assert script_path.stat().st_mode & 0o111
    assert "setup_fruit_runtime.bash" in script
    assert script.index("setup_fruit_runtime.bash") < script.index("set -u")
    assert "localization_navigation_launch.py" in script
    assert "localization_params.yaml" in script
    assert "axis_navigation.yaml" in script
    assert "fruit_hardware_bridge.launch.py" in script
    assert '"use_rviz:=false"' in script
    assert 'rviz2 -d "$RVIZ_CONFIG"' in script
    assert "DESKTOP_DISPLAY=:0" in script

    for topic in (
        "Value: map",
        "Value: gicp/localization_map",
        "Value: gicp/aligned_cloud",
        "Value: /fruit/navigation_markers",
        "Value: /fruit/final_route",
    ):
        assert topic in rviz
    assert "nav2_rviz_plugins" not in rviz
    assert "costmap" not in rviz
    assert "obstacle_scan" not in rviz


def test_waypoints_maker_uses_panel_rviz_config_and_source_paths():
    script_path = WORKSPACE / "waypoints_maker.sh"
    rviz_path = ROOT / "pb2025_nav_bringup/rviz/waypoints_maker.rviz"
    script = script_path.read_text()
    rviz = rviz_path.read_text()

    assert script_path.stat().st_mode & 0o111
    assert "setup_fruit_runtime.bash" in script
    assert script.index("setup_fruit_runtime.bash") < script.index("set -u")
    assert "waypoint_recorder.launch.py" in script
    assert "waypoints_maker.rviz" in script
    assert 'rviz2 -d "$RVIZ_CONFIG"' in script
    assert "SetGoal" not in script
    assert "Waypoint Maker panel" in script

    assert "fruit_waypoint_rviz_plugin/WaypointMakerPanel" in rviz
    assert "Value: /fruit/waypoint_maker/markers" in rviz
    assert "Value: /fruit/navigation_markers" in rviz
    assert "Value: /fruit/final_route" in rviz


def test_mapping_start_leaves_existing_maps_until_explicit_save():
    mapping_script = (WORKSPACE / "start_mapping.sh").read_text()
    save_script = (WORKSPACE / "save_glim_map.sh").read_text()

    assert "archive_existing_field_maps" not in mapping_script
    assert 'mv "${existing_maps[@]}" "$archive_dir/"' in save_script


def test_mid360_and_gicp_runtime_config_matches_live_hardware():
    params = yaml.safe_load(PARAMS.read_text())
    driver = params["livox_ros_driver2"]["ros__parameters"]
    assert driver["xfer_format"] == 1

    config = json.loads(
        (ROOT / "pb2025_nav_bringup/config/reality/mid360_user_config.json").read_text()
    )
    host = config["MID360"]["host_net_info"]
    for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
        assert host[key] == "192.168.1.5"
    assert config["lidar_configs"][0]["ip"] == "192.168.1.127"


def test_calibrated_mid360_static_transform_has_one_source():
    launch_path = ROOT / "pb2025_nav_bringup/launch/sensor_frames.launch.py"
    sensor_frames_path = ROOT / "pb2025_nav_bringup/config/reality/sensor_frames.yaml"
    launch_text = launch_path.read_text()
    sensor_frames = yaml.safe_load(sensor_frames_path.read_text())

    assert "sensor_frames.launch.py" in LAUNCH.read_text()
    assert launch_text.count('executable="static_transform_publisher"') == 2
    assert "base_to_front_mid360" in launch_text
    assert "front_mid360_to_livox_imu" in launch_text
    assert "0.144629969" not in launch_text

    expected_frames = {
        "base_to_front_mid360": ("base_link", "front_mid360"),
        "front_mid360_to_livox_imu": ("front_mid360", "livox_frame"),
    }
    for name, (parent, child) in expected_frames.items():
        transform = sensor_frames[name]
        assert transform["parent_frame"] == parent
        assert transform["child_frame"] == child
        assert len(transform["translation"]) == 3
        assert len(transform["rotation"]) == 4
        assert all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for value in transform["translation"] + transform["rotation"]
        )
