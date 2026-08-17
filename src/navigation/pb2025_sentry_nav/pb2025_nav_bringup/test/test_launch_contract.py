# Copyright 2025 Lihan Chen
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
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_removed_sentry_packages_and_launches_are_absent():
    for path in (
        "fake_vel_transform",
        "ign_sim_pointcloud_tool",
        "pointcloud_to_laserscan",
        "pb2025_nav_bringup/config/simulation",
        "pb2025_nav_bringup/map/simulation",
        "pb2025_nav_bringup/launch/bringup_launch.py",
        "pb2025_nav_bringup/launch/localization_launch.py",
        "pb2025_nav_bringup/launch/navigation_launch.py",
        "pb2025_nav_bringup/launch/robot_state_publisher_launch.py",
        "pb2025_nav_bringup/launch/slam_launch.py",
        "pb2025_nav_bringup/launch/rm_navigation_simulation_launch.py",
        "pb2025_nav_bringup/launch/rm_multi_navigation_simulation_launch.py",
    ):
        assert not (ROOT / path).exists()

    gitmodules_path = ROOT / ".gitmodules"
    gitmodules = gitmodules_path.read_text() if gitmodules_path.exists() else ""
    for removed_submodule in (
        "pb_omni_pid_pursuit_controller",
        "pb_teleop_twist_joy",
        "small_gicp_relocalization",
        "pointcloud_to_laserscan",
    ):
        assert removed_submodule not in gitmodules


def test_diff_drive_frames_and_velocity_contract():
    params = yaml.safe_load(
        (ROOT / "pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text()
    )
    controller = params["controller_server"]["ros__parameters"]
    assert controller["general_goal_checker"]["xy_goal_tolerance"] == 0.15
    assert controller["general_goal_checker"]["yaw_goal_tolerance"] == 0.15
    assert controller["FollowPath"]["plugin"].startswith(
        "pb_diff_pid_pursuit_controller"
    )
    velocity_limits = controller["FollowPath"]
    for key in ("v_linear_min", "v_linear_max", "v_angular_min", "v_angular_max"):
        assert isinstance(velocity_limits[key], float)
    assert velocity_limits["v_linear_min"] <= 0.0 <= velocity_limits["v_linear_max"]
    assert velocity_limits["v_angular_min"] <= 0.0 <= velocity_limits["v_angular_max"]
    smoother = params["velocity_smoother"]["ros__parameters"]
    assert smoother["max_velocity"] == [0.6, 0.0, 1.0]
    assert smoother["min_velocity"] == [-0.2, 0.0, -1.0]
    text = (ROOT / "pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text()
    for removed_frame in ("base_footprint", "chassis", "gimbal_yaw"):
        assert removed_frame not in text
    rviz = (ROOT / "pb2025_nav_bringup/rviz/nav2_default_view.rviz").read_text()
    assert "SetInitialPose" not in rviz


def test_mux_and_localization_contract():
    params = yaml.safe_load(
        (ROOT / "pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text()
    )
    topics = params["twist_mux"]["ros__parameters"]["topics"]
    assert topics["teleop"]["priority"] > topics["navigation"]["priority"]
    assert topics["teleop"]["timeout"] == 0.5
    localization = params["pb_gicp_localization"]["ros__parameters"]
    initial_map_to_odom = localization["initial_map_to_odom"]
    assert len(initial_map_to_odom) == 6
    assert all(
        isinstance(value, (int, float)) and math.isfinite(value)
        for value in initial_map_to_odom
    )
    assert params["laserMapping"]["ros__parameters"]["prior_pcd"]["enable"] is False
    assert params["laserMapping"]["ros__parameters"]["publish"]["tf_send_en"] is False
    assert params["laserMapping"]["ros__parameters"]["pcd_save"]["pcd_save_en"] is False
    launch = (ROOT / "pb2025_nav_bringup/launch/localization_navigation_launch.py").read_text()
    assert '("cmd_vel", "cmd_vel_nav_raw")' in launch
    assert 'system_libusb = "/lib/x86_64-linux-gnu/libusb-1.0.so.0"' in launch
    assert 'os.environ.get("LD_PRELOAD")' in launch
    assert '("cmd_vel_smoothed", "cmd_vel_nav")' in launch
    assert '("cmd_vel_out", "cmd_vel_muxed")' in launch
    assert 'package="nav2_collision_monitor"' in launch
    assert 'executable="collision_monitor"' in launch
    assert 'package="joy_linux"' in launch
    assert 'executable="joy_linux_node"' in launch
    assert 'executable="joy_convert_node"' in launch
    assert '("cmd_vel", "cmd_vel_teleop")' in launch
    assert 'package="joy"' not in launch
    assert "/home/rc4/fruit_ws/src/joy_linux/config/joy_config.yaml" in launch
    assert 'SOURCE_SHARE = (' in launch
    assert 'get_package_share_directory' not in launch
    assert 'DeclareLaunchArgument("initial_pose_file", default_value="")' in launch
    assert 'os.path.splitext(localization_map)[0] + ".initial_pose.yaml"' not in launch
    assert 'if initial_pose_file:' in launch
    assert '"initial_map_to_odom"' not in launch
    assert "default_nav_to_pose_bt_xml" in launch
    assert "default_nav_through_poses_bt_xml" in launch

    package = (ROOT / "pb2025_nav_bringup/package.xml").read_text()
    assert "<exec_depend>joy_linux</exec_depend>" in package
    assert "<exec_depend>nav2_collision_monitor</exec_depend>" in package
    assert "pb_diff_teleop_twist_joy" not in package

    collision_monitor = params["collision_monitor"]["ros__parameters"]
    assert collision_monitor["cmd_vel_in_topic"] == "cmd_vel_muxed"
    assert collision_monitor["cmd_vel_out_topic"] == "cmd_vel"
    assert collision_monitor["observation_sources"] == ["scan"]
    assert collision_monitor["scan"]["topic"] == "/obstacle_scan"
    for polygon in ("SlowFront", "StopFront", "StopRear"):
        assert collision_monitor[polygon]["max_points"] == 0


def test_navigation_uses_costmap_clearing_and_backup_recovery_without_drive_on_heading():
    behavior_tree_dir = ROOT / "pb2025_nav_bringup/behavior_trees"
    for filename in (
        "navigate_to_pose_w_replanning_and_recovery.xml",
        "navigate_through_poses_w_replanning_and_recovery.xml",
    ):
        tree = ET.parse(behavior_tree_dir / filename)
        root = tree.getroot()
        assert root.findall(".//PipelineSequence")
        rate_controllers = root.findall(".//RateController")
        assert rate_controllers
        assert all(float(node.attrib["hz"]) > 0.0 for node in rate_controllers)
        assert root.findall(".//RecoveryNode")
        assert root.findall(".//ClearEntireCostmap")
        assert root.findall(".//BackUp")
        assert not root.findall(".//DriveOnHeading")

    params = yaml.safe_load(
        (ROOT / "pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text()
    )
    behaviors = params["behavior_server"]["ros__parameters"]
    assert "backup" in behaviors["behavior_plugins"]
    assert "drive_on_heading" not in behaviors["behavior_plugins"]
    assert behaviors["backup"]["plugin"] == "pb_nav2_behaviors/BackUpFreeSpace"
    assert "drive_on_heading" not in behaviors
    bt_plugins = params["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    assert "nav2_back_up_action_bt_node" in bt_plugins
    assert "nav2_back_up_cancel_bt_node" in bt_plugins
    assert not any("drive_on_heading" in plugin for plugin in bt_plugins)


def test_xbox_series_stick_mapping_contract():
    joy_config = yaml.safe_load(
        (ROOT.parents[2] / "src/joy_linux/config/joy_config.yaml").read_text()
    )
    converter = joy_config["joy_convert_node"]["ros__parameters"]
    assert converter["linear_axis"] == 1
    assert converter["angular_axis"] == 2
    assert isinstance(converter["angular_z_scale"], (int, float))
    assert converter["angular_z_scale"] > 0.0


def test_calibrated_mid360_static_transform_has_one_source():
    launch_path = ROOT / "pb2025_nav_bringup/launch/sensor_frames.launch.py"
    launch_text = launch_path.read_text()
    sensor_frames_path = (
        ROOT / "pb2025_nav_bringup/config/reality/sensor_frames.yaml"
    )
    sensor_frames = yaml.safe_load(sensor_frames_path.read_text())
    localization_launch = (
        ROOT / "pb2025_nav_bringup/launch/localization_navigation_launch.py"
    ).read_text()
    mapping_launch = (ROOT.parents[0] / "glim_ros2/launch/mapping.launch.py").read_text()

    assert "sensor_frames.launch.py" in localization_launch
    assert "sensor_frames.launch.py" in mapping_launch
    assert "static_transform_publisher" not in localization_launch
    assert "static_transform_publisher" not in mapping_launch
    assert launch_text.count('executable="static_transform_publisher"') == 2
    assert "sensor_frames.yaml" in launch_text
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
        assert math.isclose(
            math.sqrt(sum(value * value for value in transform["rotation"])),
            1.0,
            abs_tol=1e-6,
        )

    assert not (ROOT / "pb2025_nav_bringup/scripts/map_initial_pose.py").exists()

    package = (ROOT / "pb2025_nav_bringup/package.xml").read_text()
    assert "<exec_depend>tf2_ros</exec_depend>" in package


def test_glim_2d_mapping_launch_contract():
    launch = (ROOT / "pb2025_nav_bringup/launch/glim_mapping.launch.py").read_text()

    assert "glim_ros2/launch/mapping.launch.py" in launch
    assert 'package="point_lio"' not in launch
    assert 'package="pointcloud_to_laserscan"' in launch
    assert 'package="slam_toolbox"' in launch
    assert 'executable="sync_slam_toolbox_node"' in launch
    assert '("cloud_in", "/glim_ros/aligned_points_odom")' in launch
    assert '("scan", "/obstacle_scan")' in launch


def test_glim_2d_mapping_parameter_schema():
    params = yaml.safe_load(
        (ROOT / "pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text()
    )
    cloud_to_scan = params["pointcloud_to_laserscan"]["ros__parameters"]
    slam = params["slam_toolbox"]["ros__parameters"]

    assert cloud_to_scan["target_frame"] == "base_link"
    assert isinstance(cloud_to_scan["min_height"], (int, float))
    assert isinstance(cloud_to_scan["max_height"], (int, float))
    assert cloud_to_scan["min_height"] < cloud_to_scan["max_height"]
    assert isinstance(cloud_to_scan["range_min"], (int, float))
    assert isinstance(cloud_to_scan["range_max"], (int, float))
    assert 0.0 <= cloud_to_scan["range_min"] < cloud_to_scan["range_max"]
    assert cloud_to_scan["range_max"] == 4.0
    assert cloud_to_scan["angle_increment"] == 0.01

    assert slam["map_frame"] == "map"
    assert slam["odom_frame"] == "odom"
    assert slam["base_frame"] == "base_link"
    assert slam["scan_topic"] == "/obstacle_scan"
    assert slam["transform_publish_period"] == 0.0
    assert slam["use_scan_matching"] is False
    assert slam["do_loop_closing"] is False


def test_costmap_observation_sources_use_obstacle_scan_and_valid_ranges():
    params = yaml.safe_load(
        (ROOT / "pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text()
    )
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = params["global_costmap"]["global_costmap"]["ros__parameters"]

    for costmap in (local, global_):
        obstacle_source = costmap["obstacle_layer"]["scan"]
        assert costmap["obstacle_layer"]["observation_sources"] == "scan"
        assert obstacle_source["topic"] == "/obstacle_scan"
        assert obstacle_source["data_type"] == "LaserScan"
        assert obstacle_source["marking"] is True
        assert obstacle_source["clearing"] is True
        for key in (
            "obstacle_min_range",
            "obstacle_max_range",
            "raytrace_min_range",
            "raytrace_max_range",
        ):
            assert isinstance(obstacle_source[key], (int, float))
            assert math.isfinite(obstacle_source[key])
        assert (
            0.0
            <= obstacle_source["obstacle_min_range"]
            < obstacle_source["obstacle_max_range"]
        )
        assert (
            0.0
            <= obstacle_source["raytrace_min_range"]
            < obstacle_source["raytrace_max_range"]
        )
        assert (
            obstacle_source["raytrace_max_range"]
            >= obstacle_source["obstacle_max_range"]
        )

    assert local["obstacle_layer"]["scan"]["observation_persistence"] == 0.2
    assert local["inflation_layer"]["inflation_radius"] == 0.05
    assert global_["inflation_layer"]["inflation_radius"] == 0.05


def test_costmaps_use_the_rectangular_robot_footprint():
    params = yaml.safe_load(
        (ROOT / "pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text()
    )
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = params["global_costmap"]["global_costmap"]["ros__parameters"]

    footprints = []
    for costmap in (local, global_):
        assert "robot_radius" not in costmap
        assert costmap["footprint_padding"] == 0.0
        footprint = yaml.safe_load(costmap["footprint"])
        assert len(footprint) == 4
        xs = [point[0] for point in footprint]
        ys = [point[1] for point in footprint]
        assert math.isclose(max(xs) - min(xs), 0.40, abs_tol=1e-5)
        assert math.isclose(max(ys) - min(ys), 0.30, abs_tol=1e-5)
        footprints.append(footprint)

    assert footprints[0] == footprints[1]


def test_mapping_save_and_zero_initialization_contract():
    workspace = ROOT.parents[2]
    start_mapping = (workspace / "start_mapping.sh").read_text()
    save_map = (workspace / "save_glim_map.sh").read_text()
    relocalization = (workspace / "start_relocalization_test.sh").read_text()
    cmake = (ROOT / "pb2025_nav_bringup/CMakeLists.txt").read_text()

    assert "pb2025_nav_bringup/launch/glim_mapping.launch.py" in start_mapping
    assert "nav2_map_server map_saver_cli" in start_mapping
    assert 'save_2d_map' in start_mapping
    assert '"$GLIM_DUMP_PATH/field"' in start_mapping
    assert 'archive_existing_field_maps' in start_mapping
    assert '"$MAP_DIR"/field.*' in start_mapping

    assert "map_frame_normalization.txt" in save_map
    for map_file in ("field.ply", "field.pcd", "field.pgm", "field.yaml"):
        assert map_file in save_map
    assert "map_initial_pose" not in save_map
    assert "initial_pose.yaml" not in save_map

    assert "INITIAL_POSE_FILE" not in relocalization
    assert "initial_pose.yaml" not in relocalization
    assert "initial_map_to_odom uses the identity transform" in relocalization
    assert "map_initial_pose.py" not in cmake

    mapping_rviz = (ROOT.parents[0] / "glim_ros2/rviz/glim_ros.rviz").read_text()
    for topic in (
        "/glim_ros/map",
        "/glim_ros/aligned_points",
        "/obstacle_scan",
        "/map",
    ):
        assert f"Value: {topic}" in mapping_rviz


def test_navigation_start_script_uses_source_maps_and_desktop_rviz():
    workspace = ROOT.parents[2]
    script_path = workspace / "start_navigation.sh"
    rviz_path = ROOT / "pb2025_nav_bringup/rviz/navigation.rviz"

    assert script_path.exists()
    assert script_path.stat().st_mode & 0o111
    assert rviz_path.exists()

    script = script_path.read_text()
    assert "setup_fruit_runtime.bash" in script
    assert script.index("setup_fruit_runtime.bash") < script.index("set -u")
    assert "localization_navigation_launch.py" in script
    assert "fruit_hardware_bridge.launch.py" in script
    assert "pb2025_nav_bringup/maps/field.yaml" in script
    assert "pb2025_nav_bringup/maps/field.pcd" in script
    assert '"use_rviz:=false"' in script
    assert 'DESKTOP_DISPLAY=:0' in script
    assert 'gdm/Xauthority' in script
    assert 'rviz2 -d "$RVIZ_CONFIG"' in script

    rviz = rviz_path.read_text()
    for topic in (
        "map",
        "obstacle_scan",
        "global_costmap/costmap",
        "local_costmap/costmap",
        "plan",
        "local_plan",
    ):
        assert f"Value: {topic}" in rviz
    assert "nav2_rviz_plugins/GoalTool" in rviz
    assert "nav2_rviz_plugins/Navigation 2" in rviz


def test_loam_interface_reads_mid360_extrinsic_from_tf():
    params = yaml.safe_load(
        (ROOT / "pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text()
    )
    loam = params["loam_interface"]["ros__parameters"]
    assert loam["imu_frame"] == "livox_frame"
    assert loam["lidar_frame"] == "front_mid360"
    assert "extrinsic_T" not in loam
    assert "extrinsic_R" not in loam

    loam_source = (ROOT / "loam_interface/src/loam_interface.cpp").read_text()
    loam_launch = (ROOT / "loam_interface/launch/loam_interface_launch.py").read_text()
    assert 'lookupTransform(\n        imu_frame_, lidar_frame_' in loam_source
    assert '"extrinsic_T"' not in loam_source
    assert '"extrinsic_R"' not in loam_source
    assert '"extrinsic_T"' not in loam_launch
    assert '"extrinsic_R"' not in loam_launch

    gicp = params["pb_gicp_localization"]["ros__parameters"]
    assert gicp["max_translation_jump_initial"] >= gicp["max_translation_jump"] >= 0.0
    assert gicp["max_rotation_jump_initial"] >= gicp["max_rotation_jump"] >= 0.0
    assert isinstance(gicp["lost_after_rejects"], int)
    assert gicp["lost_after_rejects"] > 0
    ivox_grid_resolution = params["laserMapping"]["ros__parameters"]["mapping"][
        "ivox_grid_resolution"
    ]
    assert isinstance(ivox_grid_resolution, (int, float))
    assert math.isfinite(ivox_grid_resolution)
    assert ivox_grid_resolution > 0.0


def test_mid360_runtime_config_matches_live_hardware():
    params = yaml.safe_load(
        (ROOT / "pb2025_nav_bringup/config/reality/nav2_params.yaml").read_text()
    )
    driver = params["livox_ros_driver2"]["ros__parameters"]
    assert driver["xfer_format"] == 1

    config = json.loads(
        (ROOT / "pb2025_nav_bringup/config/reality/mid360_user_config.json").read_text()
    )
    host = config["MID360"]["host_net_info"]
    for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
        assert host[key] == "192.168.1.5"
    assert config["lidar_configs"][0]["ip"] == "192.168.1.127"
