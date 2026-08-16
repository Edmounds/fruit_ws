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

import ast
import json
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
    assert localization["initial_map_to_odom"] == [0.0] * 6
    assert params["laserMapping"]["ros__parameters"]["prior_pcd"]["enable"] is False
    assert params["laserMapping"]["ros__parameters"]["publish"]["tf_send_en"] is False
    assert params["laserMapping"]["ros__parameters"]["pcd_save"]["pcd_save_en"] is False
    launch = (ROOT / "pb2025_nav_bringup/launch/localization_navigation_launch.py").read_text()
    assert '("cmd_vel", "cmd_vel_nav_raw")' in launch
    assert 'system_libusb = "/lib/x86_64-linux-gnu/libusb-1.0.so.0"' in launch
    assert 'os.environ.get("LD_PRELOAD")' in launch
    assert '("cmd_vel_smoothed", "cmd_vel_nav")' in launch
    assert '("cmd_vel_out", "cmd_vel")' in launch
    assert "default_nav_to_pose_bt_xml" in launch
    assert "default_nav_through_poses_bt_xml" in launch


def test_calibrated_mid360_static_transform_has_one_source():
    launch_path = ROOT / "pb2025_nav_bringup/launch/sensor_frames.launch.py"
    localization_launch = (
        ROOT / "pb2025_nav_bringup/launch/localization_navigation_launch.py"
    ).read_text()
    mapping_launch = (ROOT.parents[0] / "glim_ros2/launch/mapping.launch.py").read_text()

    assert "sensor_frames.launch.py" in localization_launch
    assert "sensor_frames.launch.py" in mapping_launch
    assert "static_transform_publisher" not in localization_launch
    assert "static_transform_publisher" not in mapping_launch

    tree = ast.parse(launch_path.read_text())
    static_tfs = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
        and any(
            keyword.arg == "executable"
            and ast.literal_eval(keyword.value) == "static_transform_publisher"
            for keyword in node.keywords
        )
    ]
    assert len(static_tfs) == 2

    arguments_by_variable = {
        assignment.targets[0].id: ast.literal_eval(assignment.value)
        for assignment in ast.walk(tree)
        if isinstance(assignment, ast.Assign)
        and len(assignment.targets) == 1
        and isinstance(assignment.targets[0], ast.Name)
        and assignment.targets[0].id in {"base_to_lidar", "lidar_to_imu"}
    }
    by_name = {}
    for static_tf in static_tfs:
        keywords = {keyword.arg: keyword.value for keyword in static_tf.keywords}
        arguments = keywords["arguments"]
        by_name[ast.literal_eval(keywords["name"])] = arguments_by_variable[arguments.id]

    assert by_name["base_to_front_mid360"] == [
        "--x",
        "0.144629969",
        "--y",
        "0.0",
        "--z",
        "0.0",
        "--qx",
        "-0.183012702",
        "--qy",
        "0.183012702",
        "--qz",
        "-0.683012702",
        "--qw",
        "0.683012702",
        "--frame-id",
        "base_link",
        "--child-frame-id",
        "front_mid360",
    ]
    assert by_name["front_mid360_to_livox_imu"] == [
        "--x",
        "0.011",
        "--y",
        "0.02329",
        "--z",
        "-0.04412",
        "--qx",
        "0.0",
        "--qy",
        "0.0",
        "--qz",
        "0.0",
        "--qw",
        "1.0",
        "--frame-id",
        "front_mid360",
        "--child-frame-id",
        "livox_frame",
    ]

    package = (ROOT / "pb2025_nav_bringup/package.xml").read_text()
    assert "<exec_depend>tf2_ros</exec_depend>" in package


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
    assert gicp["max_translation_jump_initial"] == 1.0
    assert gicp["lost_after_rejects"] == 10
    assert params["laserMapping"]["ros__parameters"]["mapping"]["ivox_grid_resolution"] == 0.5


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
