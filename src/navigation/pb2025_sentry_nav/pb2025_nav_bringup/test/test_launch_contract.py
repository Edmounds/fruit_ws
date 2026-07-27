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

    gitmodules = (ROOT / ".gitmodules").read_text()
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
    assert '("cmd_vel_smoothed", "cmd_vel_nav")' in launch
    assert '("cmd_vel_out", "cmd_vel")' in launch
    assert "default_nav_to_pose_bt_xml" in launch
    assert "default_nav_through_poses_bt_xml" in launch
