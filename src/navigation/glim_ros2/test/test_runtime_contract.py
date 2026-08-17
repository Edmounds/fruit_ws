from pathlib import Path


NAVIGATION_DIR = Path(__file__).resolve().parents[2]
GLIM_DIR = NAVIGATION_DIR / "glim"
GLIM_ROS_DIR = NAVIGATION_DIR / "glim_ros2"


def test_cpu_backend_is_the_default():
    config = (GLIM_DIR / "config" / "config.json").read_text()
    glim_cmake = (GLIM_DIR / "CMakeLists.txt").read_text()
    glim_ros_cmake = (GLIM_ROS_DIR / "CMakeLists.txt").read_text()

    assert "config_odometry_cpu.json" in config
    assert "config_sub_mapping_cpu.json" in config
    assert "config_global_mapping_cpu.json" in config
    assert 'option(BUILD_WITH_CUDA "Build with GPU support" OFF)' in glim_cmake
    assert 'option(BUILD_WITH_CUDA "Build with GPU support" OFF)' in glim_ros_cmake


def test_mid360_topics_and_internal_extrinsic_are_configured():
    ros_config = (GLIM_DIR / "config" / "config_ros.json").read_text()
    sensor_config = (GLIM_DIR / "config" / "config_sensors.json").read_text()

    assert '"imu_topic": "/livox/imu"' in ros_config
    assert '"points_topic": "/livox/lidar"' in ros_config
    assert '"base_frame_id": "base_link"' in ros_config
    assert "align_world_to_base" not in ros_config
    assert '"acc_scale": 9.80665' in ros_config
    assert "0.011" in sensor_config
    assert "0.02329" in sensor_config
    assert "-0.04412" in sensor_config
    assert '"librviz_viewer.so"' in ros_config
    assert '"libstandard_viewer.so"' not in ros_config


def test_launch_uses_source_config_and_disables_camera_input():
    launch_file = GLIM_ROS_DIR / "launch" / "mapping.launch.py"
    cmake = (GLIM_ROS_DIR / "CMakeLists.txt").read_text()
    package = (GLIM_ROS_DIR / "package.xml").read_text()

    assert launch_file.is_file()
    launch = launch_file.read_text()
    assert "/home/rc4/fruit_ws/src/navigation/glim/config" in launch
    assert "/home/rc4/fruit_ws/tmp/glim_dump" in launch
    assert "sensor_frames.launch.py" in launch
    assert "static_transform_publisher" not in launch
    assert "0.144629969" not in launch
    assert "0.183012702" not in launch
    assert "BUILD_WITH_CV_BRIDGE" not in cmake
    assert "<depend>cv_bridge</depend>" not in package
    assert "<depend>image_transport</depend>" not in package


def test_exported_glim_target_keeps_the_system_opencv_abi():
    package_config = (GLIM_DIR / "cmake" / "glim-config.cmake.in").read_text()

    assert "find_dependency(OpenCV 4.5.4 EXACT REQUIRED COMPONENTS core" in package_config
    assert 'PATHS "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4"' in package_config
    assert "NO_DEFAULT_PATH" in package_config


def test_offline_viewer_supports_noninteractive_point_export():
    offline_viewer = (GLIM_ROS_DIR / "src" / "offline_viewer.cpp").read_text()
    offline_viewer_core = (GLIM_DIR / "src" / "glim" / "viewer" / "offline_viewer.cpp").read_text()

    assert '"export_path"' in offline_viewer
    assert "glim::OfflineViewer viewer(init_map_path, export_map_path);" in offline_viewer
    assert "guik::ProgressInterface progress;" in offline_viewer_core
    assert 'open<std::string>("auto_export"' not in offline_viewer_core
    assert "loadBaseFrameAlignment" not in offline_viewer
    assert "T_export_world" not in offline_viewer_core
    assert not (GLIM_ROS_DIR / "include/glim_ros/base_frame_alignment.hpp").exists()
    assert not (GLIM_ROS_DIR / "test/base_frame_alignment_test.cpp").exists()


def test_rviz_map_publication_does_not_require_subscriber_during_submap_update():
    rviz_viewer = (GLIM_ROS_DIR / "src" / "glim_ros" / "rviz_viewer.cpp").read_text()
    rviz_header = (GLIM_ROS_DIR / "include" / "glim_ros" / "rviz_viewer.hpp").read_text()

    assert "void publish_global_map(const rclcpp::Time& stamp);" in rviz_header
    assert "std::vector<gtsam_points::PointCloud::ConstPtr> cached_submaps;" in rviz_header
    assert "cached_submap_poses" in rviz_header
    assert "this->cached_submaps = submap_frames;" in rviz_viewer
    assert "publish_global_map(now);" in rviz_viewer
    assert "!map_pub || !map_pub->get_subscription_count()" in rviz_viewer
    assert (
        "map_pub && map_pub->get_subscription_count() && !cached_submaps.empty()"
        in rviz_viewer
    )
    assert "this->submaps.push_back(latest_submap->frame);" not in rviz_viewer
