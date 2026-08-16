from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_camera_and_perception_use_one_intra_process_container():
    launch = (
        PACKAGE_ROOT / "launch" / "fruit_perception.launch.py"
    ).read_text(encoding="utf-8")

    assert "ComposableNodeContainer(" in launch
    assert "component_container" in launch
    assert "realsense2_camera::RealSenseNodeFactory" in launch
    assert "fruit_perception::FruitPerceptionNode" in launch
    assert "for node_cfg in nodes_to_launch:" in launch
    assert "extra_arguments=[{'use_intra_process_comms': True}]" in launch


def test_component_registration_keeps_standalone_executable():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "rclcpp_components_register_node(" in cmake
    assert 'PLUGIN "fruit_perception::FruitPerceptionNode"' in cmake
    assert "EXECUTABLE fruit_perception_node" in cmake


def test_serial_port_comes_from_source_protocol_config():
    launch = (
        PACKAGE_ROOT / "launch" / "fruit_perception.launch.py"
    ).read_text(encoding="utf-8")

    assert "/home/rc4/fruit_ws/src/auto_serial_bridge/config/protocol.yaml" in launch
    assert "serial_parameters['port']" not in launch


def test_perception_config_is_loaded_from_source_workspace():
    launch = (
        PACKAGE_ROOT / "launch" / "fruit_perception.launch.py"
    ).read_text(encoding="utf-8")

    assert "'/home/rc4/fruit_ws/src/fruit_perception/config/'" in launch
    assert "'fruit_perception.yaml'" in launch


def test_serial_bridge_uses_targeted_debug_logging():
    launch = (
        PACKAGE_ROOT / "launch" / "fruit_perception.launch.py"
    ).read_text(encoding="utf-8")

    assert "serial_controller:=debug" in launch
    assert "'--log-level', 'debug'" not in launch


def test_serial_launch_does_not_forward_d435_results_to_lower_controller():
    launch = (
        PACKAGE_ROOT / "launch" / "fruit_perception.launch.py"
    ).read_text(encoding="utf-8")
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

    assert "package='auto_serial_bridge'" in launch
    assert "fruit_result_bridge" not in launch
    assert "fruit_hardware_bridge.yaml" not in launch
    assert "<exec_depend>auto_serial_bridge</exec_depend>" in package_xml


def test_usb_capture_services_are_unique_and_mode_manager_is_launched():
    launch = (
        PACKAGE_ROOT / "launch" / "fruit_perception.launch.py"
    ).read_text(encoding="utf-8")

    assert "('/set_capture', '/left_camera/set_capture')" in launch
    assert "('/set_capture', '/right_camera/set_capture')" in launch
    assert "executable='camera_mode_manager'" in launch


def test_qr_planner_uses_d435i_raw_color_topic():
    launch = (
        PACKAGE_ROOT / "launch" / "fruit_perception.launch.py"
    ).read_text(encoding="utf-8")

    assert "package='fruit_qr_planner'" in launch
    assert "'/home/rc4/fruit_ws/src/fruit_qr_planner/config/'" in launch
