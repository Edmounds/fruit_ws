from pathlib import Path

import yaml


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


def test_d435_rgbd_stream_is_synchronized_and_aligned_to_color():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "fruit_perception.yaml").read_text(
            encoding="utf-8"
        )
    )
    camera = config["d435i_camera"]["ros__parameters"]

    assert camera["enable_color"] is True
    assert camera["enable_depth"] is True
    assert camera["enable_sync"] is True
    assert camera["align_depth.enable"] is True
    assert camera["enable_rgbd"] is True
    assert camera["rgb_camera.color_profile"] == "640x480x60"
    assert camera["depth_module.depth_profile"] == "640x480x60"


def test_d435_perception_consumes_rgbd_and_publishes_target_point():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "fruit_perception.yaml").read_text(
            encoding="utf-8"
        )
    )
    perception = config["fruit_perception_node"]["ros__parameters"]

    assert perception["d435i_rgbd_topic"] == "/camera/camera/rgbd"
    assert (
        perception["d435i_target_point_topic"]
        == "/fruit_perception/d435i/target_point"
    )
