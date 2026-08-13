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
