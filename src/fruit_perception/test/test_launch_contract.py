import os
from pathlib import Path
import signal
import subprocess

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


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


def test_d435i_manual_color_controls_are_present_and_valid():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "fruit_perception.yaml").read_text(
            encoding="utf-8"
        )
    )
    parameters = config["d435i_camera"]["ros__parameters"]

    required_controls = {
        "rgb_camera.enable_auto_exposure",
        "rgb_camera.exposure",
        "rgb_camera.gain",
        "rgb_camera.enable_auto_white_balance",
        "rgb_camera.white_balance",
    }
    assert required_controls <= parameters.keys()
    assert isinstance(parameters["rgb_camera.enable_auto_exposure"], bool)
    assert 1 <= parameters["rgb_camera.exposure"] <= 10000
    assert 0 <= parameters["rgb_camera.gain"] <= 128
    assert isinstance(parameters["rgb_camera.enable_auto_white_balance"], bool)
    assert 2800 <= parameters["rgb_camera.white_balance"] <= 6500


def test_annotated_images_support_reliable_viewers():
    source = (
        PACKAGE_ROOT / "src" / "fruit_perception_node.cpp"
    ).read_text(encoding="utf-8")
    normalized_source = " ".join(source.split())

    reliable_annotated_publisher = (
        "create_publisher<sensor_msgs::msg::Image>( "
        "annotated_topic, rclcpp::QoS(1).reliable())"
    )
    assert normalized_source.count(reliable_annotated_publisher) == 2


def test_d435i_arm_script_does_not_start_rviz():
    script = (WORKSPACE_ROOT / "test_d435i_arm_target.sh").read_text(
        encoding="utf-8"
    )

    assert "rviz2" not in script.lower()
    assert "RVIZ_" not in script
    assert "d435i_arm_debug_rviz" not in script
    assert "keep_running" in script
    assert "Press Ctrl-C to stop" in script
    assert "DETECTION_TIMEOUT_SECONDS=120" in script


def test_d435i_arm_script_starts_at_the_observation_position():
    script = (WORKSPACE_ROOT / "test_d435i_arm_target.sh").read_text(
        encoding="utf-8"
    )
    sequence = script.split("run_task_sequence()", 1)[1].split(
        "project_nodes_are_running()", 1
    )[0]

    expected_steps = [
        "report_current_area",
        "enter_tree_mode",
        "read_d435i_target",
        "send_arm_target",
    ]
    positions = [sequence.index(step) for step in expected_steps]

    assert positions == sorted(positions)
    assert "STATE_COMMAND_TOPIC" in script
    assert "{task_id: ${TASK_CURRENT_AREA}, task_status: ${AREA_A}}" in script
    assert "CURRENT_AREA_CALLBACK_ID=5" in script
    assert "task_id: 1" not in script
    assert "task_id: 2" not in script
    assert "LEFT_TYPES_TOPIC" not in script
    assert "RIGHT_TYPES_TOPIC" not in script


def test_d435i_mode_commands_wait_for_both_consumers():
    script = (WORKSPACE_ROOT / "test_d435i_arm_target.sh").read_text(
        encoding="utf-8"
    )
    mode_flow = script.split("publish_mode()", 1)[1].split(
        "wait_for_mode()", 1
    )[0]

    assert "--wait-matching-subscriptions 2" in mode_flow


def test_d435i_arm_task_progress_is_event_driven():
    script = (WORKSPACE_ROOT / "test_d435i_arm_target.sh").read_text(
        encoding="utf-8"
    )
    command_flow = script.split("send_command_and_wait()", 1)[1].split(
        "enter_tree_mode()", 1
    )[0]

    assert command_flow.index("publish_once") < command_flow.index("finish_callback")
    assert "sleep" not in command_flow
    assert "sleep 0.1" not in script


def test_d435i_target_filter_uses_the_projected_types_field():
    script = (WORKSPACE_ROOT / "test_d435i_arm_target.sh").read_text(
        encoding="utf-8"
    )
    target_flow = script.split("read_d435i_target()", 1)[1].split(
        "send_arm_target()", 1
    )[0]

    assert "--field types" in target_flow
    assert "--filter 'len(m) > 0'" in target_flow
    assert "len(m.types)" not in target_flow
    assert "D435i target subscription failed" in target_flow


def test_kill_script_finds_stopped_d435i_task_processes():
    runner = subprocess.Popen(
        [
            "bash",
            "-c",
            "exec -a 'bash ./test_d435i_arm_target.sh' sleep 30",
        ],
        start_new_session=True,
    )
    mode_helper = subprocess.Popen(
        [
            "bash",
            "-c",
            "exec -a 'ros2 topic pub --once --wait-matching-subscriptions 1 "
            "/fruit_mission/perception_mode' sleep 30",
        ],
        start_new_session=True,
    )
    processes = (runner, mode_helper)

    try:
        for process in processes:
            os.kill(process.pid, signal.SIGSTOP)

        result = subprocess.run(
            [str(WORKSPACE_ROOT / "kill_fruit_processes.sh"), "--dry-run"],
            check=True,
            capture_output=True,
            text=True,
        )

        assert str(runner.pid) in result.stdout
        assert str(mode_helper.pid) in result.stdout
    finally:
        for process in processes:
            if process.poll() is None:
                os.kill(process.pid, signal.SIGCONT)
                process.terminate()
                process.wait(timeout=5)


def test_kill_script_resumes_stopped_targets_before_interrupting_them():
    script = (WORKSPACE_ROOT / "kill_fruit_processes.sh").read_text(
        encoding="utf-8"
    )

    assert script.index("signal_targets CONT") < script.index("signal_targets INT")


def test_kill_script_finds_navigation_and_mapping_processes():
    command_lines = (
        "bash ./start_odometry.sh",
        "bash ./start_navigation.sh",
        "ros2 launch /home/rc4/fruit_ws/src/navigation/fruit_nav_bringup/"
        "launch/fruit_nav.launch.py",
        "/opt/ros/humble/lib/pointcloud_to_laserscan/"
        "pointcloud_to_laserscan_node",
    )

    processes = [
        subprocess.Popen(
            [
                "bash",
                "-c",
                f"exec -a '{command_line}' sleep 30",
            ],
            start_new_session=True,
        )
        for command_line in command_lines
    ]

    try:
        result = subprocess.run(
            [str(WORKSPACE_ROOT / "kill_fruit_processes.sh"), "--dry-run"],
            check=True,
            capture_output=True,
            text=True,
        )

        for process in processes:
            assert str(process.pid) in result.stdout
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
