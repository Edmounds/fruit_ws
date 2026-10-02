from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_axis_navigation_config_has_valid_runtime_parameters():
    params = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "axis_navigation.yaml").read_text()
    )
    controller = params["axis_waypoint_action_server"]["ros__parameters"]

    assert controller["map_frame"] == "map"
    assert controller["base_frame"] == "base_link"
    assert controller["cmd_vel_topic"] == "/cmd_vel_nav"
    for key in (
        "control_frequency",
        "max_linear_speed",
        "max_angular_speed",
        "position_tolerance",
        "yaw_tolerance",
        "tf_timeout_sec",
        "progress_timeout_sec",
    ):
        assert isinstance(controller[key], (int, float))
        assert controller[key] > 0.0
    assert controller["min_linear_speed"] >= 0.0


def test_package_installs_nodes_config_and_launch():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text()
    launch_path = PACKAGE_ROOT / "launch" / "axis_navigation.launch.py"

    compile(launch_path.read_text(), str(launch_path), "exec")
    assert "axis_waypoint_action_server" in setup_text
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text
