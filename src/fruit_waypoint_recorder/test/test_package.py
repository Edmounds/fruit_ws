from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_runtime_config_has_valid_required_parameters():
    config_path = PACKAGE_ROOT / "config" / "waypoint_recorder.yaml"
    parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "fruit_waypoint_recorder"
    ]["ros__parameters"]

    assert isinstance(parameters["goal_topic"], str) and parameters["goal_topic"]
    assert isinstance(parameters["frame_id"], str) and parameters["frame_id"]
    assert Path(parameters["output_directory"]).is_absolute()
    assert "src/fruit_waypoint_recorder/waypoints" in parameters["output_directory"]
    assert parameters["output_file"].endswith(".yaml")
    assert 0.0 < parameters["yaw_tolerance"] < 1.0


def test_package_has_launch_and_installs_runtime_files():
    launch_path = PACKAGE_ROOT / "launch" / "waypoint_recorder.launch.py"
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    compile(launch_path.read_text(encoding="utf-8"), str(launch_path), "exec")
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text
