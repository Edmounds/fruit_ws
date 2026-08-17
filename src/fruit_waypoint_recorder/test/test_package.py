from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_runtime_config_has_valid_required_parameters():
    config_path = PACKAGE_ROOT / "config" / "waypoint_recorder.yaml"
    parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "waypoint_recorder"
    ]["ros__parameters"]

    assert isinstance(parameters["joy_topic"], str) and parameters["joy_topic"]
    assert isinstance(parameters["frame_id"], str) and parameters["frame_id"]
    assert isinstance(parameters["child_frame_id"], str) and parameters["child_frame_id"]
    assert Path(parameters["output_directory"]).is_absolute()
    assert "src/fruit_waypoint_recorder/waypoints" in parameters["output_directory"]
    assert isinstance(parameters["save_button"], int) and parameters["save_button"] >= 0
    assert isinstance(parameters["undo_axis"], int) and parameters["undo_axis"] >= 0
    assert parameters["undo_direction"] in (-1.0, 1.0)
    assert 0.0 < parameters["axis_threshold"] <= 1.0
    assert parameters["transform_timeout_sec"] > 0.0


def test_package_has_launch_and_installs_runtime_files():
    launch_path = PACKAGE_ROOT / "launch" / "waypoint_recorder.launch.py"
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    compile(launch_path.read_text(encoding="utf-8"), str(launch_path), "exec")
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text
