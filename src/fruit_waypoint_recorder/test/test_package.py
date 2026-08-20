from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_runtime_config_has_valid_required_parameters():
    config_path = PACKAGE_ROOT / "config" / "waypoint_recorder.yaml"
    parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "fruit_waypoint_recorder"
    ]["ros__parameters"]

    assert isinstance(parameters["frame_id"], str) and parameters["frame_id"]
    assert parameters["base_frame"] == "base_link"
    assert parameters["command_topic"] == "/fruit/waypoint_maker/command"
    assert parameters["state_topic"] == "/fruit/waypoint_maker/state"
    assert parameters["marker_topic"] == "/fruit/waypoint_maker/markers"
    assert Path(parameters["source_root"]).is_absolute()
    assert parameters["source_root"].endswith("/src")
    waypoint_file = Path(parameters["waypoint_file"])
    assert waypoint_file.is_absolute()
    assert waypoint_file.suffix == ".yaml"
    assert parameters["sample_duration_sec"] == 1.0
    assert parameters["sample_period_sec"] > 0.0


def test_package_has_launch_and_installs_runtime_files():
    launch_path = PACKAGE_ROOT / "launch" / "waypoint_recorder.launch.py"
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    compile(launch_path.read_text(encoding="utf-8"), str(launch_path), "exec")
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text
