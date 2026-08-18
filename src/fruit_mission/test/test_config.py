import ast
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import yaml

from fruit_mission.config import c_positions_by_int, load_config, validate_config


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_package_xml_declares_runtime_dependencies():
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {element.text for element in root.findall("depend")}
    exec_dependencies = {element.text for element in root.findall("exec_depend")}

    assert {
        "rclpy",
        "std_msgs",
        "geometry_msgs",
        "fruit_perception",
        "fruit_hardware_interfaces",
    } <= dependencies
    assert {"python3-yaml", "launch", "launch_ros"} <= exec_dependencies


def test_launch_file_is_valid_python_and_loads_source_config():
    launch_source = (PACKAGE_ROOT / "launch" / "fruit_mission.launch.py").read_text(
        encoding="utf-8"
    )

    ast.parse(launch_source)
    assert "/home/rc4/fruit_ws/src/fruit_mission/config/" in launch_source


def test_config_has_required_mission_structure():
    config = load_config(str(PACKAGE_ROOT / "config" / "fruit_mission.yaml"))

    assert config["a_waypoints"]
    assert config["b_qr_waypoint"]
    assert config["c_qr_waypoint"]
    assert set(c_positions_by_int(config)) == set(range(1, 13))


def test_config_validation_rejects_invalid_side(tmp_path):
    document = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "fruit_mission.yaml").read_text(encoding="utf-8")
    )
    params = document["fruit_mission"]["ros__parameters"]
    params["c_positions"]["1"]["preferred_side"] = "middle"

    try:
        validate_config(params)
    except ValueError as error:
        assert "preferred_side" in str(error)
    else:
        raise AssertionError("invalid side was accepted")
