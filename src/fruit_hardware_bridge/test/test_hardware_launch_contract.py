from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_launch_starts_only_serial_bridge_from_source_config():
    launch = (
        PACKAGE_ROOT / "launch" / "fruit_hardware_bridge.launch.py"
    ).read_text(encoding="utf-8")

    assert "package': 'auto_serial_bridge'" in launch
    assert "fruit_result_bridge" not in launch
    assert "/home/rc4/fruit_ws/src/auto_serial_bridge/config/protocol.yaml" in launch
