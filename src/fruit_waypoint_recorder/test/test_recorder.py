from datetime import datetime, timezone

import pytest
import yaml

from fruit_navigation.route_model import PoseData, quaternion_from_yaw
from fruit_waypoint_recorder.recorder import WaypointTemplateStore, new_session_path


def pose(name, x, y, yaw=0.0):
    q = quaternion_from_yaw(yaw)
    return PoseData(
        name=name,
        x=x,
        y=y,
        z=0.0,
        qx=q["x"],
        qy=q["y"],
        qz=q["z"],
        qw=q["w"],
        yaw=yaw,
    )


def test_store_records_eleven_anchors_generates_and_saves_template(tmp_path):
    store = WaypointTemplateStore(tmp_path / "competition_waypoints.yaml", 0.35)

    for index, name in enumerate(
        (
            "HOME",
            "A_FIRST",
            "A_LAST",
            "B_QR",
            "B_FIRST",
            "B_LAST",
            "C_QR",
            "C1",
            "C4",
            "C9",
            "C12",
        )
    ):
        assert name in store.prompt
        store.append(pose(name, float(index), float(index)))

    assert store.prompt == "All anchors recorded"
    store.save()

    saved = yaml.safe_load(store.output_path.read_text(encoding="utf-8"))
    assert saved["schema_version"] == 1
    assert saved["frame_id"] == "map"
    assert list(saved["anchors"]) == [
        "HOME",
        "A_FIRST",
        "A_LAST",
        "B_QR",
        "B_FIRST",
        "B_LAST",
        "C_QR",
        "C1",
        "C4",
        "C9",
        "C12",
    ]
    assert [point["name"] for point in saved["generated"]["A"]] == [
        "A1",
        "A2",
        "A3",
        "A4",
    ]
    assert len(saved["generated"]["C"]) == 12


def test_store_supports_undo_and_clear(tmp_path):
    store = WaypointTemplateStore(tmp_path / "competition_waypoints.yaml", 0.35)

    assert store.undo() is None
    store.append(pose("HOME", 0.0, 0.0))
    assert store.undo().name == "HOME"
    store.append(pose("HOME", 0.0, 0.0))
    store.clear()

    assert "HOME" in store.prompt


def test_store_refuses_save_until_all_anchors_exist(tmp_path):
    store = WaypointTemplateStore(tmp_path / "competition_waypoints.yaml", 0.35)

    store.append(pose("HOME", 0.0, 0.0))

    with pytest.raises(ValueError):
        store.save()


def test_new_session_path_never_overwrites_an_existing_session(tmp_path):
    now = datetime(2026, 8, 17, 19, 30, 45, tzinfo=timezone.utc)

    first = new_session_path(tmp_path, now)
    first.touch()
    second = new_session_path(tmp_path, now)

    assert first.name == "competition_waypoints_20260817_193045.yaml"
    assert second.name == "competition_waypoints_20260817_193045_01.yaml"
