"""Check the waypoint HTTP contract using temporary files, without ROS."""

from dataclasses import replace
import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest
import yaml

from fruit_robviz_bridge.robviz_web_server import load_editor_config, WaypointEditorHTTPServer


@pytest.fixture
def editor(tmp_path):
    template = {"schema_version": 1, "points": [
        {"name": "A_left1", "type": "goal"}, {"name": "stop", "type": "goal"},
    ]}
    points = {"schema_version": 1, "frame_id": "odom", "points": [{
        "name": "stop", "type": "goal", "position": {"x": 2.0, "y": 3.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}, "yaw": 0.0,
    }]}
    for name in ("template.yaml", "template_acb.yaml", "template_acbd.yaml"):
        (tmp_path / name).write_text(yaml.safe_dump(template), encoding="utf-8")
    for name in (
        "odom_waypoints.yaml",
        "odom_waypoints_acb.yaml",
        "odom_waypoints_acbd.yaml",
        "debug.yaml",
    ):
        (tmp_path / name).write_text(yaml.safe_dump(points), encoding="utf-8")
    config_file = tmp_path / "web.yaml"
    config_file.write_text(yaml.safe_dump({
        "host": "127.0.0.1", "port": 8000, "static_dir": str(tmp_path),
        "waypoint_file": str(tmp_path / "odom_waypoints.yaml"),
        "debug_waypoint_file": str(tmp_path / "debug.yaml"),
        "template_file": str(tmp_path / "template.yaml"),
        "image_settings_file": str(tmp_path / "images.json"),
        "backup_dir": str(tmp_path / "backups"),
    }), encoding="utf-8")
    server = WaypointEditorHTTPServer(replace(load_editor_config(config_file), port=0))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        yield base, tmp_path
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def request(base, path, payload=None, origin=None):
    headers = {"Content-Type": "application/json"}
    if origin:
        headers["Origin"] = origin
    req = Request(base + path, headers=headers,
                  data=json.dumps(payload).encode() if payload is not None else None,
                  method="PUT" if payload is not None else "GET")
    try:
        with urlopen(req, timeout=3) as response:
            return response.status, json.load(response)
    except HTTPError as error:
        return error.code, json.load(error)


def test_catalog_and_partial_document(editor):
    base, _ = editor
    status, catalog = request(base, "/api/files")
    assert status == 200
    files = {item["id"]: item for item in catalog["files"]}
    assert files["odom_acb"]["file_name"] == "odom_waypoints_acb.yaml"
    assert files["template_acb"]["mode"] == "template"
    assert files["odom_acbd"]["file_name"] == "odom_waypoints_acbd.yaml"
    assert files["template_acbd"]["mode"] == "template"
    status, document = request(base, "/api/document?id=odom_acb")
    assert status == 200
    assert document["formal"]
    assert len(document["points"]) == 1
    assert document["template"][0]["dx"] is None
    assert request(base, "/api/document?id=template_acb")[0] == 200
    assert request(base, "/api/document?id=odom_acbd")[0] == 200
    assert request(base, "/api/document?id=template_acbd")[0] == 200
    assert request(base, "/api/document?id=../../unknown.yaml")[0] == 404


def test_save_and_version_conflict_leave_old_file_unchanged(editor):
    base, root = editor
    old = (root / "odom_waypoints.yaml").read_bytes()
    _, document = request(base, "/api/document?id=odom_acb")
    payload = {key: document[key] for key in ("id", "version", "points")}
    payload["points"][0]["x"] = 4.0
    status, saved = request(base, "/api/document", payload)
    assert status == 200
    assert (root / "backups" / "acb").is_dir()
    assert saved["version"] != document["version"]
    assert request(base, "/api/document", payload)[0] == 409
    assert (root / "odom_waypoints.yaml").read_bytes() == old
    assert request(base, "/api/document?id=odom_acb")[1]["points"][0]["x"] == 4.0


def test_acbd_document_can_be_saved(editor):
    base, root = editor
    _, document = request(base, "/api/document?id=odom_acbd")
    payload = {key: document[key] for key in ("id", "version", "points")}
    payload["points"][0]["x"] = 5.0
    status, saved = request(base, "/api/document", payload)
    assert status == 200
    assert saved["version"] != document["version"]
    assert (root / "backups" / "acbd").is_dir()
    assert request(base, "/api/document?id=odom_acbd")[1]["points"][0]["x"] == 5.0


def test_write_validation_preserves_document(editor):
    base, root = editor
    before = (root / "odom_waypoints_acb.yaml").read_bytes()
    _, document = request(base, "/api/document?id=odom_acb")
    payload = {key: document[key] for key in ("id", "version", "points")}
    assert request(base, "/api/document", payload, "http://evil.example")[0] == 403
    payload["points"][0]["type"] = "waypoint"
    assert request(base, "/api/document", payload)[0] == 422
    payload["points"][0]["name"] = "unknown"
    assert request(base, "/api/document", payload)[0] == 422
    assert (root / "odom_waypoints_acb.yaml").read_bytes() == before


def test_template_save_leaves_old_template_unchanged(editor):
    base, root = editor
    before = (root / "template.yaml").read_bytes()
    _, document = request(base, "/api/document?id=template_acb")
    payload = {key: document[key] for key in ("id", "version", "points")}
    payload["points"].append({"name": "new_stop", "type": "waypoint"})
    assert request(base, "/api/document", payload)[0] == 200
    _, saved = request(base, "/api/document?id=template_acb")
    assert saved["points"][-1]["name"] == "new_stop"
    assert (root / "template.yaml").read_bytes() == before
