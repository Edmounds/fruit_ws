"""Exercise recorder file switching over ROS using only temporary files."""

import json
from pathlib import Path
import time
import uuid

import pytest
from rcl_interfaces.srv import GetParameters
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from fruit_robviz_bridge.waypoint_recorder_node import latched_qos, WaypointRecorderNode


FILES = {
    "odom": ("odom_waypoints.yaml", "template.yaml", "A1", "waypoint"),
    "odom_acb": ("odom_waypoints_acb.yaml", "template_acb.yaml", "A_left1", "goal"),
}


@pytest.fixture(params=FILES)
def recorder(request, tmp_path):
    initial_id = request.param
    for point_file, template_file, anchor, point_type in FILES.values():
        (tmp_path / template_file).write_text(yaml.safe_dump({
            "schema_version": 1, "points": [
                {"name": anchor, "type": "goal"}, {"name": "stop", "type": point_type},
            ],
        }), encoding="utf-8")
        (tmp_path / point_file).write_text(yaml.safe_dump({
            "schema_version": 1, "frame_id": "odom", "points": [{
                "name": "stop", "type": point_type, "yaw": 0.0,
                "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }],
        }), encoding="utf-8")
    (tmp_path / "debug.yaml").write_text(yaml.safe_dump({
        "schema_version": 1, "frame_id": "odom", "points": [],
    }), encoding="utf-8")
    prefix = f"/waypoint_protocol_{uuid.uuid4().hex}"
    parameters = {
        "frame_id": "odom", "source_root": str(tmp_path),
        "waypoint_file": str(tmp_path / FILES[initial_id][0]),
        "template_file": str(tmp_path / FILES[initial_id][1]),
        "auto_name_file": str(tmp_path / FILES[initial_id][1]),
        "debug_waypoint_file": str(tmp_path / "debug.yaml"),
        "backup_dir": str(tmp_path / "backups"),
        "command_topic": f"{prefix}/command", "state_topic": f"{prefix}/state",
        "marker_topic": f"{prefix}/markers",
    }
    context = Context()
    context.init(args=[], domain_id=197)
    executor = SingleThreadedExecutor(context=context)
    node = client_node = None
    try:
        node = WaypointRecorderNode(context=context, namespace=prefix, parameter_overrides=[
            Parameter(key, value=value) for key, value in parameters.items()
        ])
        client_node = Node("client", context=context, namespace=prefix)
        executor.add_node(node)
        executor.add_node(client_node)
        states = []
        subscription = client_node.create_subscription(
            String, parameters["state_topic"],
            lambda message: states.append(json.loads(message.data)), latched_qos(),
        )
        publisher = client_node.create_publisher(String, parameters["command_topic"], 10)
        parameter_client = client_node.create_client(
            GetParameters, f"{node.get_fully_qualified_name()}/get_parameters",
        )

        def wait_state():
            deadline = time.monotonic() + 5
            while not states and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            assert states, "Recorder did not publish its state"
            return states[-1]

        def command(**payload):
            states.clear()
            publisher.publish(String(data=json.dumps(payload)))
            return wait_state()

        def service(name):
            states.clear()
            client = client_node.create_client(Trigger, f"{prefix}/fruit_waypoint_recorder/{name}")
            try:
                assert client.wait_for_service(timeout_sec=5)
                future = client.call_async(Trigger.Request())
                executor.spin_until_future_complete(future, timeout_sec=5)
                assert future.done()
                return future.result(), wait_state()
            finally:
                client_node.destroy_client(client)

        def file_parameters():
            assert parameter_client.wait_for_service(timeout_sec=5)
            future = parameter_client.call_async(GetParameters.Request(names=[
                "waypoint_file", "template_file", "auto_name_file",
            ]))
            executor.spin_until_future_complete(future, timeout_sec=5)
            assert future.done()
            return [value.string_value for value in future.result().values]

        initial_state = wait_state()
        assert subscription is not None
        assert publisher.get_subscription_count() == 1
        yield tmp_path, initial_id, initial_state, command, service, file_parameters
    finally:
        executor.shutdown()
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        context.shutdown()


def test_switch_and_save_use_matching_file_template_and_parameters(recorder):
    root, initial_id, initial, command, service, file_parameters = recorder
    assert initial["file_id"] == initial_id
    other_id = "odom_acb" if initial_id == "odom" else "odom"
    old_bytes = (root / FILES[initial_id][0]).read_bytes()
    state = command(command="load_waypoint_file", file_id=other_id, confirm_discard=True)
    point_file, template_file, anchor, point_type = FILES[other_id]
    assert state["file_id"] == other_id
    assert state["points"][0]["type"] == point_type
    assert state["waypoints"]["template"][0]["name"] == anchor
    assert state["waypoints"]["next_name"] == anchor
    assert file_parameters() == [str(root / point_file), str(root / template_file),
                                 str(root / template_file)]
    response, state = service("save")
    assert response.success
    assert Path(state["last_backup_path"]).is_file()
    assert (root / FILES[initial_id][0]).read_bytes() == old_bytes
    state = command(command="load_waypoint_file", file_id=initial_id, confirm_discard=True)
    assert state["file_id"] == initial_id
    assert state["waypoints"]["template"] == initial["waypoints"]["template"]
    state = command(command="load_waypoint_file", file_id="debug", confirm_discard=True)
    assert state["file_id"] == "debug"
    assert not state["formal"]


def test_discard_invalid_load_and_version_conflict_preserve_draft(recorder):
    root, initial_id, initial, command, service, file_parameters = recorder
    other_id = "odom_acb" if initial_id == "odom" else "odom"
    parameters_before = file_parameters()
    response, state = service("clear")
    assert response.success and state["dirty"]
    cleared_template = state["waypoints"]["template"]
    state = command(command="load_waypoint_file", file_id=other_id)
    assert state["file_id"] == initial_id and state["dirty"]
    assert state["points"] == []
    assert "confirm_discard=true required" in state["message"]
    other_file = root / FILES[other_id][0]
    document = yaml.safe_load(other_file.read_text())
    document["points"][0]["name"] = "unknown"
    other_file.write_text(yaml.safe_dump(document), encoding="utf-8")
    state = command(command="load_waypoint_file", file_id=other_id, confirm_discard=True)
    assert state["file_id"] == initial_id and state["dirty"]
    assert state["waypoints"]["template"] == cleared_template
    assert state["version"] == initial["version"]
    assert file_parameters() == parameters_before
    state = command(
        command="load_waypoint_file", file_id="../../outside.yaml", confirm_discard=True,
    )
    assert state["file_id"] == initial_id
    current_file = root / FILES[initial_id][0]
    external_bytes = current_file.read_bytes() + b"external_change: true\n"
    current_file.write_bytes(external_bytes)
    response, state = service("save")
    assert not response.success
    assert state["dirty"] and state["points"] == []
    assert current_file.read_bytes() == external_bytes


def test_reload_refreshes_template_and_automatic_names(recorder):
    root, initial_id, _, command, _, _ = recorder
    template_path = root / FILES[initial_id][1]
    document = yaml.safe_load(template_path.read_text())
    document["points"] = [document["points"][1], {"name": "new_stop", "type": "goal"}]
    template_path.write_text(yaml.safe_dump(document), encoding="utf-8")
    state = command(command="reload", confirm_discard=True)
    assert state["file_id"] == initial_id
    assert [point["name"] for point in state["waypoints"]["template"]] == ["stop", "new_stop"]
    assert state["waypoints"]["next_name"] == "new_stop"


def test_derived_file_paths_cannot_escape_source_root(recorder):
    root, initial_id, initial, command, _, _ = recorder
    other_id = "odom_acb" if initial_id == "odom" else "odom"
    other_file = root / FILES[other_id][0]
    outside = root.parent / f"outside_{uuid.uuid4().hex}.yaml"
    try:
        outside.write_bytes(other_file.read_bytes())
        other_file.unlink()
        other_file.symlink_to(outside)
        state = command(command="load_waypoint_file", file_id=other_id, confirm_discard=True)
        assert state["file_id"] == initial_id
        assert state["points"] == initial["points"]
        assert "waypoint file must be under" in state["message"]
    finally:
        outside.unlink(missing_ok=True)
