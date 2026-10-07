#!/usr/bin/env python3

"""Reload RobViz image settings through the local HTTP API on demand."""

from __future__ import annotations

import json
import threading
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


MAX_RESPONSE_BYTES = 64 * 1024
STREAM_FIELDS = (
    "bandwidth_mbps",
    "auto_quality",
    "quality",
    "width",
    "height",
    "max_fps",
)


class RobvizConfigClient(Node):
    """Bridge the explicit reload service to the RobViz-owned JSON settings."""

    def __init__(self) -> None:
        super().__init__("fruit_robviz_config_client")
        self.declare_parameter(
            "config_url", "http://127.0.0.1:8000/api/image-settings"
        )
        self.declare_parameter("image_node", "/fruit_robviz_image_bridge")
        self.declare_parameter("request_timeout_sec", 5.0)
        timeout = float(self.get_parameter("request_timeout_sec").value)
        if not 0.1 <= timeout <= 10.0:
            raise ValueError("request_timeout_sec must be in [0.1, 10]")
        self._config_url = str(self.get_parameter("config_url").value)
        self._timeout = timeout
        self._callback_group = ReentrantCallbackGroup()
        self._parameter_client = self.create_client(
            SetParametersAtomically,
            f"{str(self.get_parameter('image_node').value).rstrip('/')}/"
            "set_parameters_atomically",
            callback_group=self._callback_group,
        )
        self._reload_service = self.create_service(
            Trigger,
            "/fruit_robviz_bridge/reload_image_settings",
            self._on_reload,
            callback_group=self._callback_group,
        )

    def _fetch_settings(self) -> dict:
        request = Request(
            self._config_url,
            headers={"Accept": "application/json"},
            method="GET",
        )
        try:
            with urlopen(request, timeout=self._timeout) as response:
                if response.headers.get_content_type() != "application/json":
                    raise RuntimeError("RobViz settings API returned non-JSON data")
                content_length = response.headers.get("Content-Length")
                if content_length is not None and int(content_length) > MAX_RESPONSE_BYTES:
                    raise RuntimeError("RobViz settings response is too large")
                raw = response.read(MAX_RESPONSE_BYTES + 1)
        except (HTTPError, URLError, OSError) as error:
            raise RuntimeError(f"RobViz settings request failed: {error}") from error
        if len(raw) > MAX_RESPONSE_BYTES:
            raise RuntimeError("RobViz settings response is too large")
        try:
            payload = json.loads(raw.decode("utf-8"), parse_constant=self._reject_constant)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            raise RuntimeError(f"RobViz settings JSON is invalid: {error}") from error
        if not isinstance(payload, dict) or not isinstance(payload.get("settings"), dict):
            raise RuntimeError("RobViz settings response has an invalid shape")
        return payload

    @staticmethod
    def _reject_constant(value: str):
        raise ValueError(f"JSON constant {value} is not allowed")

    @staticmethod
    def _parameter_value(value) -> ParameterValue:
        parameter = ParameterValue()
        if isinstance(value, bool):
            parameter.type = ParameterType.PARAMETER_BOOL
            parameter.bool_value = value
        elif isinstance(value, int):
            parameter.type = ParameterType.PARAMETER_INTEGER
            parameter.integer_value = value
        elif isinstance(value, float):
            parameter.type = ParameterType.PARAMETER_DOUBLE
            parameter.double_value = value
        else:
            raise ValueError("image setting values must be numeric or boolean")
        return parameter

    def _to_request(self, settings: dict) -> SetParametersAtomically.Request:
        parameters = []
        for stream_name in ("rgb", "depth"):
            stream = settings.get(stream_name)
            if not isinstance(stream, dict):
                raise ValueError(f"missing {stream_name} settings")
            for field in STREAM_FIELDS:
                if field not in stream:
                    raise ValueError(f"missing {stream_name}.{field}")
                parameters.append(
                    Parameter(
                        name=f"{stream_name}.{field}",
                        value=self._parameter_value(stream[field]),
                    )
                )
        request = SetParametersAtomically.Request()
        request.parameters = parameters
        return request

    def _on_reload(self, _request: Trigger.Request, response: Trigger.Response):
        try:
            payload = self._fetch_settings()
            version = payload.get("version", payload["settings"].get("version"))
            if not isinstance(version, int) or isinstance(version, bool) or version < 1:
                raise ValueError("settings version is invalid")
            if not self._parameter_client.wait_for_service(timeout_sec=self._timeout):
                raise RuntimeError("image bridge parameter service is unavailable")
            future = self._parameter_client.call_async(self._to_request(payload["settings"]))
            done = threading.Event()
            future.add_done_callback(lambda _future: done.set())
            if not done.wait(self._timeout):
                raise RuntimeError("image bridge parameter update timed out")
            result = future.result()
            if result is None or not result.result.successful:
                reason = result.result.reason if result is not None else "no response"
                raise RuntimeError(f"image bridge rejected settings: {reason}")
            response.success = True
            response.message = f"applied RobViz image settings version {version}"
        except Exception as error:
            self.get_logger().error(f"RobViz image settings reload failed: {error}")
            response.success = False
            response.message = str(error)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobvizConfigClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
