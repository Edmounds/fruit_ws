#!/usr/bin/env python3
# Copyright (c) 2026 Edmounds
# Copyright (c) 2026 fruit_ws team
#
# Licensed under the MIT License and Apache License, Version 2.0.

"""RobViz WebRTC Video Gateway & Image Bridge ROS 2 Node."""

from __future__ import annotations

import asyncio
import ipaddress
import json
import logging
import math
import os
from pathlib import Path
import threading
import time
from typing import Dict, Optional, Set, Tuple
from urllib.parse import urlparse

import aiohttp
from aiohttp import web
import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

from fruit_robviz_bridge.video_frames import (  # noqa: E402
    encode_png,
    validate_and_extract_color,
)
from fruit_robviz_bridge.video_streamer import StreamChannel  # noqa: E402
from rcl_interfaces.msg import SetParametersResult  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("FruitRobvizImageBridge")


class VideoGatewayNode(Node):
    """ROS 2 node hosting the video gateway and managing RGB/Depth image streams."""

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        super().__init__("fruit_robviz_image_bridge")
        self.loop = loop

        # Networking
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8787)
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)

        # Topics
        self.declare_parameter("camera_frame_id", "camera_color_optical_frame")
        self.declare_parameter("arm_frame_id", "arm_base_link")
        self.declare_parameter(
            "annotated_topic", "/fruit_perception/camera/annotated_image"
        )
        self.declare_parameter(
            "depth_topic", "/camera/depth/image_rect_raw"
        )
        self.declare_parameter(
            "rgb_output_topic",
            "/fruit_perception/camera/annotated_image/compressed",
        )
        self.declare_parameter(
            "depth_output_topic",
            "/fruit_perception/camera/native_depth/compressed",
        )

        self.annotated_topic = str(self.get_parameter("annotated_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.rgb_output_topic = str(self.get_parameter("rgb_output_topic").value)
        self.depth_output_topic = str(self.get_parameter("depth_output_topic").value)

        self.declare_parameter("raw_rgb_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "raw_rgb_output_topic",
            "/fruit_robviz_bridge/camera/color/image_raw/compressed",
        )
        self.declare_parameter(
            "capture_output_dir",
            "/home/rc4/fruit_ws/data/yolo_images/camera",
        )

        self.raw_rgb_topic = str(self.get_parameter("raw_rgb_topic").value)
        self.raw_rgb_output_topic = str(
            self.get_parameter("raw_rgb_output_topic").value
        )
        self.capture_output_dir = str(self.get_parameter("capture_output_dir").value)

        # Version & session tracking
        self.declare_parameter("settings_version", 0)
        self.declare_parameter("settings_session", "")
        self.applied_version = 0
        self.applied_session = ""

        # Stream parameters
        self._declare_stream_parameters("rgb", 4.0)
        self._declare_stream_parameters("depth", 1.5)

        # Stream channels
        rgb_params = self._get_stream_params("rgb")
        depth_params = self._get_stream_params("depth")

        self.channels: Dict[str, StreamChannel] = {
            "rgb": StreamChannel(
                "rgb",
                self.annotated_topic,
                self.rgb_output_topic,
                self,
                self.loop,
                rgb_params,
            ),
            "camera": StreamChannel(
                "camera",
                self.raw_rgb_topic,
                self.raw_rgb_output_topic,
                self,
                self.loop,
                rgb_params,
            ),
            "depth": StreamChannel(
                "depth",
                self.depth_topic,
                self.depth_output_topic,
                self,
                self.loop,
                depth_params,
            ),
        }

        # Topic lookup mapping
        self.topic_to_channel: Dict[str, StreamChannel] = {
            self.annotated_topic: self.channels["rgb"],
            self.rgb_output_topic: self.channels["rgb"],
            "rgb": self.channels["rgb"],
            self.raw_rgb_topic: self.channels["camera"],
            self.raw_rgb_output_topic: self.channels["camera"],
            "camera": self.channels["camera"],
            self.depth_topic: self.channels["depth"],
            self.depth_output_topic: self.channels["depth"],
            "depth": self.channels["depth"],
        }

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.server: Optional[VideoGatewayServer] = None

    def _declare_stream_parameters(self, prefix: str, default_target_bitrate: float) -> None:
        self.declare_parameter(f"{prefix}.transport", "webrtc")
        self.declare_parameter(f"{prefix}.bandwidth_mbps", 25.0)
        self.declare_parameter(f"{prefix}.max_fps", 10.0)
        self.declare_parameter(f"{prefix}.target_bitrate_mbps", default_target_bitrate)
        self.declare_parameter(f"{prefix}.quality", 90)
        self.declare_parameter(f"{prefix}.min_quality", 70)
        self.declare_parameter(f"{prefix}.auto_quality", True)

    def _get_stream_params(self, prefix: str) -> dict:
        return {
            "transport": str(self.get_parameter(f"{prefix}.transport").value),
            "bandwidth_mbps": float(self.get_parameter(f"{prefix}.bandwidth_mbps").value),
            "max_fps": float(self.get_parameter(f"{prefix}.max_fps").value),
            "target_bitrate_mbps": float(
                self.get_parameter(f"{prefix}.target_bitrate_mbps").value
            ),
            "quality": int(self.get_parameter(f"{prefix}.quality").value),
            "min_quality": int(self.get_parameter(f"{prefix}.min_quality").value),
            "auto_quality": bool(self.get_parameter(f"{prefix}.auto_quality").value),
        }

    def _on_set_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        result = SetParametersResult()
        try:
            updates: Dict[str, dict] = {"rgb": {}, "depth": {}}
            new_version = self.applied_version
            new_session = self.applied_session

            for param in parameters:
                name = param.name
                val = param.value
                if name == "settings_version":
                    new_version = int(val)
                elif name == "settings_session":
                    new_session = str(val)
                elif name.startswith("rgb."):
                    field = name[4:]
                    updates["rgb"][field] = val
                elif name.startswith("depth."):
                    field = name[6:]
                    updates["depth"][field] = val

            if new_version <= 0 or len(new_session) != 32:
                raise ValueError(
                    "Valid settings_version and 32-char settings_session required"
                )
            if new_session == self.applied_session and new_version < self.applied_version:
                raise ValueError("Stale settings version")

            # Validate each stream's merged parameters
            for stream_name in ("rgb", "depth"):
                merged = dict(self._get_stream_params(stream_name))
                merged.update(updates[stream_name])

                transport = str(merged.get("transport", "")).lower()
                if transport not in ("webrtc", "jpeg"):
                    raise ValueError(f"Invalid {stream_name}.transport: {transport}")

                bw = float(merged["bandwidth_mbps"])
                if not (0.0 < bw <= 1000.0 and math.isfinite(bw)):
                    raise ValueError(f"Invalid {stream_name}.bandwidth_mbps: {bw}")

                max_fps = float(merged["max_fps"])
                if not (0.0 < max_fps <= 60.0 and math.isfinite(max_fps)):
                    raise ValueError(f"Invalid {stream_name}.max_fps: {max_fps}")

                target_br = float(merged["target_bitrate_mbps"])
                if not (0.0 < target_br <= bw and math.isfinite(target_br)):
                    raise ValueError(
                        f"{stream_name}.target_bitrate_mbps ({target_br}) must be in (0, {bw}]"
                    )

                quality = int(merged["quality"])
                if not (30 <= quality <= 95):
                    raise ValueError(f"Invalid {stream_name}.quality: {quality}")

                min_quality = int(merged["min_quality"])
                if not (30 <= min_quality <= quality):
                    raise ValueError(
                        f"{stream_name}.min_quality ({min_quality}) must be in [30, {quality}]"
                    )

            # Apply verified settings
            for stream_name in ("rgb", "depth"):
                merged = dict(self._get_stream_params(stream_name))
                merged.update(updates[stream_name])
                self.channels[stream_name].update_parameters(merged)

            # Camera raw stream shares full rgb stream settings
            rgb_merged = dict(self._get_stream_params("rgb"))
            rgb_merged.update(updates["rgb"])
            self.channels["camera"].update_parameters(rgb_merged)

            self.applied_version = new_version
            self.applied_session = new_session

            # Broadcast metadata update to connected WebRTC/WS clients
            if self.server:
                self.loop.call_soon_threadsafe(self.server.broadcast_metadata)

            result.successful = True
            logger.info(
                "Applied settings version %d, session %s",
                new_version,
                new_session,
            )
        except Exception as exc:
            result.successful = False
            result.reason = str(exc)
            logger.warning("Rejected parameter update: %s", exc)

        return result


def _count_png_files(directory: str) -> int:
    try:
        p = Path(directory)
        if not p.is_dir():
            return 0
        return sum(
            1 for f in p.iterdir()
            if f.is_file() and f.name.lower().endswith(".png")
        )
    except OSError:
        return 0


def _get_last_png_name(directory: str) -> Optional[str]:
    try:
        p = Path(directory)
        if not p.is_dir():
            return None
        pngs = [
            f for f in p.iterdir()
            if f.is_file() and f.name.lower().endswith(".png")
        ]
        if not pngs:
            return None
        pngs.sort(key=lambda f: f.stat().st_mtime, reverse=True)
        return pngs[0].name
    except OSError:
        return None


def _save_raw_png_disk(
    msg: Image, output_dir: str
) -> Tuple[str, str, int, int, int, int]:
    """Encode raw Image to PNG and write atomically to output_dir.

    Returns:
        (final_path, file_name, width, height, stamp_sec, stamp_nanosec)
    """
    _, bgr_mat, spec = validate_and_extract_color(msg)
    width, height = spec[0], spec[1]
    stamp_sec = int(msg.header.stamp.sec)
    stamp_nanosec = int(msg.header.stamp.nanosec)

    png_bytes = encode_png(bgr_mat, compression_level=3)

    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    now_ts = time.strftime("%Y%m%d_%H%M%S")
    millis = int((time.time() % 1.0) * 1000)
    base_name = f"frame_{now_ts}_{millis:03d}"
    final_file = out_path / f"{base_name}.png"
    suffix_idx = 1
    while final_file.exists():
        final_file = out_path / f"{base_name}_{suffix_idx}.png"
        suffix_idx += 1

    temp_file = out_path / f".tmp_{final_file.name}_{os.getpid()}_{time.monotonic_ns()}"
    try:
        with open(temp_file, "wb") as f:
            f.write(png_bytes)
            f.flush()
            os.fsync(f.fileno())

        os.replace(temp_file, final_file)
    except Exception as err:
        if temp_file.exists():
            try:
                temp_file.unlink()
            except OSError:
                pass
        raise RuntimeError(f"Failed to write image file: {err}") from err

    return str(final_file), final_file.name, width, height, stamp_sec, stamp_nanosec


class VideoGatewayServer:
    """Aiohttp WebSocket signaling server for RobViz."""

    def __init__(self, node: VideoGatewayNode, loop: asyncio.AbstractEventLoop) -> None:
        self.node = node
        self.loop = loop
        self.active_websockets: Set[web.WebSocketResponse] = set()
        self.save_lock = asyncio.Lock()
        self._active_save_future: Optional[asyncio.Future] = None

        self.app = web.Application()
        self.app.router.add_get("/", self.handle_index)
        self.app.router.add_get("/ws", self.handle_ws)

    def is_save_busy(self) -> bool:
        return (
            self.save_lock.locked()
            or (
                self._active_save_future is not None
                and not self._active_save_future.done()
            )
        )

    def is_origin_allowed(self, request: web.Request) -> bool:
        """Validate request Origin to allow only local/LAN connections."""
        origin = request.headers.get("Origin")
        if not origin:
            return True  # Direct tool connection

        try:
            parsed = urlparse(origin)
            if parsed.scheme not in ("http", "https"):
                return False
            origin_host = parsed.hostname
            if not origin_host:
                return False

            # Check localhost / loopback
            if origin_host in ("localhost", "127.0.0.1", "::1"):
                return True

            # Match host header
            req_host = request.host.split(":")[0]
            if origin_host == req_host:
                return True

            # Check private LAN address ranges (RFC 1918 / RFC 4193)
            ip = ipaddress.ip_address(origin_host)
            return ip.is_private or ip.is_loopback
        except ValueError:
            return False

    def is_capture_origin_allowed(self, request: web.Request) -> bool:
        """Validate Origin for captureFrame writes from the local network."""
        origin = request.headers.get("Origin")
        if not origin:
            return False

        try:
            parsed = urlparse(origin)
            if parsed.scheme not in ("http", "https"):
                return False
            origin_host = parsed.hostname
            if not origin_host:
                return False

            req_host = request.host.split(":")[0]
            if origin_host == req_host:
                return True

            loopbacks = ("localhost", "127.0.0.1", "::1")
            if origin_host in loopbacks and req_host in loopbacks:
                return True

            origin_ip = ipaddress.ip_address(origin_host)
            request_ip = ipaddress.ip_address(req_host)
            return origin_ip.is_private and request_ip.is_private
        except ValueError:
            return False

    def build_metadata(self) -> dict:
        streams = {}
        for stream_name, ch in self.node.channels.items():
            streams[ch.raw_topic] = {
                "name": stream_name,
                "transport": ch.transport,
                "compressedTopic": ch.compressed_topic,
                "maxFps": ch.max_fps,
                "version": self.node.applied_version,
                "bandwidthMbps": ch.bandwidth_mbps,
            }
        return {
            "type": "meta",
            "version": self.node.applied_version,
            "session": self.node.applied_session,
            "streams": streams,
        }

    def broadcast_metadata(self) -> None:
        payload = self.build_metadata()
        for ws in list(self.active_websockets):
            if not ws.closed:
                asyncio.create_task(ws.send_json(payload))

    async def handle_index(self, request: web.Request) -> web.Response:
        html = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>RobViz Video Gateway Status</title>
  <style>
    body { font-family: monospace; background: #0f172a; color: #f8fafc; padding: 24px; }
    h2 { color: #38bdf8; }
    pre { background: #1e293b; padding: 16px; border-radius: 6px; }
  </style>
</head>
<body>
  <h2>RobViz Native Video Gateway</h2>
  <pre>""" + json.dumps(self.build_metadata(), indent=2) + """</pre>
</body>
</html>"""
        return web.Response(text=html, content_type="text/html")

    async def handle_ws(self, request: web.Request) -> web.WebSocketResponse:
        if not self.is_origin_allowed(request):
            logger.warning(
                "Rejected WebSocket connection from unauthorized origin: %s",
                request.headers.get("Origin"),
            )
            return web.Response(status=403, text="Forbidden Origin")

        ws = web.WebSocketResponse(heartbeat=15, max_msg_size=64 * 1024)
        await ws.prepare(request)
        self.active_websockets.add(ws)
        ws_id = str(id(ws))
        watched_channels: Set[StreamChannel] = set()
        capture_registered = False
        logger.info("Client connected: %s", ws_id)

        # Send initial stream metadata on connect
        await ws.send_json(self.build_metadata())

        async def send_json_cb(payload: dict) -> None:
            if not ws.closed:
                await ws.send_json(payload)

        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except Exception:
                        continue
                    if not isinstance(data, dict):
                        continue

                    mtype = data.get("type")
                    topic = data.get("topic")

                    if mtype == "getMeta":
                        await ws.send_json(self.build_metadata())
                        continue

                    if mtype == "getCaptureInfo":
                        req_id = data.get("requestId")
                        if not isinstance(req_id, str) or not (1 <= len(req_id) <= 128):
                            await ws.send_json(
                                {
                                    "type": "captureInfo",
                                    "requestId": "",
                                    "error": "Invalid or missing requestId",
                                }
                            )
                            continue

                        camera_ch = self.node.channels.get("camera")
                        if camera_ch is None:
                            await ws.send_json(
                                {
                                    "type": "captureInfo",
                                    "requestId": req_id,
                                    "error": "Camera stream channel not configured",
                                }
                            )
                            continue

                        if not capture_registered:
                            camera_ch.add_capture_client(ws_id)
                            capture_registered = True

                        msg, recv_time = camera_ch.get_latest_raw_frame()
                        now = time.monotonic()
                        has_frame = msg is not None and (now - recv_time <= 1.0)
                        frame_age = (now - recv_time) if msg is not None else -1.0
                        w = int(msg.width) if msg is not None else 0
                        h = int(msg.height) if msg is not None else 0

                        total_count = _count_png_files(self.node.capture_output_dir)
                        last_name = _get_last_png_name(self.node.capture_output_dir)

                        await ws.send_json(
                            {
                                "type": "captureInfo",
                                "requestId": req_id,
                                "topic": self.node.raw_rgb_topic,
                                "outputDir": self.node.capture_output_dir,
                                "totalCount": total_count,
                                "lastFileName": last_name,
                                "hasFrame": has_frame,
                                "frameAgeSec": (
                                    round(frame_age, 3) if frame_age >= 0 else None
                                ),
                                "width": w,
                                "height": h,
                            }
                        )
                        continue

                    if mtype == "unwatchCapture":
                        if capture_registered:
                            camera_ch = self.node.channels.get("camera")
                            if camera_ch:
                                camera_ch.remove_capture_client(ws_id)
                            capture_registered = False
                        continue

                    if mtype == "captureFrame":
                        req_id = data.get("requestId")
                        if not isinstance(req_id, str) or not (1 <= len(req_id) <= 128):
                            await ws.send_json(
                                {
                                    "type": "captureResult",
                                    "requestId": "",
                                    "success": False,
                                    "error": "Invalid or missing requestId",
                                }
                            )
                            continue

                        if not self.is_capture_origin_allowed(request):
                            await ws.send_json(
                                {
                                    "type": "captureResult",
                                    "requestId": req_id,
                                    "success": False,
                                    "error": "Forbidden Origin for capture write command",
                                }
                            )
                            continue

                        camera_ch = self.node.channels.get("camera")
                        if camera_ch is None:
                            await ws.send_json(
                                {
                                    "type": "captureResult",
                                    "requestId": req_id,
                                    "success": False,
                                    "error": "Camera stream channel not configured",
                                }
                            )
                            continue

                        if self.is_save_busy():
                            await ws.send_json(
                                {
                                    "type": "captureResult",
                                    "requestId": req_id,
                                    "success": False,
                                    "error": "A capture task is already in progress",
                                }
                            )
                            continue

                        now = time.monotonic()
                        msg, recv_time = camera_ch.get_latest_raw_frame()
                        if msg is None:
                            await ws.send_json(
                                {
                                    "type": "captureResult",
                                    "requestId": req_id,
                                    "success": False,
                                    "error": "No camera frame received yet",
                                }
                            )
                            continue

                        age = now - recv_time
                        if age > 1.0:
                            await ws.send_json(
                                {
                                    "type": "captureResult",
                                    "requestId": req_id,
                                    "success": False,
                                    "error": (
                                        f"Frame expired (age: {age:.3f}s > 1.0s)"
                                    ),
                                }
                            )
                            continue

                        async with self.save_lock:
                            if (
                                self._active_save_future is not None
                                and not self._active_save_future.done()
                            ):
                                await ws.send_json(
                                    {
                                        "type": "captureResult",
                                        "requestId": req_id,
                                        "success": False,
                                        "error": (
                                            "A capture task is already in progress"
                                        ),
                                    }
                                )
                                continue

                            save_fut = self.loop.create_future()
                            self._active_save_future = save_fut

                            def _worker():
                                try:
                                    return _save_raw_png_disk(
                                        msg, self.node.capture_output_dir
                                    )
                                except Exception as exc:
                                    return exc

                            async def _run_save():
                                try:
                                    res = await self.loop.run_in_executor(
                                        None, _worker
                                    )
                                    if isinstance(res, Exception):
                                        if not save_fut.done():
                                            save_fut.set_exception(res)
                                    else:
                                        if not save_fut.done():
                                            save_fut.set_result(res)
                                except BaseException as exc:
                                    if not save_fut.done():
                                        save_fut.set_exception(exc)
                                finally:
                                    if self._active_save_future is save_fut:
                                        self._active_save_future = None

                            self.loop.create_task(_run_save())

                            try:
                                fpath, fname, w, h, s_sec, s_nsec = (
                                    await asyncio.shield(save_fut)
                                )
                                total_count = _count_png_files(
                                    self.node.capture_output_dir
                                )
                                await ws.send_json(
                                    {
                                        "type": "captureResult",
                                        "requestId": req_id,
                                        "success": True,
                                        "filePath": fpath,
                                        "fileName": fname,
                                        "totalCount": total_count,
                                        "width": w,
                                        "height": h,
                                        "stampSec": s_sec,
                                        "stampNsec": s_nsec,
                                    }
                                )
                            except asyncio.CancelledError:
                                logger.info(
                                    "Client disconnected during capture; "
                                    "background save proceeds to completion"
                                )
                                raise
                            except Exception as exc:
                                logger.exception("Capture frame save failed")
                                await ws.send_json(
                                    {
                                        "type": "captureResult",
                                        "requestId": req_id,
                                        "success": False,
                                        "error": str(exc),
                                    }
                                )
                        continue

                    if not isinstance(topic, str):
                        continue

                    channel = self.node.topic_to_channel.get(topic)
                    if channel is None:
                        await ws.send_json(
                            {
                                "type": "error",
                                "topic": topic,
                                "message": f"Unauthorized or unmanaged topic: {topic}",
                            }
                        )
                        continue

                    if mtype == "watch":
                        requested_fps = float(data.get("targetFps", channel.max_fps))
                        channel.add_viewer(ws_id, send_json_cb, requested_fps)
                        watched_channels.add(channel)
                        # Notify client of channel's mode
                        await ws.send_json(
                            {
                                "type": "meta",
                                "topic": channel.raw_topic,
                                "transport": channel.transport,
                                "compressedTopic": channel.compressed_topic,
                                "maxFps": channel.max_fps,
                                "version": self.node.applied_version,
                            }
                        )
                    elif mtype == "unwatch":
                        channel.remove_viewer(ws_id)
                        watched_channels.discard(channel)
                    elif mtype == "setFps":
                        target_fps = float(data.get("targetFps", channel.max_fps))
                        channel.update_viewer_fps(ws_id, target_fps)
                    elif mtype == "answer":
                        sdp = data.get("sdp")
                        if isinstance(sdp, str) and len(sdp) <= 60_000:
                            channel.handle_answer(ws_id, sdp)
                    elif mtype == "ice":
                        candidate = data.get("candidate")
                        mline = data.get("sdpMLineIndex", 0)
                        if (
                            isinstance(candidate, str)
                            and len(candidate) <= 4096
                            and isinstance(mline, int)
                            and 0 <= mline < 16
                        ):
                            channel.handle_ice(ws_id, candidate, mline)
                    elif mtype == "telemetry":
                        channel.handle_telemetry(ws_id, data)
        finally:
            self.active_websockets.discard(ws)
            logger.info("Client disconnected: %s", ws_id)
            for ch in list(watched_channels):
                ch.remove_viewer(ws_id)
            if capture_registered:
                camera_ch = self.node.channels.get("camera")
                if camera_ch:
                    camera_ch.remove_capture_client(ws_id)

        return ws


def main(args=None) -> None:
    rclpy.init(args=args)
    Gst.init(None)

    # GLib loop in separate thread for GStreamer WebRTC events
    glib_loop = GLib.MainLoop()
    glib_thread = threading.Thread(target=glib_loop.run, daemon=True)
    glib_thread.start()

    # Asyncio event loop for aiohttp
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    node = VideoGatewayNode(loop)
    server = VideoGatewayServer(node, loop)
    node.server = server

    # ROS 2 spin in background daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app_runner = web.AppRunner(server.app)
    loop.run_until_complete(app_runner.setup())
    site = web.TCPSite(app_runner, host=node.host, port=node.port)
    loop.run_until_complete(site.start())

    logger.info(
        "RobViz Video Gateway listening on http://%s:%d (RGB: %s, Depth: %s)",
        node.host,
        node.port,
        node.annotated_topic,
        node.depth_topic,
    )

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(app_runner.cleanup())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
