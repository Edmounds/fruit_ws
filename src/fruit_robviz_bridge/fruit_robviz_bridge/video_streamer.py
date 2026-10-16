# Copyright (c) 2026 Edmounds
# Copyright (c) 2026 fruit_ws team
#
# Licensed under the MIT License and Apache License, Version 2.0.

"""GStreamer WebRTC streaming engine for ROS 2 image streams."""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from typing import Callable, Dict, Optional, Set, Tuple

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstWebRTC", "1.0")
gi.require_version("GstSdp", "1.0")
from gi.repository import Gst, GstSdp, GstWebRTC  # noqa: E402

from fruit_robviz_bridge.video_frames import (  # noqa: E402
    convert_depth_to_jet,
    validate_and_extract_color,
)
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image  # noqa: E402

logger = logging.getLogger("VideoStreamer")


class StreamChannel:
    """Manages one raw image source for WebRTC output."""

    def __init__(
        self,
        name: str,
        raw_topic: str,
        node: Node,
        loop: asyncio.AbstractEventLoop,
        initial_params: dict,
    ) -> None:
        self.name = name
        self.raw_topic = raw_topic
        self.node = node
        self.loop = loop

        # Configuration parameters
        self.transport = "webrtc"
        self.bandwidth_mbps = float(initial_params.get("bandwidth_mbps", 25.0))
        self.max_fps = float(initial_params.get("max_fps", 10.0))
        self.target_bitrate_mbps = float(
            initial_params.get(
                "target_bitrate_mbps", 4.0 if name == "rgb" else 1.5
            )
        )
        self.settings_applied = False

        # WebRTC Bitrate bounds (kbps)
        target_kbps = int(self.target_bitrate_mbps * 1000)
        self.max_bitrate_kbps = min(int(self.bandwidth_mbps * 1000), target_kbps)
        self.min_bitrate_kbps = max(150, int(self.max_bitrate_kbps * 0.15))
        self.current_bitrate_kbps = self.max_bitrate_kbps

        # State and synchronization
        self.lock = threading.Lock()
        self.latest_msg: Optional[Image] = None
        self.latest_raw_msg: Optional[Image] = None
        self.latest_raw_time: float = 0.0
        self.capture_viewers: Set[str] = set()
        self.last_admitted_time = 0.0
        self.viewers: Dict[str, dict] = {}
        self.peer_counter = 0
        self.viewer_telemetry: Dict[str, dict] = {}
        self.last_bitrate_adjustment = 0.0

        # ROS 2 Image subscription (created once and kept for node lifetime)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.node.create_subscription(
            Image, self.raw_topic, self.on_raw_image, qos
        )

        # GStreamer pipeline
        self.pipeline: Optional[Gst.Pipeline] = None
        self.appsrc: Optional[Gst.Element] = None
        self.encoder: Optional[Gst.Element] = None
        self.rtp_tee: Optional[Gst.Element] = None
        self.bus: Optional[Gst.Bus] = None
        self.video_spec: Optional[Tuple[int, int, str]] = None

        # Worker push task
        self.push_task: Optional[asyncio.Task] = None
        self.is_active = False

    def update_parameters(self, params: dict) -> None:
        """Update stream configuration atomically."""
        self.bandwidth_mbps = float(
            params.get("bandwidth_mbps", self.bandwidth_mbps)
        )
        self.max_fps = float(params.get("max_fps", self.max_fps))
        self.target_bitrate_mbps = float(
            params.get("target_bitrate_mbps", self.target_bitrate_mbps)
        )
        self.settings_applied = True

        target_kbps = int(self.target_bitrate_mbps * 1000)
        self.max_bitrate_kbps = min(int(self.bandwidth_mbps * 1000), target_kbps)
        self.min_bitrate_kbps = max(150, int(self.max_bitrate_kbps * 0.15))
        self.current_bitrate_kbps = min(
            self.max_bitrate_kbps,
            max(self.min_bitrate_kbps, self.current_bitrate_kbps),
        )

        if self.encoder:
            try:
                self.encoder.set_property("bitrate", self.current_bitrate_kbps)
            except Exception:
                logger.exception(
                    "[%s] Could not update encoder bitrate", self.name
                )

        self.update_demand()

    def add_capture_client(self, viewer_id: str) -> None:
        """Register a client needing raw frame caching."""
        self.capture_viewers.add(viewer_id)
        self.update_demand()

    def remove_capture_client(self, viewer_id: str) -> None:
        """Unregister a client needing raw frame caching."""
        self.capture_viewers.discard(viewer_id)
        self.update_demand()

    def get_latest_raw_frame(self) -> Tuple[Optional[Image], float]:
        """Return the latest unthrottled raw frame and its monotonic receive time."""
        with self.lock:
            return self.latest_raw_msg, self.latest_raw_time

    def has_video_demand(self) -> bool:
        """Return True if there is an active subscriber/viewer for this stream video."""
        if not self.settings_applied:
            return False
        return len(self.viewers) > 0

    def has_demand(self) -> bool:
        """Return True if there is video demand or raw capture demand."""
        return self.has_video_demand() or len(self.capture_viewers) > 0

    def update_demand(self) -> None:
        """Ensure streaming pipeline is active only when video demand exists."""
        demand = self.has_video_demand()
        if demand and not self.is_active:
            self.start_active()
        elif not demand and self.is_active:
            self.stop_active()

    def start_active(self) -> None:
        if self.is_active:
            return
        self.is_active = True
        logger.info("[%s] Activating stream encoding: %s", self.name, self.raw_topic)
        if self.push_task is None or self.push_task.done():
            self.push_task = self.loop.create_task(self.frame_push_loop())

    def stop_active(self) -> None:
        if not self.is_active:
            return
        self.is_active = False
        logger.info("[%s] Deactivating stream encoding: %s", self.name, self.raw_topic)
        if self.push_task is not None:
            self.push_task.cancel()
            self.push_task = None
        with self.lock:
            self.latest_msg = None
        self.teardown_pipeline()

    def destroy(self) -> None:
        """Tear down pipeline and release subscription when ROS executor is stopped."""
        self.stop_active()
        if self.subscription is not None:
            try:
                self.node.destroy_subscription(self.subscription)
            except Exception:
                pass
            self.subscription = None

    def on_raw_image(self, msg: Image) -> None:
        now = time.monotonic()
        with self.lock:
            self.latest_raw_msg = msg
            self.latest_raw_time = now

        if not self.has_video_demand():
            return

        interval = 1.0 / max(0.1, self.max_fps)
        if now - self.last_admitted_time < interval:
            return  # Drop frame before processing
        self.last_admitted_time = now
        with self.lock:
            self.latest_msg = msg

    def add_viewer(
        self,
        viewer_id: str,
        send_callback: Callable[[dict], None],
        requested_fps: float,
        requested_topic: str = "",
    ) -> None:
        target_topic = requested_topic if requested_topic else self.raw_topic
        if viewer_id in self.viewers:
            self.viewers[viewer_id]["targetFps"] = requested_fps
            self.viewers[viewer_id]["topic"] = target_topic
            return
        self.viewers[viewer_id] = {
            "targetFps": requested_fps,
            "send": send_callback,
            "topic": target_topic,
            "webrtc": None,
            "queue": None,
            "capsfilter": None,
            "tee_pad": None,
            "pending_ice": [],
            "remote_description_set": False,
        }
        logger.info(
            "[%s] Added viewer %s for topic %s (total: %d)",
            self.name,
            viewer_id,
            target_topic,
            len(self.viewers),
        )
        self.update_demand()
        if self.transport == "webrtc" and self.pipeline:
            self.attach_viewer(viewer_id)

    def remove_viewer(self, viewer_id: str) -> None:
        self.viewer_telemetry.pop(viewer_id, None)
        self.detach_viewer(viewer_id)
        self.viewers.pop(viewer_id, None)
        logger.info(
            "[%s] Removed viewer %s (remaining: %d)",
            self.name,
            viewer_id,
            len(self.viewers),
        )
        self.update_demand()

    def update_viewer_fps(self, viewer_id: str, target_fps: float) -> None:
        if viewer_id in self.viewers:
            self.viewers[viewer_id]["targetFps"] = target_fps

    def notify_all(self, payload: dict) -> None:
        for viewer in list(self.viewers.values()):
            asyncio.run_coroutine_threadsafe(viewer["send"](payload), self.loop)

    def notify_error(self, message: str, viewer_id: Optional[str] = None) -> None:
        logger.error(
            "[%s] Stream error for viewer %s: %s",
            self.name,
            viewer_id or "all",
            message,
        )
        targets = (
            [self.viewers[viewer_id]]
            if viewer_id in self.viewers
            else list(self.viewers.values())
        )
        for viewer in targets:
            payload = {
                "type": "error",
                "topic": viewer.get("topic", self.raw_topic),
                "message": message,
            }
            asyncio.run_coroutine_threadsafe(viewer["send"](payload), self.loop)

    def setup_pipeline(self, width: int, height: int, image_format: str) -> None:
        fps_fraction = f"{int(max(1, round(self.max_fps)))}/1"
        caps_str = (
            f"video/x-raw,format={image_format},width={width},height={height},"
            f"framerate={fps_fraction}"
        )
        pipe_str = f"""
            appsrc name=src is-live=true format=time caps="{caps_str}" do-timestamp=false
                block=false max-buffers=1 max-bytes=0 max-time=0 leaky-type=downstream !
            videoconvert !
            x264enc name=enc tune=zerolatency bitrate={self.current_bitrate_kbps}
                speed-preset=ultrafast key-int-max=30 bframes=0 !
            video/x-h264,profile=constrained-baseline !
            h264parse config-interval=1 ! rtph264pay config-interval=1 pt=96 !
            tee name=rtptee allow-not-linked=true
        """
        logger.info(
            "[%s] Launching GStreamer x264 pipeline: %dx%d %s at %d kbps",
            self.name,
            width,
            height,
            image_format,
            self.current_bitrate_kbps,
        )
        self.pipeline = Gst.parse_launch(pipe_str)
        self.appsrc = self.pipeline.get_by_name("src")
        self.encoder = self.pipeline.get_by_name("enc")
        self.rtp_tee = self.pipeline.get_by_name("rtptee")
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message::error", self.on_pipeline_error)
        self.video_spec = (width, height, image_format)
        self.pipeline.set_state(Gst.State.PLAYING)

    def on_pipeline_error(self, _bus, message) -> None:
        error, debug = message.parse_error()
        logger.error("[%s] GStreamer pipeline error: %s (%s)", self.name, error, debug)
        self.notify_error(f"GStreamer error: {error.message}")
        self.teardown_pipeline()

    def teardown_pipeline(self) -> None:
        for viewer_id in list(self.viewers):
            self.detach_viewer(viewer_id)
        if self.pipeline:
            logger.info("[%s] Tearing down GStreamer pipeline", self.name)
            if self.bus:
                self.bus.remove_signal_watch()
                self.bus = None
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None
        self.appsrc = None
        self.encoder = None
        self.rtp_tee = None
        self.video_spec = None

    def attach_viewer(self, viewer_id: str) -> None:
        viewer = self.viewers.get(viewer_id)
        if not viewer or not self.pipeline or not self.rtp_tee:
            return
        self.peer_counter += 1
        queue = Gst.ElementFactory.make("queue", f"queue_{self.name}_{self.peer_counter}")
        capsfilter = Gst.ElementFactory.make("capsfilter", f"caps_{self.name}_{self.peer_counter}")
        webrtc = Gst.ElementFactory.make("webrtcbin", f"webrtc_{self.name}_{self.peer_counter}")

        if not queue or not capsfilter or not webrtc:
            self.notify_error("Could not create WebRTC pipeline elements", viewer_id)
            return

        queue.set_property("max-size-buffers", 128)
        queue.set_property("max-size-bytes", 64 * 1024)
        queue.set_property("max-size-time", 0)
        webrtc.set_property("bundle-policy", "max-bundle")
        webrtc.set_property("latency", 0)
        capsfilter.set_property(
            "caps",
            Gst.Caps.from_string(
                "application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000"
            ),
        )

        try:
            self.pipeline.add(queue)
            self.pipeline.add(capsfilter)
            self.pipeline.add(webrtc)
        except Exception as exc:
            self.notify_error(f"Could not add WebRTC elements to pipeline: {exc}", viewer_id)
            return
        if not queue.link(capsfilter) or not capsfilter.link(webrtc):
            for elem in (webrtc, capsfilter, queue):
                self.pipeline.remove(elem)
            self.notify_error("Could not link WebRTC elements", viewer_id)
            return

        tee_pad = self.rtp_tee.request_pad_simple("src_%u")
        queue_sink = queue.get_static_pad("sink")
        if not tee_pad or tee_pad.link(queue_sink) != Gst.PadLinkReturn.OK:
            if tee_pad:
                self.rtp_tee.release_request_pad(tee_pad)
            for elem in (webrtc, capsfilter, queue):
                elem.set_state(Gst.State.NULL)
                self.pipeline.remove(elem)
            self.notify_error("Could not connect RTP tee pad to WebRTC peer", viewer_id)
            return

        viewer.update(
            {
                "webrtc": webrtc,
                "queue": queue,
                "capsfilter": capsfilter,
                "tee_pad": tee_pad,
                "pending_ice": [],
                "remote_description_set": False,
            }
        )

        webrtc.connect(
            "on-ice-candidate",
            lambda elem, mline, cand: self.on_gst_ice(viewer_id, cand, mline),
        )
        webrtc.connect(
            "on-negotiation-needed",
            lambda elem: self.create_offer(viewer_id),
        )
        queue.sync_state_with_parent()
        capsfilter.sync_state_with_parent()
        webrtc.sync_state_with_parent()

    def detach_viewer(self, viewer_id: str) -> None:
        viewer = self.viewers.get(viewer_id)
        if not viewer:
            return
        webrtc = viewer.get("webrtc")
        queue = viewer.get("queue")
        capsfilter = viewer.get("capsfilter")
        tee_pad = viewer.get("tee_pad")
        if tee_pad and self.rtp_tee:
            if queue:
                queue_sink = queue.get_static_pad("sink")
                if queue_sink:
                    tee_pad.unlink(queue_sink)
            self.rtp_tee.release_request_pad(tee_pad)
        for elem in (webrtc, capsfilter, queue):
            if elem and self.pipeline:
                elem.set_state(Gst.State.NULL)
                self.pipeline.remove(elem)
        viewer.update(
            {
                "webrtc": None,
                "queue": None,
                "capsfilter": None,
                "tee_pad": None,
                "pending_ice": [],
                "remote_description_set": False,
            }
        )

    def on_gst_ice(self, viewer_id: str, candidate: str, mline_index: int) -> None:
        viewer = self.viewers.get(viewer_id)
        if not viewer:
            return
        payload = {
            "type": "ice",
            "topic": viewer.get("topic", self.raw_topic),
            "candidate": candidate,
            "sdpMLineIndex": mline_index,
        }
        asyncio.run_coroutine_threadsafe(viewer["send"](payload), self.loop)

    def create_offer(self, viewer_id: str) -> None:
        viewer = self.viewers.get(viewer_id)
        webrtc = viewer.get("webrtc") if viewer else None
        if not webrtc:
            return

        def on_offer_ready(promise, _):
            current = self.viewers.get(viewer_id)
            if not current or current.get("webrtc") is not webrtc:
                return
            reply = promise.get_reply()
            offer = reply.get_value("offer") if reply else None
            if not offer:
                self.notify_error("GStreamer failed to generate SDP offer", viewer_id)
                return
            webrtc.emit("set-local-description", offer, Gst.Promise.new())
            offer_topic = current.get("topic", self.raw_topic)
            payload = {
                "type": "offer",
                "topic": offer_topic,
                "sdp": offer.sdp.as_text(),
                "encoder": "x264",
                "targetFps": self.max_fps,
                "bitrateKbps": self.current_bitrate_kbps,
            }
            asyncio.run_coroutine_threadsafe(current["send"](payload), self.loop)

        promise = Gst.Promise.new_with_change_func(on_offer_ready, None)
        webrtc.emit("create-offer", None, promise)

    def handle_answer(self, viewer_id: str, sdp_text: str) -> None:
        viewer = self.viewers.get(viewer_id)
        webrtc = viewer.get("webrtc") if viewer else None
        if not webrtc or not sdp_text:
            return
        res, sdp_msg = GstSdp.SDPMessage.new()
        if res != GstSdp.SDPResult.OK:
            self.notify_error("Could not allocate SDP message", viewer_id)
            return
        res = GstSdp.sdp_message_parse_buffer(sdp_text.encode("utf-8"), sdp_msg)
        if res != GstSdp.SDPResult.OK:
            self.notify_error("Client sent invalid SDP answer", viewer_id)
            return
        answer = GstWebRTC.WebRTCSessionDescription.new(
            GstWebRTC.WebRTCSDPType.ANSWER, sdp_msg
        )

        def on_remote_desc_set(promise, _):
            current = self.viewers.get(viewer_id)
            if not current or current.get("webrtc") is not webrtc:
                return
            current["remote_description_set"] = True
            logger.info("[%s] Remote answer applied for %s", self.name, viewer_id)
            for mline, candidate in current["pending_ice"]:
                webrtc.emit("add-ice-candidate", mline, candidate)
            current["pending_ice"].clear()

        promise = Gst.Promise.new_with_change_func(on_remote_desc_set, None)
        webrtc.emit("set-remote-description", answer, promise)

    def handle_ice(self, viewer_id: str, candidate_str: str, mline_index: int) -> None:
        viewer = self.viewers.get(viewer_id)
        webrtc = viewer.get("webrtc") if viewer else None
        if not webrtc or not candidate_str:
            return
        if viewer["remote_description_set"]:
            webrtc.emit("add-ice-candidate", mline_index, candidate_str)
        elif len(viewer["pending_ice"]) < 256:
            viewer["pending_ice"].append((mline_index, candidate_str))

    def handle_telemetry(self, viewer_id: str, telemetry: dict) -> None:
        try:
            rtt = max(0.0, float(telemetry.get("rtt", 0)))
            loss = max(0.0, min(1.0, float(telemetry.get("packetLoss", 0))))
            inbound_kbps = max(0.0, float(telemetry.get("inboundKbps", 0)))
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(v) for v in (rtt, loss, inbound_kbps)):
            return

        self.viewer_telemetry[viewer_id] = {
            "rtt": rtt,
            "loss": loss,
            "inboundKbps": inbound_kbps,
            "receivedAt": time.monotonic(),
        }

        now = time.monotonic()
        if now - self.last_bitrate_adjustment < 1.0:
            return
        samples = [
            s for s in self.viewer_telemetry.values() if now - s["receivedAt"] <= 5.0
        ]
        if not samples:
            return

        old_bitrate = self.current_bitrate_kbps
        worst_loss = max(s["loss"] for s in samples)
        worst_rtt = max(s["rtt"] for s in samples)

        if worst_loss > 0.02 or worst_rtt > 0.250:
            self.current_bitrate_kbps = max(
                self.min_bitrate_kbps, int(self.current_bitrate_kbps * 0.75)
            )
        elif worst_loss < 0.005 and worst_rtt < 0.100:
            self.current_bitrate_kbps = min(
                self.max_bitrate_kbps, self.current_bitrate_kbps + 50
            )

        if old_bitrate != self.current_bitrate_kbps:
            self.last_bitrate_adjustment = now
            if self.encoder:
                try:
                    self.encoder.set_property("bitrate", self.current_bitrate_kbps)
                except Exception:
                    logger.exception("[%s] Could not set encoder bitrate", self.name)
            logger.info(
                "[%s] Encoder bitrate adjusted %d -> %d kbps (loss=%.2f%%, rtt=%.0fms)",
                self.name,
                old_bitrate,
                self.current_bitrate_kbps,
                worst_loss * 100,
                worst_rtt * 1000,
            )

        for viewer in list(self.viewers.values()):
            payload = {
                "type": "stats",
                "topic": viewer.get("topic", self.raw_topic),
                "encoderBitrateKbps": self.current_bitrate_kbps,
                "targetFps": self.max_fps,
                "encoder": "x264",
            }
            asyncio.run_coroutine_threadsafe(viewer["send"](payload), self.loop)

    async def frame_push_loop(self) -> None:
        pts = 0
        while self.is_active:
            if not self.has_video_demand():
                if self.pipeline:
                    self.teardown_pipeline()
                await asyncio.sleep(0.1)
                continue

            interval = 1.0 / max(0.1, self.max_fps)
            loop_start = time.monotonic()

            with self.lock:
                frame_msg = self.latest_msg
                self.latest_msg = None

            if frame_msg is not None:
                try:
                    if self.name == "depth":
                        gst_bytes, _, spec = convert_depth_to_jet(frame_msg)
                        spec = (spec[0], spec[1], "RGB")
                    else:
                        gst_bytes, _, spec = validate_and_extract_color(frame_msg)
                except ValueError as exc:
                    logger.warning("[%s] Image processing rejected: %s", self.name, exc)
                    self.notify_error(f"Image error: {exc}")
                else:
                    if not self.pipeline or self.video_spec != spec:
                        self.teardown_pipeline()
                        try:
                            self.setup_pipeline(*spec)
                            for vid in list(self.viewers):
                                self.attach_viewer(vid)
                        except Exception as exc:
                            logger.exception(
                                "[%s] Failed to initialize pipeline: %s", self.name, exc
                            )
                            self.notify_error(f"Could not init pipeline: {exc}")
                            await asyncio.sleep(0.5)
                            continue

                    if self.appsrc:
                        duration = int(interval * Gst.SECOND)
                        buf = Gst.Buffer.new_wrapped(gst_bytes)
                        buf.pts = pts
                        buf.dts = pts
                        buf.duration = duration
                        pts += duration
                        ret = self.appsrc.emit("push-buffer", buf)
                        if ret != Gst.FlowReturn.OK:
                            logger.warning(
                                "[%s] appsrc returned %s", self.name, ret.value_nick
                            )

            elapsed = time.monotonic() - loop_start
            await asyncio.sleep(max(0.001, interval - elapsed))
