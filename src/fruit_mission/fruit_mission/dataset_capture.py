"""Asynchronously capture short raw RGB bursts at successful Look poses."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
import threading
import time
from typing import Any, Optional

import cv2
from cv_bridge import CvBridge


@dataclass
class _LookCapture:
    area: str
    sequence: int
    frame_limit: int
    next_capture_at: float
    captured: int = 0
    last_stamp_ns: int = 0
    active: bool = True


@dataclass
class _PendingImage:
    path: Path
    frame: Any
    encoded: Optional[bytes] = None
    next_attempt_at: float = 0.0
    last_error_log_at: float = 0.0


class DatasetCapture:
    """Keep image capture and disk I/O outside the mission control path."""

    def __init__(
        self,
        logger,
        output_root: str,
        frame_limit: int = 5,
        frame_interval_sec: float = 0.1,
        shutdown_timeout_sec: float = 10.0,
    ) -> None:
        self._logger = logger
        self._output_root = Path(output_root)
        self._frame_limit = int(frame_limit)
        self._frame_interval_sec = float(frame_interval_sec)
        self._shutdown_timeout_sec = float(shutdown_timeout_sec)
        self._bridge = CvBridge()
        self._run_dir = self._create_run_dir()
        self._area_sequences: dict[str, int] = {}
        self._latest_frame: Any = None
        self._latest_stamp_ns = 0
        self._active: Optional[_LookCapture] = None
        self._pending: deque[_PendingImage] = deque()
        self._condition = threading.Condition()
        self._closing = False
        self._force_stop = False
        self._worker = threading.Thread(
            target=self._run,
            name="fruit-mission-dataset-writer",
            daemon=True,
        )
        self._worker.start()

    def on_frame(self, frame: Any, stamp_ns: int) -> None:
        """Cache the newest ROS image without converting it in the callback."""
        with self._condition:
            self._latest_frame = frame
            self._latest_stamp_ns = int(stamp_ns)
            self._condition.notify()

    def start_look(self, area: str) -> None:
        """Start one Look burst and enqueue the current frame immediately."""
        with self._condition:
            if self._active is not None:
                self._active.active = False
            sequence = self._area_sequences.get(area, 0) + 1
            self._area_sequences[area] = sequence
            self._active = _LookCapture(
                area=area,
                sequence=sequence,
                frame_limit=self._frame_limit,
                next_capture_at=time.monotonic(),
            )
            self._capture_latest_locked(self._active)
            self._condition.notify()

    def stop_look(self) -> None:
        """Stop collecting new frames while keeping queued writes retryable."""
        with self._condition:
            if self._active is not None:
                self._active.active = False
                self._active = None
            self._condition.notify()

    def close(self) -> None:
        """Stop capture and wait briefly for already captured images to be written."""
        self.stop_look()
        with self._condition:
            self._closing = True
            self._condition.notify()
        self._worker.join(timeout=self._shutdown_timeout_sec)
        if self._worker.is_alive():
            with self._condition:
                self._force_stop = True
                self._condition.notify()
            self._worker.join(timeout=1.0)
            self._logger.warning("Dataset image writer did not finish before shutdown")

    def _create_run_dir(self) -> Path:
        mission_id = time.strftime("%Y%m%d_%H%M%S")
        for suffix in range(100):
            name = mission_id if suffix == 0 else f"{mission_id}_{suffix:02d}"
            candidate = self._output_root / name
            try:
                candidate.mkdir(parents=True, exist_ok=False)
                return candidate
            except FileExistsError:
                continue
            except OSError as error:
                self._logger.warning(f"Cannot create dataset directory {candidate}: {error}")
                return candidate
        raise RuntimeError(f"Cannot allocate dataset mission directory under {self._output_root}")

    def _capture_latest_locked(self, look: _LookCapture) -> bool:
        if look.captured >= look.frame_limit:
            look.active = False
            return False
        if self._latest_frame is None or self._latest_stamp_ns == look.last_stamp_ns:
            return False
        frame_number = look.captured + 1
        path = self._run_dir / look.area / (
            f"look_{look.sequence:03d}_f{frame_number:02d}.png"
        )
        self._pending.append(
            _PendingImage(path=path, frame=self._latest_frame, next_attempt_at=time.monotonic())
        )
        look.captured = frame_number
        look.last_stamp_ns = self._latest_stamp_ns
        look.next_capture_at = time.monotonic() + self._frame_interval_sec
        if look.captured >= look.frame_limit:
            look.active = False
            if self._active is look:
                self._active = None
        return True

    def _run(self) -> None:
        while True:
            with self._condition:
                if self._force_stop:
                    return
                now = time.monotonic()
                active = self._active
                if active is not None and active.active:
                    if now >= active.next_capture_at:
                        self._capture_latest_locked(active)
                    if not active.active and self._active is active:
                        self._active = None
                pending = next(
                    (item for item in self._pending if item.next_attempt_at <= now),
                    None,
                )
                if self._closing and not self._pending and self._active is None:
                    return
                if pending is None:
                    self._condition.wait(timeout=0.05)
                    continue
            self._write_pending(pending)

    def _write_pending(self, pending: _PendingImage) -> None:
        try:
            if pending.encoded is None:
                rgb_frame = self._bridge.imgmsg_to_cv2(
                    pending.frame,
                    desired_encoding='rgb8',
                )
                bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                encoded_ok, encoded = cv2.imencode(".png", bgr_frame)
                if not encoded_ok:
                    raise OSError("OpenCV PNG encoding failed")
                pending.encoded = encoded.tobytes()
                pending.frame = None
            pending.path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = pending.path.with_name(pending.path.name + ".tmp")
            temporary_path.write_bytes(pending.encoded)
            temporary_path.replace(pending.path)
        except Exception as error:  # noqa: BLE001
            now = time.monotonic()
            if now - pending.last_error_log_at >= 5.0:
                self._logger.warning(f"Cannot save dataset image {pending.path}: {error}")
                pending.last_error_log_at = now
            pending.next_attempt_at = now + self._frame_interval_sec
            return

        with self._condition:
            try:
                self._pending.remove(pending)
            except ValueError:
                return
            self._condition.notify()
