"""Capture the clearest raw RGB frame after each successful Look pose."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
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
    start_stamp_ns: int
    captured: int = 0
    last_stamp_ns: int = 0
    pending_count: int = 0
    candidates: list[tuple[float, Any]] = field(default_factory=list)
    active: bool = True
    finalize_requested: bool = False
    finalized: bool = False


@dataclass
class _PendingFrame:
    look: _LookCapture
    frame: Any


@dataclass
class _PendingImage:
    path: Path
    frame: Any
    encoded: Optional[bytes] = None
    next_attempt_at: float = 0.0
    last_error_log_at: float = 0.0


def sharpness_score(bgr_frame: Any) -> float:
    """Return a focus score based on the variance of the image Laplacian."""
    gray_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray_frame, cv2.CV_64F).var())


class DatasetCapture:
    """Keep frame conversion, scoring, and disk I/O outside the mission path."""

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
        if (
            self._frame_limit <= 0
            or not math.isfinite(self._frame_interval_sec)
            or self._frame_interval_sec <= 0
            or not math.isfinite(self._shutdown_timeout_sec)
            or self._shutdown_timeout_sec <= 0
        ):
            raise ValueError("Capture frame count and timeouts must be finite and positive")
        self._bridge = CvBridge()
        self._run_dir = self._create_run_dir()
        self._area_sequences: dict[str, int] = {}
        self._latest_frame: Any = None
        self._latest_stamp_ns = 0
        self._active: Optional[_LookCapture] = None
        self._pending: deque[_PendingFrame | _PendingImage] = deque()
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
        """Cache the newest ROS image without conversion in the ROS callback."""
        with self._condition:
            self._latest_frame = frame
            self._latest_stamp_ns = int(stamp_ns)
            self._condition.notify()

    def start_look(self, area: str, start_stamp_ns: int) -> None:
        """Start collecting new frames after the Look completion timestamp."""
        if area not in ("A", "B", "C") or start_stamp_ns <= 0:
            self.stop_look()
            self._logger.warning("Dataset Look skipped: invalid area or Look timestamp")
            return
        with self._condition:
            self._finish_active_locked()
            if self._closing:
                return
            sequence = self._area_sequences.get(area, 0) + 1
            self._area_sequences[area] = sequence
            self._active = _LookCapture(
                area=area,
                sequence=sequence,
                frame_limit=self._frame_limit,
                next_capture_at=time.monotonic(),
                start_stamp_ns=int(start_stamp_ns),
            )
            self._condition.notify()

    def stop_look(self) -> None:
        """Stop collecting new frames and finalize the current candidate set."""
        with self._condition:
            self._finish_active_locked()
            self._condition.notify()

    def close(self) -> None:
        """Stop capture and wait briefly for selected images to be written."""
        self.stop_look()
        with self._condition:
            self._closing = True
            self._condition.notify()
        self._worker.join(timeout=min(self._shutdown_timeout_sec, 10.0))
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

    def _finish_active_locked(self) -> None:
        if self._active is None:
            return
        look = self._active
        look.active = False
        look.finalize_requested = True
        self._active = None
        self._maybe_finalize_locked(look)

    def _capture_latest_locked(self, look: _LookCapture) -> None:
        if (
            not look.active
            or look.captured >= look.frame_limit
            or self._latest_frame is None
            or self._latest_stamp_ns <= max(look.start_stamp_ns, look.last_stamp_ns)
        ):
            return
        look.captured += 1
        look.last_stamp_ns = self._latest_stamp_ns
        look.pending_count += 1
        look.next_capture_at = time.monotonic() + self._frame_interval_sec
        self._pending.append(_PendingFrame(look=look, frame=self._latest_frame))
        if look.captured >= look.frame_limit:
            look.active = False
            look.finalize_requested = True
            if self._active is look:
                self._active = None

    def _maybe_finalize_locked(self, look: _LookCapture) -> None:
        if (
            look.finalized
            or not look.finalize_requested
            or look.pending_count != 0
        ):
            return
        look.finalized = True
        if not look.candidates:
            self._logger.warning(
                f"Dataset Look {look.area}/{look.sequence}: no valid RGB frame"
            )
            return
        score, frame = max(look.candidates, key=lambda candidate: candidate[0])
        path = self._run_dir / look.area / f"look_{look.sequence:03d}.png"
        self._pending.append(_PendingImage(path=path, frame=frame))
        self._logger.info(
            f"Dataset Look {look.area}/{look.sequence}: saved best of "
            f"{len(look.candidates)} frames, sharpness={score:.2f}"
        )

    def _run(self) -> None:
        while True:
            with self._condition:
                if self._force_stop:
                    return
                now = time.monotonic()
                active = self._active
                if active is not None and active.active and now >= active.next_capture_at:
                    self._capture_latest_locked(active)
                if self._closing and not self._pending and self._active is None:
                    return
                pending = next(
                    (
                        item
                        for item in self._pending
                        if not isinstance(item, _PendingImage)
                        or item.next_attempt_at <= now
                    ),
                    None,
                )
                if pending is None:
                    self._condition.wait(timeout=0.05)
                    continue
                self._pending.remove(pending)
            if isinstance(pending, _PendingFrame):
                self._score_pending_frame(pending)
            else:
                self._write_pending_image(pending)

    def _score_pending_frame(self, pending: _PendingFrame) -> None:
        try:
            rgb_frame = self._bridge.imgmsg_to_cv2(
                pending.frame,
                desired_encoding="rgb8",
            )
            bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            score = sharpness_score(bgr_frame)
        except Exception as error:  # noqa: BLE001
            self._logger.warning(f"Dataset RGB frame skipped during scoring: {error}")
            score = None
            bgr_frame = None
        with self._condition:
            pending.look.pending_count -= 1
            if score is not None:
                pending.look.candidates.append((score, bgr_frame))
            self._maybe_finalize_locked(pending.look)
            self._condition.notify()

    def _write_pending_image(self, pending: _PendingImage) -> None:
        try:
            if pending.encoded is None:
                encoded_ok, encoded = cv2.imencode(".png", pending.frame)
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
            with self._condition:
                self._pending.append(pending)
                self._condition.notify()
