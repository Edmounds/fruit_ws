"""Pure joystick edge detection and waypoint persistence logic."""

import math
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import yaml


def new_session_path(output_directory: Path, now: datetime) -> Path:
    """Return a timestamped session path without replacing an existing file."""
    stem = f"waypoints_{now.strftime('%Y%m%d_%H%M%S')}"
    candidate = output_directory / f"{stem}.yaml"
    suffix = 1
    while candidate.exists():
        candidate = output_directory / f"{stem}_{suffix:02d}.yaml"
        suffix += 1
    return candidate


class JoyEdgeDetector:
    """Detect rising edges for the configured save button and undo axis."""

    def __init__(
        self,
        save_button: int,
        undo_axis: int,
        undo_direction: float,
        axis_threshold: float,
    ) -> None:
        self._save_button = save_button
        self._undo_axis = undo_axis
        self._undo_direction = undo_direction
        self._axis_threshold = axis_threshold
        self._save_was_pressed = False
        self._undo_was_pressed = False

    def observe(
        self, buttons: Sequence[int], axes: Sequence[float]
    ) -> Tuple[bool, bool]:
        """Return save and undo rising edges for one Joy sample."""
        save_pressed = (
            0 <= self._save_button < len(buttons)
            and buttons[self._save_button] != 0
        )
        undo_pressed = (
            0 <= self._undo_axis < len(axes)
            and axes[self._undo_axis] * self._undo_direction
            >= self._axis_threshold
        )
        save_edge = save_pressed and not self._save_was_pressed
        undo_edge = undo_pressed and not self._undo_was_pressed
        self._save_was_pressed = save_pressed
        self._undo_was_pressed = undo_pressed
        return save_edge, undo_edge


class WaypointStore:
    """Persist an ordered waypoint session to one YAML file."""

    def __init__(
        self,
        output_path: Path,
        frame_id: str,
        child_frame_id: str,
        created_at: datetime,
    ) -> None:
        self.output_path = output_path
        self._document = {
            "frame_id": frame_id,
            "child_frame_id": child_frame_id,
            "created_at": created_at.isoformat(),
            "waypoints": [],
        }
        self._write()

    def append(
        self,
        translation: Dict[str, float],
        rotation: Dict[str, float],
        recorded_at: datetime,
    ) -> Dict[str, object]:
        """Append one transform and return the generated waypoint entry."""
        index = len(self._document["waypoints"]) + 1
        waypoint = {
            "name": f"waypoint_{index:03d}",
            "recorded_at": recorded_at.isoformat(),
            "position": {key: float(translation[key]) for key in ("x", "y", "z")},
            "orientation": {key: float(rotation[key]) for key in ("x", "y", "z", "w")},
            "yaw": round(self._yaw(rotation), 6),
        }
        self._document["waypoints"].append(waypoint)
        self._write()
        return waypoint

    def undo(self) -> Optional[Dict[str, object]]:
        """Remove and return the last waypoint, or return None when empty."""
        if not self._document["waypoints"]:
            return None
        waypoint = self._document["waypoints"].pop()
        self._write()
        return waypoint

    @staticmethod
    def _yaw(rotation: Dict[str, float]) -> float:
        x = float(rotation["x"])
        y = float(rotation["y"])
        z = float(rotation["z"])
        w = float(rotation["w"])
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _write(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", encoding="utf-8") as output:
            yaml.safe_dump(
                self._document,
                output,
                allow_unicode=True,
                sort_keys=False,
            )
