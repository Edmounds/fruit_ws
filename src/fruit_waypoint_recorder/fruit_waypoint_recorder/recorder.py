# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Route-template recording helpers for the fixed competition field."""

from datetime import datetime
from pathlib import Path
from typing import Optional

from fruit_navigation.route_model import AnchorSession, PoseData


def new_session_path(output_directory: Path, now: datetime) -> Path:
    """Return a timestamped route-template path without replacing a file."""
    stem = f"competition_waypoints_{now.strftime('%Y%m%d_%H%M%S')}"
    candidate = output_directory / f"{stem}.yaml"
    suffix = 1
    while candidate.exists():
        candidate = output_directory / f"{stem}_{suffix:02d}.yaml"
        suffix += 1
    return candidate


class WaypointTemplateStore:
    """Maintain one eleven-anchor recording session and save generated points."""

    def __init__(self, output_path: Path, yaw_tolerance: float) -> None:
        self.output_path = output_path
        self._session = AnchorSession(yaw_tolerance=yaw_tolerance)

    @property
    def prompt(self) -> str:
        """Return the next click prompt."""
        return self._session.next_prompt()

    @property
    def anchors(self):
        """Return recorded anchors in click order."""
        return self._session.anchors

    def append(self, pose: PoseData) -> PoseData:
        """Append one RViz-clicked pose as the next canonical anchor."""
        return self._session.append(pose)

    def undo(self) -> Optional[PoseData]:
        """Undo the latest anchor."""
        return self._session.undo()

    def clear(self) -> None:
        """Clear the current session."""
        self._session.clear()

    def save(self) -> None:
        """Validate anchors, generate points, and write YAML."""
        self._session.save(self.output_path)
