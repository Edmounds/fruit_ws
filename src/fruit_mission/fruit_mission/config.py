"""Configuration loading and validation for fruit_mission."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping

import yaml


def load_config(path: str) -> Dict[str, Any]:
    """Load the node parameter YAML used by the mission package."""
    with Path(path).open(encoding="utf-8") as config_file:
        document = yaml.safe_load(config_file)
    params = document["fruit_mission"]["ros__parameters"]
    validate_config(params)
    return params


def validate_config(params: Mapping[str, Any]) -> None:
    """Validate stable structure without freezing field-tuning values."""
    waypoint_file = str(params.get("waypoint_file", "")).strip()
    if not waypoint_file:
        raise ValueError("waypoint_file must not be empty")
    if not Path(waypoint_file).exists():
        raise ValueError(f"waypoint_file does not exist: {waypoint_file}")
    if not str(params.get("follow_waypoints_action", "")).strip():
        raise ValueError("follow_waypoints_action must not be empty")
    if params.get("preferred_side") not in ("left", "right"):
        raise ValueError("preferred_side must be left or right")
