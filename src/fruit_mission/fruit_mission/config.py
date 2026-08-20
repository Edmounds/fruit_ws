"""Configuration loading and validation for fruit_mission."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping

import yaml


def load_config(path: str, waypoint_file: str | None = None) -> Dict[str, Any]:
    """Load the node parameter YAML used by the mission package."""
    with Path(path).open(encoding="utf-8") as config_file:
        document = yaml.safe_load(config_file)
    params = document["fruit_mission"]["ros__parameters"]
    if waypoint_file is not None and waypoint_file.strip():
        params["waypoint_file"] = waypoint_file.strip()
    validate_config(params)
    return params


def validate_config(params: Mapping[str, Any]) -> None:
    """Validate stable structure without freezing field-tuning values."""
    waypoint_file = str(params.get("waypoint_file", "")).strip()
    if not waypoint_file:
        raise ValueError("waypoint_file must not be empty")
    waypoint_path = Path(waypoint_file)
    if waypoint_path.suffix not in (".yaml", ".yml"):
        raise ValueError("waypoint_file must be a YAML file")
    if not waypoint_path.is_file():
        raise ValueError(f"waypoint_file does not exist: {waypoint_file}")
    try:
        with waypoint_path.open("rb"):
            pass
    except OSError as error:
        raise ValueError(f"waypoint_file is not readable: {waypoint_file}") from error
    if not str(params.get("follow_waypoints_action", "")).strip():
        raise ValueError("follow_waypoints_action must not be empty")
    if not str(params.get("map_frame", "")).strip():
        raise ValueError("map_frame must not be empty")
    if not str(params.get("base_frame", "")).strip():
        raise ValueError("base_frame must not be empty")
    if float(params.get("tf_lookup_timeout_sec", 0.0)) <= 0.0:
        raise ValueError("tf_lookup_timeout_sec must be positive")
    if float(params.get("c_graph_max_start_distance", 0.0)) <= 0.0:
        raise ValueError("c_graph_max_start_distance must be positive")
    if not str(params.get("expected_fruit_type_topic", "")).strip():
        raise ValueError("expected_fruit_type_topic must not be empty")
    if not str(params.get("decision_topic", "")).strip():
        raise ValueError("decision_topic must not be empty")
    if params.get("preferred_side") not in ("left", "right"):
        raise ValueError("preferred_side must be left or right")
