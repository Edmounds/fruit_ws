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
    for timeout_name in (
        "qr_plan_timeout_sec",
        "callback_timeout_sec",
        "hardware_endpoint_timeout_sec",
        "d435i_detection_timeout_sec",
        "navigation_action_server_timeout_sec",
    ):
        if float(params.get(timeout_name, 0.0)) <= 0.0:
            raise ValueError(f"{timeout_name} must be positive")
    if not str(params.get("map_frame", "")).strip():
        raise ValueError("map_frame must not be empty")
    if not str(params.get("base_frame", "")).strip():
        raise ValueError("base_frame must not be empty")
    if float(params.get("tf_lookup_timeout_sec", 0.0)) <= 0.0:
        raise ValueError("tf_lookup_timeout_sec must be positive")
    if float(params.get("c_graph_max_start_distance", 0.0)) <= 0.0:
        raise ValueError("c_graph_max_start_distance must be positive")
    if float(params.get("c_qr_announce_lead_sec", 4.0)) < 0.0:
        raise ValueError("c_qr_announce_lead_sec must not be negative")
    if not str(params.get("expected_fruit_type_topic", "")).strip():
        raise ValueError("expected_fruit_type_topic must not be empty")
    if not str(params.get("decision_topic", "")).strip():
        raise ValueError("decision_topic must not be empty")
    if params.get("preferred_side") not in ("left", "right"):
        raise ValueError("preferred_side must be left or right")
    _validate_qr_fallbacks(params)
    tts = params.get("tts", {})
    if "qr_length_scale" in tts and float(tts["qr_length_scale"]) <= 0.0:
        raise ValueError("tts.qr_length_scale must be positive")
    mission_flow = str(params.get("mission_flow", "full")).strip().lower()
    if mission_flow not in {"full", "ac", "a", "b", "c"}:
        raise ValueError("mission_flow must be full, ac, a, b, or c")


def _validate_qr_fallbacks(params: Mapping[str, Any]) -> None:
    b_targets = params.get("b_qr_fallback_targets")
    if not isinstance(b_targets, list) or len(b_targets) != 8:
        raise ValueError("b_qr_fallback_targets must contain 8 targets")
    if any(target not in {"苹果", "梨子"} for target in b_targets):
        raise ValueError("b_qr_fallback_targets may only contain 苹果 or 梨子")

    c_targets = params.get("c_qr_fallback_targets")
    if not isinstance(c_targets, list) or len(c_targets) != 8:
        raise ValueError("c_qr_fallback_targets must contain 8 targets")
    if any(target not in {"南瓜", "西红柿", "辣椒", "洋葱"} for target in c_targets):
        raise ValueError(
            "c_qr_fallback_targets may only contain 南瓜, 西红柿, 辣椒, or 洋葱"
        )

    c_positions = params.get("c_qr_fallback_positions")
    if (
        not isinstance(c_positions, list)
        or len(c_positions) != 8
        or any(
            type(position) is not int or not 1 <= position <= 12
            for position in c_positions
        )
        or len(set(c_positions)) != 8
    ):
        raise ValueError(
            "c_qr_fallback_positions must contain 8 unique position IDs "
            "from 1 to 12"
        )
