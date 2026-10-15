"""Configuration loading and validation for fruit_mission."""

from __future__ import annotations

from pathlib import Path
import math
from typing import Any, Dict, Mapping

import yaml


def load_config(path: str, waypoint_file: str | None = None) -> Dict[str, Any]:
    """Load the node parameter YAML used by the mission package."""
    with Path(path).open(encoding="utf-8") as config_file:
        document = yaml.safe_load(config_file)
    params = document["fruit_mission"]["ros__parameters"]
    if waypoint_file is not None and waypoint_file.strip():
        params["waypoint_file"] = waypoint_file.strip()
    fallback_path = Path(params['qr_fallback_file'])
    with fallback_path.open(encoding='utf-8') as stream:
        params['qr_fallback'] = yaml.safe_load(stream)
    validate_config(params)
    params["target_offset_ac_m"] = validate_target_offset(
        params["target_offset_ac_m"], f"target_offset_ac_m in {path}")
    params["target_offset_b_up_m"] = validate_target_offset(
        params["target_offset_b_up_m"], f"target_offset_b_up_m in {path}")
    params["target_offset_b_down_m"] = validate_target_offset(
        params["target_offset_b_down_m"], f"target_offset_b_down_m in {path}")
    return params


def validate_target_offset(
    values: Any, context: str = "target offset"
) -> tuple[float, float, float]:
    """Validate target offset as a 3-element finite float sequence (meters)."""
    if (not isinstance(values, (list, tuple)) or len(values) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                   for v in values)):
        raise ValueError(f"{context} must contain three finite numbers")
    return (float(values[0]), float(values[1]), float(values[2]))


def load_target_offsets(
    config_path: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Load and validate AC, B-Up and B-Down target offsets from mission config YAML."""
    path = Path(config_path)
    if not path.is_file():
        raise ValueError(f"mission config file does not exist: {config_path}")
    with path.open(encoding="utf-8") as stream:
        doc = yaml.safe_load(stream) or {}
    params = doc.get("fruit_mission", {}).get("ros__parameters", {})
    if "target_offset_ac_m" not in params:
        raise ValueError(f"target_offset_ac_m missing in {config_path}")
    if "target_offset_b_up_m" not in params:
        raise ValueError(f"target_offset_b_up_m missing in {config_path}")
    if "target_offset_b_down_m" not in params:
        raise ValueError(f"target_offset_b_down_m missing in {config_path}")
    ac_offset = validate_target_offset(
        params["target_offset_ac_m"], f"target_offset_ac_m in {config_path}"
    )
    b_up_offset = validate_target_offset(
        params["target_offset_b_up_m"], f"target_offset_b_up_m in {config_path}"
    )
    b_down_offset = validate_target_offset(
        params["target_offset_b_down_m"], f"target_offset_b_down_m in {config_path}"
    )
    return ac_offset, b_up_offset, b_down_offset


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
        "camera_detection_timeout_sec",
        "navigation_action_server_timeout_sec",
    ):
        if float(params.get(timeout_name, 0.0)) <= 0.0:
            raise ValueError(f"{timeout_name} must be positive")
    if (
        "camera_empty_detection_timeout_sec" in params
        and float(params.get("camera_empty_detection_timeout_sec", 0.0)) <= 0.0
    ):
        raise ValueError("camera_empty_detection_timeout_sec must be positive")
    if (
        "camera_early_exit_confirm_frames" in params
        and int(params.get("camera_early_exit_confirm_frames", 0)) <= 0
    ):
        raise ValueError("camera_early_exit_confirm_frames must be positive")
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
    if not str(params.get("decision_topic", "")).strip():
        raise ValueError("decision_topic must not be empty")
    if not str(params.get("dataset_output_dir", "")).strip():
        raise ValueError("dataset_output_dir must not be empty")
    if not str(params.get("dataset_image_topic", "")).strip():
        raise ValueError("dataset_image_topic must not be empty")
    if int(params.get("dataset_frames_per_look", 0)) <= 0:
        raise ValueError("dataset_frames_per_look must be positive")
    if float(params.get("dataset_frame_interval_sec", 0.0)) <= 0.0:
        raise ValueError("dataset_frame_interval_sec must be positive")
    if float(params.get("dataset_shutdown_timeout_sec", 0.0)) <= 0.0:
        raise ValueError("dataset_shutdown_timeout_sec must be positive")
    if not str(params.get("camera_extrinsic_config", "")).strip():
        raise ValueError("camera_extrinsic_config must not be empty")
    if "target_offset_ac_m" not in params:
        raise ValueError("target_offset_ac_m must be specified in config")
    validate_target_offset(params["target_offset_ac_m"], "target_offset_ac_m")
    if "target_offset_b_up_m" not in params:
        raise ValueError("target_offset_b_up_m must be specified in config")
    validate_target_offset(params["target_offset_b_up_m"], "target_offset_b_up_m")
    if "target_offset_b_down_m" not in params:
        raise ValueError("target_offset_b_down_m must be specified in config")
    validate_target_offset(params["target_offset_b_down_m"], "target_offset_b_down_m")
    if not str(params.get("observation_window_topic", "")).strip():
        raise ValueError("observation_window_topic must not be empty")
    _validate_qr_fallbacks(params['qr_fallback'])
    stationary = params.get('stationary')
    if not isinstance(stationary, dict):
        raise ValueError('stationary must be a mapping')
    joint_speed = stationary.get('joint_speed_rps')
    if (isinstance(joint_speed, bool) or not isinstance(joint_speed, (int, float))
            or not math.isfinite(joint_speed) or joint_speed <= 0):
        raise ValueError('stationary.joint_speed_rps must be finite and positive')
    tts = params.get("tts", {})
    if "playback_timeout_sec" in tts and float(tts["playback_timeout_sec"]) <= 0.0:
        raise ValueError("tts.playback_timeout_sec must be positive")
    mission_flow = str(params.get("mission_flow", "full")).strip().lower()
    if mission_flow not in {"full", "ac", "acb", "a", "b", "c"}:
        raise ValueError("mission_flow must be full, ac, acb, a, b, or c")


def _validate_qr_fallbacks(params: Mapping[str, Any]) -> None:
    if not isinstance(params, dict):
        raise ValueError('QR fallback config must be a mapping')
    b_targets = params.get("b_targets")
    if not isinstance(b_targets, list) or len(b_targets) != 8:
        raise ValueError("b_targets must contain 8 targets")
    if any(target not in {"苹果", "梨子"} for target in b_targets):
        raise ValueError("b_targets may only contain 苹果 or 梨子")

    c_targets = params.get("c_targets")
    if not isinstance(c_targets, list) or len(c_targets) != 8:
        raise ValueError("c_targets must contain 8 targets")
    if any(target not in {"南瓜", "西红柿", "辣椒", "洋葱"} for target in c_targets):
        raise ValueError(
            "c_targets may only contain 南瓜, 西红柿, 辣椒, or 洋葱"
        )

    c_positions = params.get("c_positions")
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
            "c_positions must contain 8 unique position IDs "
            "from 1 to 12"
        )
