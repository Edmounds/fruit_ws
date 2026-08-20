#!/usr/bin/env bash

# Resolve all field-dependent runtime files from one explicit profile ID.
# Callers must source this file and then invoke resolve_map_profile.

resolve_map_profile() {
  local workspace=$1
  local map_id=$2

  case "$map_id" in
    A|B) ;;
    *)
      printf 'map profile must be A or B, got: %s\n' "$map_id" >&2
      return 2
      ;;
  esac

  MAP_ID="$map_id"
  MAP_DIR="$workspace/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/$map_id"
  MAP_YAML="$MAP_DIR/field.yaml"
  LOCALIZATION_MAP="$MAP_DIR/field.pcd"
  INITIAL_POSE_FILE="$MAP_DIR/field.initial_pose.yaml"
  MAP_PGM="$MAP_DIR/field.pgm"
  MAP_PLY="$MAP_DIR/field.ply"
  GLIM_DUMP_PATH="$workspace/tmp/glim_map/$map_id"

  local profile_config="$workspace/src/fruit_waypoint_recorder/config/map_profiles.yaml"
  [[ -r "$profile_config" ]] || {
    printf 'map profile configuration is unavailable: %s\n' "$profile_config" >&2
    return 1
  }
  local -a waypoint_paths
  if ! mapfile -t waypoint_paths < <(
    python3 - "$profile_config" "$map_id" <<'PY'
import sys

import yaml


config_path, map_id = sys.argv[1:]
with open(config_path, encoding="utf-8") as stream:
    document = yaml.safe_load(stream) or {}
profile = (document.get("maps") or {}).get(map_id)
if not isinstance(profile, dict):
    raise SystemExit(f"missing map profile: {map_id}")
global_waypoint_file = str(
    profile.get("waypoint_file") or document.get("waypoint_file") or ""
).strip()
if global_waypoint_file:
    print(global_waypoint_file)
    print(global_waypoint_file)
else:
    for key in ("mission_waypoint_file", "recorder_waypoint_file"):
        value = str(profile.get(key, "")).strip()
        if not value:
            raise SystemExit(f"missing {key} for map profile {map_id}")
        print(value)
PY
  ); then
    printf 'failed to parse map profile configuration: %s\n' "$profile_config" >&2
    return 1
  fi
  ((${#waypoint_paths[@]} == 2)) || {
    printf 'map profile %s must define mission and recorder waypoint files\n' "$map_id" >&2
    return 1
  }
  local waypoint_path
  for waypoint_path in "${!waypoint_paths[@]}"; do
    if [[ "${waypoint_paths[waypoint_path]}" != /* ]]; then
      waypoint_paths[waypoint_path]="$workspace/${waypoint_paths[waypoint_path]}"
    fi
    waypoint_paths[waypoint_path]="$(realpath -m "${waypoint_paths[waypoint_path]}")"
  done
  MISSION_WAYPOINT_FILE="${waypoint_paths[0]}"
  RECORDER_WAYPOINT_FILE="${waypoint_paths[1]}"
  export MAP_ID MAP_DIR MAP_YAML LOCALIZATION_MAP INITIAL_POSE_FILE
  export MAP_PGM MAP_PLY GLIM_DUMP_PATH MISSION_WAYPOINT_FILE
  export RECORDER_WAYPOINT_FILE
}

require_map_profile_file() {
  local path=$1
  [[ -f "$path" && -r "$path" ]] || {
    printf 'required map profile file is unavailable: %s\n' "$path" >&2
    return 1
  }
}

require_map_profile_runtime() {
  require_map_profile_file "$MAP_YAML" || return 1
  require_map_profile_file "$LOCALIZATION_MAP" || return 1
  require_map_profile_file "$INITIAL_POSE_FILE" || return 1
  require_map_profile_file "$MISSION_WAYPOINT_FILE" || return 1
  require_map_profile_file "$RECORDER_WAYPOINT_FILE" || return 1
}
