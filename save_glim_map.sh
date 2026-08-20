#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
MAP_PROFILE_HELPER="$WORKSPACE/scripts/map_profile.sh"
GLIM_CONFIG="$WORKSPACE/src/navigation/glim/config"
EXPORT_DIR="$WORKSPACE/tmp/glim_map_export_$$"
MAP_ID=""
DESKTOP_RUNTIME_DIR="/run/user/$(id -u)"
DESKTOP_DISPLAY=:0
DESKTOP_XAUTHORITY="$DESKTOP_RUNTIME_DIR/gdm/Xauthority"

cleanup() {
  rm -rf "$EXPORT_DIR"
}
trap cleanup EXIT

usage() {
  cat <<'EOF'
Usage: ./save_glim_map.sh --map A|B

Export the selected mapping dump into its field profile.
EOF
}

while (($# > 0)); do
  case "$1" in
    --map)
      (($# >= 2)) || { usage >&2; exit 2; }
      MAP_ID=$2
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "$MAP_ID" ]] || { usage >&2; exit 2; }
SENSOR_FRAMES="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/sensor_frames.yaml"

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u
source "$MAP_PROFILE_HELPER"
resolve_map_profile "$WORKSPACE" "$MAP_ID"
GLIM_DUMP="$GLIM_DUMP_PATH"

for required_file in \
  "$GLIM_DUMP/map_frame_normalization.txt" \
  "$GLIM_DUMP/field.pgm" \
  "$GLIM_DUMP/field.yaml" \
  "$GLIM_DUMP/traj_lidar.txt" \
  "$SENSOR_FRAMES" \
  "$DESKTOP_XAUTHORITY"; do
  if [[ ! -r "$required_file" ]]; then
    printf '[save-glim-map] required file is unavailable: %s\n' "$required_file" >&2
    exit 1
  fi
done

mkdir -p "$MAP_DIR" "$EXPORT_DIR"

printf '[save-glim-map] exporting optimized GLIM map...\n'
env \
  "DISPLAY=$DESKTOP_DISPLAY" \
  "XAUTHORITY=$DESKTOP_XAUTHORITY" \
  "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
  "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
  ros2 run glim_ros offline_viewer \
    "$GLIM_DUMP" \
    --export_path "$EXPORT_DIR/field.ply" \
    --config_path "$GLIM_CONFIG"

printf '[save-glim-map] converting localization map...\n'
ros2 run pb_glim_map_tools glim_ply_to_pcd \
  --input "$EXPORT_DIR/field.ply" \
  --output "$EXPORT_DIR/field.pcd" \
  --leaf-size 0.05

cp "$GLIM_DUMP/field.pgm" "$EXPORT_DIR/field.pgm"
cp "$GLIM_DUMP/field.yaml" "$EXPORT_DIR/field.yaml"

if [[ -s "$GLIM_DUMP/field.initial_pose.yaml" ]]; then
  cp "$GLIM_DUMP/field.initial_pose.yaml" "$EXPORT_DIR/field.initial_pose.yaml"
else
  printf '[save-glim-map] generating initial GICP pose from the first GLIM trajectory frame...\n'
  python3 - "$GLIM_DUMP/traj_lidar.txt" "$SENSOR_FRAMES" \
    "$EXPORT_DIR/field.initial_pose.yaml" <<'PY'
import math
import sys

import yaml


def q_mul(lhs, rhs):
    return (
        lhs[3] * rhs[0] + lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[3] * rhs[1] - lhs[0] * rhs[2] + lhs[1] * rhs[3] + lhs[2] * rhs[0],
        lhs[3] * rhs[2] + lhs[0] * rhs[1] - lhs[1] * rhs[0] + lhs[2] * rhs[3],
        lhs[3] * rhs[3] - lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2],
    )


def q_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rotate(q, vector):
    rotated = q_mul(q_mul(q, (*vector, 0.0)), q_conjugate(q))
    return rotated[:3]


def first_pose(path):
    for line in open(path, encoding="utf-8"):
        fields = line.split()
        if len(fields) == 8 and not line.lstrip().startswith("#"):
            values = [float(value) for value in fields[1:]]
            return values[:3], tuple(values[3:])
    raise ValueError(f"no TUM pose found in {path}")


trajectory_path, frames_path, output_path = sys.argv[1:]
translation_map_lidar, q_map_lidar = first_pose(trajectory_path)
frames = yaml.safe_load(open(frames_path, encoding="utf-8"))
base_to_lidar = frames["base_to_front_mid360"]
t_base_lidar = tuple(float(value) for value in base_to_lidar["translation"])
q_base_lidar = tuple(float(value) for value in base_to_lidar["rotation"])
q_lidar_base = q_conjugate(q_base_lidar)
t_lidar_base = q_rotate(q_lidar_base, tuple(-value for value in t_base_lidar))
t_map_base = tuple(
    translation_map_lidar[index] + q_rotate(q_map_lidar, t_lidar_base)[index]
    for index in range(3)
)
q_map_base = q_mul(q_map_lidar, q_lidar_base)
qx, qy, qz, qw = q_map_base
roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
pitch_term = 2.0 * (qw * qy - qz * qx)
pitch = math.copysign(math.pi / 2.0, pitch_term) if abs(pitch_term) >= 1.0 else math.asin(pitch_term)
yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
document = {
    "pb_gicp_localization": {
        "ros__parameters": {
            "initial_map_to_odom": [*t_map_base, roll, pitch, yaw],
        }
    }
}
with open(output_path, "w", encoding="utf-8") as stream:
    yaml.safe_dump(document, stream, sort_keys=False)
PY
fi

for exported_file in \
  "$EXPORT_DIR/field.ply" \
  "$EXPORT_DIR/field.pcd" \
  "$EXPORT_DIR/field.pgm" \
  "$EXPORT_DIR/field.yaml" \
  "$EXPORT_DIR/field.initial_pose.yaml"; do
  if [[ ! -s "$exported_file" ]]; then
    printf '[save-glim-map] exported map file is empty or missing: %s\n' "$exported_file" >&2
    exit 1
  fi
done

existing_maps=()
for map_file in "$MAP_DIR"/field.*; do
  [[ -e "$map_file" ]] || continue
  existing_maps+=("$map_file")
done
if ((${#existing_maps[@]} > 0)); then
  archive_dir="$WORKSPACE/tmp/map_backups/$MAP_ID/$(date +%Y%m%d_%H%M%S)_$$"
  mkdir -p "$archive_dir"
  mv "${existing_maps[@]}" "$archive_dir/"
  printf '[save-glim-map] archived previous field map set to %s\n' "$archive_dir"
fi

mv \
  "$EXPORT_DIR/field.ply" \
  "$EXPORT_DIR/field.pcd" \
  "$EXPORT_DIR/field.pgm" \
  "$EXPORT_DIR/field.yaml" \
  "$EXPORT_DIR/field.initial_pose.yaml" \
  "$MAP_DIR/"

printf '[save-glim-map] saved profile %s map files in %s\n' "$MAP_ID" "$MAP_DIR"
