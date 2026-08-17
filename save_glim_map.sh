#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
GLIM_DUMP="$WORKSPACE/tmp/glim_map"
MAP_DIR="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps"
GLIM_CONFIG="$WORKSPACE/src/navigation/glim/config"
EXPORT_DIR="$WORKSPACE/tmp/glim_map_export_$$"
DESKTOP_RUNTIME_DIR="/run/user/$(id -u)"
DESKTOP_DISPLAY=:0
DESKTOP_XAUTHORITY="$DESKTOP_RUNTIME_DIR/gdm/Xauthority"

cleanup() {
  rm -rf "$EXPORT_DIR"
}
trap cleanup EXIT

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

for required_file in \
  "$GLIM_DUMP/map_frame_normalization.txt" \
  "$GLIM_DUMP/field.pgm" \
  "$GLIM_DUMP/field.yaml" \
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
  --leaf-size 0.10

cp "$GLIM_DUMP/field.pgm" "$EXPORT_DIR/field.pgm"
cp "$GLIM_DUMP/field.yaml" "$EXPORT_DIR/field.yaml"

for exported_file in \
  "$EXPORT_DIR/field.ply" \
  "$EXPORT_DIR/field.pcd" \
  "$EXPORT_DIR/field.pgm" \
  "$EXPORT_DIR/field.yaml"; do
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
  archive_dir="$WORKSPACE/tmp/field_map_backup_$(date +%Y%m%d_%H%M%S)_$$"
  mkdir -p "$archive_dir"
  mv "${existing_maps[@]}" "$archive_dir/"
  printf '[save-glim-map] archived previous field map set to %s\n' "$archive_dir"
fi

mv \
  "$EXPORT_DIR/field.ply" \
  "$EXPORT_DIR/field.pcd" \
  "$EXPORT_DIR/field.pgm" \
  "$EXPORT_DIR/field.yaml" \
  "$MAP_DIR/"

printf '[save-glim-map] saved matched map files in %s\n' "$MAP_DIR"
