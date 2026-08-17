#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
BAG_ROOT="$WORKSPACE/tmp/nav_bags"

usage() {
  cat <<'EOF'
Usage: ./record_nav_debug.sh [label]

Record the navigation topics needed to diagnose localization, planning,
control, TF, LiDAR input, and lower-controller issues.

The bag is written to:
  tmp/nav_bags/<label>_YYYYMMDD_HHMMSS/

Press Ctrl-C once to stop recording cleanly.
EOF
}

case "${1:-}" in
  --help|-h)
    usage
    exit 0
    ;;
  "")
    BAG_LABEL=nav_debug
    ;;
  *)
    BAG_LABEL=$1
    ;;
esac

if (($# > 1)); then
  usage >&2
  exit 2
fi

if [[ ! "$BAG_LABEL" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  printf '[nav-bag] label must contain only letters, numbers, dot, underscore, or hyphen: %s\n' \
    "$BAG_LABEL" >&2
  exit 2
fi

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

mkdir -p "$BAG_ROOT"

bag_stamp=$(date +%Y%m%d_%H%M%S)
BAG_PATH="$BAG_ROOT/${BAG_LABEL}_${bag_stamp}"
if [[ -e "$BAG_PATH" ]]; then
  BAG_PATH="${BAG_PATH}_$$"
fi

# Keep the input and output of every navigation stage in the same bag so that
# a failure can be localized without starting the navigation stack again.
TOPICS=(
  # Runtime logs, diagnostics, and the complete TF tree.
  /rosout
  /parameter_events
  /diagnostics
  /tf
  /tf_static

  # MID-360 input and Point-LIO outputs.
  /livox/lidar
  /livox/lidar/pointcloud
  /livox/imu
  /cloud_registered
  /cloud_registered_body
  /cloud_effected
  /Laser_map
  /aft_mapped_to_init

  # Navigation-frame conversion and terrain pipeline.
  /registered_scan
  /lidar_odometry
  /wheel/odom
  /sensor_scan
  /odometry
  /terrain_map
  /terrain_map_ext
  /obstacle_scan

  # GICP localization result and debug visualization.
  /gicp/localization_map
  /gicp/source_cloud
  /gicp/aligned_cloud
  /gicp/pose
  /gicp/path

  # Map, costmaps, planner/controller debug topics, and the selected goal.
  /map
  /map_metadata
  /global_costmap/costmap
  /global_costmap/costmap_raw
  /global_costmap/footprint
  /global_costmap/published_footprint
  /local_costmap/costmap
  /local_costmap/costmap_raw
  /local_costmap/footprint
  /local_costmap/published_footprint
  /plan
  /local_plan
  /lookahead_point
  /curvature_points_marker_array
  /follow_path_target_pose
  /goal_pose

  # Velocity arbitration and lower-controller protocol traffic.
  /cmd_vel_nav_raw
  /cmd_vel_nav
  /cmd_vel_teleop
  /cmd_vel
  /fruit_hardware_bridge/state_machine/command
  /fruit_hardware_bridge/arm/target
  /fruit_hardware_bridge/execution/callback

  # Navigation action state and lifecycle transitions.
  /navigate_to_pose/_action/goal
  /navigate_to_pose/_action/result
  /navigate_to_pose/_action/feedback
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/cancel
  /navigate_through_poses/_action/goal
  /navigate_through_poses/_action/result
  /navigate_through_poses/_action/feedback
  /navigate_through_poses/_action/status
  /navigate_through_poses/_action/cancel
  /map_server/transition_event
  /controller_server/transition_event
  /planner_server/transition_event
  /behavior_server/transition_event
  /bt_navigator/transition_event
  /waypoint_follower/transition_event
  /velocity_smoother/transition_event
  /lifecycle_manager_navigation/transition_event
)

printf '[nav-bag] output: %s\n' "$BAG_PATH"
printf '[nav-bag] recording raw LiDAR and point-cloud topics; monitor free disk space.\n'
printf '[nav-bag] press Ctrl-C once when the failure has been captured.\n'

ros2 bag record \
  --output "$BAG_PATH" \
  --compression-mode file \
  --compression-format zstd \
  "${TOPICS[@]}"
