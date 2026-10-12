#!/usr/bin/env bash

set -u
set -o pipefail

target_pids=()
competition=0
dry_run=0

# These tokens cover the launch files and executable nodes in the current
# perception, odometry, navigation, and teleop design.
target_tokens=(
  # Project-level runners. They use setsid, so matching the runner as well as
  # its children is necessary when a runner was suspended with Ctrl-Z.
  "bash ./start_fruit_perception.sh"
  "bash ./start_fruit_mission.sh"
  "bash ./start_navigation.sh"
  "bash ./start_tele.sh"
  "bash ./waypoints_maker.sh"
  "bash ./record_rosbag.sh"
  "/home/rc4/fruit_ws/start_fruit_perception.sh"
  "/home/rc4/fruit_ws/start_fruit_mission.sh"
  "/home/rc4/fruit_ws/start_navigation.sh"
  "/home/rc4/fruit_ws/start_tele.sh"
  "/home/rc4/fruit_ws/waypoints_maker.sh"
  "/home/rc4/fruit_ws/record_rosbag.sh"
  "/home/rc4/fruit_ws/scripts/robot_debug_monitor.sh"
  "fruit_robot_debug_monitor"
  "fruit_robviz_image_bridge"
  "python3 - fruit-mission "
  "python3 - navigation "
  "python3 - perception "
  "python3 - rosbag "
  "python3 - waypoints-maker "
  "/home/rc4/fruit_ws/test_camera_detector.py"
  "test_camera_detector.py"
  "/home/rc4/fruit_ws/collect_cam_img.py"
  "collect_cam_img.py"
  "bash ./test_camera_arm_target.sh"
  "bash test_camera_arm_target.sh"
  "bash /home/rc4/fruit_ws/test_camera_arm_target.sh"
  "ros2 topic pub --once /fruit_mission/observation_window"
  "ros2 bag record "
  "/lib/rosbag2_transport/record"
  "ros2 launch fruit_perception "
  "ros2 launch fruit_mission "
  "ros2 launch fruit_arm_planner "
  "ros2 launch fruit_qr_planner "
  "ros2 launch auto_serial_bridge "
  "ros2 launch fruit_nav_bringup "
  "ros2 launch fruit_navigation "
  "ros2 launch point_lio "
  "ros2 launch loam_interface "
  "ros2 launch joy_linux "
  "ros2 launch livox_ros_driver2 "
  "/home/rc4/fruit_ws/src/auto_serial_bridge/launch/serial_bridge_by_component.launch.py"
  "/home/rc4/fruit_ws/src/auto_serial_bridge/launch/serial_bridge_by_node.launch.py"
  "/home/rc4/fruit_ws/src/auto_serial_bridge/launch/auto_serial_bridge.launch.py"
  "/home/rc4/fruit_ws/src/fruit_perception/launch/fruit_perception.launch.py"
  "/home/rc4/fruit_ws/src/fruit_mission/launch/fruit_mission.launch.py"
  "/home/rc4/fruit_ws/src/joy_linux/launch/joy_cmd_vel_serial.launch.py"
  "/home/rc4/fruit_ws/src/joy_linux/launch/joy_conventor.launch.py"
  "/home/rc4/fruit_ws/src/joy_linux/launch/joy_conventor_component.launch.py"
  "/home/rc4/fruit_ws/src/navigation/fruit_nav_bringup/launch/fruit_nav.launch.py"
  "/home/rc4/fruit_ws/src/navigation/fruit_nav_bringup/launch/pointlio_odometry.launch.py"
  "/home/rc4/fruit_ws/src/navigation/fruit_nav_bringup/launch/sensor_frames.launch.py"
  "/home/rc4/fruit_ws/src/navigation/fruit_navigation/launch/axis_navigation.launch.py"
  "/home/rc4/fruit_ws/src/navigation/point_lio/launch/point_lio.launch.py"
  "/home/rc4/fruit_ws/src/fruit_robviz_bridge/launch/waypoint_recorder.launch.py"
  "ros2 launch fruit_robviz_bridge waypoint_recorder.launch.py"
  "__node:=fruit_perception_container"
  "__node:=auto_serial_bridge_container"
  "__node:=joy_container"
  "__node:=base_to_lidar"
  "__node:=lidar_to_imu"
  "-c test_camera_detector_"
  "__node:=test_camera_detector_"
  "-c collect_cam_img_"
  "__node:=collect_cam_img_"
  "/lib/fruit_perception/"
  "/lib/fruit_mission/"
  "/lib/fruit_arm_planner/"
  "/home/rc4/fruit_ws/src/fruit_arm_planner/launch/fruit_arm_planner.launch.py"
  "/home/rc4/fruit_ws/src/fruit_arm_planner/launch/fruit_arm_debug.launch.py"
  "/home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py"
  "/lib/fruit_qr_planner/"
  "/lib/auto_serial_bridge/"
  "/lib/livox_ros_driver2/"
  "/lib/fruit_navigation/"
  "/lib/fruit_nav_bringup/"
  "/lib/fruit_robviz_bridge/waypoint_recorder_node"
  "/lib/point_lio/"
  "/lib/loam_interface/"
  "/lib/twist_mux/"
  "/lib/joy_linux/"
  "/lib/tf2_ros/static_transform_publisher"
  "/lib/tf2_ros/tf2_echo"
  "ros2 run fruit_perception "
  "ros2 run fruit_mission "
  "ros2 run fruit_qr_planner "
  "ros2 run auto_serial_bridge "
  "ros2 run livox_ros_driver2 "
  "ros2 run point_lio "
  "ros2 run loam_interface "
  "ros2 run fruit_navigation "
  "ros2 run fruit_robviz_bridge waypoint_recorder_node"
  "ros2 run joy_linux "
  "ros2 run tf2_ros static_transform_publisher "
  "ros2 run tf2_ros tf2_echo "
)

usage() {
  cat <<'EOF'
Usage: ./kill_fruit_processes.sh [--competition] [--dry-run]

Stop task scripts, helper processes, nodes, and launch processes in the current
fruit robot project design. This includes perception, odometry, navigation,
teleop, camera tuning, direct serial diagnostics, and rosbag recording. RViz,
foxglove_bridge, and the system-managed waypoint editor are intentionally
excluded.

--dry-run  Print matching processes without sending signals.
--competition  Stop only competition stacks and motion clients. Preserve
               independent RobViz, waypoint recorder, and rosbag processes.
               The caller and its ancestors are never targets.
EOF
}


is_competition_process() {
  local command_line=$1
  local pid=$2
  local executable package script
  local -a words
  read -r -a words <<< "$command_line"
  ((${#words[@]})) || return 1
  executable=${words[0]}
  case "${executable##*/}" in
    bash|python3|python3.*)
      executable=${words[1]:-}
      words=("${words[@]:1}")
      ;;
  esac
  case "$executable" in
    /home/rc4/fruit_ws/install/*)
      for package in fruit_mission fruit_arm_planner fruit_perception fruit_qr_planner \
        auto_serial_bridge livox_ros_driver2 fruit_navigation point_lio loam_interface joy_linux; do
        [[ "$executable" == */lib/"$package"/* ]] && return 0
      done
      ;;
    /opt/ros/humble/lib/twist_mux/twist_mux) return 0 ;;
    /opt/ros/humble/lib/rclcpp_components/component_container*)
      [[ " $command_line " == *" __node:=fruit_perception_container "* \
        || " $command_line " == *" __node:=auto_serial_bridge_container "* \
        || " $command_line " == *" __node:=joy_container "* ]] && return 0
      ;;
    /opt/ros/humble/lib/tf2_ros/static_transform_publisher)
      [[ " $command_line " == *" __node:=base_to_lidar "* \
        || " $command_line " == *" __node:=lidar_to_imu "* ]] && return 0
      ;;
    ros2|/opt/ros/humble/bin/ros2)
      case "${words[1]:-}" in
        launch|run)
          case "${words[2]:-}" in
            fruit_mission|fruit_arm_planner|fruit_perception|fruit_qr_planner|\
            auto_serial_bridge|livox_ros_driver2|fruit_navigation|fruit_nav_bringup|\
            point_lio|loam_interface|joy_linux) return 0 ;;
            /home/rc4/fruit_ws/src/*/launch/*.launch.py)
              for package in fruit_mission fruit_arm_planner fruit_perception fruit_qr_planner \
                auto_serial_bridge livox_ros_driver2 fruit_navigation fruit_nav_bringup \
                point_lio loam_interface joy_linux; do
                [[ "${words[2]}" == */"$package"/launch/* ]] && return 0
              done
              ;;
          esac
          ;;
      esac
      ;;
  esac
  for script in start_fruit_mission.sh start_fruit_perception.sh start_navigation.sh \
    start_tele.sh waypoints_maker.sh test_camera_arm_target.sh; do
    [[ "$executable" == "/home/rc4/fruit_ws/$script" ]] && return 0
    if [[ "$executable" == "./$script" || "$executable" == "$script" ]]; then
      [[ "$(readlink "/proc/$pid/cwd")" == /home/rc4/fruit_ws ]] && return 0
    fi
  done
  case "$executable" in
    /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py) return 0 ;;
    src/fruit_arm_planner/test/cartesian_console.py|./src/fruit_arm_planner/test/cartesian_console.py)
      [[ "$(readlink "/proc/$pid/cwd")" == /home/rc4/fruit_ws ]] && return 0
      ;;
  esac
  return 1
}

is_target_process() {
  local command_line=$1
  local token

  if ((competition)); then
    is_competition_process "$command_line" "$2"
    return
  fi
  for token in "${target_tokens[@]}"; do
    if [[ "$command_line" == *"$token"* ]]; then
      return 0
    fi
  done

  return 1
}

collect_target_pids() {
  local pid command_line

  while read -r pid command_line; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    [[ " $protected_pids " == *" $pid "* ]] && continue

    if is_target_process "$command_line" "$pid"; then
      printf '%s\n' "$pid"
    fi
  done < <(ps -eo pid=,args=)
}

refresh_targets() {
  mapfile -t target_pids < <(collect_target_pids | sort -n -u)
}

get_target_command() {
  local pid=$1
  ps -p "$pid" -o args= 2>/dev/null
}

is_live_target() {
  local pid=$1
  local command_line

  command_line=$(get_target_command "$pid") || return 1
  is_target_process "$command_line" "$pid"
}

print_targets() {
  local pid

  for pid in "${target_pids[@]}"; do
    ps -p "$pid" -o pid=,ppid=,stat=,args= 2>/dev/null || true
  done
}

signal_targets() {
  local signal=$1
  local pid command_line

  for pid in "${target_pids[@]}"; do
    command_line=$(get_target_command "$pid") || continue
    is_target_process "$command_line" "$pid" || continue

    printf 'Sending SIG%s to PID %s\n' "$signal" "$pid"
    kill "-$signal" "$pid" 2>/dev/null || true
  done
}

wait_for_targets() {
  local attempts=$1
  local pid

  while (( attempts > 0 )); do
    for pid in "${target_pids[@]}"; do
      if is_live_target "$pid"; then
        sleep 0.1
        ((attempts--))
        continue 2
      fi
    done
    return 0
  done

  return 1
}

stop_ros_daemon() {
  command -v ros2 >/dev/null 2>&1 || return 0
  if ros2 daemon stop >/dev/null 2>&1; then
    printf 'Stopped the ROS 2 daemon to discard stale graph entries.\n'
  else
    printf 'Warning: failed to stop the ROS 2 daemon.\n' >&2
  fi
}

while (($#)); do
case "$1" in
  --competition) competition=1 ;;
  --dry-run) dry_run=1 ;;
  --help|-h)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
shift
done

protected_pids="$$"
ancestor=$PPID
while [[ "$ancestor" =~ ^[0-9]+$ ]] && ((ancestor > 0)); do
  protected_pids+=" $ancestor"
  ancestor=$(ps -p "$ancestor" -o ppid=) || break
  ancestor=${ancestor//[[:space:]]/}
done

if command -v systemctl >/dev/null 2>&1 && \
  systemctl is-active --quiet fruit-perception.service 2>/dev/null; then
  printf 'Warning: fruit-perception.service is active and may restart the launch.\n' >&2
  printf 'Stop it separately with: sudo systemctl stop fruit-perception.service\n' >&2
  if ((competition && !dry_run)); then
    exit 1
  fi
fi

refresh_targets

if ((${#target_pids[@]} == 0)); then
  printf 'No project processes found.\n'
  if ((${dry_run:-0} == 0)); then
    stop_ros_daemon
  fi
  exit 0
fi

printf 'Matching project processes:\n'
print_targets

if ((${dry_run:-0} == 1)); then
  exit 0
fi

# Queue interruption before resuming suspended scripts, so they cannot submit another motion first.
signal_targets INT
signal_targets CONT
if wait_for_targets 50; then
  printf 'All matching processes stopped gracefully.\n'
  stop_ros_daemon
  exit 0
fi

refresh_targets
signal_targets TERM
if wait_for_targets 30; then
  printf 'All matching processes stopped after SIGTERM.\n'
  stop_ros_daemon
  exit 0
fi

refresh_targets
signal_targets KILL
if wait_for_targets 10; then
  printf 'All matching processes stopped after SIGKILL.\n'
  stop_ros_daemon
  exit 0
fi

refresh_targets
printf 'Some matching processes are still running:\n' >&2
print_targets >&2
exit 1
