#!/usr/bin/env bash
# Shared debug recording ownership and topic profiles for fruit_ws entrypoints.

FRUIT_WS_ROOT="${FRUIT_WS_ROOT:-/home/rc4/fruit_ws}"
FRUIT_RECORDING_PROFILE="${FRUIT_RECORDING_PROFILE:-}"
FRUIT_RECORDING_OWNER_PID="${FRUIT_RECORDING_OWNER_PID:-}"
FRUIT_RECORDING_PID="${FRUIT_RECORDING_PID:-}"
FRUIT_RECORDING_LOG="${FRUIT_RECORDING_LOG:-}"
FRUIT_RECORDING_SKIPPED="${FRUIT_RECORDING_SKIPPED:-0}"

fruit_recording_profile_topics() {
    local profile=$1

    case "$profile" in
        navigation)
            cat <<'EOF'
/tf
/tf_static
/livox/lidar
/livox/imu
/aft_mapped_to_init
/cloud_registered
/odometry
/cmd_vel_nav
/cmd_vel
/follow_waypoints/_action/feedback
/follow_waypoints/_action/status
EOF
            ;;
        waypoints)
            fruit_recording_profile_topics navigation
            cat <<'EOF'
/fruit/waypoint_maker/command
/fruit/waypoint_maker/state
/fruit/waypoint_maker/markers
EOF
            ;;
        waypoints_tele)
            fruit_recording_profile_topics waypoints
            cat <<'EOF'
/joy
/cmd_vel_teleop
EOF
            ;;
        teleop)
            cat <<'EOF'
/joy
/cmd_vel_teleop
/cmd_vel
/fruit_hardware_bridge/arm/status
/auto_serial_bridge/ready
EOF
            ;;
        perception)
            cat <<'EOF'
/fruit_perception/decision
/fruit_perception/camera/preview_detections
/fruit_perception/camera/rgbd_heartbeat
/fruit_mission/observation_window
/fruit_mission/current_area
/fruit/b_pick_plan
/fruit/c_pick_plan
/fruit_hardware_bridge/arm/status
/auto_serial_bridge/ready
EOF
            ;;
        camera_arm)
            fruit_recording_profile_topics perception
            cat <<'EOF'
/arm_planner_node/joint_states
/fruit_hardware_bridge/arm/trajectory_header
/fruit_hardware_bridge/arm/trajectory_point
/fruit_hardware_bridge/arm/control_mode
EOF
            ;;
        mission)
            fruit_recording_profile_topics navigation
            fruit_recording_profile_topics camera_arm
            ;;
        full)
            fruit_recording_profile_topics navigation
            fruit_recording_profile_topics waypoints_tele
            fruit_recording_profile_topics camera_arm
            ;;
        *)
            printf '[recording][ERROR] unknown profile: %s\n' "$profile" >&2
            return 2
            ;;
    esac
}

fruit_recording_profile_uses_camera() {
    case "$1" in
        perception|camera_arm|mission|full)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

fruit_recording_prepare() {
    local profile=$1

    if [[ -n "${FRUIT_RECORDING_OWNER_PID}" ]]; then
        return 0
    fi
    export FRUIT_RECORDING_PROFILE="$profile"
    export FRUIT_RECORDING_OWNER_PID="$$"
    export FRUIT_RECORDING_RESERVED=1
}

fruit_recording_node_running() {
    /usr/bin/grep -Fxq '/rosbag2_recorder' < <(ros2 node list 2>/dev/null)
}

fruit_recording_start() {
    local record_script=$1
    local profile=${2:-${FRUIT_RECORDING_PROFILE}}
    local filter_script="${FRUIT_WS_ROOT}/scripts/runtime_log_filter.py"

    [[ "${FRUIT_RECORDING_OWNER_PID}" == "$$" ]] || return 0
    [[ -z "${FRUIT_RECORDING_PID}" ]] || return 0
    if fruit_recording_node_running; then
        FRUIT_RECORDING_SKIPPED=1
        export FRUIT_RECORDING_SKIPPED
        printf '[recording] rosbag recorder already exists; skip automatic recording\n'
        return 0
    fi

    FRUIT_RECORDING_PROFILE="$profile"
    FRUIT_RECORDING_LOG="${FRUIT_RUNTIME_SESSION_DIR}/rosbag.log"
    : > "$FRUIT_RECORDING_LOG"
    printf '[recording] start profile=%s\n' "$profile"
    printf '[recording] log=%s\n' "$FRUIT_RECORDING_LOG"

    setsid env \
        FRUIT_RECORDING_WORKER=1 \
        FRUIT_RECORDING_PROFILE="$profile" \
        FRUIT_RECORDING_LOG_FILE="$FRUIT_RECORDING_LOG" \
        FRUIT_RECORDING_FILTER_SCRIPT="$filter_script" \
        FRUIT_RECORDING_DEBUG="${FRUIT_LOGGING_DEBUG:-0}" \
        bash -c '
            set -o pipefail
            filter_args=(--output-file "$FRUIT_RECORDING_LOG_FILE" --component rosbag)
            [[ "$FRUIT_RECORDING_DEBUG" == "1" ]] && filter_args+=(--debug)
            "$@" 2>&1 | /usr/bin/python3 "$FRUIT_RECORDING_FILTER_SCRIPT" "${filter_args[@]}"
        ' fruit-recording "$record_script" &
    FRUIT_RECORDING_PID=$!
    export FRUIT_RECORDING_PID FRUIT_RECORDING_LOG
}

fruit_recording_wait_ready() {
    local timeout_sec=${1:-10}
    local deadline=$((SECONDS + timeout_sec))

    [[ -n "${FRUIT_RECORDING_PID}" ]] || return 0
    while (( SECONDS < deadline )); do
        if fruit_recording_node_running; then
            printf '[recording] recorder ready node=/rosbag2_recorder\n'
            return 0
        fi
        sleep 0.2
    done
    printf '[recording][WARN] recorder was not ready within %ss\n' "$timeout_sec" >&2
    return 1
}

fruit_recording_stop() {
    [[ -n "${FRUIT_RECORDING_PID}" ]] || return 0
    fruit_stop_process_group "$FRUIT_RECORDING_PID" "rosbag" 10 2
    FRUIT_RECORDING_PID=""
    export FRUIT_RECORDING_PID
}
