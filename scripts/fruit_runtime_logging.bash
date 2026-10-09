#!/usr/bin/env bash
# Shared runtime logging helper for fruit_ws scripts.
# Manages session directories under /home/rc4/fruit_ws/log/runtime,
# retention of the latest 20 completed sessions with flock protection,
# and streaming of component stdout/stderr into dedicated log files.

FRUIT_WS_ROOT="/home/rc4/fruit_ws"
RUNTIME_BASE_DIR="${FRUIT_WS_ROOT}/log/runtime"
FILTER_SCRIPT="${FRUIT_WS_ROOT}/scripts/runtime_log_filter.py"

fruit_logging_init() {
    local entry_name="${1:-entry}"
    local debug_mode="${2:-0}"

    if [[ -n "${FRUIT_RUNTIME_SESSION_DIR:-}" && -d "${FRUIT_RUNTIME_SESSION_DIR}" ]]; then
        # Inherit existing parent session directory
        export FRUIT_LOGGING_DEBUG="${debug_mode}"
        export FRUIT_RUNTIME_TOP_ENTRY=0
        return 0
    fi

    local timestamp
    timestamp="$(date +%Y-%m-%d_%H-%M-%S)"
    local session_dir="${RUNTIME_BASE_DIR}/${timestamp}_${entry_name}_$$"
    mkdir -p "${session_dir}/ros"

    # Acquire active execution lock for this session
    exec 200>"${session_dir}/.active.lock"
    if ! flock -n 200; then
        echo "[logging][ERROR] Failed to acquire active execution lock on ${session_dir}" >&2
        return 1
    fi

    # Update atomic latest symlink
    ln -sfn "$(basename "${session_dir}")" "${RUNTIME_BASE_DIR}/latest"

    export FRUIT_RUNTIME_SESSION_DIR="${session_dir}"
    export ROS_LOG_DIR="${session_dir}/ros"
    export FRUIT_RUNTIME_TOP_ENTRY=1
    export FRUIT_LOGGING_DEBUG="${debug_mode}"

    echo "[logging] Session logs directory: ${session_dir}"

    # Log orchestrator launch info
    local orchestrator_log="${session_dir}/orchestrator.log"
    {
        echo "=== Session Started ==="
        echo "Entry: ${entry_name}"
        echo "PID: $$"
        echo "Timestamp: $(date -Iseconds)"
        echo "Debug: ${debug_mode}"
        echo "Session Dir: ${session_dir}"
    } >> "${orchestrator_log}"

    # Perform cleanup of old completed sessions
    fruit_logging_cleanup_old_sessions
}

fruit_logging_cleanup_old_sessions() {
    local max_keep=20
    mkdir -p "${RUNTIME_BASE_DIR}"

    local completed_dirs=()
    local dir
    for dir in "${RUNTIME_BASE_DIR}"/*; do
        [[ -d "${dir}" ]] || continue
        [[ -L "${dir}" ]] && continue
        local bname
        bname="$(basename "${dir}")"
        [[ "${bname}" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}_ ]] || continue

        # Check if dir is currently locked by an active process
        local lock_file="${dir}/.active.lock"
        if [[ -f "${lock_file}" ]]; then
            # Test lock non-blockingly using a subshell
            if ( exec 201>"${lock_file}" && flock -n 201 ) 2>/dev/null; then
                completed_dirs+=("${dir}")
            fi
        else
            # No lock file means it's an old finished directory
            completed_dirs+=("${dir}")
        fi
    done

    # Sort completed directories by name (which has timestamp prefix YYYY-MM-DD_HH-MM-SS)
    if (( ${#completed_dirs[@]} > max_keep )); then
        IFS=$'\n' sorted_dirs=($(sort <<<"${completed_dirs[*]}"))
        unset IFS
        local count=${#sorted_dirs[@]}
        local to_delete=$((count - max_keep))
        for ((i=0; i<to_delete; i++)); do
            local del_dir="${sorted_dirs[i]}"
            # Ensure it is strictly inside RUNTIME_BASE_DIR and not symlink
            if [[ "${del_dir}" == "${RUNTIME_BASE_DIR}"/* && ! -L "${del_dir}" && -d "${del_dir}" ]]; then
                # Double check lock before removing
                if [[ ! -f "${del_dir}/.active.lock" ]] || ( exec 202>"${del_dir}/.active.lock" && flock -n 202 ) 2>/dev/null; then
                    rm -rf "${del_dir}" 2>/dev/null || true
                fi
            fi
        done
    fi
}

fruit_run_logged() {
    local component_name="$1"
    shift
    local log_file="${FRUIT_RUNTIME_SESSION_DIR}/${component_name}.log"

    local filter_args=("--output-file" "${log_file}" "--component" "${component_name}")
    if [[ "${FRUIT_LOGGING_DEBUG:-0}" == "1" ]]; then
        filter_args+=("--debug")
    fi

    # Run the command and pipe stdout+stderr through the log filter,
    # preserving the exit code of the original command.
    local restore_e=0
    [[ "$-" =~ e ]] && restore_e=1
    set +e
    "$@" 2>&1 | /usr/bin/python3 "${FILTER_SCRIPT}" "${filter_args[@]}"
    local exit_code="${PIPESTATUS[0]}"
    if [[ "$restore_e" -eq 1 ]]; then
        set -e
    else
        set +e
    fi
    return "${exit_code}"
}

fruit_stop_process_group() {
    local pid="$1"
    local name="${2:-process}"
    local int_timeout="${3:-5}"
    local term_timeout="${4:-2}"

    [[ -n "$pid" ]] || return 0
    kill -0 "$pid" 2>/dev/null || return 0

    # 1. SIGINT to process group, fallback to individual pid
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true

    local deadline=$((SECONDS + int_timeout))
    while (( SECONDS < deadline )) && kill -0 "$pid" 2>/dev/null; do
        sleep 0.1
    done

    # 2. SIGTERM
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
        deadline=$((SECONDS + term_timeout))
        while (( SECONDS < deadline )) && kill -0 "$pid" 2>/dev/null; do
            sleep 0.1
        done
    fi

    # 3. SIGKILL
    if kill -0 "$pid" 2>/dev/null; then
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    fi

    wait "$pid" 2>/dev/null || true
}

fruit_stop_all_process_groups() {
    local -a pids=("$@")
    local pid

    ((${#pids[@]} == 0)) && return 0

    # Phase 1: send SIGINT to all groups simultaneously
    for pid in "${pids[@]}"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
        fi
    done

    # Phase 2: wait up to 5s for all to finish
    local deadline=$((SECONDS + 5))
    while (( SECONDS < deadline )); do
        local any_alive=0
        for pid in "${pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                any_alive=1
                break
            fi
        done
        (( any_alive == 0 )) && break
        sleep 0.1
    done

    # Phase 3: escalate to SIGTERM
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    deadline=$((SECONDS + 2))
    while (( SECONDS < deadline )); do
        local any_alive=0
        for pid in "${pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                any_alive=1
                break
            fi
        done
        (( any_alive == 0 )) && break
        sleep 0.1
    done

    # Phase 4: escalate to SIGKILL
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
        fi
    done

    # Phase 5: reap all
    for pid in "${pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
}

fruit_logging_close() {
    local exit_code="${1:-0}"
    if [[ "${FRUIT_RUNTIME_TOP_ENTRY:-0}" == "1" && -n "${FRUIT_RUNTIME_SESSION_DIR:-}" ]]; then
        local orchestrator_log="${FRUIT_RUNTIME_SESSION_DIR}/orchestrator.log"
        if [[ -d "${FRUIT_RUNTIME_SESSION_DIR}" ]]; then
            {
                echo "=== Session Finished ==="
                echo "Timestamp: $(date -Iseconds)"
                echo "Exit Code: ${exit_code}"
            } >> "${orchestrator_log}" 2>/dev/null || true
        fi
        # Run cleanup again on exit
        fruit_logging_cleanup_old_sessions
        # Release lock descriptor 200
        exec 200>&- 2>/dev/null || true
    fi
}
