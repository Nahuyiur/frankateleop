#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_ROOT="${PIPELINE_LOG_ROOT:-$REPO_ROOT/logs/fr3_pipeline}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$RUN_ID"
READY_TIMEOUT="${PIPELINE_READY_TIMEOUT:-90}"

SCRIPT_PIDS=()
SCRIPT_NAMES=()
SCRIPT_LOGS=()
RECORD_PID=""
SUDO_KEEPALIVE_PID=""
CLEANING_UP=0
ENABLE_CLEANUP=0

usage() {
    cat <<EOF
Usage:
  bash 0_record_fr3_pipeline.sh <task> [output_root] [extra 6_record_fr3 args...]

Examples:
  bash 0_record_fr3_pipeline.sh pick_block
  bash 0_record_fr3_pipeline.sh pick_block /home/pnp/Desktop/franka_record_data --timeout-ms 3000

Recording keys:
  s=start/resume, w=pause, e=save current episode, d=discard current episode,
  k=keyframe, q=save and quit

Environment:
  PIPELINE_READY_TIMEOUT=90      seconds to wait for each startup step
  PIPELINE_LOG_ROOT=...          directory for background script logs
EOF
}

log() {
    printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

tail_log() {
    local file="$1"
    if [[ -f "$file" ]]; then
        log "Last 80 lines from $file:"
        tail -n 80 "$file" || true
    fi
}

process_alive() {
    local pid="$1"
    kill -0 "$pid" 2>/dev/null
}

sudo_cmd() {
    if command -v sudo >/dev/null 2>&1; then
        sudo "$@"
    else
        "$@"
    fi
}

terminate_pid_tree() {
    local pid="$1"
    local label="${2:-process}"

    [[ -z "$pid" ]] && return 0
    process_alive "$pid" || return 0

    local pids=()
    collect_pid_tree "$pid" pids
    ((${#pids[@]} == 0)) && return 0

    log "Stopping $label process tree: ${pids[*]}"
    kill -TERM "${pids[@]}" 2>/dev/null || sudo_cmd kill -TERM "${pids[@]}" 2>/dev/null || true
    sleep 1
    kill -KILL "${pids[@]}" 2>/dev/null || sudo_cmd kill -KILL "${pids[@]}" 2>/dev/null || true
}

collect_pid_tree() {
    local root_pid="$1"
    local array_name="$2"
    local -n out_ref="$array_name"
    local children=()
    local child

    mapfile -t children < <(pgrep -P "$root_pid" 2>/dev/null || true)
    for child in "${children[@]}"; do
        collect_pid_tree "$child" "$array_name"
    done
    out_ref+=("$root_pid")
}

kill_matches() {
    local label="$1"
    local pattern="$2"

    mapfile -t pids < <(pgrep -f "$pattern" 2>/dev/null || true)
    ((${#pids[@]} == 0)) && return 0

    log "Cleaning stale $label process(es): ${pids[*]}"
    for pid in "${pids[@]}"; do
        [[ "$pid" == "$$" || "$pid" == "$BASHPID" ]] && continue
        terminate_pid_tree "$pid" "$label"
    done
}

cleanup_stale_pipeline_processes() {
    log "Cleaning stale 1-7 script processes and known child processes ..."

    kill_matches "script 1" '(^|[ /])1_launch_robot\.sh($| )'
    kill_matches "script 2" '(^|[ /])2_launch_gripper\.sh($| )'
    kill_matches "script 3" '(^|[ /])3_launch_node\.sh($| )'
    kill_matches "script 4" '(^|[ /])4_run_env\.sh($| )'
    kill_matches "script 5" '(^|[ /])5_capture_image\.sh($| )'
    kill_matches "script 6" '(^|[ /])6_record_fr3\.sh($| )'
    kill_matches "script 7" '(^|[ /])7_replay_fr3\.sh($| )'

    kill_matches "Polymetis robot launcher" 'scripts/launch_robot\.py'
    kill_matches "Polymetis gripper launcher" 'scripts/launch_gripper\.py'
    kill_matches "Teleop robot node" 'experiments/launch_nodes\.py'
    kill_matches "Teleop env" 'experiments/run_env\.py'
    kill_matches "Capture image module" 'franka_capture\.scripts\.capture_image'
    kill_matches "Record FR3 module" 'franka_capture\.scripts\.record_fr3'
    kill_matches "Replay FR3 module" 'franka_replay\.replay_fr3'
    kill_matches "run_server" '(^|[ /])run_server($| )'
    kill_matches "franka_hand_client" '(^|[ /])franka_hand_client($| )'
}

cleanup_started_processes() {
    local i
    for ((i=${#SCRIPT_PIDS[@]}-1; i>=0; i--)); do
        terminate_pid_tree "${SCRIPT_PIDS[$i]}" "${SCRIPT_NAMES[$i]}"
    done
    if [[ -n "$RECORD_PID" ]]; then
        terminate_pid_tree "$RECORD_PID" "6_record_fr3.sh"
    fi
}

cleanup_all() {
    ((ENABLE_CLEANUP == 0)) && return 0
    ((CLEANING_UP == 1)) && return 0
    CLEANING_UP=1

    log "Cleaning up pipeline processes ..."
    cleanup_started_processes
    cleanup_stale_pipeline_processes

    if [[ -n "$SUDO_KEEPALIVE_PID" ]]; then
        kill "$SUDO_KEEPALIVE_PID" 2>/dev/null || true
    fi

    log "Cleanup done. Logs: $LOG_DIR"
}

abort() {
    local label="$1"
    local logfile="${2:-}"
    log "ERROR: $label"
    [[ -n "$logfile" ]] && tail_log "$logfile"
    cleanup_all
    exit 1
}

on_signal() {
    local sig="$1"
    log "Received $sig, stopping everything."
    cleanup_all
    exit 130
}

trap 'on_signal INT' INT
trap 'on_signal TERM' TERM
trap 'cleanup_all' EXIT

require_args() {
    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
        usage
        exit 0
    fi
}

prepare_sudo() {
    if command -v sudo >/dev/null 2>&1; then
        log "Checking sudo access up front because script 1 and cleanup may need it."
        sudo -v
        (
            while true; do
                sudo -n true 2>/dev/null || exit 0
                sleep 60
            done
        ) &
        SUDO_KEEPALIVE_PID="$!"
    fi
}

conda_python() {
    local env_name="$1"
    shift
    timeout 10 bash -lc '
        set -e
        source "$(conda info --base)/etc/profile.d/conda.sh"
        conda activate "$1"
        shift
        python "$@"
    ' bash "$env_name" "$@"
}

check_robot_grpc_ready() {
    conda_python polymetis - <<'PY' >/dev/null 2>&1
from polymetis import RobotInterface

robot = RobotInterface(ip_address="127.0.0.1", port=50051, enforce_version=False)
robot.get_joint_positions()
PY
}

check_gripper_grpc_ready() {
    conda_python polymetis - <<'PY' >/dev/null 2>&1
import grpc
import polymetis_pb2
import polymetis_pb2_grpc

channel = grpc.insecure_channel("127.0.0.1:50052")
grpc.channel_ready_future(channel).result(timeout=2)
stub = polymetis_pb2_grpc.GripperServerStub(channel)
stub.GetRobotClientMetadata(polymetis_pb2.Empty(), timeout=2)
PY
}

check_robot_zmq_ready() {
    conda_python polymetis - <<'PY' >/dev/null 2>&1
import pickle
import zmq

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 2000)
sock.setsockopt(zmq.SNDTIMEO, 2000)
sock.setsockopt(zmq.LINGER, 0)
sock.connect("tcp://127.0.0.1:6001")

for method in ("num_dofs", "get_observations"):
    sock.send(pickle.dumps({"method": method, "args": {}}))
    result = pickle.loads(sock.recv())
    if isinstance(result, dict) and "error" in result:
        raise RuntimeError(result["error"])
    if method == "num_dofs" and int(result) <= 0:
        raise RuntimeError(f"bad num_dofs: {result}")
    if method == "get_observations" and "ee_pose_euler" not in result:
        raise RuntimeError("robot node is missing ee_pose_euler")

sock.close(0)
ctx.term()
PY
}

check_env_loop_ready() {
    local logfile="$1"
    grep -q 'Time passed:' "$logfile" 2>/dev/null
}

wait_until_ready() {
    local label="$1"
    local pid="$2"
    local logfile="$3"
    local check_fn="$4"
    local started
    started="$(date +%s)"

    while true; do
        if ! process_alive "$pid"; then
            abort "$label exited before becoming ready." "$logfile"
        fi

        if "$check_fn" "$logfile"; then
            log "$label is ready."
            return 0
        fi

        if (( $(date +%s) - started >= READY_TIMEOUT )); then
            abort "$label did not become ready within ${READY_TIMEOUT}s." "$logfile"
        fi

        sleep 1
    done
}

start_background_script() {
    local label="$1"
    local script="$2"
    local check_fn="$3"
    local logfile="$LOG_DIR/${label// /_}.log"

    log "Starting $label ..."
    (
        cd "$REPO_ROOT"
        export PYTHONUNBUFFERED=1
        exec bash "$script"
    ) >"$logfile" 2>&1 &

    local pid="$!"
    SCRIPT_PIDS+=("$pid")
    SCRIPT_NAMES+=("$label")
    SCRIPT_LOGS+=("$logfile")

    wait_until_ready "$label" "$pid" "$logfile" "$check_fn"
}

run_recording() {
    log "Starting 6_record_fr3.sh in the foreground workflow."
    log "Use RGB window controls: s=start, w=pause, e=save, d=discard, k=keyframe, q=save+quit."
    log "When script 6 exits, scripts 1-4 will be stopped."

    (
        cd "$REPO_ROOT"
        export PYTHONUNBUFFERED=1
        exec bash "$REPO_ROOT/6_record_fr3.sh" "$@"
    ) &
    RECORD_PID="$!"

    set +e
    wait "$RECORD_PID"
    local rc="$?"
    set -e
    RECORD_PID=""

    if ((rc != 0)); then
        abort "6_record_fr3.sh exited with code $rc."
    fi

    log "6_record_fr3.sh exited normally."
}

main() {
    require_args "$@"
    ENABLE_CLEANUP=1
    mkdir -p "$LOG_DIR"

    log "Repo: $REPO_ROOT"
    log "Logs: $LOG_DIR"

    prepare_sudo
    cleanup_stale_pipeline_processes

    start_background_script "1_launch_robot" "$REPO_ROOT/1_launch_robot.sh" check_robot_grpc_ready
    start_background_script "2_launch_gripper" "$REPO_ROOT/2_launch_gripper.sh" check_gripper_grpc_ready
    start_background_script "3_launch_node" "$REPO_ROOT/3_launch_node.sh" check_robot_zmq_ready
    start_background_script "4_run_env" "$REPO_ROOT/4_run_env.sh" check_env_loop_ready

    run_recording "$@"
}

main "$@"
