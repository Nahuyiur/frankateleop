#!/bin/bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
CLIENT="$REPO_ROOT/adaptor/franka_eraser_closed_loop_client.py"
LOG_ROOT="${ERASER_LOG_ROOT:-$REPO_ROOT/logs/eraser_closed_loop}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$RUN_ID"
READY_TIMEOUT="${ERASER_READY_TIMEOUT:-120}"
STACK_DIR="${ERASER_STACK_DIR:-$REPO_ROOT}"
ROBOT_GRPC_PORT="${ERASER_ROBOT_GRPC_PORT:-50051}"
GRIPPER_GRPC_PORT="${ERASER_GRIPPER_GRPC_PORT:-50052}"
ROBOT_ZMQ_PORT="${ERASER_ROBOT_ZMQ_PORT:-6001}"
KEEP_STACK="${ERASER_KEEP_STACK:-0}"
STARTED_PIDS=()
STARTED_NAMES=()
STARTED_LOGS=()

log() {
    printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

process_alive() {
    kill -0 "$1" 2>/dev/null
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

terminate_pid_tree() {
    local pid="$1"
    local label="$2"
    local pids=()
    process_alive "$pid" || return 0
    collect_pid_tree "$pid" pids
    ((${#pids[@]} == 0)) && return 0
    log "Stopping $label process tree: ${pids[*]}"
    kill -TERM "${pids[@]}" 2>/dev/null || true
    sleep 1
    kill -KILL "${pids[@]}" 2>/dev/null || true
}

cleanup_started_stack() {
    if [[ "$KEEP_STACK" == "1" ]]; then
        log "Keeping auto-started robot stack because ERASER_KEEP_STACK=1"
        return 0
    fi
    local i
    for ((i=${#STARTED_PIDS[@]}-1; i>=0; i--)); do
        terminate_pid_tree "${STARTED_PIDS[$i]}" "${STARTED_NAMES[$i]}"
    done
}

trap cleanup_started_stack EXIT

has_arg() {
    local needle="$1"
    shift
    local arg
    for arg in "$@"; do
        [[ "$arg" == "$needle" ]] && return 0
    done
    return 1
}

arg_value() {
    local name="$1"
    local default="$2"
    shift 2
    local prev=""
    local arg
    for arg in "$@"; do
        if [[ "$prev" == "$name" ]]; then
            printf '%s\n' "$arg"
            return 0
        fi
        if [[ "$arg" == "$name="* ]]; then
            printf '%s\n' "${arg#*=}"
            return 0
        fi
        prev="$arg"
    done
    printf '%s\n' "$default"
}

if [[ ! -f "$CLIENT" ]]; then
    echo "ERROR: missing client script: $CLIENT"
    exit 1
fi

echo ">>> Activating conda environment: franka_capture"
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate franka_capture || { echo "ERROR: failed to activate franka_capture"; exit 1; }

for arg in "$@"; do
    if [[ "$arg" == "-h" || "$arg" == "--help" ]]; then
        python "$CLIENT" "$@"
        exit 0
    fi
done

python - <<'PY'
import importlib.util
import subprocess
import sys

required = {
    "numpy": "numpy",
    "cv2": "opencv-python",
    "zmq": "pyzmq",
    "pyrealsense2": "pyrealsense2",
    "websockets": "websockets",
}

missing = [module for module in required if importlib.util.find_spec(module) is None]
if missing:
    packages = [required[module] for module in missing]
    print(
        ">>> Missing Python dependencies in franka_capture: "
        + ", ".join(missing)
    )
    print(">>> Installing: " + " ".join(packages))
    subprocess.check_call([sys.executable, "-m", "pip", "install", *packages])

still_missing = [
    module for module in required if importlib.util.find_spec(module) is None
]
if still_missing:
    raise SystemExit(
        "ERROR: dependencies are still missing after auto-install: "
        + ", ".join(still_missing)
    )
PY

conda_python() {
    local env_name="$1"
    shift
    timeout 15 bash -lc '
        set -e
        source "$(conda info --base)/etc/profile.d/conda.sh"
        conda activate "$1"
        shift
        python "$@"
    ' bash "$env_name" "$@"
}

check_robot_grpc_ready() {
    conda_python polymetis - "$ROBOT_GRPC_PORT" <<'PY' >/dev/null 2>&1
import sys
from polymetis import RobotInterface

port = int(sys.argv[1])
robot = RobotInterface(ip_address="127.0.0.1", port=port, enforce_version=False)
robot.get_joint_positions()
PY
}

check_gripper_grpc_ready() {
    conda_python polymetis - "$GRIPPER_GRPC_PORT" <<'PY' >/dev/null 2>&1
import sys
import grpc
import polymetis_pb2
import polymetis_pb2_grpc

port = int(sys.argv[1])
channel = grpc.insecure_channel(f"127.0.0.1:{port}")
grpc.channel_ready_future(channel).result(timeout=2)
stub = polymetis_pb2_grpc.GripperServerStub(channel)
stub.GetRobotClientMetadata(polymetis_pb2.Empty(), timeout=2)
PY
}

check_robot_zmq_ready() {
    conda_python franka_capture - "$ROBOT_ZMQ_PORT" <<'PY' >/dev/null 2>&1
import pickle
import sys
import zmq

port = int(sys.argv[1])
ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 2000)
sock.setsockopt(zmq.SNDTIMEO, 2000)
sock.setsockopt(zmq.LINGER, 0)
sock.connect(f"tcp://127.0.0.1:{port}")

for method in ("num_dofs", "get_observations", "get_control_mode"):
    sock.send(pickle.dumps({"method": method, "args": {}}))
    result = pickle.loads(sock.recv())
    if isinstance(result, dict) and "error" in result:
        raise RuntimeError(result["error"])
    if method == "num_dofs" and int(result) <= 0:
        raise RuntimeError(f"bad num_dofs: {result}")
    if method == "get_observations" and "ee_pose_euler" not in result:
        raise RuntimeError("robot node is missing ee_pose_euler")
    if method == "get_control_mode" and str(result) != "ee":
        raise RuntimeError(f"robot node is running control_mode={result!r}, expected 'ee'")

sock.close(0)
ctx.term()
PY
}

check_robot_zmq_any_ready() {
    conda_python franka_capture - "$ROBOT_ZMQ_PORT" <<'PY' >/dev/null 2>&1
import pickle
import sys
import zmq

port = int(sys.argv[1])
ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 2000)
sock.setsockopt(zmq.SNDTIMEO, 2000)
sock.setsockopt(zmq.LINGER, 0)
sock.connect(f"tcp://127.0.0.1:{port}")

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

robot_zmq_control_mode() {
    conda_python franka_capture - "$ROBOT_ZMQ_PORT" <<'PY'
import pickle
import sys
import zmq

port = int(sys.argv[1])
ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 2000)
sock.setsockopt(zmq.SNDTIMEO, 2000)
sock.setsockopt(zmq.LINGER, 0)
sock.connect(f"tcp://127.0.0.1:{port}")
sock.send(pickle.dumps({"method": "get_control_mode", "args": {}}))
result = pickle.loads(sock.recv())
sock.close(0)
ctx.term()
if isinstance(result, dict) and "error" in result:
    raise RuntimeError(result["error"])
print(result)
PY
}

tail_log() {
    local logfile="$1"
    if [[ -f "$logfile" ]]; then
        log "Last 80 lines from $logfile:"
        tail -n 80 "$logfile" || true
    fi
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
            log "ERROR: $label exited before becoming ready."
            tail_log "$logfile"
            exit 1
        fi
        if "$check_fn"; then
            log "$label is ready."
            return 0
        fi
        if (( $(date +%s) - started >= READY_TIMEOUT )); then
            log "ERROR: $label did not become ready within ${READY_TIMEOUT}s."
            tail_log "$logfile"
            exit 1
        fi
        sleep 1
    done
}

start_background_script() {
    local label="$1"
    local script="$2"
    local check_fn="$3"
    shift 3
    local logfile="$LOG_DIR/${label// /_}.log"
    local script_dir
    script_dir="$(cd "$(dirname "$script")"; pwd)"

    mkdir -p "$LOG_DIR"
    log "Starting $label; log: $logfile"
    (
        cd "$script_dir"
        export PYTHONUNBUFFERED=1
        exec bash "$script" "$@"
    ) >"$logfile" 2>&1 &

    local pid="$!"
    STARTED_PIDS+=("$pid")
    STARTED_NAMES+=("$label")
    STARTED_LOGS+=("$logfile")
    wait_until_ready "$label" "$pid" "$logfile" "$check_fn"
}

prepare_sudo_if_needed() {
    if [[ -f "$STACK_DIR/1_launch_robot.sh" ]] && command -v sudo >/dev/null 2>&1; then
        log "Checking sudo access up front because robot launch scripts may need it."
        sudo -v
    fi
}

ensure_execute_stack() {
    local robot_script="$STACK_DIR/1_launch_robot.sh"
    local gripper_script="$STACK_DIR/2_launch_gripper.sh"
    local node_script="$STACK_DIR/3_launch_node.sh"

    if [[ ! -f "$robot_script" || ! -f "$gripper_script" || ! -f "$node_script" ]]; then
        log "ERROR: cannot find 1/2/3 launch scripts in STACK_DIR=$STACK_DIR"
        exit 1
    fi

    mkdir -p "$LOG_DIR"
    log "Robot stack requested. Ensuring robot stack is ready."
    log "Stack dir: $STACK_DIR"
    log "Logs: $LOG_DIR"
    log "Note: 4_run_env.sh is intentionally not started for policy execution."

    prepare_sudo_if_needed

    if check_robot_grpc_ready; then
        log "Robot gRPC service is already ready on port $ROBOT_GRPC_PORT."
    else
        start_background_script "1_launch_robot" "$robot_script" check_robot_grpc_ready
    fi

    if check_gripper_grpc_ready; then
        log "Gripper gRPC service is already ready on port $GRIPPER_GRPC_PORT."
    else
        start_background_script "2_launch_gripper" "$gripper_script" check_gripper_grpc_ready
    fi

    if check_robot_zmq_ready; then
        log "Robot ZMQ node is already ready on port $ROBOT_ZMQ_PORT."
    elif check_robot_zmq_any_ready; then
        mode="$(robot_zmq_control_mode 2>/dev/null || printf 'unknown')"
        log "ERROR: Robot ZMQ node is already running on port $ROBOT_ZMQ_PORT but is not ee-ready (control_mode=$mode)."
        log "Stop the existing node or 4_run_env.sh, then rerun this script so it can start 3_launch_node.sh --control-mode ee."
        exit 1
    else
        start_background_script \
            "3_launch_node_ee" \
            "$node_script" \
            check_robot_zmq_ready \
            --control-mode ee \
            --no-home-on-init \
            --no-open-gripper-on-init
    fi
}

if has_arg "--execute" "$@" || has_arg "--capture-policy-start" "$@"; then
    ROBOT_ZMQ_PORT="$(arg_value "--robot-port" "$ROBOT_ZMQ_PORT" "$@")"
    ensure_execute_stack
fi

echo ">>> Running FR3 eraser closed-loop client"
if has_arg "--capture-policy-start" "$@"; then
    echo ">>> CAPTURE POLICY START mode: saving current EE pose/gripper, then exiting."
elif has_arg "--execute" "$@"; then
    echo ">>> EXECUTE mode: robot commands may be sent."
else
    echo ">>> Default mode is dry-run. Add --execute only after dry-run checks pass."
fi
python "$CLIENT" "$@"
