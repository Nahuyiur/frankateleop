#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RIGHT_SSH="${BI_ARM_RIGHT_SSH:-192.168.1.131}"
RIGHT_REPO="${BI_ARM_RIGHT_REPO:-/home/pnp/frankateleop}"
RIGHT_REMOTE_ZMQ_PORT="${BI_ARM_RIGHT_REMOTE_ZMQ_PORT:-6001}"
RIGHT_LOCAL_ZMQ_PORT="${BI_ARM_RIGHT_LOCAL_ZMQ_PORT:-16001}"
RIGHT_LOCAL_GRIPPER_PORT="${BI_ARM_RIGHT_LOCAL_GRIPPER_PORT:-15053}"
LEFT_ZMQ_PORT="${BI_ARM_LEFT_ZMQ_PORT:-6002}"
LEFT_ROBOT_PORT="${BI_ARM_LEFT_ROBOT_PORT:-50052}"
LEFT_GRIPPER_PORT="${BI_ARM_LEFT_GRIPPER_PORT:-50054}"
RIGHT_ROBOT_PORT="${BI_ARM_RIGHT_ROBOT_PORT:-50051}"
RIGHT_GRIPPER_PORT="${BI_ARM_RIGHT_GRIPPER_PORT:-50053}"
READY_TIMEOUT="${BI_ARM_READY_TIMEOUT:-120}"
SSH_PASSWORD="${BI_ARM_SSH_PASSWORD:-}"
LOCAL_SUDO_PASSWORD="${BI_ARM_LOCAL_SUDO_PASSWORD:-}"
REMOTE_SUDO_PASSWORD="${BI_ARM_REMOTE_SUDO_PASSWORD:-}"
MOVE_TO_INITIAL_POSE="${BI_ARM_REPLAY_MOVE_TO_INITIAL_POSE:-0}"
LOG_ROOT="${BI_ARM_REPLAY_LOG_ROOT:-$REPO_ROOT/logs/bi_arm_replay}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$RUN_ID"
REMOTE_LOG_DIR="${BI_ARM_REMOTE_REPLAY_LOG_DIR:-$RIGHT_REPO/logs/bi_arm_replay/$RUN_ID}"
SYNC_REMOTE_RIGHT_SCRIPTS="${BI_ARM_SYNC_REMOTE_RIGHT_SCRIPTS:-1}"
LEFT_ROBOTIQ_COMPORT_OVERRIDE="${BI_ARM_LEFT_ROBOTIQ_COMPORT:-${LEFT_ROBOTIQ_COMPORT:-${FRANKA_ROBOTIQ_COMPORT:-}}}"
RIGHT_ROBOTIQ_COMPORT_OVERRIDE="${BI_ARM_RIGHT_ROBOTIQ_COMPORT:-${RIGHT_ROBOTIQ_COMPORT:-}}"

DEFAULT_REPLAY_SPEED="${DEFAULT_REPLAY_SPEED:-1.0}"
DEFAULT_GRIPPER_SPEED="${DEFAULT_GRIPPER_SPEED:-0.1}"
DEFAULT_GRIPPER_FORCE="${DEFAULT_GRIPPER_FORCE:-10.0}"
DEFAULT_GRIPPER_EVENT_DELTA="${DEFAULT_GRIPPER_EVENT_DELTA:-0.01}"
DEFAULT_GRIPPER_REPLAY_MODE="${DEFAULT_GRIPPER_REPLAY_MODE:-event}"
DEFAULT_GRIPPER_COMMAND_HZ="${DEFAULT_GRIPPER_COMMAND_HZ:-15.0}"
DEFAULT_GRIPPER_HOLD_SEC="${DEFAULT_GRIPPER_HOLD_SEC:-2.0}"
DEFAULT_APPROACH_START="${DEFAULT_APPROACH_START:-1}"
DEFAULT_APPROACH_START_MAX_DELTA="${DEFAULT_APPROACH_START_MAX_DELTA:-0.75}"
DEFAULT_APPROACH_START_STEP_DELTA="${DEFAULT_APPROACH_START_STEP_DELTA:-0.02}"
DEFAULT_APPROACH_START_HZ="${DEFAULT_APPROACH_START_HZ:-5.0}"

LOCAL_PIDS=()
LOCAL_NAMES=()
REMOTE_PID_FILES=()
REMOTE_NAMES=()
TUNNEL_PID=""
TUNNEL_LOG=""
SUDO_KEEPALIVE_PID=""
SSH_ASKPASS_FILE=""
CLEANING_UP=0
ENABLE_CLEANUP=0

usage() {
    cat <<EOF
Usage:
  bash 16_replay_bi_arm_pipeline.sh <dual-arm episode dir, metadata.json, or pkl.gz> [replay args...]

Examples:
  bash 16_replay_bi_arm_pipeline.sh /home/pnp/Desktop/franka_record_data/test/High_Quality/0
  bash 16_replay_bi_arm_pipeline.sh /home/pnp/Desktop/franka_record_data/test/High_Quality --latest
  bash 16_replay_bi_arm_pipeline.sh /home/pnp/Desktop/franka_record_data/test/High_Quality/0 --skip-robot-check
  bash 16_replay_bi_arm_pipeline.sh /home/pnp/Desktop/franka_record_data/test/High_Quality/0 --execute
  bash 16_replay_bi_arm_pipeline.sh /home/pnp/Desktop/franka_record_data/test/High_Quality/0 --execute --speed 0.5

Default mode is dry-run: it starts/checks both robot nodes but sends no replay
trajectory unless --execute is passed. If DEFAULT_APPROACH_START=1 and the
start delta is within DEFAULT_APPROACH_START_MAX_DELTA, dry-run reports that
execute mode can auto-approach frame 0 instead of failing.

--skip-robot-check only validates the episode file and exits before sudo, SSH,
port cleanup, or hardware stack startup.
Environment:
  BI_ARM_RIGHT_SSH=192.168.1.131
  BI_ARM_RIGHT_REPO=/home/pnp/frankateleop
  BI_ARM_RIGHT_LOCAL_ZMQ_PORT=16001
  BI_ARM_RIGHT_LOCAL_GRIPPER_PORT=15053
  BI_ARM_READY_TIMEOUT=120
  BI_ARM_REPLAY_LOG_ROOT=$REPO_ROOT/logs/bi_arm_replay
  BI_ARM_CLEAN_STALE=1
  BI_ARM_SYNC_REMOTE_RIGHT_SCRIPTS=1
  BI_ARM_SSH_PASSWORD=
  BI_ARM_LOCAL_SUDO_PASSWORD=
  BI_ARM_REMOTE_SUDO_PASSWORD=
  BI_ARM_REPLAY_MOVE_TO_INITIAL_POSE=0
  BI_ARM_LEFT_ROBOTIQ_COMPORT=
  BI_ARM_RIGHT_ROBOTIQ_COMPORT=
  DEFAULT_GRIPPER_REPLAY_MODE=event
  DEFAULT_GRIPPER_COMMAND_HZ=15.0
  DEFAULT_APPROACH_START=1
  DEFAULT_APPROACH_START_MAX_DELTA=0.75

This script starts local left_franka/1-3, starts remote right_franka/1-3
through SSH, opens SSH tunnels for right ZMQ and right gripper gRPC, then runs
the dual-arm replay on this host. It intentionally does not start 4_run_env.sh.
The replay resolver accepts the current task/High_Quality|Low_Quality|Failure/index
layout, the old task/index layout, metadata.json, or the exact .pkl.gz file.
EOF
}

log() {
    printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

q() {
    printf '%q' "$1"
}

is_truthy() {
    case "${1,,}" in
        1|true|yes|on) return 0 ;;
        *) return 1 ;;
    esac
}

resolve_conda_base() {
    if command -v conda >/dev/null 2>&1; then
        conda info --base
    elif [[ -x "$HOME/miniconda3/bin/conda" ]]; then
        printf '%s\n' "$HOME/miniconda3"
    elif [[ -x "/home/pnp/miniconda3/bin/conda" ]]; then
        printf '%s\n' "/home/pnp/miniconda3"
    elif [[ -x "$HOME/anaconda3/bin/conda" ]]; then
        printf '%s\n' "$HOME/anaconda3"
    else
        return 1
    fi
}

process_alive() {
    local pid="$1"
    [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

sudo_cmd() {
    if command -v sudo >/dev/null 2>&1; then
        if [[ -n "$LOCAL_SUDO_PASSWORD" ]]; then
            printf '%s\n' "$LOCAL_SUDO_PASSWORD" | sudo -S -p '' "$@"
        else
            sudo "$@"
        fi
    else
        "$@"
    fi
}

setup_ssh_askpass() {
    [[ -z "$SSH_PASSWORD" ]] && return 0
    SSH_ASKPASS_FILE="$LOG_DIR/ssh_askpass.sh"
    printf '#!/usr/bin/env sh\nprintf "%%s\\n" %s\n' "$(q "$SSH_PASSWORD")" > "$SSH_ASKPASS_FILE"
    chmod 700 "$SSH_ASKPASS_FILE"
}

ssh_cmd() {
    local opts=(
        -o ServerAliveInterval=5
        -o ServerAliveCountMax=3
        -o NumberOfPasswordPrompts=1
        -o StrictHostKeyChecking=accept-new
    )
    if [[ -n "$SSH_PASSWORD" ]]; then
        setsid env \
            SSH_ASKPASS="$SSH_ASKPASS_FILE" \
            SSH_ASKPASS_REQUIRE=force \
            DISPLAY=none \
            ssh "${opts[@]}" "$@"
    else
        ssh "${opts[@]}" "$@"
    fi
}

sync_remote_right_scripts() {
    [[ "$SYNC_REMOTE_RIGHT_SCRIPTS" == "0" ]] && return 0
    log "Syncing right_franka, teleop, and Robotiq gripper files to remote repo ..."
    tar -C "$REPO_ROOT" -cf - \
        right_franka \
        polymetis/polymetis/conf/launch_right_gripper.yaml \
        polymetis/polymetis/conf/gripper/robotiq_2f.yaml \
        polymetis/polymetis/python/polymetis/robot_client/robotiq_gripper/robotiq_gripper_client.py \
        teleop/experiments/launch_nodes.py \
        teleop/experiments/run_env.py \
        teleop/teleop/agents/teleop_agent.py \
        teleop/teleop/robots/fr3.py | \
        ssh_cmd "$RIGHT_SSH" "mkdir -p $(q "$RIGHT_REPO") && tar -C $(q "$RIGHT_REPO") -xf - && chmod +x $(q "$RIGHT_REPO/right_franka")/*.sh"
}

remote_bash() {
    local cmd="$1"
    ssh_cmd "$RIGHT_SSH" "bash -lc $(q "$cmd")"
}

tail_log() {
    local file="$1"
    if [[ -f "$file" ]]; then
        log "Last 80 lines from $file:"
        tail -n 80 "$file" || true
    fi
}

tail_remote_log() {
    local file="$1"
    log "Last 80 lines from remote $file:"
    remote_bash "test -f $(q "$file") && tail -n 80 $(q "$file") || true" || true
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

terminate_remote_pid_file() {
    local pid_file="$1"
    local label="$2"
    local cmd
    cmd=$(cat <<EOF
pid_file=$(q "$pid_file")
label=$(q "$label")
if [[ -f "\$pid_file" ]]; then
    pid="\$(cat "\$pid_file" 2>/dev/null || true)"
    if [[ -n "\$pid" ]] && kill -0 "\$pid" 2>/dev/null; then
        echo "Stopping remote \$label process group: \$pid"
        kill -TERM -"\$pid" 2>/dev/null || kill -TERM "\$pid" 2>/dev/null || true
        sleep 1
        kill -KILL -"\$pid" 2>/dev/null || kill -KILL "\$pid" 2>/dev/null || true
    fi
fi
EOF
)
    remote_bash "$cmd" || true
}

kill_port_local() {
    local label="$1"
    local port="$2"
    local pids=()
    if ! command -v lsof >/dev/null 2>&1; then
        log "Skipping local port cleanup for $label; lsof is not installed."
        return 0
    fi
    mapfile -t pids < <(sudo_cmd lsof -t -i:"$port" 2>/dev/null || true)
    ((${#pids[@]} == 0)) && return 0
    log "Cleaning stale local $label on port $port: ${pids[*]}"
    local pid
    for pid in "${pids[@]}"; do
        [[ "$pid" == "$$" || "$pid" == "$BASHPID" ]] && continue
        terminate_pid_tree "$pid" "$label"
    done
}

kill_port_remote() {
    local label="$1"
    local port="$2"
    local cmd
    cmd=$(cat <<EOF
label=$(q "$label")
port=$(q "$port")
remote_sudo_password=$(q "$REMOTE_SUDO_PASSWORD")
if ! command -v lsof >/dev/null 2>&1; then
    echo "Skipping remote port cleanup for \$label; lsof is not installed."
    exit 0
fi
pids="\$(printf '%s\n' "\$remote_sudo_password" | sudo -S -p '' lsof -t -i:"\$port" 2>/dev/null || lsof -t -i:"\$port" 2>/dev/null || true)"
if [[ -n "\$pids" ]]; then
    echo "Cleaning stale remote \$label on port \$port: \$pids"
    kill -TERM \$pids 2>/dev/null || printf '%s\n' "\$remote_sudo_password" | sudo -S -p '' kill -TERM \$pids 2>/dev/null || true
    sleep 1
    kill -KILL \$pids 2>/dev/null || printf '%s\n' "\$remote_sudo_password" | sudo -S -p '' kill -KILL \$pids 2>/dev/null || true
fi
EOF
)
    remote_bash "$cmd" || true
}

kill_matches_local() {
    local label="$1"
    local pattern="$2"
    local pids=()

    mapfile -t pids < <(pgrep -f "$pattern" 2>/dev/null || true)
    ((${#pids[@]} == 0)) && return 0

    log "Cleaning stale local $label process(es): ${pids[*]}"
    local pid
    for pid in "${pids[@]}"; do
        [[ "$pid" == "$$" || "$pid" == "$BASHPID" ]] && continue
        terminate_pid_tree "$pid" "$label"
    done
}

kill_matches_remote() {
    local label="$1"
    local pattern="$2"
    local cmd
    cmd=$(cat <<EOF
label=$(q "$label")
pattern=$(q "$pattern")
pids="\$(pgrep -f "\$pattern" 2>/dev/null || true)"
if [[ -n "\$pids" ]]; then
    echo "Cleaning stale remote \$label process(es): \$pids"
    kill_targets=()
    for pid in \$pids; do
        [[ "\$pid" == "\$\$" || "\$pid" == "\$BASHPID" ]] && continue
        kill_targets+=("\$pid")
    done
    ((\${#kill_targets[@]} == 0)) && exit 0
    kill -TERM "\${kill_targets[@]}" 2>/dev/null || true
    sleep 1
    kill -KILL "\${kill_targets[@]}" 2>/dev/null || true
fi
EOF
)
    remote_bash "$cmd" || true
}

cleanup_stale_replay_conflicts() {
    log "Cleaning stale teleop env processes before replay ..."
    kill_matches_local "left teleop env" '(^|[ /])4_run_env\.sh($| )|experiments/run_env\.py'
    kill_matches_remote "right teleop env" '(^|[ /])4_run_env\.sh($| )|experiments/run_env\.py'
}

cleanup_stale_ports() {
    [[ "${BI_ARM_CLEAN_STALE:-1}" == "0" ]] && return 0
    log "Cleaning stale bi-arm replay ports ..."
    kill_port_local "left robot server" "$LEFT_ROBOT_PORT"
    kill_port_local "left gripper server" "$LEFT_GRIPPER_PORT"
    kill_port_local "left ZMQ node" "$LEFT_ZMQ_PORT"
    kill_port_local "right ZMQ tunnel" "$RIGHT_LOCAL_ZMQ_PORT"
    kill_port_local "right gripper tunnel" "$RIGHT_LOCAL_GRIPPER_PORT"
    kill_port_remote "right robot server" "$RIGHT_ROBOT_PORT"
    kill_port_remote "right gripper server" "$RIGHT_GRIPPER_PORT"
    kill_port_remote "right ZMQ node" "$RIGHT_REMOTE_ZMQ_PORT"
}

cleanup_local_started_processes() {
    local i
    for ((i=${#LOCAL_PIDS[@]}-1; i>=0; i--)); do
        terminate_pid_tree "${LOCAL_PIDS[$i]}" "${LOCAL_NAMES[$i]}"
    done
}

cleanup_remote_started_processes() {
    local i
    for ((i=${#REMOTE_PID_FILES[@]}-1; i>=0; i--)); do
        terminate_remote_pid_file "${REMOTE_PID_FILES[$i]}" "${REMOTE_NAMES[$i]}"
    done
}

cleanup_tunnel() {
    if [[ -n "$TUNNEL_PID" ]]; then
        terminate_pid_tree "$TUNNEL_PID" "right-arm replay SSH tunnels"
        TUNNEL_PID=""
    fi
}

cleanup_all() {
    ((ENABLE_CLEANUP == 0)) && return 0
    ((CLEANING_UP == 1)) && return 0
    CLEANING_UP=1

    log "Cleaning up bi-arm replay processes ..."
    cleanup_tunnel
    cleanup_local_started_processes
    cleanup_remote_started_processes

    if [[ -n "$SUDO_KEEPALIVE_PID" ]]; then
        kill "$SUDO_KEEPALIVE_PID" 2>/dev/null || true
    fi
    if [[ -n "$SSH_ASKPASS_FILE" ]]; then
        rm -f "$SSH_ASKPASS_FILE"
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

abort_remote() {
    local label="$1"
    local logfile="${2:-}"
    log "ERROR: $label"
    [[ -n "$logfile" ]] && tail_remote_log "$logfile"
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
        log "Checking local sudo access."
        sudo_cmd -v
        (
            while true; do
                sudo_cmd -v >/dev/null 2>&1 || exit 0
                sleep 60
            done
        ) &
        SUDO_KEEPALIVE_PID="$!"
    fi
}

prepare_remote() {
    log "Checking remote SSH and repo: $RIGHT_SSH:$RIGHT_REPO"
    remote_bash "test -d $(q "$RIGHT_REPO/right_franka") && test -d $(q "$RIGHT_REPO/teleop") && test -d $(q "$RIGHT_REPO/polymetis")"
    if [[ "${BI_ARM_REMOTE_SUDO_VALIDATE:-1}" == "1" ]]; then
        log "Checking remote sudo access."
        remote_bash "printf '%s\n' $(q "$REMOTE_SUDO_PASSWORD") | sudo -S -p '' -v"
    fi
}

conda_python() {
    local env_name="$1"
    local code="$2"
    timeout 15 bash -lc '
        set -e
        if command -v conda >/dev/null 2>&1; then
            CONDA_BASE="$(conda info --base)"
        elif [[ -x "$HOME/miniconda3/bin/conda" ]]; then
            CONDA_BASE="$HOME/miniconda3"
        elif [[ -x "/home/pnp/miniconda3/bin/conda" ]]; then
            CONDA_BASE="/home/pnp/miniconda3"
        elif [[ -x "$HOME/anaconda3/bin/conda" ]]; then
            CONDA_BASE="$HOME/anaconda3"
        else
            echo "ERROR: conda not found" >&2
            exit 1
        fi
        source "$CONDA_BASE/etc/profile.d/conda.sh"
        conda activate "$1"
        python -c "$2"
    ' bash "$env_name" "$code"
}

remote_conda_python() {
    local env_name="$1"
    local code="$2"
    local cmd
    cmd=$(cat <<EOF
set -e
if command -v conda >/dev/null 2>&1; then
    CONDA_BASE="\$(conda info --base)"
elif [[ -x "\$HOME/miniconda3/bin/conda" ]]; then
    CONDA_BASE="\$HOME/miniconda3"
elif [[ -x "/home/pnp/miniconda3/bin/conda" ]]; then
    CONDA_BASE="/home/pnp/miniconda3"
elif [[ -x "\$HOME/anaconda3/bin/conda" ]]; then
    CONDA_BASE="\$HOME/anaconda3"
else
    echo "ERROR: conda not found" >&2
    exit 1
fi
source "\$CONDA_BASE/etc/profile.d/conda.sh"
conda activate $(q "$env_name")
python -c $(q "$code")
EOF
)
    remote_bash "$cmd"
}

check_robot_grpc_ready_local() {
    local port="$1"
    conda_python polymetis "from polymetis import RobotInterface; robot = RobotInterface(ip_address='127.0.0.1', port=$port, enforce_version=False); robot.get_joint_positions()" >/dev/null 2>&1
}

check_robot_grpc_ready_remote() {
    local port="$1"
    remote_conda_python polymetis "from polymetis import RobotInterface; robot = RobotInterface(ip_address='127.0.0.1', port=$port, enforce_version=False); robot.get_joint_positions()" >/dev/null 2>&1
}

check_gripper_grpc_ready_local() {
    local port="$1"
    conda_python polymetis "import grpc, polymetis_pb2, polymetis_pb2_grpc; channel = grpc.insecure_channel('127.0.0.1:$port'); grpc.channel_ready_future(channel).result(timeout=2); stub = polymetis_pb2_grpc.GripperServerStub(channel); stub.GetRobotClientMetadata(polymetis_pb2.Empty(), timeout=2)" >/dev/null 2>&1
}

check_gripper_grpc_ready_remote() {
    local port="$1"
    remote_conda_python polymetis "import grpc, polymetis_pb2, polymetis_pb2_grpc; channel = grpc.insecure_channel('127.0.0.1:$port'); grpc.channel_ready_future(channel).result(timeout=2); stub = polymetis_pb2_grpc.GripperServerStub(channel); stub.GetRobotClientMetadata(polymetis_pb2.Empty(), timeout=2)" >/dev/null 2>&1
}

check_robot_zmq_ready_local() {
    local port="$1"
    local code
    code=$(cat <<PY
import pickle
import zmq
ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 2000)
sock.setsockopt(zmq.SNDTIMEO, 2000)
sock.setsockopt(zmq.LINGER, 0)
sock.connect("tcp://127.0.0.1:$port")
for method in ("num_dofs", "get_observations"):
    sock.send(pickle.dumps({"method": method, "args": {}}))
    result = pickle.loads(sock.recv())
    if isinstance(result, dict) and "error" in result:
        raise RuntimeError(result["error"])
    if method == "num_dofs" and int(result) != 8:
        raise RuntimeError(f"bad num_dofs: {result}")
    if method == "get_observations":
        missing = [key for key in ("ee_pose_euler",) if key not in result]
        if missing:
            raise RuntimeError(f"robot node is missing observation fields: {missing}")
        missing_gripper = [key for key in ("gripper_closedness", "gripper_01closedness", "gripper_target_width", "gripper_width") if key not in result]
        if missing_gripper:
            raise RuntimeError(f"robot node is missing continuous gripper observation fields: {missing_gripper}")
sock.close(0)
ctx.term()
PY
)
    conda_python polymetis "$code" >/dev/null 2>&1
}

check_robot_zmq_ready_remote() {
    local port="$1"
    local code
    code=$(cat <<PY
import pickle
import zmq
ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 2000)
sock.setsockopt(zmq.SNDTIMEO, 2000)
sock.setsockopt(zmq.LINGER, 0)
sock.connect("tcp://127.0.0.1:$port")
for method in ("num_dofs", "get_observations"):
    sock.send(pickle.dumps({"method": method, "args": {}}))
    result = pickle.loads(sock.recv())
    if isinstance(result, dict) and "error" in result:
        raise RuntimeError(result["error"])
    if method == "num_dofs" and int(result) != 8:
        raise RuntimeError(f"bad num_dofs: {result}")
    if method == "get_observations":
        missing = [key for key in ("ee_pose_euler",) if key not in result]
        if missing:
            raise RuntimeError(f"robot node is missing observation fields: {missing}")
        missing_gripper = [key for key in ("gripper_closedness", "gripper_01closedness", "gripper_target_width", "gripper_width") if key not in result]
        if missing_gripper:
            raise RuntimeError(f"robot node is missing continuous gripper observation fields: {missing_gripper}")
sock.close(0)
ctx.term()
PY
)
    remote_conda_python polymetis "$code" >/dev/null 2>&1
}

check_right_tunnels_ready() {
    check_robot_zmq_ready_local "$RIGHT_LOCAL_ZMQ_PORT" && \
        check_gripper_grpc_ready_local "$RIGHT_LOCAL_GRIPPER_PORT"
}

remote_process_alive() {
    local pid_file="$1"
    remote_bash "pid=\"\$(cat $(q "$pid_file") 2>/dev/null || true)\"; [[ -n \"\$pid\" ]] && kill -0 \"\$pid\" 2>/dev/null" >/dev/null 2>&1
}

wait_until_ready() {
    local label="$1"
    local pid="$2"
    local logfile="$3"
    local check_fn="$4"
    shift 4
    local started
    started="$(date +%s)"

    while true; do
        if ! process_alive "$pid"; then
            abort "$label exited before becoming ready." "$logfile"
        fi
        if "$check_fn" "$@"; then
            log "$label is ready."
            return 0
        fi
        if (( $(date +%s) - started >= READY_TIMEOUT )); then
            abort "$label did not become ready within ${READY_TIMEOUT}s." "$logfile"
        fi
        sleep 1
    done
}

wait_until_remote_ready() {
    local label="$1"
    local pid_file="$2"
    local logfile="$3"
    local check_fn="$4"
    shift 4
    local started
    started="$(date +%s)"

    while true; do
        if ! remote_process_alive "$pid_file"; then
            abort_remote "$label exited before becoming ready." "$logfile"
        fi
        if "$check_fn" "$@"; then
            log "$label is ready."
            return 0
        fi
        if (( $(date +%s) - started >= READY_TIMEOUT )); then
            abort_remote "$label did not become ready within ${READY_TIMEOUT}s." "$logfile"
        fi
        sleep 1
    done
}

start_local_script() {
    local label="$1"
    local script="$2"
    local check_fn="$3"
    shift 3
    local safe_label="${label// /_}"
    local logfile="$LOG_DIR/${safe_label}.log"

    log "Starting $label ..."
    (
        cd "$REPO_ROOT"
        export PYTHONUNBUFFERED=1
        export FRANKA_SUDO_PASSWORD="$LOCAL_SUDO_PASSWORD"
        export FRANKA_MOVE_TO_INITIAL_POSE="$MOVE_TO_INITIAL_POSE"
        export LEFT_ROBOTIQ_COMPORT="$LEFT_ROBOTIQ_COMPORT_OVERRIDE"
        export LEFT_GRIPPER_SERVER_PORT="$LEFT_GRIPPER_PORT"
        exec bash "$script"
    ) >"$logfile" 2>&1 &

    local pid="$!"
    LOCAL_PIDS+=("$pid")
    LOCAL_NAMES+=("$label")
    wait_until_ready "$label" "$pid" "$logfile" "$check_fn" "$@"
}

start_remote_script() {
    local label="$1"
    local script_name="$2"
    local check_fn="$3"
    shift 3
    local safe_label="${label// /_}"
    local logfile="$REMOTE_LOG_DIR/${safe_label}.log"
    local pid_file="$REMOTE_LOG_DIR/${safe_label}.pid"
    local cmd

    log "Starting remote $label ..."
    cmd=$(cat <<EOF
mkdir -p $(q "$REMOTE_LOG_DIR")
cd $(q "$RIGHT_REPO/right_franka")
export PYTHONUNBUFFERED=1
export FRANKA_SUDO_PASSWORD=$(q "$REMOTE_SUDO_PASSWORD")
export FRANKA_MOVE_TO_INITIAL_POSE=$(q "$MOVE_TO_INITIAL_POSE")
export RIGHT_ROBOTIQ_COMPORT=$(q "$RIGHT_ROBOTIQ_COMPORT_OVERRIDE")
export RIGHT_GRIPPER_SERVER_PORT=$(q "$RIGHT_GRIPPER_PORT")
nohup setsid bash $(q "$script_name") > $(q "$logfile") 2>&1 < /dev/null &
echo \$! > $(q "$pid_file")
EOF
)
    remote_bash "$cmd"
    REMOTE_PID_FILES+=("$pid_file")
    REMOTE_NAMES+=("$label")
    wait_until_remote_ready "$label" "$pid_file" "$logfile" "$check_fn" "$@"
}

start_tunnels() {
    TUNNEL_LOG="$LOG_DIR/right_replay_tunnels.log"
    log "Starting SSH tunnels:"
    log "  ZMQ:     127.0.0.1:$RIGHT_LOCAL_ZMQ_PORT -> $RIGHT_SSH:127.0.0.1:$RIGHT_REMOTE_ZMQ_PORT"
    log "  Gripper: 127.0.0.1:$RIGHT_LOCAL_GRIPPER_PORT -> $RIGHT_SSH:127.0.0.1:$RIGHT_GRIPPER_PORT"
    ssh_cmd -N \
        -o ExitOnForwardFailure=yes \
        -L "127.0.0.1:$RIGHT_LOCAL_ZMQ_PORT:127.0.0.1:$RIGHT_REMOTE_ZMQ_PORT" \
        -L "127.0.0.1:$RIGHT_LOCAL_GRIPPER_PORT:127.0.0.1:$RIGHT_GRIPPER_PORT" \
        "$RIGHT_SSH" >"$TUNNEL_LOG" 2>&1 &
    TUNNEL_PID="$!"
    wait_until_ready "right ZMQ/gripper SSH tunnels" "$TUNNEL_PID" "$TUNNEL_LOG" check_right_tunnels_ready
}

run_replay() {
    local episode="$1"
    shift

    local approach_args=()
    if is_truthy "$DEFAULT_APPROACH_START"; then
        approach_args+=(
            --approach-start
            --approach-start-max-delta "$DEFAULT_APPROACH_START_MAX_DELTA"
            --approach-start-step-delta "$DEFAULT_APPROACH_START_STEP_DELTA"
            --approach-start-hz "$DEFAULT_APPROACH_START_HZ"
        )
    fi

    log "Starting dual-arm replay in foreground."
    log "Default dry-run: add --execute to command both arms."

    set +e
    (
        cd "$REPO_ROOT"
        export PYTHONUNBUFFERED=1
        if ! CONDA_BASE="$(resolve_conda_base)"; then
            echo "ERROR: conda not found" >&2
            exit 1
        fi
        set +u
        source "$CONDA_BASE/etc/profile.d/conda.sh"
        conda activate polymetis
        set -u
        python -m franka_replay.replay_fr3_dual \
            --speed "$DEFAULT_REPLAY_SPEED" \
            --gripper-speed "$DEFAULT_GRIPPER_SPEED" \
            --gripper-force "$DEFAULT_GRIPPER_FORCE" \
            --gripper-event-delta "$DEFAULT_GRIPPER_EVENT_DELTA" \
            --gripper-replay-mode "$DEFAULT_GRIPPER_REPLAY_MODE" \
            --gripper-command-hz "$DEFAULT_GRIPPER_COMMAND_HZ" \
            --gripper-hold-sec "$DEFAULT_GRIPPER_HOLD_SEC" \
            --left-host 127.0.0.1 \
            --left-port "$LEFT_ZMQ_PORT" \
            --right-host 127.0.0.1 \
            --right-port "$RIGHT_LOCAL_ZMQ_PORT" \
            --left-gripper-host 127.0.0.1 \
            --left-gripper-port "$LEFT_GRIPPER_PORT" \
            --right-gripper-host 127.0.0.1 \
            --right-gripper-port "$RIGHT_LOCAL_GRIPPER_PORT" \
            "${approach_args[@]}" \
            "$episode" \
            "$@"
    )
    local rc="$?"
    set -e

    if ((rc != 0)); then
        abort "dual-arm replay exited with code $rc."
    fi
    log "dual-arm replay exited normally."
}

preflight_dual_episode() {
    local episode="$1"
    shift

    log "Preflight checking dual-arm episode format before starting hardware stack."
    set +e
    (
        cd "$REPO_ROOT"
        export PYTHONUNBUFFERED=1
        if ! CONDA_BASE="$(resolve_conda_base)"; then
            echo "ERROR: conda not found" >&2
            exit 1
        fi
        set +u
        source "$CONDA_BASE/etc/profile.d/conda.sh"
        conda activate polymetis
        set -u
        python - "$episode" "$@" <<'PY'
import argparse

from franka_replay.replay_fr3 import (
    infer_episode_kind,
    load_episode,
    load_episode_metadata,
    resolve_episode_file,
)

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("episode")
parser.add_argument("--latest", action="store_true")
args, _ = parser.parse_known_args()

episode_path = resolve_episode_file(args.episode, latest=args.latest)
_, frames = load_episode(episode_path)
metadata = load_episode_metadata(episode_path)
kind = infer_episode_kind(frames, metadata)
if kind != "dual":
    raise SystemExit(
        f"Episode appears to be {kind}; use 7_replay_fr3.sh for single-arm replay."
    )
first_frame = frames[0] if isinstance(frames[0], dict) else {}
schema = metadata.get("schema_version") or first_frame.get("schema_version", "<missing>")
print(f"Preflight OK: {episode_path} schema={schema} frames={len(frames)}")
PY
    )
    local rc="$?"
    set -e

    if ((rc != 0)); then
        abort "dual-arm episode preflight failed with code $rc."
    fi
}

main() {
    require_args "$@"
    ENABLE_CLEANUP=1
    mkdir -p "$LOG_DIR"

    local episode="$1"
    local extra_args=()
    if [[ "$#" -gt 1 ]]; then
        extra_args=("${@:2}")
    fi

    local skip_robot_check=0
    local execute_replay=0
    local arg
    for arg in "${extra_args[@]}"; do
        case "$arg" in
            --skip-robot-check) skip_robot_check=1 ;;
            --execute) execute_replay=1 ;;
        esac
    done
    if ((skip_robot_check == 1 && execute_replay == 1)); then
        log "ERROR: --skip-robot-check cannot be used with --execute"
        exit 1
    fi

    log "Repo: $REPO_ROOT"
    log "Local logs: $LOG_DIR"
    log "Remote: $RIGHT_SSH"
    log "Remote repo: $RIGHT_REPO"
    log "Remote logs: $REMOTE_LOG_DIR"
    log "Episode: $episode"
    log "Move to initial joint pose before replay checks: $MOVE_TO_INITIAL_POSE"
    log "Replay speed: $DEFAULT_REPLAY_SPEED"
    log "Gripper speed/force: $DEFAULT_GRIPPER_SPEED / $DEFAULT_GRIPPER_FORCE"
    log "Gripper replay mode/Hz: $DEFAULT_GRIPPER_REPLAY_MODE / $DEFAULT_GRIPPER_COMMAND_HZ"
    log "Right local gripper tunnel port: $RIGHT_LOCAL_GRIPPER_PORT"

    setup_ssh_askpass
    preflight_dual_episode "$episode" "${extra_args[@]}"
    if ((skip_robot_check == 1)); then
        log "Robot check skipped. No command was sent."
        exit 0
    fi
    prepare_sudo
    prepare_remote
    sync_remote_right_scripts
    cleanup_stale_ports
    cleanup_stale_replay_conflicts

    start_local_script "left 1_launch_robot" "$REPO_ROOT/left_franka/1_launch_robot.sh" check_robot_grpc_ready_local "$LEFT_ROBOT_PORT"
    start_remote_script "right 1_launch_robot" "1_launch_robot.sh" check_robot_grpc_ready_remote "$RIGHT_ROBOT_PORT"

    start_local_script "left 2_launch_gripper" "$REPO_ROOT/left_franka/2_launch_gripper.sh" check_gripper_grpc_ready_local "$LEFT_GRIPPER_PORT"
    start_remote_script "right 2_launch_gripper" "2_launch_gripper.sh" check_gripper_grpc_ready_remote "$RIGHT_GRIPPER_PORT"

    start_local_script "left 3_launch_node" "$REPO_ROOT/left_franka/3_launch_node.sh" check_robot_zmq_ready_local "$LEFT_ZMQ_PORT"
    start_remote_script "right 3_launch_node" "3_launch_node.sh" check_robot_zmq_ready_remote "$RIGHT_REMOTE_ZMQ_PORT"

    start_tunnels
    run_replay "$episode" "${extra_args[@]}"
}

main "$@"
