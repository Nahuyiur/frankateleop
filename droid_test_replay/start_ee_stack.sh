#!/usr/bin/env bash
set -euo pipefail

REPO="/home/muka/frankateleop"
LOG_DIR="${REPO}/droid_test_replay/logs"
mkdir -p "${LOG_DIR}"
export PATH="${HOME}/miniconda3/bin:${PATH}"

# Acquire the sudo ticket while this script still owns an interactive stdin.
# Polymetis needs it for realtime scheduling; background launch cannot prompt.
sudo -v

port_open() {
    timeout 1 bash -c "</dev/tcp/127.0.0.1/$1" >/dev/null 2>&1
}

start_and_wait() {
    local name="$1" port="$2"
    shift 2
    if port_open "${port}"; then
        echo "${name}: port ${port} already open"
        return
    fi
    nohup "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
    local pid=$!
    for _ in $(seq 1 60); do
        if port_open "${port}"; then
            echo "${name}: ready (pid=${pid}, port=${port})"
            return
        fi
        if ! kill -0 "${pid}" 2>/dev/null; then
            tail -50 "${LOG_DIR}/${name}.log"
            return 1
        fi
        sleep 1
    done
    echo "${name}: timed out; see ${LOG_DIR}/${name}.log" >&2
    return 1
}

start_and_wait robot 50051 "${REPO}/1_launch_robot.sh"
start_and_wait gripper 50052 "${REPO}/2_launch_gripper.sh"
start_and_wait ee_node 6001 "${REPO}/3_launch_node.sh" \
    --control-mode ee --no-home-on-init --no-open-gripper-on-init \
    --no-move-to-initial-pose

(
    source "${HOME}/miniconda3/etc/profile.d/conda.sh"
    conda activate polymetis
    cd "${REPO}"
    python droid_test_replay/ensure_ee_controller.py
)

echo "EE stack ready. Run a dry-run before --execute."
