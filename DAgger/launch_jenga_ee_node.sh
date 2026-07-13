#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROBOT_NAME="${JENGA_EE_ROBOT_NAME:-fr3_left}"
TELE_PORT="${JENGA_EE_TELE_PORT:-6002}"
ROBOT_IP="${JENGA_EE_ROBOT_IP:-127.0.0.1}"
ROBOT_PORT="${JENGA_EE_ROBOT_PORT:-50052}"
GRIPPER_PORT="${JENGA_EE_GRIPPER_PORT:-50054}"
CONTROL_MODE="${JENGA_EE_CONTROL_MODE:-ee}"
CONDA_ENV="${JENGA_EE_CONDA_ENV:-polymetis}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
Usage:
  bash DAgger/launch_jenga_ee_node.sh [extra launch_nodes.py args]

Default launch:
  robot=$ROBOT_NAME
  tele_port=$TELE_PORT
  robot_ip=$ROBOT_IP
  robot_port=$ROBOT_PORT
  gripper_port=$GRIPPER_PORT
  control_mode=$CONTROL_MODE

This starts the robot ZMQ node in Cartesian impedance mode for
DAgger/replay_jenga_cartesian.sh. It refuses to start if tele_port is already
listening; stop the existing joint-mode node first.

Override defaults with environment variables:
  JENGA_EE_TELE_PORT=6002
  JENGA_EE_ROBOT_PORT=50052
  JENGA_EE_GRIPPER_PORT=50054
EOF
    exit 0
fi

if ss -ltn "sport = :$TELE_PORT" | grep -q LISTEN; then
    echo "error: tcp port $TELE_PORT is already listening." >&2
    echo "Current listener:" >&2
    ss -ltnp "sport = :$TELE_PORT" >&2 || true
    echo "Stop the existing robot node, then rerun this script." >&2
    exit 1
fi

if command -v conda >/dev/null 2>&1; then
    CONDA_BASE="$(conda info --base)"
elif [[ -x "$HOME/miniconda3/bin/conda" ]]; then
    CONDA_BASE="$HOME/miniconda3"
elif [[ -x "/home/pnp/miniconda3/bin/conda" ]]; then
    CONDA_BASE="/home/pnp/miniconda3"
elif [[ -x "$HOME/anaconda3/bin/conda" ]]; then
    CONDA_BASE="$HOME/anaconda3"
else
    echo "error: conda not found" >&2
    exit 1
fi

# shellcheck source=/dev/null
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$CONDA_ENV"

SCRIPT_PATH="$REPO_ROOT/teleop/experiments/launch_nodes.py"
if [[ ! -f "$SCRIPT_PATH" ]]; then
    echo "error: missing $SCRIPT_PATH" >&2
    exit 1
fi

echo "Starting Jenga Cartesian robot node:"
echo "  robot=$ROBOT_NAME"
echo "  tele_port=$TELE_PORT"
echo "  robot_ip=$ROBOT_IP"
echo "  robot_port=$ROBOT_PORT"
echo "  gripper_port=$GRIPPER_PORT"
echo "  control_mode=$CONTROL_MODE"

exec python3 "$SCRIPT_PATH" \
    --robot="$ROBOT_NAME" \
    --tele_port="$TELE_PORT" \
    --robot_ip="$ROBOT_IP" \
    --robot_port="$ROBOT_PORT" \
    --gripper_port="$GRIPPER_PORT" \
    --control_mode="$CONTROL_MODE" \
    "$@"
