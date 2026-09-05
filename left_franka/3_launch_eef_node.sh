#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE_PORT="${LEFT_TELEOP_NODE_PORT:-6002}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<'EOF'
Usage: bash left_franka/3_launch_eef_node.sh [extra launch_nodes.py args]

Starts the existing left robot node in Cartesian impedance mode.  It does not
move home, move to calibrated joints, or open the gripper during node startup;
the EEF adapter anchors to the robot's current pose.
EOF
    exit 0
fi

if ss -ltn "sport = :$NODE_PORT" 2>/dev/null | grep -q LISTEN; then
    echo "error: robot-node port $NODE_PORT is already in use." >&2
    echo "Stop the existing joint-mode 3_launch_node.sh before starting EE mode." >&2
    exit 1
fi

# left_franka/3_launch_node.sh historically uses LEFT_TELEOP_PORT for the ZMQ
# node port, while script 4 uses that name for the serial device.  Scope the
# numeric value to this child process and expose a clearer EEF-specific name.
export LEFT_TELEOP_PORT="$NODE_PORT"
# Do not start the Cartesian policy while Pico is still preview-only.  The
# first anchored EEF command starts it and immediately seeds the desired pose.
export FRANKA_LAZY_EE_START="${FRANKA_LAZY_EE_START:-1}"

exec bash "$SCRIPT_DIR/3_launch_node.sh" \
    --control-mode ee \
    --no-home-on-init \
    --no-open-gripper-on-init \
    --no-move-to-initial-pose \
    "$@"
