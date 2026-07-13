#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ "$#" -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
Usage:
  bash DAgger/replay_jenga_cartesian.sh <retargeted.pkl.gz> [options]

Dry-run example:
  bash DAgger/replay_jenga_cartesian.sh ~/Desktop/Muka_NAS/stack_jenga/DAgger/live_retargeted.pkl.gz \\
    --host 127.0.0.1 --port 6002 \\
    --gripper-host 127.0.0.1 --gripper-port 50054

Execute only after dry-run passes:
  bash DAgger/replay_jenga_cartesian.sh <retargeted.pkl.gz> --execute \\
    --host 127.0.0.1 --port 6002 \\
    --gripper-host 127.0.0.1 --gripper-port 50054

Important:
  This sends pose commands through command_ee_pose. The robot node must be
  launched with --control-mode ee on the same port.
EOF
    exit 0
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
conda activate polymetis

cd "$REPO_ROOT"
exec python -m DAgger.jenga_cartesian_replay "$@"
