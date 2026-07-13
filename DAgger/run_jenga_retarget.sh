#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONDA_ENV="${FRANKA_CAPTURE_CONDA_ENV:-franka_capture}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
Usage:
  bash DAgger/run_jenga_retarget.sh [jenga_retarget.py options]

Live current-scene example:
  bash DAgger/run_jenga_retarget.sh \\
    --demo ~/Desktop/Muka_NAS/stack_jenga/High_Quality/0/0.pkl.gz \\
    --output ~/Desktop/Muka_NAS/stack_jenga/DAgger/live_retargeted.pkl.gz \\
    --debug-dir ~/Desktop/Muka_NAS/stack_jenga/DAgger/live_debug

Offline current-episode example:
  bash DAgger/run_jenga_retarget.sh \\
    --demo ~/Desktop/Muka_NAS/stack_jenga/High_Quality/0/0.pkl.gz \\
    --current-episode ~/Desktop/Muka_NAS/stack_jenga/High_Quality/0/0.pkl.gz \\
    --output ~/Desktop/Muka_NAS/stack_jenga/DAgger/retargeted_0.pkl.gz \\
    --debug-dir ~/Desktop/Muka_NAS/stack_jenga/DAgger/debug_0
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
conda activate "$CONDA_ENV"

cd "$REPO_ROOT"
exec python -m DAgger.jenga_retarget "$@"
