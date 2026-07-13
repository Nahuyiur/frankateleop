#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONDA_ENV="${FRANKA_CAPTURE_CONDA_ENV:-franka_capture}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
Usage:
  bash DAgger/build_jenga_mapping_model.sh [jenga_mapping_model.py options]

Default output:
  ~/Desktop/Muka_NAS/stack_jenga/DAgger/mapping_model.json

Recommended quick validation build:
  bash DAgger/build_jenga_mapping_model.sh \\
    --model-type affine \\
    --train-demos 10 \\
    --validation-demos 5 \\
    --random-seed 7

Useful debug build:
  bash DAgger/build_jenga_mapping_model.sh \\
    --model-type affine \\
    --train-demos 10 \\
    --validation-demos 5 \\
    --random-seed 7 \\
    --debug-dir ~/Desktop/Muka_NAS/stack_jenga/DAgger/mapping_model_debug

Optional full split:
  bash DAgger/build_jenga_mapping_model.sh --max-demos 100 --validation-fraction 0.2
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
exec python -m DAgger.jenga_mapping_model "$@"
