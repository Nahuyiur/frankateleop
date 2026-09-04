#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_SH="${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"

if [[ ! -f "$CONDA_SH" ]]; then
    echo "ERROR: conda activation script not found: $CONDA_SH" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$CONDA_SH"
conda activate polymetis
export PYTHONPATH="$REPO_ROOT/teleop:$REPO_ROOT/teleop/third_party/DynamixelSDK/python/src${PYTHONPATH:+:$PYTHONPATH}"

exec python "$REPO_ROOT/scripts/dynamixel_damping_tuner.py" "$@"
