#!/usr/bin/env bash
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"

source_conda() {
    local conda_base="${CONDA_BASE:-$HOME/miniconda3}"
    if [[ ! -f "$conda_base/etc/profile.d/conda.sh" ]]; then
        echo "ERROR: Cannot find conda.sh under $conda_base" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    source "$conda_base/etc/profile.d/conda.sh"
}

echo ">>> 激活 conda 环境 franka_capture ..."
source_conda
conda activate franka_capture || {
    echo "ERROR: failed to activate franka_capture" >&2
    exit 1
}

cd "$REPO_ROOT"
python -m validate.validate_task "$@"
