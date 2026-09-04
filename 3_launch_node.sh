#!/bin/bash
set -e

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "Failed to activate polymetis"; exit 1; }

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
POLYMETIS_ROOT="$REPO_ROOT/polymetis"
POLYMETIS_TORCH_ISOLATION="$POLYMETIS_ROOT/polymetis/build/torch_isolation"
POLYMETIS_PINOCCHIO_ISOLATION="$POLYMETIS_TORCH_ISOLATION/pinocchio_isolation"
POLYMETIS_TORCH_LIB="$(python -c 'from pathlib import Path; import torch; print(Path(torch.__file__).resolve().parent / "lib")')"

for required_dir in \
    "$POLYMETIS_TORCH_ISOLATION" \
    "$POLYMETIS_PINOCCHIO_ISOLATION" \
    "$POLYMETIS_TORCH_LIB"; do
    if [[ ! -d "$required_dir" ]]; then
        echo "Missing Polymetis runtime library directory: $required_dir" >&2
        exit 1
    fi
done

export LD_LIBRARY_PATH="$POLYMETIS_PINOCCHIO_ISOLATION:$POLYMETIS_TORCH_ISOLATION:$POLYMETIS_TORCH_LIB:$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

TELEOP_ROOT="$REPO_ROOT/teleop"

SCRIPT_PATH="$TELEOP_ROOT/experiments/launch_nodes.py"
python3 "$SCRIPT_PATH" --robot=fr3 --robot_ip=127.0.0.1 "$@"
