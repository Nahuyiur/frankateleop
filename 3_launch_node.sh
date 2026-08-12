#!/bin/bash
set -e

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "Failed to activate polymetis"; exit 1; }

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
POLY_ROOT="$REPO_ROOT/teleop"

SCRIPT_PATH="$POLY_ROOT/experiments/launch_nodes.py"
python3 "$SCRIPT_PATH" --robot=fr3 --robot_ip=127.0.0.1 "$@"
