#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_ENV="${FRANKA_CAPTURE_CONDA_ENV:-franka_capture}"
MUKA_NAS_ROOT="$HOME/Desktop/Muka_NAS"
DEFAULT_OUTPUT_ROOT="$MUKA_NAS_ROOT"
DEFAULT_TASK="rgb_pointcloud"

source_conda() {
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
}

require_muka_nas_mounted() {
    local output_root="$1"
    case "$output_root" in
        "$MUKA_NAS_ROOT"|"$MUKA_NAS_ROOT"/*)
            if ! mountpoint -q "$MUKA_NAS_ROOT"; then
                echo "error: NAS is not mounted at $MUKA_NAS_ROOT. Mount Muka_NAS before recording." >&2
                exit 1
            fi
            ;;
    esac
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
Usage:
  bash 18_record_rgb_pointclouds.sh [options]

Purpose:
  Record RealSense RGB-D data only. This does not start robot, gripper, teleop,
  or robot-node processes. Saved episodes are compatible with script 19.

Defaults:
  output_root=$DEFAULT_OUTPUT_ROOT
  task=$DEFAULT_TASK
  fps=30
  camera_names=all configured cameras
  depth_cameras=all recorded cameras

Examples:
  bash 18_record_rgb_pointclouds.sh
  bash 18_record_rgb_pointclouds.sh --camera-names middle,left_wrist
  bash 18_record_rgb_pointclouds.sh --depth-cameras middle
  bash 18_record_rgb_pointclouds.sh --camera-fps 15
  bash 18_record_rgb_pointclouds.sh --camera-names right --depth-cameras right
  bash 18_record_rgb_pointclouds.sh --auto-start --duration-sec 5

Recording keys:
  s=start/resume, w=pause, e=save current episode, d=discard current episode,
  k=keyframe, q=save and quit

After recording:
  bash 19_view_recorded_rgb_pointclouds.sh
  bash 19_view_recorded_rgb_pointclouds.sh $DEFAULT_OUTPUT_ROOT/$DEFAULT_TASK/0 --frame middle --open
EOF
    exit 0
fi

echo ">>> Activating conda env: $CONDA_ENV"
source_conda
conda activate "$CONDA_ENV" || {
    echo "error: failed to activate conda env $CONDA_ENV" >&2
    exit 1
}

require_muka_nas_mounted "$DEFAULT_OUTPUT_ROOT"
mkdir -p "$DEFAULT_OUTPUT_ROOT"

echo ">>> Camera-only RGB-D recording"
echo ">>> Default output root: $DEFAULT_OUTPUT_ROOT"
echo ">>> Default task: $DEFAULT_TASK"
echo ">>> Compatible viewer: bash 19_view_recorded_rgb_pointclouds.sh"

cd "$REPO_ROOT"
python -m pointcloud.record_rgb_pointclouds \
    --output-root "$DEFAULT_OUTPUT_ROOT" \
    --task "$DEFAULT_TASK" \
    "$@"
