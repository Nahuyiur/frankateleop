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
                echo "error: NAS is not mounted at $MUKA_NAS_ROOT. Mount Muka_NAS or pass an explicit episode_dir." >&2
                exit 1
            fi
            ;;
    esac
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<'EOF'
Usage:
  bash 19_view_recorded_rgb_pointclouds.sh [episode_dir] [options]

Purpose:
  Export recorded RGB, depth preview, and per-camera PLY point clouds from one
  episode frame. This does not require multi-camera extrinsics; each PLY is in
  that camera's own color optical frame.

Examples:
  bash 19_view_recorded_rgb_pointclouds.sh
  bash 19_view_recorded_rgb_pointclouds.sh /home/pnp/Desktop/Muka_NAS/test_depth/0
  bash 19_view_recorded_rgb_pointclouds.sh /home/pnp/Desktop/Muka_NAS/test_depth/0 --frame middle --open
  bash 19_view_recorded_rgb_pointclouds.sh /home/pnp/Desktop/Muka_NAS/test_depth/0 --pointcloud-stride 1

Default:
  With no episode_dir, the latest episode under
  /home/pnp/Desktop/Muka_NAS/rgb_pointcloud is used.

Outputs:
  <episode_dir>/rgb_pointcloud_view/frame_XXXXXX/
    all_cameras_rgb.png
    summary.json
    <camera>_rgb.png
    <camera>_depth.png
    <camera>_cloud.ply

Options are passed through to:
  python -m pointcloud.inspect_rgb_pointcloud_episode --help
EOF
    exit 0
fi

echo ">>> Activating conda env: $CONDA_ENV"
source_conda
conda activate "$CONDA_ENV" || {
    echo "error: failed to activate conda env $CONDA_ENV" >&2
    exit 1
}

if [[ "$#" -lt 1 ]]; then
    require_muka_nas_mounted "$DEFAULT_OUTPUT_ROOT"
    echo ">>> No episode_dir supplied; using latest under $DEFAULT_OUTPUT_ROOT/$DEFAULT_TASK"
fi

cd "$REPO_ROOT"
python -m pointcloud.inspect_rgb_pointcloud_episode \
    --output-root "$DEFAULT_OUTPUT_ROOT" \
    --task "$DEFAULT_TASK" \
    "$@"
