#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_ENV="${FRANKA_CAPTURE_CONDA_ENV:-franka_capture}"
DEFAULT_OUTPUT_ROOT="$HOME/Desktop/franka_record_data/calibration"
DEFAULT_BOARD="$DEFAULT_OUTPUT_ROOT/targets/charuco_7x5_35mm_26mm.png"

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

usage() {
    cat <<EOF
Usage:
  bash 20_pointcloud_calibration.sh <command> [options]

Commands:
  workflow
      Print the recommended real-hardware calibration workflow.

  doctor | preflight [session_dir]
      Check environment, or check a saved calibration session if session_dir is passed.

  latest
      List recent calibration sessions.

  self-test
      Run py_compile, synthetic hand-eye, and synthetic fusion tests. No hardware required.

  board | print-board [options]
      Generate a printable ChArUco board.
      Default output: $DEFAULT_BOARD

  capture [options]
      Start manual ChArUco sample capture. Reads robot pose, does not move robot.
      Common: --camera middle

  solve <session_dir> [options]
      Solve eye-to-hand calibration, write calibration_result.json and validation_report.json,
      then run doctor on the session.

  validate | check <session_dir>
      Re-run validation and doctor on an existing session.

  apply <episode_dir> --camera <name> --extrinsic <calibration_result.json> [options]
      Transform one recorded RGB-D frame into robot-base/world-frame PLY.

  fuse <episode_dir> (--extrinsic camera=result.json [...] | --extrinsics-map map.json)
      Fuse multiple calibrated RGB-D cameras from one frame into one robot-base/world-frame PLY.

Examples:
  bash 20_pointcloud_calibration.sh self-test
  bash 20_pointcloud_calibration.sh board
  bash 20_pointcloud_calibration.sh capture --camera middle
  bash 20_pointcloud_calibration.sh solve ~/Desktop/franka_record_data/calibration/middle_eye_to_hand_YYYYMMDD_HHMMSS
  bash 20_pointcloud_calibration.sh apply ~/Desktop/franka_record_data/rgb_pointcloud/0 --camera middle --extrinsic ~/Desktop/franka_record_data/calibration/middle_eye_to_hand_YYYYMMDD_HHMMSS/calibration_result.json
  bash 20_pointcloud_calibration.sh fuse ~/Desktop/franka_record_data/rgb_pointcloud/0 --extrinsic middle=~/Desktop/franka_record_data/calibration/middle_eye_to_hand_YYYYMMDD_HHMMSS/calibration_result.json
EOF
}

workflow() {
    cat <<EOF
Recommended workflow:
  1. Generate and print board:
       bash 20_pointcloud_calibration.sh board

  2. Mount the board rigidly on the gripper/end-effector.
     Measure square_length_m and marker_length_m; keep them consistent with capture.

  3. Capture samples while manually moving the robot:
       bash 20_pointcloud_calibration.sh capture --camera middle
     Press s only when the robot is still and the overlay is valid.
     Collect 25-40 poses with strong rotation diversity.

  4. Solve and validate:
       bash 20_pointcloud_calibration.sh solve <session_dir>

  5. Use only if doctor status is PASS or PASS_WITH_WARNINGS and warnings are understood.
     Then apply to a smoke RGB-D episode:
       bash 20_pointcloud_calibration.sh apply <episode_dir> --camera middle --extrinsic <session_dir>/calibration_result.json
EOF
}

has_option() {
    local needle="$1"
    shift || true
    local arg
    for arg in "$@"; do
        if [[ "$arg" == "$needle" || "$arg" == "$needle="* ]]; then
            return 0
        fi
    done
    return 1
}

reject_option() {
    local option="$1"
    shift || true
    local arg
    for arg in "$@"; do
        if [[ "$arg" == "$option" || "$arg" == "$option="* ]]; then
            echo "error: $COMMAND does not allow $option in the wrapper; use the default session files or call the Python module directly for experiments" >&2
            exit 2
        fi
    done
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

COMMAND="$1"
shift || true

case "$COMMAND" in
    workflow)
        workflow
        exit 0
        ;;
    latest)
        find "$DEFAULT_OUTPUT_ROOT" -maxdepth 1 -type d -name '*_eye_to_hand_*' -printf '%T@ %p\n' 2>/dev/null \
            | sort -nr \
            | head -20 \
            | cut -d' ' -f2-
        exit 0
        ;;
esac

source_conda
conda activate "$CONDA_ENV" || {
    echo "error: failed to activate conda env $CONDA_ENV" >&2
    exit 1
}

cd "$REPO_ROOT"

case "$COMMAND" in
    doctor)
        python -m pointcloud.calibration.doctor "$@"
        ;;
    preflight)
        python -m pointcloud.calibration.doctor "$@"
        ;;
    self-test)
        python -m py_compile pointcloud/*.py pointcloud/calibration/*.py
        python -m pointcloud.calibration.synthetic_handeye_test
        python -m pointcloud.calibration.synthetic_fusion_test
        python -m pointcloud.calibration.doctor --env-only
        ;;
    board)
        mkdir -p "$(dirname "$DEFAULT_BOARD")"
        BOARD_ARGS=("$@")
        if ! has_option "--output" "${BOARD_ARGS[@]}"; then
            BOARD_ARGS=(--output "$DEFAULT_BOARD" "${BOARD_ARGS[@]}")
        fi
        python -m pointcloud.calibration.targets "${BOARD_ARGS[@]}"
        ;;
    print-board)
        mkdir -p "$(dirname "$DEFAULT_BOARD")"
        BOARD_ARGS=("$@")
        if ! has_option "--output" "${BOARD_ARGS[@]}"; then
            BOARD_ARGS=(--output "$DEFAULT_BOARD" "${BOARD_ARGS[@]}")
        fi
        python -m pointcloud.calibration.targets "${BOARD_ARGS[@]}"
        ;;
    capture)
        mkdir -p "$DEFAULT_OUTPUT_ROOT"
        python -m pointcloud.calibration.capture_samples --output-root "$DEFAULT_OUTPUT_ROOT" "$@"
        ;;
    solve)
        if [[ $# -lt 1 ]]; then
            echo "error: solve requires session_dir" >&2
            usage
            exit 1
        fi
        SESSION_DIR="$1"
        shift
        reject_option "--output" "$@"
        python -m pointcloud.calibration.solve_eye_to_hand "$SESSION_DIR" "$@"
        python -m pointcloud.calibration.doctor --strict "$SESSION_DIR"
        ;;
    validate)
        if [[ $# -lt 1 ]]; then
            echo "error: validate requires session_dir" >&2
            usage
            exit 1
        fi
        SESSION_DIR="$1"
        shift
        reject_option "--result" "$@"
        reject_option "--output" "$@"
        python -m pointcloud.calibration.validate_extrinsic "$SESSION_DIR" "$@"
        python -m pointcloud.calibration.doctor --strict "$SESSION_DIR"
        ;;
    check)
        if [[ $# -lt 1 ]]; then
            echo "error: check requires session_dir" >&2
            usage
            exit 1
        fi
        SESSION_DIR="$1"
        shift
        reject_option "--result" "$@"
        reject_option "--output" "$@"
        python -m pointcloud.calibration.validate_extrinsic "$SESSION_DIR" "$@"
        python -m pointcloud.calibration.doctor --strict "$SESSION_DIR"
        ;;
    apply)
        python -m pointcloud.calibration.apply_extrinsics "$@"
        ;;
    fuse)
        python -m pointcloud.calibration.fuse_world_pointclouds "$@"
        ;;
    *)
        echo "error: unknown command: $COMMAND" >&2
        usage
        exit 1
        ;;
esac
