#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONDA_ENV="${STACK_RING_CAPTURE_ENV:-franka_capture}"
CALIBRATION_DIR="${STACK_RING_CALIBRATION_DIR:-$SCRIPT_DIR/calibration}"
CALIBRATION_MODULE="${STACK_RING_CALIBRATION_MODULE:-$CALIBRATION_DIR/ring_calibration_v1.py}"
MAPPING_MODEL="${STACK_RING_MAPPING_MODEL:-$CALIBRATION_DIR/ring_mapping_model_selected.json}"
PROJECTION_REPORT="${STACK_RING_PROJECTION_REPORT:-$CALIBRATION_DIR/ring_mapping_report.json}"

if command -v conda >/dev/null 2>&1; then
  CONDA_BASE="$(conda info --base)"
elif [[ -x "$HOME/miniconda3/bin/conda" ]]; then
  CONDA_BASE="$HOME/miniconda3"
else
  echo "error: conda not found" >&2
  exit 1
fi
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$CONDA_ENV"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$REPO_ROOT"

args=(
  --calibration-module "$CALIBRATION_MODULE"
  --mapping-model "$MAPPING_MODEL"
)
if [[ -f "$PROJECTION_REPORT" ]]; then
  args+=(--projection-report "$PROJECTION_REPORT")
fi
exec python "$SCRIPT_DIR/stack_ring_retarget.py" "${args[@]}" "$@"
