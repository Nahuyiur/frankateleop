#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
STATE_READER="${FRANKA_STATE_READER:-/home/muka/libfranka/build/examples/echo_robot_state}"
OUTPUT_ROOT="${CALIBRATED_DEMO_OUTPUT_ROOT:-$HOME/Desktop/calibrated_human_demos}"
CALIBRATION="${MULTICAMERA_CALIBRATION:-$REPO_ROOT/config/calibration/multicamera_extrinsics_final.json}"
LABEL="${1:-human_demo}"
if [[ $# -gt 0 ]]; then
    shift
fi

if [[ ! "$LABEL" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "ERROR: label may contain only letters, digits, dot, underscore, and dash" >&2
    exit 2
fi
if [[ ! -x "$STATE_READER" ]]; then
    echo "ERROR: Franka state reader is unavailable: $STATE_READER" >&2
    exit 1
fi
if [[ ! -f "$CALIBRATION" ]]; then
    echo "ERROR: final multi-camera calibration is unavailable: $CALIBRATION" >&2
    exit 1
fi
python - "$CALIBRATION" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
assert payload["schema_version"] == "franka_multicamera_extrinsics_v2"
assert payload["acceptance"]["left_passed"] is True
assert payload["acceptance"]["middle_passed"] is True
assert payload["cameras"]["left"]["serial_number"] == "347622072146"
assert payload["cameras"]["middle"]["serial_number"] == "332322072361"
PY

timestamp="$(date +%Y%m%d_%H%M%S)"
session_dir="$OUTPUT_ROOT/${LABEL}_${timestamp}"
suffix=0
while [[ -e "$session_dir" ]]; do
    suffix=$((suffix + 1))
    session_dir="$OUTPUT_ROOT/${LABEL}_${timestamp}_$suffix"
done
mkdir -p "$session_dir"
cp "$CALIBRATION" "$session_dir/multicamera_extrinsics.json"

capture_robot_state() {
    local destination="$1"
    local temporary="${destination}.tmp"
    set +o pipefail
    "$STATE_READER" "$ROBOT_IP" 2>/dev/null | head -n 1 > "$temporary"
    local read_status=$?
    set -o pipefail
    if [[ $read_status -ne 0 || ! -s "$temporary" ]]; then
        rm -f "$temporary"
        echo "ERROR: failed to read Franka state from $ROBOT_IP" >&2
        return 1
    fi
    python -c 'import json,sys; p=sys.argv[1]; d=json.load(open(p)); assert len(d["O_T_EE"]) == 16' "$temporary"
    mv "$temporary" "$destination"
}

echo ">>> Recording directory: $session_dir"
echo ">>> Cameras: left_wrist / left / middle, all RGB + aligned metric depth"
echo ">>> Primary human-demo view: left; secondary cross-check view: middle"
echo ">>> The left_wrist stream is retained as an additional calibrated reference."
echo ">>> Keep the Franka completely stationary throughout the recording."
capture_robot_state "$session_dir/robot_before.json"

set +e
DISPLAY="${DISPLAY:-:0}" /home/muka/miniconda3/bin/conda run --no-capture-output -n franka_capture \
    python -m pointcloud.record_rgb_pointclouds \
    --output-root "$session_dir" \
    --task rgbd \
    --camera-names left_wrist,left,middle \
    --depth-cameras left_wrist,left,middle \
    --camera-fps 30 \
    --width 640 \
    --height 480 \
    --no-depth-proof \
    "$@"
record_status=$?
set -e

capture_robot_state "$session_dir/robot_after.json"
if [[ $record_status -ne 0 ]]; then
    echo "ERROR: camera recorder exited with status $record_status" >&2
    exit "$record_status"
fi

echo ">>> Recording complete: $session_dir"
echo ">>> Controls: s=start/resume, w=pause, e=save episode, q=save and quit"
echo ">>> Multi-camera calibration copied to: $session_dir/multicamera_extrinsics.json"
