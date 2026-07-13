#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DEMO="${JENGA_DEMO:-$HOME/Desktop/Muka_NAS/stack_jenga/High_Quality/0/0.pkl.gz}"
DEMO_ROOT="${JENGA_DEMO_ROOT:-$HOME/Desktop/Muka_NAS/stack_jenga/High_Quality}"
MAPPING_MODEL="${JENGA_MAPPING_MODEL:-$HOME/Desktop/Muka_NAS/stack_jenga/DAgger/mapping_model.json}"
REQUIRE_MAPPING_MODEL="${JENGA_REQUIRE_MAPPING_MODEL:-1}"
MAX_DEMOS="${JENGA_MAX_DEMOS:-6}"
MIN_VALID_DEMOS="${JENGA_MIN_VALID_DEMOS:-2}"
MAX_PICK_SHIFT_M="${JENGA_MAX_PICK_SHIFT_M:-0.30}"
MAX_PLACE_SHIFT_M="${JENGA_MAX_PLACE_SHIFT_M:-0.45}"
OUTPUT="${JENGA_OUTPUT:-$HOME/Desktop/Muka_NAS/stack_jenga/DAgger/live_retargeted.pkl.gz}"
DEBUG_DIR="${JENGA_DEBUG_DIR:-$HOME/Desktop/Muka_NAS/stack_jenga/DAgger/live_debug}"
CAMERA_WARMUP_SEC="${JENGA_CAMERA_WARMUP_SEC:-3.0}"
STACK_HEIGHT_CAMERA="${JENGA_STACK_HEIGHT_CAMERA:-left}"
STACK_COUNT_OVERRIDE="${JENGA_STACK_COUNT_OVERRIDE:-}"
BLOCK_HEIGHT_M="${JENGA_BLOCK_HEIGHT_M:-}"
PLACE_OFFSET_M="${JENGA_PLACE_OFFSET_M:-0,0,0}"
MAX_STACK_COUNT="${JENGA_MAX_STACK_COUNT:-20}"
STACK_LAYER_PITCH_PX="${JENGA_STACK_LAYER_PITCH_PX:-}"
STACK_LAYER_PITCH_RATIO="${JENGA_STACK_LAYER_PITCH_RATIO:-0.17}"
MIN_STACK_HEIGHT_QUALITY="${JENGA_MIN_STACK_HEIGHT_QUALITY:-0.70}"
HOST="${JENGA_REPLAY_HOST:-127.0.0.1}"
PORT="${JENGA_REPLAY_PORT:-6002}"
GRIPPER_HOST="${JENGA_GRIPPER_HOST:-127.0.0.1}"
GRIPPER_PORT="${JENGA_GRIPPER_PORT:-50054}"
CURRENT_EPISODE=""
CURRENT_FRAME="0"
EXECUTE=0
APPROACH_START=0
WITH_REPLAY=0
NO_MAPPING_MODEL=0
NO_DYNAMIC_PLACE_Z=0

usage() {
    cat <<EOF
Usage:
  bash DAgger/run_live_jenga_stack.sh [options]

Default behavior:
  1. Capture live left/middle side-camera frames.
  2. Retarget through the validated pixel-to-robot mapping model.
  3. Stop after writing the mapped trajectory and debug overlays.

Options:
  --demo PATH              Successful demo pkl/episode. Default: $DEMO
  --demo-root DIR          High_Quality demos used for mapping. Default: $DEMO_ROOT
  --mapping-model PATH     Validated mapping model. Default: $MAPPING_MODEL
  --allow-old-mapping      If model is missing, fall back to legacy two-point mapping.
  --no-mapping-model       Disable validated model and use legacy two-point mapping.
  --max-demos N            Max successful demos to aggregate. Default: $MAX_DEMOS
  --min-valid-demos N      Min usable demo mappings required. Default: $MIN_VALID_DEMOS
  --max-pick-shift-m M     Pickup safety gate. Default: $MAX_PICK_SHIFT_M
  --max-place-shift-m M    Place safety gate. Default: $MAX_PLACE_SHIFT_M
  --output PATH            Retargeted output pkl. Default: $OUTPUT
  --debug-dir DIR          Detection overlays/summary. Default: $DEBUG_DIR
  --camera-warmup-sec S    Live camera warmup before capture. Default: $CAMERA_WARMUP_SEC
  --current-episode PATH   Use an episode frame instead of live cameras.
  --current-frame N        Frame for --current-episode. Default: $CURRENT_FRAME
  --stack-height-camera C  Camera for right-stack counting. Default: $STACK_HEIGHT_CAMERA
  --stack-count-override N Manually set existing right-side stack count.
  --block-height-m M       Override per-layer Z increment in meters.
  --place-offset-m X,Y,Z   Final place offset in meters. Default: $PLACE_OFFSET_M
  --place-z-offset-m Z     Convenience for --place-offset-m 0,0,Z.
  --max-stack-count N      Clamp detected stack count. Default: $MAX_STACK_COUNT
  --stack-layer-pitch-px P Apparent pixel increment per layer.
  --stack-layer-pitch-ratio R
                           Default pitch = demo block height * R. Default: $STACK_LAYER_PITCH_RATIO
  --min-stack-height-quality Q
                           Min confidence before auto Z change. Default: $MIN_STACK_HEIGHT_QUALITY
  --no-dynamic-place-z     Keep demo place z instead of counting stack height.
  --host HOST              Robot node host. Default: $HOST
  --port PORT              Robot node port. Default: $PORT
  --gripper-host HOST      Gripper server host. Default: $GRIPPER_HOST
  --gripper-port PORT      Gripper server port. Default: $GRIPPER_PORT
  --approach-start         Let Cartesian replay approach frame 0 before execute.
  --with-replay            After mapping, run Cartesian replay dry-run.
  --execute                Execute after mapping. Implies --with-replay.
  --no-replay              Accepted for compatibility; mapping-only is default.
  -h, --help               Show this help.

Before --execute:
  Stop any joint-mode node on port 6002 and start:
    bash DAgger/launch_jenga_ee_node.sh

Run this script first without --with-replay or --execute. Inspect:
  $DEBUG_DIR/current_left.jpg
  $DEBUG_DIR/stack_height_$STACK_HEIGHT_CAMERA.jpg
  $DEBUG_DIR/mapping_summary.json

Build/validate the mapping model first if needed:
  bash DAgger/build_jenga_mapping_model.sh --max-demos 100 --validation-fraction 0.2
EOF
}

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --demo)
            DEMO="$2"; shift 2 ;;
        --demo-root)
            DEMO_ROOT="$2"; shift 2 ;;
        --mapping-model)
            MAPPING_MODEL="$2"; shift 2 ;;
        --allow-old-mapping)
            REQUIRE_MAPPING_MODEL=0; shift ;;
        --no-mapping-model)
            NO_MAPPING_MODEL=1; REQUIRE_MAPPING_MODEL=0; shift ;;
        --max-demos)
            MAX_DEMOS="$2"; shift 2 ;;
        --min-valid-demos)
            MIN_VALID_DEMOS="$2"; shift 2 ;;
        --max-pick-shift-m)
            MAX_PICK_SHIFT_M="$2"; shift 2 ;;
        --max-place-shift-m)
            MAX_PLACE_SHIFT_M="$2"; shift 2 ;;
        --output)
            OUTPUT="$2"; shift 2 ;;
        --debug-dir)
            DEBUG_DIR="$2"; shift 2 ;;
        --camera-warmup-sec)
            CAMERA_WARMUP_SEC="$2"; shift 2 ;;
        --current-episode)
            CURRENT_EPISODE="$2"; shift 2 ;;
        --current-frame)
            CURRENT_FRAME="$2"; shift 2 ;;
        --stack-height-camera)
            STACK_HEIGHT_CAMERA="$2"; shift 2 ;;
        --stack-count-override)
            STACK_COUNT_OVERRIDE="$2"; shift 2 ;;
        --block-height-m)
            BLOCK_HEIGHT_M="$2"; shift 2 ;;
        --place-offset-m)
            PLACE_OFFSET_M="$2"; shift 2 ;;
        --place-z-offset-m)
            PLACE_OFFSET_M="0,0,$2"; shift 2 ;;
        --max-stack-count)
            MAX_STACK_COUNT="$2"; shift 2 ;;
        --stack-layer-pitch-px)
            STACK_LAYER_PITCH_PX="$2"; shift 2 ;;
        --stack-layer-pitch-ratio)
            STACK_LAYER_PITCH_RATIO="$2"; shift 2 ;;
        --min-stack-height-quality)
            MIN_STACK_HEIGHT_QUALITY="$2"; shift 2 ;;
        --no-dynamic-place-z)
            NO_DYNAMIC_PLACE_Z=1; shift ;;
        --host)
            HOST="$2"; shift 2 ;;
        --port)
            PORT="$2"; shift 2 ;;
        --gripper-host)
            GRIPPER_HOST="$2"; shift 2 ;;
        --gripper-port)
            GRIPPER_PORT="$2"; shift 2 ;;
        --approach-start)
            APPROACH_START=1; shift ;;
        --with-replay)
            WITH_REPLAY=1; shift ;;
        --execute)
            EXECUTE=1; WITH_REPLAY=1; shift ;;
        --no-replay)
            WITH_REPLAY=0; shift ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            echo "error: unknown option: $1" >&2
            usage >&2
            exit 2 ;;
    esac
done

retarget_args=(
    --demo "$DEMO"
    --demo-root "$DEMO_ROOT"
    --output "$OUTPUT"
    --debug-dir "$DEBUG_DIR"
    --max-pick-shift-m "$MAX_PICK_SHIFT_M"
    --max-place-shift-m "$MAX_PLACE_SHIFT_M"
    --camera-warmup-sec "$CAMERA_WARMUP_SEC"
    --stack-height-camera "$STACK_HEIGHT_CAMERA"
    --place-offset-m "$PLACE_OFFSET_M"
    --max-stack-count "$MAX_STACK_COUNT"
    --stack-layer-pitch-ratio "$STACK_LAYER_PITCH_RATIO"
    --min-stack-height-quality "$MIN_STACK_HEIGHT_QUALITY"
)
if [[ "$NO_DYNAMIC_PLACE_Z" -eq 1 ]]; then
    retarget_args+=(--no-dynamic-place-z)
fi
if [[ -n "$STACK_COUNT_OVERRIDE" ]]; then
    retarget_args+=(--stack-count-override "$STACK_COUNT_OVERRIDE")
fi
if [[ -n "$BLOCK_HEIGHT_M" ]]; then
    retarget_args+=(--block-height-m "$BLOCK_HEIGHT_M")
fi
if [[ -n "$STACK_LAYER_PITCH_PX" ]]; then
    retarget_args+=(--stack-layer-pitch-px "$STACK_LAYER_PITCH_PX")
fi
if [[ "$NO_MAPPING_MODEL" -eq 1 ]]; then
    retarget_args+=(--no-mapping-model --max-demos "$MAX_DEMOS" --min-valid-demos "$MIN_VALID_DEMOS")
else
    retarget_args+=(--mapping-model "$MAPPING_MODEL")
    if [[ "$REQUIRE_MAPPING_MODEL" -eq 1 ]]; then
        retarget_args+=(--require-mapping-model)
    else
        retarget_args+=(--max-demos "$MAX_DEMOS" --min-valid-demos "$MIN_VALID_DEMOS")
    fi
fi
if [[ -n "$CURRENT_EPISODE" ]]; then
    retarget_args+=(--current-episode "$CURRENT_EPISODE" --current-frame "$CURRENT_FRAME")
fi

echo "==> Retargeting Jenga trajectory"
echo "    demo: $DEMO"
echo "    demo_root: $DEMO_ROOT"
if [[ "$NO_MAPPING_MODEL" -eq 1 ]]; then
    echo "    mapping_model: disabled"
else
    echo "    mapping_model: $MAPPING_MODEL"
fi
echo "    output: $OUTPUT"
echo "    debug: $DEBUG_DIR"
echo "    max_pick/place_shift_m: $MAX_PICK_SHIFT_M/$MAX_PLACE_SHIFT_M"
if [[ -z "$CURRENT_EPISODE" ]]; then
    echo "    camera_warmup_sec: $CAMERA_WARMUP_SEC"
fi
if [[ "$NO_DYNAMIC_PLACE_Z" -eq 1 ]]; then
    echo "    dynamic_place_z: disabled"
else
    echo "    dynamic_place_z: camera=$STACK_HEIGHT_CAMERA max_stack=$MAX_STACK_COUNT pitch_ratio=$STACK_LAYER_PITCH_RATIO min_quality=$MIN_STACK_HEIGHT_QUALITY"
fi
echo "    place_offset_m: $PLACE_OFFSET_M"
bash "$SCRIPT_DIR/run_jenga_retarget.sh" "${retarget_args[@]}"

echo "==> Detection overlays"
echo "    $DEBUG_DIR/current_left.jpg"
if [[ "$NO_DYNAMIC_PLACE_Z" -eq 0 ]]; then
    echo "    $DEBUG_DIR/stack_height_$STACK_HEIGHT_CAMERA.jpg"
fi
echo "    $DEBUG_DIR/mapping_summary.json"

if [[ "$WITH_REPLAY" -eq 0 ]]; then
    echo "==> Mapping only; replay skipped."
    exit 0
fi

replay_args=(
    "$OUTPUT"
    --host "$HOST"
    --port "$PORT"
    --gripper-host "$GRIPPER_HOST"
    --gripper-port "$GRIPPER_PORT"
)
if [[ "$APPROACH_START" -eq 1 ]]; then
    replay_args+=(--approach-start)
fi
if [[ "$EXECUTE" -eq 1 ]]; then
    replay_args+=(--execute)
fi

MODE_LABEL="dry-run"
if [[ "$EXECUTE" -eq 1 ]]; then
    MODE_LABEL="EXECUTE"
fi
echo "==> Cartesian replay $MODE_LABEL"
bash "$SCRIPT_DIR/replay_jenga_cartesian.sh" "${replay_args[@]}"
