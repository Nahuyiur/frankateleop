#!/bin/bash
set -e

# 这里是脚本7的默认参数；需要调轨迹速度或夹爪参数时优先改这里。
DEFAULT_REPLAY_SPEED="${DEFAULT_REPLAY_SPEED:-1.0}"
DEFAULT_GRIPPER_SPEED="${DEFAULT_GRIPPER_SPEED:-0.1}"
DEFAULT_GRIPPER_FORCE="${DEFAULT_GRIPPER_FORCE:-10.0}"
DEFAULT_GRIPPER_EVENT_DELTA="${DEFAULT_GRIPPER_EVENT_DELTA:-0.01}"
DEFAULT_GRIPPER_REPLAY_MODE="${DEFAULT_GRIPPER_REPLAY_MODE:-event}"
DEFAULT_GRIPPER_COMMAND_HZ="${DEFAULT_GRIPPER_COMMAND_HZ:-15.0}"
DEFAULT_GRIPPER_HOLD_SEC="${DEFAULT_GRIPPER_HOLD_SEC:-2.0}"
DEFAULT_GRIPPER_HOST="${DEFAULT_GRIPPER_HOST:-127.0.0.1}"
DEFAULT_GRIPPER_PORT="${DEFAULT_GRIPPER_PORT:-50052}"
DEFAULT_APPROACH_START="${DEFAULT_APPROACH_START:-1}"
DEFAULT_APPROACH_START_MAX_DELTA="${DEFAULT_APPROACH_START_MAX_DELTA:-0.75}"
DEFAULT_APPROACH_START_STEP_DELTA="${DEFAULT_APPROACH_START_STEP_DELTA:-0.02}"
DEFAULT_APPROACH_START_HZ="${DEFAULT_APPROACH_START_HZ:-5.0}"

is_truthy() {
    case "${1,,}" in
        1|true|yes|on) return 0 ;;
        *) return 1 ;;
    esac
}

echo ">>> 激活 conda 环境 polymetis ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败，请确认 polymetis 环境存在"; exit 1; }

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    echo "用法: bash 7_replay_fr3.sh <episode目录或pkl.gz文件> [replay参数...]"
    echo "默认 dry-run，不会控制机械臂。加 --execute 才会发送命令。"
    echo "默认轨迹速度: $DEFAULT_REPLAY_SPEED"
    echo "默认夹爪 speed/force: $DEFAULT_GRIPPER_SPEED / $DEFAULT_GRIPPER_FORCE"
    echo "默认夹爪事件阈值: $DEFAULT_GRIPPER_EVENT_DELTA m"
    echo "默认夹爪 replay 模式/Hz: $DEFAULT_GRIPPER_REPLAY_MODE / $DEFAULT_GRIPPER_COMMAND_HZ"
    echo "默认夹爪事件后 joint 暂停: $DEFAULT_GRIPPER_HOLD_SEC s"
    echo "默认夹爪 server: $DEFAULT_GRIPPER_HOST:$DEFAULT_GRIPPER_PORT"
    echo "默认起点自动慢速靠近: $DEFAULT_APPROACH_START"
    echo "默认起点靠近 max/step/hz: $DEFAULT_APPROACH_START_MAX_DELTA / $DEFAULT_APPROACH_START_STEP_DELTA / $DEFAULT_APPROACH_START_HZ"
    echo "示例: bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/3"
    echo "示例: bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --execute --speed 0.5 --gripper-force 20"
    exit 0
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"

echo ">>> 使用仓库根目录 franka_replay ..."
echo ">>> 输入 episode: $1"
echo ">>> 默认轨迹速度: $DEFAULT_REPLAY_SPEED"
echo ">>> 默认夹爪 speed/force: $DEFAULT_GRIPPER_SPEED / $DEFAULT_GRIPPER_FORCE"
echo ">>> 默认夹爪事件阈值: $DEFAULT_GRIPPER_EVENT_DELTA m"
echo ">>> 默认夹爪 replay 模式/Hz: $DEFAULT_GRIPPER_REPLAY_MODE / $DEFAULT_GRIPPER_COMMAND_HZ"
echo ">>> 默认夹爪事件后 joint 暂停: $DEFAULT_GRIPPER_HOLD_SEC s"
echo ">>> 默认夹爪 server: $DEFAULT_GRIPPER_HOST:$DEFAULT_GRIPPER_PORT"
echo ">>> 默认起点自动慢速靠近: $DEFAULT_APPROACH_START"
echo ">>> 默认起点靠近 max/step/hz: $DEFAULT_APPROACH_START_MAX_DELTA / $DEFAULT_APPROACH_START_STEP_DELTA / $DEFAULT_APPROACH_START_HZ"

approach_args=()
if is_truthy "$DEFAULT_APPROACH_START"; then
    approach_args+=(
        --approach-start
        --approach-start-max-delta "$DEFAULT_APPROACH_START_MAX_DELTA"
        --approach-start-step-delta "$DEFAULT_APPROACH_START_STEP_DELTA"
        --approach-start-hz "$DEFAULT_APPROACH_START_HZ"
    )
fi

cd "$REPO_ROOT"
python -m franka_replay.replay_fr3 \
    --speed "$DEFAULT_REPLAY_SPEED" \
    --gripper-speed "$DEFAULT_GRIPPER_SPEED" \
    --gripper-force "$DEFAULT_GRIPPER_FORCE" \
    --gripper-event-delta "$DEFAULT_GRIPPER_EVENT_DELTA" \
    --gripper-replay-mode "$DEFAULT_GRIPPER_REPLAY_MODE" \
    --gripper-command-hz "$DEFAULT_GRIPPER_COMMAND_HZ" \
    --gripper-hold-sec "$DEFAULT_GRIPPER_HOLD_SEC" \
    --gripper-host "$DEFAULT_GRIPPER_HOST" \
    --gripper-port "$DEFAULT_GRIPPER_PORT" \
    "${approach_args[@]}" \
    "$@"
