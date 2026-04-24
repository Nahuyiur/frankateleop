#!/bin/bash
set -e

# 这里是脚本7的默认参数；需要调轨迹速度或夹爪参数时优先改这里。
DEFAULT_REPLAY_SPEED="${DEFAULT_REPLAY_SPEED:-1.0}"
DEFAULT_GRIPPER_SPEED="${DEFAULT_GRIPPER_SPEED:-0.1}"
DEFAULT_GRIPPER_FORCE="${DEFAULT_GRIPPER_FORCE:-10.0}"
DEFAULT_GRIPPER_EVENT_DELTA="${DEFAULT_GRIPPER_EVENT_DELTA:-0.01}"
DEFAULT_GRIPPER_HOST="${DEFAULT_GRIPPER_HOST:-127.0.0.1}"
DEFAULT_GRIPPER_PORT="${DEFAULT_GRIPPER_PORT:-50052}"

echo ">>> 激活 conda 环境 polymetis ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败，请确认 polymetis 环境存在"; exit 1; }

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    echo "用法: bash 7_replay_fr3.sh <episode目录或pkl.gz文件> [replay参数...]"
    echo "默认 dry-run，不会控制机械臂。加 --execute 才会发送命令。"
    echo "默认轨迹速度: $DEFAULT_REPLAY_SPEED"
    echo "默认夹爪 speed/force: $DEFAULT_GRIPPER_SPEED / $DEFAULT_GRIPPER_FORCE"
    echo "默认夹爪事件阈值: $DEFAULT_GRIPPER_EVENT_DELTA m"
    echo "默认夹爪 server: $DEFAULT_GRIPPER_HOST:$DEFAULT_GRIPPER_PORT"
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
echo ">>> 默认夹爪 server: $DEFAULT_GRIPPER_HOST:$DEFAULT_GRIPPER_PORT"

cd "$REPO_ROOT"
python -m franka_replay.replay_fr3 \
    --speed "$DEFAULT_REPLAY_SPEED" \
    --gripper-speed "$DEFAULT_GRIPPER_SPEED" \
    --gripper-force "$DEFAULT_GRIPPER_FORCE" \
    --gripper-event-delta "$DEFAULT_GRIPPER_EVENT_DELTA" \
    --gripper-host "$DEFAULT_GRIPPER_HOST" \
    --gripper-port "$DEFAULT_GRIPPER_PORT" \
    "$@"
