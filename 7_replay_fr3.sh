#!/bin/bash
set -e

# Script 7 is the single-arm replay entrypoint.
#
# Supported inputs:
#   1) Current GUI episode dir:
#      /home/pnp/Desktop/franka_record_data/<task>/<High_Quality|Low_Quality|Failure>/<index>
#   2) Current GUI sidecar:
#      /home/pnp/Desktop/franka_record_data/<task>/<quality>/<index>/metadata.json
#   3) Old episode dir:
#      /home/pnp/Desktop/franka_record_data/<task>/<index>
#   4) Exact pickle:
#      .../<index>.pkl.gz
#
# Common workflow:
#   bash 7_replay_fr3.sh <episode> --skip-robot-check
#       Only inspect the file. This never connects to the robot.
#   bash 7_replay_fr3.sh <episode>
#       Dry-run robot start check. This connects to robot node but sends no command.
#       If auto-approach is enabled and the start is within its safety limit,
#       dry-run reports that execute mode can approach frame 0 instead of failing.
#   bash 7_replay_fr3.sh <episode> --execute
#       Execute replay. If enabled and within limits, it first approaches frame 0.
#
# This script is for left/right single-arm data only. Use
# 16_replay_bi_arm_pipeline.sh for dual-arm episodes. Do not use --arm left/right
# to extract one arm from a dual-arm episode; dual episodes are rejected.
#
# Safety notes:
#   - Without --execute, no joint/gripper replay command is sent.
#   - --skip-robot-check is file inspection only, not a way to bypass safety
#     checks. The Python replay rejects --execute --skip-robot-check.
#   - DEFAULT_APPROACH_START=1 makes dry-run validate whether execute mode can
#     auto-approach frame 0. In --execute mode it may slowly move the arm to
#     frame 0 if the start delta is within the configured safety limit.
#     Set DEFAULT_APPROACH_START=0 to disable this auto-approach behavior.
#   - Prefer episode directories or metadata.json. A bare right-arm pkl.gz has
#     no arm_side metadata, so you must pass --arm right and, if needed, explicit
#     --host/--port/--gripper-host/--gripper-port.
#   - Auto endpoint defaults target physical hardware: left gripper defaults to
#     127.0.0.1:50052; right gripper defaults to the robot host with port 50053.
#     A copied local file can still contain metadata pointing at remote hardware.
#   - --latest picks the newest pkl.gz by file mtime. Passing a task directory
#     can select across High_Quality, Low_Quality, and Failure.
#   - Do not replay while 4_run_env.sh or a GUI teleop run-env process is active.
#   - Replay is not motion planning or collision checking. Execute only with the
#     same arm/calibration, a clear workspace, and reachable E-stop.
#
# 这里是脚本7的默认参数；需要调轨迹速度或夹爪参数时优先改这里。
DEFAULT_REPLAY_SPEED="${DEFAULT_REPLAY_SPEED:-1.0}"
DEFAULT_GRIPPER_SPEED="${DEFAULT_GRIPPER_SPEED:-0.1}"
DEFAULT_GRIPPER_FORCE="${DEFAULT_GRIPPER_FORCE:-10.0}"
DEFAULT_GRIPPER_EVENT_DELTA="${DEFAULT_GRIPPER_EVENT_DELTA:-0.01}"
DEFAULT_GRIPPER_REPLAY_MODE="${DEFAULT_GRIPPER_REPLAY_MODE:-event}"
DEFAULT_GRIPPER_COMMAND_HZ="${DEFAULT_GRIPPER_COMMAND_HZ:-15.0}"
DEFAULT_GRIPPER_HOLD_SEC="${DEFAULT_GRIPPER_HOLD_SEC:-2.0}"
DEFAULT_GRIPPER_HOST="${DEFAULT_GRIPPER_HOST:-}"
DEFAULT_GRIPPER_PORT="${DEFAULT_GRIPPER_PORT:-}"
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

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    echo "脚本7：单臂 FR3 replay。双臂数据请使用 16_replay_bi_arm_pipeline.sh。"
    echo "用法: bash 7_replay_fr3.sh <episode目录、metadata.json 或 pkl.gz文件> [replay参数...]"
    echo "支持当前格式: task/High_Quality|Low_Quality|Failure/index"
    echo "支持旧格式: task/index"
    echo "不要用脚本7处理双臂 episode，也不要用 --arm left/right 从双臂 episode 抽单臂。"
    echo ""
    echo "SAFETY:"
    echo "默认 dry-run，不会控制机械臂。加 --execute 才会发送命令。"
    echo "dry-run 会按当前 approach 参数检查 execute 是否可行；若起点偏差在自动靠近安全上限内，会提示 execute 会先慢速靠近 frame 0。"
    echo "加 --execute 后会发送 joint/gripper 命令；默认 DEFAULT_APPROACH_START=1 时，必要时会先慢速靠近 frame 0。"
    echo "如果不希望自动靠近起点，运行前设置 DEFAULT_APPROACH_START=0。"
    echo "--skip-robot-check 只检查文件，不连接 robot node；它不是绕过硬件安全检查的开关，也不能和 --execute 同用。"
    echo "replay 不是 motion planner 或碰撞检查器；只在同一 arm、同一标定、初始姿态接近、工作区清空且 E-stop 可触达时执行。"
    echo "执行时不要同时运行 4_run_env.sh、GUI teleop、recording，或任何占用同一 arm/gripper 的控制栈。"
    echo "执行前先看 --skip-robot-check，再做不带 --execute 的 dry-run，确认 Episode、Inferred arm side、Robot node 和夹爪 endpoint。"
    echo ""
    echo "INPUT/ENDPOINT:"
    echo "默认 arm=auto：优先读取 metadata.json 的 arm_side；没有 metadata 时按 left 处理。"
    echo "右臂数据优先传 episode 目录或 metadata.json；如果只有裸 pkl.gz，必须显式加 --arm right。"
    echo "endpoint auto 是物理硬件目标：left gripper 默认 127.0.0.1:50052；right gripper 默认 robot host:50053。"
    echo "本地文件路径不等于本地硬件目标；复制来的 metadata 仍可能指向右臂远端机器。"
    echo "如果传 task 或 quality 目录，必须加 --latest；--latest 按 pkl.gz 文件 mtime 选择最新。"
    echo "注意: 对 task 根目录使用 --latest 可能跨 High_Quality/Low_Quality/Failure 选择最新数据。"
    echo ""
    echo "DEFAULTS:"
    echo "默认轨迹速度: $DEFAULT_REPLAY_SPEED"
    echo "默认夹爪 speed/force: $DEFAULT_GRIPPER_SPEED / $DEFAULT_GRIPPER_FORCE"
    echo "默认夹爪事件阈值: $DEFAULT_GRIPPER_EVENT_DELTA m"
    echo "默认夹爪 replay 模式/Hz: $DEFAULT_GRIPPER_REPLAY_MODE / $DEFAULT_GRIPPER_COMMAND_HZ"
    echo "默认夹爪事件后 joint 暂停: $DEFAULT_GRIPPER_HOLD_SEC s"
    echo "默认夹爪 server: auto；可用 DEFAULT_GRIPPER_HOST/DEFAULT_GRIPPER_PORT 或 --gripper-host/--gripper-port 覆盖。"
    echo "默认起点自动慢速靠近: $DEFAULT_APPROACH_START"
    echo "默认起点靠近 max/step/hz: $DEFAULT_APPROACH_START_MAX_DELTA / $DEFAULT_APPROACH_START_STEP_DELTA / $DEFAULT_APPROACH_START_HZ"
    echo ""
    echo "常用示例:"
    echo "  只检查文件，不碰机器人:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0 --skip-robot-check"
    echo "  连接 robot node 做起点检查，但不发送 replay 命令:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0"
    echo "  真正执行 replay:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0 --execute"
    echo "  执行 replay，但禁止自动靠近 frame 0:"
    echo "    DEFAULT_APPROACH_START=0 bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0 --execute"
    echo "  直接传 metadata.json:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0/metadata.json --skip-robot-check"
    echo "  直接传裸 pkl.gz:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0/0.pkl.gz --skip-robot-check"
    echo "  右臂裸 pkl.gz，必须显式指定右臂:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0/0.pkl.gz --arm right --skip-robot-check"
    echo "  在某个质量目录下选择最新一条:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality --latest --skip-robot-check"
    echo "  在 Failure 目录下选择最新一条:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/Failure --latest --skip-robot-check"
    echo "  从 task 根目录选择最新一条，会跨质量桶，谨慎使用:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block --latest --skip-robot-check"
    echo "  强制指定右臂并慢速 replay:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0 --arm right --execute --speed 0.5"
    echo "  临时覆盖夹爪 server:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/High_Quality/0 --gripper-host 127.0.0.1 --gripper-port 50052"
    echo "  旧格式 episode:"
    echo "    bash 7_replay_fr3.sh /home/pnp/Desktop/franka_record_data/pick_block/3 --skip-robot-check"
    exit 0
fi

echo ">>> 激活 conda 环境 polymetis ..."
if command -v conda >/dev/null 2>&1; then
    CONDA_BASE="$(conda info --base)"
elif [[ -x "$HOME/miniconda3/bin/conda" ]]; then
    CONDA_BASE="$HOME/miniconda3"
elif [[ -x "/home/pnp/miniconda3/bin/conda" ]]; then
    CONDA_BASE="/home/pnp/miniconda3"
elif [[ -x "$HOME/anaconda3/bin/conda" ]]; then
    CONDA_BASE="$HOME/anaconda3"
else
    echo "❌ 找不到 conda，请确认 miniconda/anaconda 已安装"
    exit 1
fi
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败，请确认 polymetis 环境存在"; exit 1; }

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"

echo ">>> 使用仓库根目录 franka_replay ..."
echo ">>> 输入 episode: $1"
echo ">>> 默认轨迹速度: $DEFAULT_REPLAY_SPEED"
echo ">>> 默认夹爪 speed/force: $DEFAULT_GRIPPER_SPEED / $DEFAULT_GRIPPER_FORCE"
echo ">>> 默认夹爪事件阈值: $DEFAULT_GRIPPER_EVENT_DELTA m"
echo ">>> 默认夹爪 replay 模式/Hz: $DEFAULT_GRIPPER_REPLAY_MODE / $DEFAULT_GRIPPER_COMMAND_HZ"
echo ">>> 默认夹爪事件后 joint 暂停: $DEFAULT_GRIPPER_HOLD_SEC s"
if [[ -n "$DEFAULT_GRIPPER_HOST" || -n "$DEFAULT_GRIPPER_PORT" ]]; then
    echo ">>> 默认夹爪 server override: ${DEFAULT_GRIPPER_HOST:-auto}:${DEFAULT_GRIPPER_PORT:-auto}"
else
    echo ">>> 默认夹爪 server: auto"
fi
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

gripper_endpoint_args=()
if [[ -n "$DEFAULT_GRIPPER_HOST" ]]; then
    gripper_endpoint_args+=(--gripper-host "$DEFAULT_GRIPPER_HOST")
fi
if [[ -n "$DEFAULT_GRIPPER_PORT" ]]; then
    gripper_endpoint_args+=(--gripper-port "$DEFAULT_GRIPPER_PORT")
fi

cd "$REPO_ROOT"
exec python -m franka_replay.replay_fr3 \
    --speed "$DEFAULT_REPLAY_SPEED" \
    --gripper-speed "$DEFAULT_GRIPPER_SPEED" \
    --gripper-force "$DEFAULT_GRIPPER_FORCE" \
    --gripper-event-delta "$DEFAULT_GRIPPER_EVENT_DELTA" \
    --gripper-replay-mode "$DEFAULT_GRIPPER_REPLAY_MODE" \
    --gripper-command-hz "$DEFAULT_GRIPPER_COMMAND_HZ" \
    --gripper-hold-sec "$DEFAULT_GRIPPER_HOLD_SEC" \
    "${approach_args[@]}" \
    "${gripper_endpoint_args[@]}" \
    "$@"
