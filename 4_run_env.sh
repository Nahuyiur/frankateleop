#!/bin/bash
set -e

echo ">>> 激活 conda 环境 polymetis ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败，请确认 polymetis 环境存在"; exit 1; }

echo ">>> 使用仓库根目录 teleop ..."
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
POLY_ROOT="$REPO_ROOT/teleop"

if [[ -z "$POLY_ROOT" ]]; then
    echo "❌ 当前目录下未找到 teleop 文件夹"
    exit 1
fi

SCRIPT_PATH="$POLY_ROOT/experiments/run_env.py"
if [[ ! -f "$SCRIPT_PATH" ]]; then
    echo "❌ 未找到脚本：$SCRIPT_PATH"
    exit 1
fi

echo ">>> 启动Gripper 客户端 ..."

resolve_teleop_port() {
    local default_port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBJTKP2-if00-port0"
    local teleop_port="${LEFT_TELEOP_PORT:-${FRANKA_TELEOP_PORT:-${TELEOP_PORT:-$default_port}}}"

    if [[ ! -e "$teleop_port" ]]; then
        echo "❌ 串口不存在：$teleop_port"
        echo "   默认固定左臂同构臂串口：$default_port"
        echo "   如需临时覆盖，请设置 LEFT_TELEOP_PORT/FRANKA_TELEOP_PORT/TELEOP_PORT。"
        exit 1
    fi

    printf '%s\n' "$teleop_port"
}

TELEOP_PORT_RESOLVED="$(resolve_teleop_port)"
echo ">>> 使用同构臂串口：$TELEOP_PORT_RESOLVED"
python3 "$SCRIPT_PATH" --agent=teleop --teleop_port="$TELEOP_PORT_RESOLVED" "$@"
#如果要启用采集数据，需要在后面增加“--use_save_interface”
