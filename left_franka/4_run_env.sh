#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo ">>> 激活 conda 环境 polymetis ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败，请确认 polymetis 环境存在"; exit 1; }

echo ">>> 使用仓库根目录 teleop ..."
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
python3 "$SCRIPT_PATH" --agent=teleop --tele_port=6002 --teleop_port=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB2UWSZ-if00-port0
#如果要启用采集数据，需要在后面增加“--use_save_interface”
