#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

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
        echo "❌ 错误：未找到 conda"
        exit 1
    fi
    source "$CONDA_BASE/etc/profile.d/conda.sh"
}

echo ">>> 激活 conda 环境 polymetis ..."
source_conda
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

PORT_RESOLVER="$REPO_ROOT/scripts/resolve_left_teleop_port.sh"
if [[ ! -f "$PORT_RESOLVER" ]]; then
    echo "❌ 未找到左臂同构臂串口解析脚本：$PORT_RESOLVER" >&2
    exit 1
fi
TELEOP_PORT_RESOLVED="$(bash "$PORT_RESOLVER")"

echo ">>> 启动同构臂 Teleop 客户端 ..."
echo ">>> 使用同构臂串口：$TELEOP_PORT_RESOLVED"
python3 "$SCRIPT_PATH" --agent=teleop --tele_port=6002 --teleop_port="$TELEOP_PORT_RESOLVED" "$@"
#如果要启用采集数据，需要在后面增加“--use_save_interface”
