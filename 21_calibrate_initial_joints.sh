#!/usr/bin/env bash
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"

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

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<'EOF'
用法:
  bash 21_calibrate_initial_joints.sh save --arm left --host 127.0.0.1 --port 6002
  bash 21_calibrate_initial_joints.sh save --arm right --host 192.168.1.131 --port 6001
  bash 21_calibrate_initial_joints.sh check --arm right --host 192.168.1.131 --port 6001
  bash 21_calibrate_initial_joints.sh validate

说明:
  这个脚本只读取当前 robot node 的 7 维 joint_positions 并保存/校验。
  它不会发送 robot/gripper 运动命令。
  默认配置文件: config/initial_joints.json
EOF
    exit 0
fi

echo ">>> 激活 conda 环境 franka_capture ..."
source_conda
conda activate franka_capture || { echo "❌ 激活失败，请确认 franka_capture 环境存在"; exit 1; }

cd "$REPO_ROOT"
python3 scripts/calibrate_initial_joints.py "$@"
