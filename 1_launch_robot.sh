#!/bin/bash
set -e

echo ">>> 激活 conda 环境 polymetis ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败"; exit 1; }

sudo_run() {
    if command -v sudo >/dev/null 2>&1; then
        if [[ -n "${FRANKA_SUDO_PASSWORD:-}" ]]; then
            printf '%s\n' "$FRANKA_SUDO_PASSWORD" | sudo -S -p '' "$@"
        else
            sudo "$@"
        fi
    else
        "$@"
    fi
}

install_sudo_wrapper() {
    [[ -z "${FRANKA_SUDO_PASSWORD:-}" ]] && return 0
    local sudo_bin
    sudo_bin="$(command -v sudo || true)"
    [[ -z "$sudo_bin" ]] && return 0

    local wrapper_dir
    wrapper_dir="$(mktemp -d /tmp/franka-sudo-wrapper.XXXXXX)"
    cat > "$wrapper_dir/sudo" <<EOF
#!/usr/bin/env bash
printf '%s\n' "\${FRANKA_SUDO_PASSWORD}" | "$sudo_bin" -S -p '' "\$@"
EOF
    chmod 700 "$wrapper_dir/sudo"
    export PATH="$wrapper_dir:$PATH"
}

echo ">>> 清理旧 run_server 进程 ..."
sudo_run pkill -9 run_server || echo "⚠️ 未发现 run_server 进程或无需清理"

POLY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)/polymetis"

WORK_DIR="$POLY_ROOT/polymetis/python/polymetis"
if [[ ! -d "$WORK_DIR/conf" ]]; then
    echo "❌ 配置目录 $WORK_DIR/conf 不存在"
    exit 1
fi

echo ">>> 启动Franka 客户端 ..."
cd "$WORK_DIR"
install_sudo_wrapper
python ../scripts/launch_robot.py robot_client=franka_hardware
