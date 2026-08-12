#!/bin/bash
set -e

echo ">>> 激活 conda 环境 polymetis ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate polymetis || { echo "❌ 激活失败"; exit 1; }

sudo_run() {
    if command -v sudo >/dev/null 2>&1; then
        if sudo -n true >/dev/null 2>&1; then
            sudo -n "$@"
            return
        fi
        local sudo_password="${FRANKA_SUDO_PASSWORD:-}"
        if [[ -z "$sudo_password" && -n "${FRANKA_SUDO_PASSWORD_FILE:-}" ]]; then
            validate_sudo_password_file "$FRANKA_SUDO_PASSWORD_FILE" || return 1
            IFS= read -r sudo_password < "$FRANKA_SUDO_PASSWORD_FILE" || true
        fi
        if [[ -n "$sudo_password" ]]; then
            printf '%s\n' "$sudo_password" | sudo -S -p '' "$@"
        else
            sudo "$@"
        fi
    else
        "$@"
    fi
}

validate_sudo_password_file() {
    local file="$1"
    local mode owner
    [[ -f "$file" ]] || { echo "ERROR: sudo credential file not found: $file" >&2; return 1; }
    mode="$(stat -c '%a' "$file")"
    owner="$(stat -c '%u' "$file")"
    [[ "$owner" == "$(id -u)" && ( "$mode" == "600" || "$mode" == "400" ) ]] || {
        echo "ERROR: sudo credential file must be owned by the current user with mode 600 or 400: $file" >&2
        return 1
    }
}

install_sudo_wrapper() {
    if command -v sudo >/dev/null 2>&1 && sudo -n true >/dev/null 2>&1; then
        return 0
    fi
    if [[ -z "${FRANKA_SUDO_PASSWORD_FILE:-}" && -n "${FRANKA_SUDO_PASSWORD:-}" ]]; then
        FRANKA_SUDO_PASSWORD_FILE="$(mktemp /tmp/franka-sudo-password.XXXXXX)"
        chmod 600 "$FRANKA_SUDO_PASSWORD_FILE"
        printf '%s\n' "$FRANKA_SUDO_PASSWORD" > "$FRANKA_SUDO_PASSWORD_FILE"
        export FRANKA_SUDO_PASSWORD_FILE
        unset FRANKA_SUDO_PASSWORD
    fi
    [[ -z "${FRANKA_SUDO_PASSWORD_FILE:-}" ]] && return 0
    validate_sudo_password_file "$FRANKA_SUDO_PASSWORD_FILE"
    local sudo_bin
    sudo_bin="$(command -v sudo || true)"
    [[ -z "$sudo_bin" ]] && return 0

    local wrapper_dir
    wrapper_dir="$(mktemp -d /tmp/franka-sudo-wrapper.XXXXXX)"
    umask 077
    cat > "$wrapper_dir/sudo" <<EOF
#!/usr/bin/env bash
mode="\$(stat -c '%a' "\${FRANKA_SUDO_PASSWORD_FILE}")"
owner="\$(stat -c '%u' "\${FRANKA_SUDO_PASSWORD_FILE}")"
[[ "\$owner" == "\$(id -u)" && ( "\$mode" == "600" || "\$mode" == "400" ) ]] || exit 1
IFS= read -r sudo_password < "\${FRANKA_SUDO_PASSWORD_FILE}"
printf '%s\n' "\$sudo_password" | "$sudo_bin" -S -p '' "\$@"
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
