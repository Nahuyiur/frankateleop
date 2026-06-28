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

resolve_teleop_port() {
    local default_ports=(
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBJKECV-if00-port0"
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAUMOPA-if00-port0"
    )
    local teleop_port="${RIGHT_TELEOP_PORT:-${FRANKA_TELEOP_PORT:-${TELEOP_PORT:-}}}"

    if [[ -z "$teleop_port" ]]; then
        local default_port
        for default_port in "${default_ports[@]}"; do
            if [[ -e "$default_port" ]]; then
                teleop_port="$default_port"
                break
            fi
        done
    fi

    if [[ -z "$teleop_port" ]]; then
        local ftdi_ports=()
        if [[ -d /dev/serial/by-id ]]; then
            mapfile -t ftdi_ports < <(
                find /dev/serial/by-id -maxdepth 1 -type l -name 'usb-FTDI_USB__-__Serial_Converter_*' | sort
            )
        fi
        if [[ "${#ftdi_ports[@]}" -eq 1 ]]; then
            teleop_port="${ftdi_ports[0]}"
            echo ">>> 未找到右臂默认串口，使用当前唯一同构臂 FTDI 作为实际串口：$teleop_port" >&2
            echo ">>> 如需强制右臂 mapping，请设置 RIGHT_TELEOP_CONFIG_PORT。" >&2
        else
            echo "❌ 未找到右臂同构臂默认串口，且无法唯一确定实际同构臂串口。" >&2
            echo ">>> 右臂默认候选:" >&2
            printf '  %s\n' "${default_ports[@]}" >&2
            echo ">>> 当前检测到的同构臂 FTDI 串口:" >&2
            printf '  %s\n' "${ftdi_ports[@]}" >&2
            if [[ -d /dev/serial/by-id ]]; then
                echo ">>> 当前 /dev/serial/by-id:" >&2
                find /dev/serial/by-id -maxdepth 1 -type l -printf '  %p -> %l\n' 2>/dev/null | sort >&2 || true
            fi
            echo ">>> 请显式设置 BI_ARM_RIGHT_TELEOP_PORT 或 RIGHT_TELEOP_PORT。" >&2
            exit 1
        fi
    fi

    if [[ ! -e "$teleop_port" ]]; then
        echo "❌ 串口不存在：$teleop_port" >&2
        exit 1
    fi

    printf '%s\n' "$teleop_port"
}

TELEOP_PORT_RESOLVED="$(resolve_teleop_port)"
TELEOP_CONFIG_PORT_RESOLVED="${RIGHT_TELEOP_CONFIG_PORT:-${FRANKA_TELEOP_CONFIG_PORT:-${TELEOP_CONFIG_PORT:-}}}"
TELEOP_ZMQ_HOST="${RIGHT_TELEOP_ZMQ_HOST:-${FRANKA_TELEOP_ZMQ_HOST:-127.0.0.1}}"
TELEOP_ZMQ_PORT="${RIGHT_TELEOP_ZMQ_PORT:-${FRANKA_TELEOP_ZMQ_PORT:-6001}}"
export TELEOP_DEBUG_ACTION="${TELEOP_DEBUG_ACTION:-1}"
export TELEOP_DEBUG_INTERVAL_SEC="${TELEOP_DEBUG_INTERVAL_SEC:-1.0}"

echo ">>> 启动同构臂 teleop 客户端 ..."
echo ">>> 使用同构臂串口：$TELEOP_PORT_RESOLVED"
if [[ -n "$TELEOP_CONFIG_PORT_RESOLVED" ]]; then
    echo ">>> 使用同构臂 mapping：$TELEOP_CONFIG_PORT_RESOLVED"
else
    echo ">>> 使用同构臂 mapping：跟随实际串口"
fi
echo ">>> 使用 robot ZMQ：$TELEOP_ZMQ_HOST:$TELEOP_ZMQ_PORT"
if [[ "$TELEOP_PORT_RESOLVED" == *"FTBJTKP2"* && -z "$TELEOP_CONFIG_PORT_RESOLVED" ]]; then
    echo ">>> 注意：正在按显式配置使用 FTBJTKP2；源码里该串口原先标注为 left fr3。"
fi
extra_args=()
if [[ -n "$TELEOP_CONFIG_PORT_RESOLVED" ]]; then
    extra_args+=(--teleop_config_port="$TELEOP_CONFIG_PORT_RESOLVED")
fi
python3 "$SCRIPT_PATH" --agent=teleop --hostname="$TELEOP_ZMQ_HOST" --tele_port="$TELEOP_ZMQ_PORT" --teleop_port="$TELEOP_PORT_RESOLVED" "${extra_args[@]}" "$@"
#如果要启用采集数据，需要在后面增加“--use_save_interface”
