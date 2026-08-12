#!/usr/bin/env bash

FRANKA_GUI_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/franka_runtime_config.sh
source "$FRANKA_GUI_SCRIPTS_DIR/franka_runtime_config.sh"

franka_gui_reexec_with_dialout() {
    local script_path="$1"
    shift
    if [[ "${FRANKA_GUI_DIALOUT_REEXEC:-}" == "1" ]]; then
        return 0
    fi
    local current_user="${USER:-$(id -un)}"
    local account_groups=""
    local current_groups=""
    account_groups="$(id -nG "$current_user" 2>/dev/null || true)"
    current_groups="$(id -nG 2>/dev/null || true)"
    if [[ " $account_groups " != *" dialout "* || " $current_groups " == *" dialout "* ]]; then
        return 0
    fi
    if ! command -v sg >/dev/null 2>&1; then
        return 0
    fi
    local quoted_cmd=""
    quoted_cmd="$(printf '%q ' "$script_path" "$@")"
    echo ">>> 当前会话未继承 dialout 组，使用 sg dialout 重新启动统一 GUI ..."
    exec sg dialout -c "FRANKA_GUI_DIALOUT_REEXEC=1 $quoted_cmd"
}

franka_gui_source_conda() {
    local conda_base=""
    if command -v conda >/dev/null 2>&1; then
        conda_base="$(conda info --base)"
    elif [[ -x "$HOME/miniconda3/bin/conda" ]]; then
        conda_base="$HOME/miniconda3"
    elif [[ -x "/home/pnp/miniconda3/bin/conda" ]]; then
        conda_base="/home/pnp/miniconda3"
    elif [[ -x "$HOME/anaconda3/bin/conda" ]]; then
        conda_base="$HOME/anaconda3"
    else
        echo "ERROR: 未找到 conda" >&2
        return 1
    fi
    # shellcheck source=/dev/null
    source "$conda_base/etc/profile.d/conda.sh"
    conda activate franka_capture || {
        echo "ERROR: 激活 franka_capture 环境失败" >&2
        return 1
    }
}

franka_gui_check_dependencies() {
    python - <<'PY'
missing = []
for module in ("PyQt6", "pyrealsense2", "cv2", "numpy", "zmq"):
    try:
        __import__(module)
    except ImportError:
        missing.append(module)
if missing:
    print("ERROR: franka_capture 环境缺少 GUI 依赖:", ", ".join(missing))
    raise SystemExit(1)
PY

    if [[ "${QT_QPA_PLATFORM:-}" != "offscreen" && "${XDG_SESSION_TYPE:-}" == "x11" ]]; then
        if ! ldconfig -p 2>/dev/null | grep 'libxcb-cursor\.so\.0' >/dev/null; then
            echo "ERROR: X11 桌面缺少 Qt6 依赖 libxcb-cursor0" >&2
            echo "请运行: sudo apt-get install -y libxcb-cursor0" >&2
            return 1
        fi
    fi
}

franka_gui_export_defaults() {
    franka_runtime_export_defaults
    local password_file="$HOME/.franka_gui_sudo_password"
    local right_identity_file="$HOME/.ssh/frankateleop_right_ed25519"
    if [[ -z "${FRANKA_GUI_SUDO_PASSWORD_FILE:-}" && -f "$password_file" ]]; then
        export FRANKA_GUI_SUDO_PASSWORD_FILE="$password_file"
    fi
    export BI_ARM_REMOTE_SUDO_PASSWORD_FILE="${BI_ARM_REMOTE_SUDO_PASSWORD_FILE:-/home/pnp/.franka_gui_sudo_password}"
    if [[ -z "${BI_ARM_SSH_IDENTITY_FILE:-}" && -f "$right_identity_file" ]]; then
        export BI_ARM_SSH_IDENTITY_FILE="$right_identity_file"
    fi
    export FRANKA_GUI_STORAGE_MODE="${FRANKA_GUI_STORAGE_MODE:-direct-nas}"
    export BI_ARM_RIGHT_ROBOTIQ_COMPORT="${BI_ARM_RIGHT_ROBOTIQ_COMPORT:-${RIGHT_ROBOTIQ_COMPORT:-}}"
    export BI_ARM_LEFT_ROBOTIQ_COMPORT="${BI_ARM_LEFT_ROBOTIQ_COMPORT:-${LEFT_ROBOTIQ_COMPORT:-}}"
    export BI_ARM_SSH_PASSWORD="${BI_ARM_SSH_PASSWORD:-}"
    export BI_ARM_LOCAL_SUDO_PASSWORD_FILE="${BI_ARM_LOCAL_SUDO_PASSWORD_FILE:-${FRANKA_GUI_SUDO_PASSWORD_FILE:-$HOME/.franka_gui_sudo_password}}"
    export BI_ARM_STACK_ONLY=1
    export BI_ARM_RIGHT_ONLY=0
}

franka_gui_print_help() {
    local mode="${1:-hub}"
    local command="run_data_collection_hub.sh"
    local cameras="在首页选择左臂、右臂或双臂"
    case "$mode" in
        single)
            command="A_run_left_arm_capture_gui.sh"
            cameras="left_wrist,left,middle"
            ;;
        right)
            command="B_run_right_arm_capture_gui.sh"
            cameras="middle,right,right_wrist"
            ;;
        dual)
            command="C_run_dual_arm_capture_gui.sh"
            cameras="全部配置相机"
            ;;
    esac
    cat <<EOF
用法:
  bash $command [GUI参数...]

统一入口会要求确认数采员姓名，并在同一应用中打开采集、工时监控和数据复核。
预选相机: $cameras

默认:
  固定录制频率=30 Hz
  固定保存根目录=$HOME/Desktop/Muka_NAS
  保存模式=$FRANKA_GUI_STORAGE_MODE
  右机网络地址=$BI_ARM_RIGHT_HOST
  右机=$BI_ARM_RIGHT_SSH
  右机仓库=$BI_ARM_RIGHT_REPO
  左臂 ZMQ 端口=$BI_ARM_LEFT_ZMQ_PORT
  右臂直连 ZMQ=$FRANKA_RIGHT_ZMQ_HOST:$FRANKA_RIGHT_ZMQ_PORT
  右臂本地 ZMQ 隧道端口=$BI_ARM_RIGHT_LOCAL_ZMQ_PORT

常用参数:
  --mock
  --storage-mode {direct-nas,local-outbox}
  --open-monitor
  --open-review
EOF
}
