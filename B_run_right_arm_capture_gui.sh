#!/usr/bin/env bash
set -e

MUKA_NAS_ROOT="$HOME/Desktop/Muka_NAS"
DEFAULT_OUTPUT_ROOT="$MUKA_NAS_ROOT"
DEFAULT_SUDO_PASSWORD_FILE="$HOME/.franka_gui_sudo_password"
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

if [[ -z "${FRANKA_GUI_SUDO_PASSWORD_FILE:-}" && -f "$DEFAULT_SUDO_PASSWORD_FILE" ]]; then
    export FRANKA_GUI_SUDO_PASSWORD_FILE="$DEFAULT_SUDO_PASSWORD_FILE"
fi

# Default to NAS staging with an atomic final publish. Set local-outbox only
# when deliberately using the legacy delayed sync workflow.
export FRANKA_GUI_STORAGE_MODE="${FRANKA_GUI_STORAGE_MODE:-direct-nas}"

export BI_ARM_RIGHT_HOST="${BI_ARM_RIGHT_HOST:-192.168.1.131}"
BI_ARM_RIGHT_SSH="${BI_ARM_RIGHT_SSH:-pnp@$BI_ARM_RIGHT_HOST}"
if [[ "$BI_ARM_RIGHT_SSH" != *@* ]]; then
    BI_ARM_RIGHT_SSH="pnp@$BI_ARM_RIGHT_SSH"
fi
export BI_ARM_RIGHT_SSH
export BI_ARM_RIGHT_REPO="${BI_ARM_RIGHT_REPO:-/home/pnp/frankateleop}"
export BI_ARM_RIGHT_LOCAL_ZMQ_PORT="${BI_ARM_RIGHT_LOCAL_ZMQ_PORT:-16001}"
export BI_ARM_RIGHT_REMOTE_ZMQ_PORT="${BI_ARM_RIGHT_REMOTE_ZMQ_PORT:-6001}"
export FRANKA_RIGHT_ZMQ_HOST="${FRANKA_RIGHT_ZMQ_HOST:-$BI_ARM_RIGHT_HOST}"
export FRANKA_RIGHT_ZMQ_PORT="${FRANKA_RIGHT_ZMQ_PORT:-$BI_ARM_RIGHT_REMOTE_ZMQ_PORT}"
export BI_ARM_RIGHT_ROBOTIQ_COMPORT="${BI_ARM_RIGHT_ROBOTIQ_COMPORT:-${RIGHT_ROBOTIQ_COMPORT:-}}"
export BI_ARM_SSH_PASSWORD="${BI_ARM_SSH_PASSWORD:-}"
export BI_ARM_LOCAL_SUDO_PASSWORD="${BI_ARM_LOCAL_SUDO_PASSWORD:-}"
export BI_ARM_REMOTE_SUDO_PASSWORD="${BI_ARM_REMOTE_SUDO_PASSWORD:-}"
export BI_ARM_STACK_ONLY=1
export BI_ARM_RIGHT_ONLY=1

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
用法:
  bash B_run_right_arm_capture_gui.sh [GUI参数...]

示例:
  bash B_run_right_arm_capture_gui.sh
  bash B_run_right_arm_capture_gui.sh --mock

B 右臂单臂 GUI 会优先尝试打开相机:
  middle,right,right_wrist
  缺失相机时仍可启动预览用于排查，但会拒绝开始录制，避免保存不完整数据。

相机布局:
  第一行: middle,right
  第二行: right_wrist

默认:
  固定录制频率=30 Hz
  固定保存根目录=$DEFAULT_OUTPUT_ROOT
  右机网络地址=$BI_ARM_RIGHT_HOST
  右机=$BI_ARM_RIGHT_SSH
  右机仓库=$BI_ARM_RIGHT_REPO
  右臂直连 ZMQ=$FRANKA_RIGHT_ZMQ_HOST:$FRANKA_RIGHT_ZMQ_PORT
  回滚到隧道: FRANKA_RIGHT_ZMQ_HOST=127.0.0.1 FRANKA_RIGHT_ZMQ_PORT=$BI_ARM_RIGHT_LOCAL_ZMQ_PORT
  右臂本地 ZMQ 隧道端口=$BI_ARM_RIGHT_LOCAL_ZMQ_PORT
  右臂 Robotiq 串口=自动检测；如需固定，设置 BI_ARM_RIGHT_ROBOTIQ_COMPORT
EOF
    exit 0
fi

echo ">>> 激活 conda 环境 franka_capture ..."
source_conda
conda activate franka_capture || { echo "❌ 激活失败，请确认 franka_capture 环境存在"; exit 1; }

python - <<'PY'
missing = []
for module in ("PyQt6", "pyrealsense2", "cv2", "numpy", "zmq"):
    try:
        __import__(module)
    except ImportError:
        missing.append(module)
if missing:
    print("❌ franka_capture 环境缺少 GUI 依赖:", ", ".join(missing))
    print("建议安装: pip install PyQt6 opencv-python pyzmq numpy")
    print("RealSense 依赖按本机环境安装 pyrealsense2。")
    raise SystemExit(1)
PY

if [[ "${QT_QPA_PLATFORM:-}" != "offscreen" && "${XDG_SESSION_TYPE:-}" == "x11" ]]; then
    if ! ldconfig -p 2>/dev/null | grep -q 'libxcb-cursor\.so\.0'; then
        cat <<'EOF'
❌ 当前是 X11 桌面会话，但系统缺少 Qt6 需要的 libxcb-cursor0。

请先运行：
  sudo apt-get update
  sudo apt-get install -y libxcb-cursor0

安装完成后重新启动：
  bash B_run_right_arm_capture_gui.sh
EOF
        exit 1
    fi
fi

echo ">>> 使用仓库根目录 franka_gui ..."
echo ">>> 默认保存根目录: $DEFAULT_OUTPUT_ROOT"
echo ">>> 保存模式: ${FRANKA_GUI_STORAGE_MODE}（默认 NAS staging，完成后原子发布）"
echo ">>> 固定录制频率: 30 Hz"
echo ">>> B 右臂单臂相机: middle,right,right_wrist；缺失时可预览但禁止录制"
echo ">>> 右臂直连 ZMQ: $FRANKA_RIGHT_ZMQ_HOST:$FRANKA_RIGHT_ZMQ_PORT"
echo ">>> 右臂本地 ZMQ 隧道端口(回滚用): $BI_ARM_RIGHT_LOCAL_ZMQ_PORT"
echo ">>> 右机网络地址: $BI_ARM_RIGHT_HOST"
echo ">>> 右机: $BI_ARM_RIGHT_SSH"
echo ">>> 右机仓库: $BI_ARM_RIGHT_REPO"
if [[ -n "${BI_ARM_RIGHT_ROBOTIQ_COMPORT:-}" ]]; then
    echo ">>> 右臂 Robotiq 串口: $BI_ARM_RIGHT_ROBOTIQ_COMPORT"
else
    echo ">>> 右臂 Robotiq 串口: 自动检测远端唯一 RS485 设备"
fi
if [[ -n "${FRANKA_GUI_SUDO_PASSWORD_FILE:-}" ]]; then
    echo ">>> sudo 密码文件: $FRANKA_GUI_SUDO_PASSWORD_FILE"
fi

cd "$REPO_ROOT"
python -m franka_gui.app \
    --mode right \
    --profile-key right \
    --camera-names middle,right,right_wrist \
    --right-host "$FRANKA_RIGHT_ZMQ_HOST" \
    --right-port "$FRANKA_RIGHT_ZMQ_PORT" \
    "$@"
