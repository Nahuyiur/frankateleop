#!/usr/bin/env bash
set -e

DEFAULT_OUTPUT_ROOT="$HOME/Desktop/franka_record_data"
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

export BI_ARM_RIGHT_SSH="${BI_ARM_RIGHT_SSH:-192.168.1.131}"
export BI_ARM_RIGHT_REPO="${BI_ARM_RIGHT_REPO:-/home/pnp/frankateleop}"
export BI_ARM_SSH_PASSWORD="${BI_ARM_SSH_PASSWORD:-}"
export BI_ARM_LOCAL_SUDO_PASSWORD="${BI_ARM_LOCAL_SUDO_PASSWORD:-}"
export BI_ARM_REMOTE_SUDO_PASSWORD="${BI_ARM_REMOTE_SUDO_PASSWORD:-}"
export BI_ARM_STACK_ONLY=1

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
用法:
  bash B_run_bi_arm_capture_gui.sh [GUI参数...]

示例:
  bash B_run_bi_arm_capture_gui.sh
  bash B_run_bi_arm_capture_gui.sh --mock

双臂 GUI 使用全部配置相机:
  left_wrist,left,middle,right,right_wrist

默认:
  固定录制频率=30 Hz
  固定保存根目录=$DEFAULT_OUTPUT_ROOT
  右机=$BI_ARM_RIGHT_SSH
  右机仓库=$BI_ARM_RIGHT_REPO
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
  bash B_run_bi_arm_capture_gui.sh
EOF
        exit 1
    fi
fi

echo ">>> 使用仓库根目录 franka_gui ..."
echo ">>> 默认保存根目录: $DEFAULT_OUTPUT_ROOT"
echo ">>> 固定录制频率: 30 Hz"
echo ">>> 双臂相机: 全部配置相机"
echo ">>> 右机: $BI_ARM_RIGHT_SSH"
echo ">>> 右机仓库: $BI_ARM_RIGHT_REPO"
if [[ -n "${FRANKA_GUI_SUDO_PASSWORD_FILE:-}" ]]; then
    echo ">>> sudo 密码文件: $FRANKA_GUI_SUDO_PASSWORD_FILE"
fi

cd "$REPO_ROOT"
python -m franka_gui.app --mode dual "$@"
