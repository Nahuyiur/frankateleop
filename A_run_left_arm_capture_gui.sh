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

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
用法:
  bash A_run_left_arm_capture_gui.sh [GUI参数...]

示例:
  bash A_run_left_arm_capture_gui.sh
  bash A_run_left_arm_capture_gui.sh --mock

A 左臂单臂 GUI 会优先尝试打开相机:
  left_wrist,left,middle
  缺失相机时仍可启动预览用于排查，但会拒绝开始录制，避免保存不完整数据。

默认:
  固定录制频率=30 Hz
  固定保存根目录=$DEFAULT_OUTPUT_ROOT
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
  bash A_run_left_arm_capture_gui.sh
EOF
        exit 1
    fi
fi

echo ">>> 使用仓库根目录 franka_gui ..."
echo ">>> 默认保存根目录: $DEFAULT_OUTPUT_ROOT"
echo ">>> 保存模式: ${FRANKA_GUI_STORAGE_MODE}（默认 NAS staging，完成后原子发布）"
echo ">>> 固定录制频率: 30 Hz"
echo ">>> A 左臂单臂相机: left_wrist,left,middle；缺失时可预览但禁止录制"
if [[ -n "${FRANKA_GUI_SUDO_PASSWORD_FILE:-}" ]]; then
    echo ">>> sudo 密码文件: $FRANKA_GUI_SUDO_PASSWORD_FILE"
fi

cd "$REPO_ROOT"
python -m franka_gui.app \
    --mode single \
    --profile-key left \
    --camera-names left_wrist,left,middle \
    "$@"
