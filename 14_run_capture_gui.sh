#!/bin/bash
set -e

echo ">>> 激活 conda 环境 franka_capture ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate franka_capture || { echo "❌ 激活失败，请确认 franka_capture 环境存在"; exit 1; }

DEFAULT_OUTPUT_ROOT="$HOME/Desktop/franka_record_data"
DEFAULT_SUDO_PASSWORD_FILE="$HOME/.franka_gui_sudo_password"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"

if [[ -z "${FRANKA_GUI_SUDO_PASSWORD_FILE:-}" && -f "$DEFAULT_SUDO_PASSWORD_FILE" ]]; then
    export FRANKA_GUI_SUDO_PASSWORD_FILE="$DEFAULT_SUDO_PASSWORD_FILE"
fi

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<EOF
用法:
  bash 14_run_capture_gui.sh [GUI参数...]

示例:
  bash 14_run_capture_gui.sh
  bash 14_run_capture_gui.sh --mock

快捷键:
  s 开始/继续, w 暂停, e 保存当前, d 丢弃当前, k 关键帧, q 保存当前

默认:
  固定录制频率=30 Hz
  固定保存根目录=$DEFAULT_OUTPUT_ROOT
EOF
    exit 0
fi

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
  bash 14_run_capture_gui.sh
EOF
        exit 1
    fi
fi

echo ">>> 使用仓库根目录 franka_gui ..."
echo ">>> 默认保存根目录: $DEFAULT_OUTPUT_ROOT"
echo ">>> 固定录制频率: 30 Hz"
if [[ -n "${FRANKA_GUI_SUDO_PASSWORD_FILE:-}" ]]; then
    echo ">>> sudo 密码文件: $FRANKA_GUI_SUDO_PASSWORD_FILE"
fi

cd "$REPO_ROOT"
python -m franka_gui.app "$@"
