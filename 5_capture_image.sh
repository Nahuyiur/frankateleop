#!/bin/bash
set -e

echo ">>> 激活 conda 环境 franka_capture ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate franka_capture || { echo "❌ 激活失败，请确认 franka_capture 环境存在"; exit 1; }

ensure_cv2_qt_fonts() {
    local font_src=""
    local candidate
    for candidate in \
        /usr/share/fonts/truetype/dejavu \
        /usr/share/fonts/truetype/liberation \
        /usr/share/fonts/truetype/open-sans \
        /usr/share/fonts/truetype \
        /usr/share/fonts; do
        if [[ -d "$candidate" ]] && find "$candidate" -maxdepth 1 -type f \( -iname '*.ttf' -o -iname '*.otf' \) | grep -q .; then
            font_src="$candidate"
            break
        fi
    done

    if [[ -z "$font_src" ]]; then
        echo "⚠️ 未找到系统字体目录，OpenCV/Qt 可能继续提示字体 warning"
        return 0
    fi

    local cv2_font_dir
    cv2_font_dir="$(python - <<'PY'
import importlib.util
from pathlib import Path

spec = importlib.util.find_spec("cv2")
if spec and spec.origin:
    print(Path(spec.origin).resolve().parent / "qt" / "fonts")
PY
)"

    if [[ -z "$cv2_font_dir" ]]; then
        return 0
    fi

    if [[ -L "$cv2_font_dir" && ! -e "$cv2_font_dir" ]]; then
        rm -f "$cv2_font_dir"
    fi
    if [[ ! -e "$cv2_font_dir" ]]; then
        mkdir -p "$(dirname "$cv2_font_dir")"
        ln -s "$font_src" "$cv2_font_dir"
    fi
}

ensure_cv2_qt_fonts

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    echo "用法: bash 5_capture_image.sh [保存目录]"
    echo "默认保存目录: $HOME/Desktop/franka_capture_images"
    echo "示例: bash 5_capture_image.sh /home/pnp/camera_checks"
    exit 0
fi

CALL_DIR="$(pwd)"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
DEFAULT_SAVE_DIR="$HOME/Desktop/franka_capture_images"
SAVE_DIR="${1:-$DEFAULT_SAVE_DIR}"

if [[ "$SAVE_DIR" != /* ]]; then
    SAVE_DIR="$CALL_DIR/$SAVE_DIR"
fi

echo ">>> 使用仓库根目录 franka_capture ..."
echo ">>> 截图保存目录: $SAVE_DIR"

cd "$REPO_ROOT"
python -m franka_capture.scripts.capture_image --save-dir "$SAVE_DIR"
