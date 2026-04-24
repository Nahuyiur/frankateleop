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

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    echo "用法: bash 6_record_fr3.sh <任务名> [保存根目录] [额外 record_fr3 参数...]"
    echo "默认保存根目录: $HOME/Desktop/franka_record_data"
    echo "示例: bash 6_record_fr3.sh pick_block"
    echo "示例: bash 6_record_fr3.sh pick_block /home/pnp/Desktop/franka_record_data --port 6001"
    echo "可选起始编号: bash 6_record_fr3.sh pick_block --index 10"
    echo "录制键位: s=开始/继续, w=暂停, e=保存当前episode并等待下一次s, k=关键帧, q=保存并退出"
    exit 0
fi

TASK="$1"
DEFAULT_OUTPUT_ROOT="$HOME/Desktop/franka_record_data"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"

EXTRA_ARGS=()
if [[ "$#" -ge 2 && "${2:0:2}" != "--" ]]; then
    OUTPUT_ROOT="$2"
    if [[ "$#" -gt 2 ]]; then
        EXTRA_ARGS=("${@:3}")
    fi
else
    OUTPUT_ROOT="$DEFAULT_OUTPUT_ROOT"
    if [[ "$#" -gt 1 ]]; then
        EXTRA_ARGS=("${@:2}")
    fi
fi

mkdir -p "$OUTPUT_ROOT"

echo ">>> 使用仓库根目录 franka_capture ..."
echo ">>> 任务名: $TASK"
echo ">>> 数据保存根目录: $OUTPUT_ROOT"
echo ">>> Episode 编号: 自动从当前最大编号继续"
echo ">>> 录制键位: s=开始/继续, w=暂停, e=保存当前episode, k=关键帧, q=保存并退出"

cd "$REPO_ROOT"
python -m franka_capture.scripts.record_fr3 \
    --task "$TASK" \
    --output-root "$OUTPUT_ROOT" \
    "${EXTRA_ARGS[@]}"
