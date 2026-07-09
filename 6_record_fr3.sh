#!/bin/bash
set -e

FIXED_RECORDING_FPS=30
MUKA_NAS_ROOT="$HOME/Desktop/Muka_NAS"
DEFAULT_OUTPUT_ROOT="$MUKA_NAS_ROOT"

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

require_muka_nas_mounted() {
    local output_root="$1"
    case "$output_root" in
        "$MUKA_NAS_ROOT"|"$MUKA_NAS_ROOT"/*)
            if ! mountpoint -q "$MUKA_NAS_ROOT"; then
                echo "❌ 错误：NAS 未挂载到 $MUKA_NAS_ROOT，请先挂载 Muka_NAS。" >&2
                exit 1
            fi
            ;;
    esac
}

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

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    echo "用法: bash 6_record_fr3.sh <任务名> [保存根目录] [额外 record_fr3 参数...]"
    echo "默认保存根目录: $DEFAULT_OUTPUT_ROOT"
    echo "示例: bash 6_record_fr3.sh pick_block"
    echo "示例: bash 6_record_fr3.sh pick_block /home/pnp/Desktop/Muka_NAS --port 6001"
    echo "可选起始编号: bash 6_record_fr3.sh pick_block --index 10"
    echo "可选深度录制: bash 6_record_fr3.sh pick_block --enable-depth --depth-cameras middle,left_wrist"
    echo "关闭深度录制: bash 6_record_fr3.sh pick_block --no-depth"
    echo "固定录制频率: ${FIXED_RECORDING_FPS} Hz"
    echo "录制键位: s=开始/继续, w=暂停, e=保存当前episode并等待下一次s, d=丢弃当前episode, k=关键帧, q=保存并退出"
    exit 0
fi

TASK="$1"
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

require_muka_nas_mounted "$OUTPUT_ROOT"
mkdir -p "$OUTPUT_ROOT"

echo ">>> 激活 conda 环境 franka_capture ..."
source_conda
conda activate franka_capture || { echo "❌ 激活失败，请确认 franka_capture 环境存在"; exit 1; }
ensure_cv2_qt_fonts

echo ">>> 使用仓库根目录 franka_capture ..."
echo ">>> 任务名: $TASK"
echo ">>> 数据保存根目录: $OUTPUT_ROOT"
echo ">>> Episode 编号: 自动从当前最大编号继续"
echo ">>> 固定录制频率: ${FIXED_RECORDING_FPS} Hz"
echo ">>> 录制键位: s=开始/继续, w=暂停, e=保存当前episode, d=丢弃当前episode, k=关键帧, q=保存并退出"

cd "$REPO_ROOT"
python -m franka_capture.scripts.record_fr3 \
    --task "$TASK" \
    --output-root "$OUTPUT_ROOT" \
    "${EXTRA_ARGS[@]}"
