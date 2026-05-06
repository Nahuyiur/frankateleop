#!/bin/bash
set -e

echo ">>> 激活 conda 环境 data_convert ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate data_convert || { echo "❌ 激活失败，请确认 data_convert 环境存在"; exit 1; }
export PYTHONDONTWRITEBYTECODE=1

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    echo "用法: bash 12_downsample_task_to_10hz.sh <输入task目录> [输出task目录] [额外参数...]"
    echo "默认输出目录: $HOME/Desktop/franka_record_data_10hz/<task>"
    echo "示例: bash 12_downsample_task_to_10hz.sh /home/pnp/Desktop/franka_record_data/put_eraser_into_drawer"
    echo "示例: bash 12_downsample_task_to_10hz.sh /home/pnp/Desktop/franka_record_data/put_eraser_into_drawer /home/pnp/Desktop/franka_record_data_10hz/put_eraser_into_drawer --overwrite"
    echo "常用参数: --camera right, --camera all, --source-fps 30, --target-fps 10, --overwrite"
    exit 0
fi

check_deps() {
    python - <<'PY'
import importlib
import sys

missing = []
for name in ["numpy", "cv2", "imageio"]:
    try:
        importlib.import_module(name)
    except ImportError:
        missing.append(name)

if missing:
    print("❌ 降采样依赖缺失: " + ", ".join(missing))
    print("请安装到 data_convert 环境:")
    print("  conda activate data_convert")
    print("  python -m pip install opencv-python 'imageio[ffmpeg]'")
    sys.exit(1)
PY
}

check_deps

INPUT_TASK="$1"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
EXTRA_ARGS=()

if [[ "$#" -ge 2 && "${2:0:2}" != "--" ]]; then
    OUTPUT_TASK="$2"
    if [[ "$#" -gt 2 ]]; then
        EXTRA_ARGS=("${@:3}")
    fi
else
    OUTPUT_TASK=""
    if [[ "$#" -gt 1 ]]; then
        EXTRA_ARGS=("${@:2}")
    fi
fi

echo ">>> 使用仓库根目录 franka_downsample ..."
echo ">>> 输入 task: $INPUT_TASK"
if [[ -n "$OUTPUT_TASK" ]]; then
    echo ">>> 输出 task: $OUTPUT_TASK"
else
    echo ">>> 输出 task: 使用默认路径"
fi

cd "$REPO_ROOT"
if [[ -n "$OUTPUT_TASK" ]]; then
    python -m franka_downsample.scripts.downsample_task "$INPUT_TASK" "$OUTPUT_TASK" "${EXTRA_ARGS[@]}"
else
    python -m franka_downsample.scripts.downsample_task "$INPUT_TASK" "${EXTRA_ARGS[@]}"
fi
