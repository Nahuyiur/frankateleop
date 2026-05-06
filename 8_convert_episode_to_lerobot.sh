#!/bin/bash
set -e

echo ">>> 激活 conda 环境 data_convert ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate data_convert || { echo "❌ 激活失败，请确认 data_convert 环境存在"; exit 1; }
export PYTHONDONTWRITEBYTECODE=1

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    echo "用法: bash 8_convert_episode_to_lerobot.sh <episode目录或pkl.gz文件> [输出dataset目录] [额外参数...]"
    echo "默认输出目录: $HOME/Desktop/franka_lerobot_data/<task>_episode_<source_index>"
    echo "示例: bash 8_convert_episode_to_lerobot.sh /home/pnp/Desktop/franka_record_data/pick_block/3"
    echo "示例: bash 8_convert_episode_to_lerobot.sh /home/pnp/Desktop/franka_record_data/pick_block/3 /home/pnp/Desktop/franka_lerobot_data/debug --overwrite"
    echo "常用参数: --fps 10, --task-description pick_block, --overwrite"
    echo "旧 15fps 数据请显式加: --fps 15"
    exit 0
fi

check_deps() {
    python - <<'PY'
import importlib
import sys

missing = []
for name in ["numpy", "cv2", "imageio", "pandas", "pyarrow"]:
    try:
        importlib.import_module(name)
    except ImportError:
        missing.append(name)

if missing:
    print("❌ LeRobot 转换依赖缺失: " + ", ".join(missing))
    print("请安装到 data_convert 环境:")
    print("  conda activate data_convert")
    print("  python -m pip install --extra-index-url https://download.pytorch.org/whl/cpu -e '/home/pnp/lerobot[dataset]'")
    print("  python -m pip install 'imageio[ffmpeg]'")
    sys.exit(1)
PY
}

check_deps

EPISODE="$1"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
EXTRA_ARGS=()

if [[ "$#" -ge 2 && "${2:0:2}" != "--" ]]; then
    OUTPUT_ROOT="$2"
    if [[ "$#" -gt 2 ]]; then
        EXTRA_ARGS=("${@:3}")
    fi
else
    OUTPUT_ROOT=""
    if [[ "$#" -gt 1 ]]; then
        EXTRA_ARGS=("${@:2}")
    fi
fi

echo ">>> 使用仓库根目录 franka_lerobot ..."
echo ">>> 输入 episode: $EPISODE"
if [[ -n "$OUTPUT_ROOT" ]]; then
    echo ">>> 输出 dataset: $OUTPUT_ROOT"
else
    echo ">>> 输出 dataset: 使用默认路径"
fi

cd "$REPO_ROOT"
if [[ -n "$OUTPUT_ROOT" ]]; then
    python -m franka_lerobot.scripts.convert_episode "$EPISODE" --output-root "$OUTPUT_ROOT" "${EXTRA_ARGS[@]}"
else
    python -m franka_lerobot.scripts.convert_episode "$EPISODE" "${EXTRA_ARGS[@]}"
fi
