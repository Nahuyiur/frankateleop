#!/bin/bash
set -e

echo ">>> 激活 conda 环境 data_convert ..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate data_convert || { echo "❌ 激活失败，请确认 data_convert 环境存在"; exit 1; }
export PYTHONDONTWRITEBYTECODE=1

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    echo "用法: bash 10_convert_episode_to_hdf5.sh <episode目录或pkl.gz文件> [输出hdf5文件] [额外参数...]"
    echo "默认输出文件: $HOME/Desktop/franka_hdf5_data/<task>_episode_<source_index>.hdf5"
    echo "示例: bash 10_convert_episode_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block/3"
    echo "示例: bash 10_convert_episode_to_hdf5.sh /home/pnp/Desktop/franka_record_data/pick_block/3 /home/pnp/Desktop/debug.hdf5 --overwrite"
    echo "常用参数: --camera right, --fps 10, --task-description pick_block, --compression gzip, --overwrite"
    echo "旧 15fps 数据请显式加: --fps 15"
    exit 0
fi

check_deps() {
    python - <<'PY'
import importlib
import sys

missing = []
for name in ["numpy", "h5py"]:
    try:
        importlib.import_module(name)
    except ImportError:
        missing.append(name)

if missing:
    print("❌ HDF5 转换依赖缺失: " + ", ".join(missing))
    print("请安装到 data_convert 环境:")
    print("  conda activate data_convert")
    print("  python -m pip install h5py")
    sys.exit(1)
PY
}

check_deps

EPISODE="$1"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
EXTRA_ARGS=()

if [[ "$#" -ge 2 && "${2:0:2}" != "--" ]]; then
    OUTPUT_FILE="$2"
    if [[ "$#" -gt 2 ]]; then
        EXTRA_ARGS=("${@:3}")
    fi
else
    OUTPUT_FILE=""
    if [[ "$#" -gt 1 ]]; then
        EXTRA_ARGS=("${@:2}")
    fi
fi

echo ">>> 使用仓库根目录 franka_hdf5 ..."
echo ">>> 输入 episode: $EPISODE"
if [[ -n "$OUTPUT_FILE" ]]; then
    echo ">>> 输出 HDF5: $OUTPUT_FILE"
else
    echo ">>> 输出 HDF5: 使用默认路径"
fi

cd "$REPO_ROOT"
if [[ -n "$OUTPUT_FILE" ]]; then
    python -m franka_hdf5.scripts.convert_episode "$EPISODE" --output-file "$OUTPUT_FILE" "${EXTRA_ARGS[@]}"
else
    python -m franka_hdf5.scripts.convert_episode "$EPISODE" "${EXTRA_ARGS[@]}"
fi
