#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="/home/muka/frankateleop"
PYTHON_BIN="/home/muka/miniconda3/envs/franka_capture/bin/python"

cd "$REPO_DIR"
export DISPLAY="${DISPLAY:-:0}"

echo "三相机预览将打开：left_wrist / left / middle"
echo "按键：S 保存三路 RGB-D；D 切换 RGB/深度；Q 或 Esc 退出"
exec "$PYTHON_BIN" -m pointcloud.calibration.preview_multicamera_charuco "$@"
