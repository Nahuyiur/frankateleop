#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"

echo ">>> 14_run_capture_gui.sh 是统一采集 GUI 的兼容入口"
echo ">>> 确认数采员后请选择左臂、右臂或双臂模式 ..."

exec bash "$SCRIPT_DIR/A_run_left_arm_capture_gui.sh" "$@"
