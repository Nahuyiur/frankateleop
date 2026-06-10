#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"

echo ">>> 14_run_capture_gui.sh 已更名为 A_run_single_arm_capture_gui.sh"
echo ">>> 正在转发到新的单臂三相机 GUI 入口 ..."

exec bash "$SCRIPT_DIR/A_run_single_arm_capture_gui.sh" "$@"
