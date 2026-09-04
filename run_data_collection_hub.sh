#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/franka_gui_bootstrap.sh
source "$REPO_ROOT/scripts/franka_gui_bootstrap.sh"

franka_gui_reexec_with_dialout "$REPO_ROOT/run_data_collection_hub.sh" "$@"
franka_gui_export_defaults
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    franka_gui_print_help hub
    exit 0
fi
echo ">>> 激活 conda 环境 franka_capture ..."
franka_gui_source_conda
franka_gui_check_dependencies
echo ">>> 启动 Franka 数据采集中心"
echo ">>> 保存模式: $FRANKA_GUI_STORAGE_MODE"
echo ">>> 右臂服务: $FRANKA_RIGHT_ZMQ_HOST:$FRANKA_RIGHT_ZMQ_PORT"
cd "$REPO_ROOT"
exec python -m franka_gui.hub "$@"
