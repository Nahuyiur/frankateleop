#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/franka_gui_bootstrap.sh
source "$REPO_ROOT/scripts/franka_gui_bootstrap.sh"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<'EOF'
用法:
  bash M_run_worktime_monitor.sh

打开只读工时监控页面。该页面读取本地 SQLite 账本，不扫描 NAS，也不启动机械臂。
EOF
    exit 0
fi
franka_gui_source_conda
cd "$REPO_ROOT"
exec python -m work_monitor.dashboard "$@"
