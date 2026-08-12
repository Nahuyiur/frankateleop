#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/franka_gui_bootstrap.sh
source "$REPO_ROOT/scripts/franka_gui_bootstrap.sh"
franka_gui_export_defaults
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    { set +x; } 2>/dev/null
    franka_gui_print_help single
    exit 0
fi
exec bash "$REPO_ROOT/run_data_collection_hub.sh" --preset-mode single "$@"
