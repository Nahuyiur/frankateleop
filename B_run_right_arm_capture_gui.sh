#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/franka_gui_bootstrap.sh
source "$REPO_ROOT/scripts/franka_gui_bootstrap.sh"
franka_gui_export_defaults
# The right Franka is physically connected to this 67/muka host.  Right-mode
# capture must use the local Robot Node instead of the legacy remote pnp host.
export FRANKA_RIGHT_ZMQ_HOST="${FRANKA_RIGHT_LOCAL_GUI_HOST:-127.0.0.1}"
export FRANKA_RIGHT_ZMQ_PORT="${FRANKA_RIGHT_LOCAL_GUI_PORT:-6001}"
export BI_ARM_RIGHT_HOST="$FRANKA_RIGHT_ZMQ_HOST"
export BI_ARM_RIGHT_SSH="muka@$FRANKA_RIGHT_ZMQ_HOST"
export BI_ARM_RIGHT_REPO="$REPO_ROOT"
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    { set +x; } 2>/dev/null
    franka_gui_print_help right
    exit 0
fi
exec bash "$REPO_ROOT/run_data_collection_hub.sh" --preset-mode right "$@"
