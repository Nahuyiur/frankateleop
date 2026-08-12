#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/franka_gui_bootstrap.sh
source "$REPO_ROOT/scripts/franka_gui_bootstrap.sh"
franka_gui_source_conda
cd "$REPO_ROOT"
exec python -m data_review.window "$@"
