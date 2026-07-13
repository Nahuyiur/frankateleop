#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
PYTHON_BIN="${FRANKA_SYNC_PYTHON:-python3}"

cd "$REPO_ROOT"
if command -v ionice >/dev/null 2>&1; then
    exec ionice -c 3 nice -n 15 "$PYTHON_BIN" -m franka_sync "$@"
fi
exec nice -n 15 "$PYTHON_BIN" -m franka_sync "$@"
