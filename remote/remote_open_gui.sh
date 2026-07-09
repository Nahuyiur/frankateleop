#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
    cat <<'EOF'
Usage:
  bash remote/remote_open_gui.sh A [extra GUI args...]
  bash remote/remote_open_gui.sh B [extra GUI args...]
  bash remote/remote_open_gui.sh C [extra GUI args...]

This is the stable server-side entrypoint for Xpra-launched GUI capture.
It intentionally only maps A/B/C to the current GUI scripts in this repo.
All capture behavior remains in A/B/C_run_*_gui.sh.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    usage
    exit 0
fi

mode="$1"
shift
mode_key="$(printf "%s" "$mode" | tr '[:upper:]' '[:lower:]')"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "$mode_key" in
    a|left|single)
        script="$repo_root/A_run_left_arm_capture_gui.sh"
        ;;
    b|right)
        script="$repo_root/B_run_right_arm_capture_gui.sh"
        ;;
    c|dual|bi|biarm|bi_arm)
        script="$repo_root/C_run_dual_arm_capture_gui.sh"
        ;;
    *)
        echo "ERROR: unknown GUI mode: $mode" >&2
        usage >&2
        exit 2
        ;;
esac

if [[ ! -x "$script" ]]; then
    echo "ERROR: GUI script is not executable or missing: $script" >&2
    exit 1
fi

export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"

echo ">>> Remote GUI entrypoint: $script"
echo ">>> DISPLAY: ${DISPLAY:-<unset>}"
exec bash "$script" "$@"
