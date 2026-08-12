#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/franka_runtime_config.sh
source "$REPO_ROOT/scripts/franka_runtime_config.sh"
franka_runtime_export_xpra_defaults

usage() {
    cat <<'EOF'
Usage:
  bash open_franka_gui_xpra.sh A [extra GUI args...]
  bash open_franka_gui_xpra.sh B [extra GUI args...]
  bash open_franka_gui_xpra.sh C [extra GUI args...]

Environment overrides:
  FRANKA_XPRA_HOST=muka@192.168.1.170
  FRANKA_XPRA_REPO=/home/muka/frankateleop
  FRANKA_XPRA_SSH_SOCKET=/tmp/codex-franka-170.sock
  FRANKA_XPRA_DISPLAY=124
  FRANKA_XPRA_SSH="ssh ..."
  FRANKA_XPRA_BIN=/Applications/Xpra.app/Contents/MacOS/Xpra
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" -lt 1 ]]; then
    usage
    exit 0
fi

mode="$1"
shift
mode_key="$(printf "%s" "$mode" | tr '[:upper:]' '[:lower:]')"

case "$mode_key" in
    a|left|single) default_display="124" ;;
    b|right) default_display="125" ;;
    c|dual|bi|biarm|bi_arm) default_display="126" ;;
    *)
        echo "ERROR: unknown GUI mode: $mode" >&2
        usage >&2
        exit 2
        ;;
esac

host="$FRANKA_XPRA_HOST"
repo="$FRANKA_XPRA_REPO"
display_no="${FRANKA_XPRA_DISPLAY:-$default_display}"
xpra_bin="${FRANKA_XPRA_BIN:-/Applications/Xpra.app/Contents/MacOS/Xpra}"

if [[ ! -x "$xpra_bin" ]]; then
    if command -v xpra >/dev/null 2>&1; then
        xpra_bin="$(command -v xpra)"
    else
        echo "ERROR: xpra not found. Install Xpra or set FRANKA_XPRA_BIN." >&2
        exit 1
    fi
fi

if [[ -n "${FRANKA_XPRA_SSH:-}" ]]; then
    ssh_cmd="$FRANKA_XPRA_SSH"
elif [[ -n "$FRANKA_XPRA_SSH_SOCKET" && -S "$FRANKA_XPRA_SSH_SOCKET" ]]; then
    ssh_cmd="ssh -S $FRANKA_XPRA_SSH_SOCKET -o BatchMode=yes"
else
    ssh_cmd="ssh"
fi

remote_args=("$mode" "$@")
printf -v quoted_args "%q " "${remote_args[@]}"
remote_cmd="cd $(printf "%q" "$repo") && exec bash remote/remote_open_gui.sh $quoted_args"
printf -v quoted_remote_cmd "%q" "$remote_cmd"
start_child="bash -lc $quoted_remote_cmd"

echo ">>> Xpra host: $host"
echo ">>> Xpra display: $display_no"
echo ">>> Remote repo: $repo"
echo ">>> GUI mode: $mode"
echo ">>> SSH command: $ssh_cmd"

exec "$xpra_bin" start "ssh://$host/$display_no" \
    --ssh="$ssh_cmd" \
    --speaker=off \
    --microphone=off \
    --printing=no \
    --notifications=no \
    --start-child="$start_child" \
    --exit-with-children=yes
