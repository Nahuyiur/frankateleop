#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
    cat <<'EOF'
Usage:
  bash remote/install_xpra_ubuntu.sh

Installs or upgrades Xpra on Ubuntu using the official xpra.org apt repository.
Ubuntu's distro package can be too old for the macOS Xpra client.

Environment:
  SUDO_PASSWORD=...   Optional. If unset, sudo may prompt in an interactive shell.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

if [[ -r /etc/os-release ]]; then
    . /etc/os-release
else
    echo "ERROR: /etc/os-release not found; this installer expects Ubuntu." >&2
    exit 1
fi

if [[ "${ID:-}" != "ubuntu" ]]; then
    echo "ERROR: this installer expects Ubuntu, got ID=${ID:-unknown}." >&2
    exit 1
fi

codename="${VERSION_CODENAME:-}"
if [[ -z "$codename" ]]; then
    codename="$(lsb_release -cs 2>/dev/null || true)"
fi
if [[ -z "$codename" ]]; then
    echo "ERROR: could not determine Ubuntu codename." >&2
    exit 1
fi

sudo_run() {
    if [[ -n "${SUDO_PASSWORD:-}" ]]; then
        printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' "$@"
    else
        sudo "$@"
    fi
}

echo ">>> Ubuntu codename: $codename"
echo ">>> Installing Xpra apt repository prerequisites ..."
sudo_run apt-get update
sudo_run apt-get install -y apt-transport-https software-properties-common ca-certificates wget

tmpdir="$(mktemp -d /tmp/xpra-install.XXXXXX)"
trap 'rm -rf "$tmpdir"' EXIT

echo ">>> Installing xpra.org apt key and source ..."
wget -q -O "$tmpdir/xpra.asc" https://xpra.org/xpra.asc
sudo_run install -m 0644 "$tmpdir/xpra.asc" /usr/share/keyrings/xpra.asc

source_url="https://raw.githubusercontent.com/Xpra-org/xpra/master/packaging/repos/${codename}/xpra.sources"
if ! wget -q -O "$tmpdir/xpra.sources" "$source_url"; then
    echo "ERROR: failed to download Xpra source file for Ubuntu codename '$codename'." >&2
    echo "       URL: $source_url" >&2
    exit 1
fi
sudo_run install -m 0644 "$tmpdir/xpra.sources" /etc/apt/sources.list.d/xpra.sources

echo ">>> Installing Xpra ..."
sudo_run apt-get update
sudo_run apt-get install -y xpra

echo ">>> Installed:"
xpra --version
