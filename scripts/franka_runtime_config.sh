#!/usr/bin/env bash

# Shared runtime defaults for launch scripts. Keep environment variables as the
# public interface so existing commands and machine-local overrides keep working.

_franka_runtime_trim() {
    local value="$1"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"
    printf '%s' "$value"
}

franka_runtime_load_env_file() {
    local config_file="${FRANKA_RUNTIME_CONFIG_FILE:-$HOME/.config/frankateleop/runtime.env}"
    [[ -f "$config_file" ]] || return 0

    local line key value line_number=0
    while IFS= read -r line || [[ -n "$line" ]]; do
        line_number=$((line_number + 1))
        line="$(_franka_runtime_trim "$line")"
        [[ -z "$line" || "$line" == \#* ]] && continue
        if [[ "$line" != *=* ]]; then
            echo "ERROR: invalid runtime config at $config_file:$line_number (expected KEY=VALUE)" >&2
            return 1
        fi
        key="$(_franka_runtime_trim "${line%%=*}")"
        value="$(_franka_runtime_trim "${line#*=}")"
        if [[ ! "$key" =~ ^(FRANKA|BI_ARM|LEFT|RIGHT)_[A-Z0-9_]+$ ]]; then
            echo "ERROR: unsupported runtime config key at $config_file:$line_number: $key" >&2
            return 1
        fi
        if [[ -z "${!key+x}" ]]; then
            printf -v "$key" '%s' "$value"
            export "$key"
        fi
    done < "$config_file"
}

franka_runtime_export_defaults() {
    franka_runtime_load_env_file

    export FRANKA_LEFT_HOST="${FRANKA_LEFT_HOST:-192.168.100.67}"
    export FRANKA_LEFT_SSH="${FRANKA_LEFT_SSH:-muka@$FRANKA_LEFT_HOST}"
    if [[ "$FRANKA_LEFT_SSH" != *@* ]]; then
        export FRANKA_LEFT_SSH="muka@$FRANKA_LEFT_SSH"
    fi
    export FRANKA_LEFT_REPO="${FRANKA_LEFT_REPO:-/home/muka/frankateleop}"

    export BI_ARM_RIGHT_HOST="${BI_ARM_RIGHT_HOST:-192.168.1.131}"
    export BI_ARM_RIGHT_SSH="${BI_ARM_RIGHT_SSH:-pnp@$BI_ARM_RIGHT_HOST}"
    if [[ "$BI_ARM_RIGHT_SSH" != *@* ]]; then
        export BI_ARM_RIGHT_SSH="pnp@$BI_ARM_RIGHT_SSH"
    fi
    export BI_ARM_RIGHT_REPO="${BI_ARM_RIGHT_REPO:-/home/pnp/frankateleop}"

    export BI_ARM_LEFT_ZMQ_PORT="${BI_ARM_LEFT_ZMQ_PORT:-6002}"
    export BI_ARM_RIGHT_REMOTE_ZMQ_PORT="${BI_ARM_RIGHT_REMOTE_ZMQ_PORT:-6001}"
    export BI_ARM_RIGHT_LOCAL_ZMQ_PORT="${BI_ARM_RIGHT_LOCAL_ZMQ_PORT:-16001}"
    export BI_ARM_RIGHT_LOCAL_GRIPPER_PORT="${BI_ARM_RIGHT_LOCAL_GRIPPER_PORT:-15053}"
    export BI_ARM_LEFT_ROBOT_PORT="${BI_ARM_LEFT_ROBOT_PORT:-50052}"
    export BI_ARM_LEFT_GRIPPER_PORT="${BI_ARM_LEFT_GRIPPER_PORT:-50054}"
    export BI_ARM_RIGHT_ROBOT_PORT="${BI_ARM_RIGHT_ROBOT_PORT:-50051}"
    export BI_ARM_RIGHT_GRIPPER_PORT="${BI_ARM_RIGHT_GRIPPER_PORT:-50053}"
    export BI_ARM_RIGHT_ROBOT_IP="${BI_ARM_RIGHT_ROBOT_IP:-172.16.0.2}"
    export BI_ARM_RIGHT_FCI_PORT="${BI_ARM_RIGHT_FCI_PORT:-1337}"
    export BI_ARM_REQUIRE_RIGHT_FCI_READY="${BI_ARM_REQUIRE_RIGHT_FCI_READY:-1}"

    export FRANKA_RIGHT_ZMQ_HOST="${FRANKA_RIGHT_ZMQ_HOST:-${BI_ARM_RIGHT_RECORD_ZMQ_HOST:-$BI_ARM_RIGHT_HOST}}"
    export FRANKA_RIGHT_ZMQ_PORT="${FRANKA_RIGHT_ZMQ_PORT:-${BI_ARM_RIGHT_RECORD_ZMQ_PORT:-$BI_ARM_RIGHT_REMOTE_ZMQ_PORT}}"
}

franka_runtime_export_xpra_defaults() {
    franka_runtime_export_defaults
    export FRANKA_XPRA_HOST="${FRANKA_XPRA_HOST:-$FRANKA_LEFT_SSH}"
    export FRANKA_XPRA_REPO="${FRANKA_XPRA_REPO:-$FRANKA_LEFT_REPO}"
    if [[ -z "${FRANKA_XPRA_SSH_SOCKET+x}" ]]; then
        if [[ "$FRANKA_XPRA_HOST" == "muka@192.168.100.67" ]]; then
            FRANKA_XPRA_SSH_SOCKET="/tmp/codex-franka-67.sock"
        else
            FRANKA_XPRA_SSH_SOCKET=""
        fi
    fi
    export FRANKA_XPRA_SSH_SOCKET
}
