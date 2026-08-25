#!/usr/bin/env bash
set -euo pipefail

PRIMARY_PORT="${LEFT_TELEOP_PRIMARY_PORT:-/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBJKECV-if00-port0}"
FALLBACK_PORT="${LEFT_TELEOP_FALLBACK_PORT:-/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB2UWSZ-if00-port0}"
EXPLICIT_PORT="${LEFT_TELEOP_PORT:-${FRANKA_TELEOP_PORT:-${TELEOP_PORT:-}}}"

print_port_details() {
    local port="$1"
    if [[ -L "$port" ]]; then
        printf '   %s -> %s\n' "$port" "$(readlink -f "$port" 2>/dev/null || readlink "$port")" >&2
    elif [[ -e "$port" ]]; then
        printf '   %s\n' "$port" >&2
    fi
}

print_detected_ports() {
    local found=0
    echo ">>> 当前检测到的同构臂 FTDI 串口：" >&2
    if [[ -d /dev/serial/by-id ]]; then
        while IFS= read -r port; do
            found=1
            print_port_details "$port"
        done < <(
            find /dev/serial/by-id -maxdepth 1 -type l \
                -name 'usb-FTDI_USB__-__Serial_Converter_*' -print 2>/dev/null | sort
        )
    fi
    if [[ "$found" -eq 0 ]]; then
        echo "   （未检测到同构臂 FTDI 串口）" >&2
    fi
}

require_accessible_port() {
    local port="$1"
    local source_label="$2"

    if [[ ! -e "$port" ]]; then
        echo "❌ ${source_label}不存在：$port" >&2
        return 1
    fi
    if [[ ! -r "$port" || ! -w "$port" ]]; then
        echo "❌ ${source_label}当前用户无读写权限：$port" >&2
        print_port_details "$port"
        echo "   当前用户/用户组：$(id)" >&2
        return 1
    fi
    printf '%s\n' "$port"
}

if [[ -n "$EXPLICIT_PORT" ]]; then
    if ! require_accessible_port "$EXPLICIT_PORT" "显式设置的左臂同构臂串口"; then
        echo "   已设置 LEFT_TELEOP_PORT/FRANKA_TELEOP_PORT/TELEOP_PORT，因此拒绝自动改选其他设备。" >&2
        print_detected_ports
        exit 1
    fi
    exit 0
fi

if [[ -e "$PRIMARY_PORT" ]]; then
    require_accessible_port "$PRIMARY_PORT" "左臂同构臂主串口"
    exit 0
fi

if [[ -e "$FALLBACK_PORT" ]]; then
    echo ">>> 左臂主串口未检测到，使用已登记的备用左臂串口：$FALLBACK_PORT" >&2
    require_accessible_port "$FALLBACK_PORT" "左臂同构臂备用串口"
    exit 0
fi

echo "❌ 未检测到任何已登记的左臂同构臂串口。" >&2
echo "   主串口：$PRIMARY_PORT" >&2
echo "   备用串口：$FALLBACK_PORT" >&2
echo "   请检查左臂同构臂电源和 USB 连接；如硬件确实更换，请显式设置 LEFT_TELEOP_PORT。" >&2
print_detected_ports
exit 1
