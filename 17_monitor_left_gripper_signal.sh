#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_ENV="${MONITOR_CONDA_ENV:-polymetis}"

source_conda() {
    if command -v conda >/dev/null 2>&1; then
        CONDA_BASE="$(conda info --base)"
    elif [[ -x "$HOME/miniconda3/bin/conda" ]]; then
        CONDA_BASE="$HOME/miniconda3"
    elif [[ -x "/home/pnp/miniconda3/bin/conda" ]]; then
        CONDA_BASE="/home/pnp/miniconda3"
    elif [[ -x "$HOME/anaconda3/bin/conda" ]]; then
        CONDA_BASE="$HOME/anaconda3"
    else
        echo "error: conda not found" >&2
        exit 1
    fi
    # shellcheck source=/dev/null
    source "$CONDA_BASE/etc/profile.d/conda.sh"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<'EOF'
Usage:
  bash 17_monitor_left_gripper_signal.sh [options]

Purpose:
  Read-only monitor for the left FR3 gripper. It prints continuous gripper
  feedback and command values, and marks the continuous homomorphic-arm
  closedness value where the close signal turns on/off.

Default:
  host=127.0.0.1
  port=6002        left robot node
  hz=10

Examples:
  # After left/dual stack is running
  bash 17_monitor_left_gripper_signal.sh

  # If using the old single-arm 0 pipeline on port 6001
  bash 17_monitor_left_gripper_signal.sh --port 6001

  # Additionally read the leader/homomorphic arm serial directly
  bash 17_monitor_left_gripper_signal.sh --read-teleop

Options:
  --host HOST
  --port PORT
  --hz HZ
  --teleop-port PATH
  --read-teleop
  --event-delta VALUE
  --close-threshold VALUE
  --rows N
  --no-dashboard

Notes:
  robot_open_norm / robot_closed_norm are continuous feedback values.
  cmd_closed_raw is the continuous command from the robot node, 0=open, 1=closed.
  needs_close becomes 1 when cmd_closed_raw >= --close-threshold.
  The current code default threshold is 0.5.
EOF
    exit 0
fi

echo ">>> Activating conda env: $CONDA_ENV"
source_conda
conda activate "$CONDA_ENV" || {
    echo "error: failed to activate conda env $CONDA_ENV" >&2
    exit 1
}

cd "$REPO_ROOT"
export PYTHONPATH="$REPO_ROOT/teleop:${PYTHONPATH:-}"
python - "$@" <<'PY'
from __future__ import annotations

import argparse
import collections
import datetime as _dt
import math
import os
import time
from typing import Any

import numpy as np

from franka_capture.core.robot_zmq_client import RobotZMQClient

MAX_GRIPPER_WIDTH = 0.09


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only monitor for left FR3 gripper feedback and teleop command signal."
    )
    parser.add_argument("--host", default=os.environ.get("LEFT_ROBOT_HOST", "127.0.0.1"))
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("LEFT_ZMQ_PORT", "6002")),
        help="Robot node ZMQ port. Left arm default is 6002; old single-arm pipeline is usually 6001.",
    )
    parser.add_argument("--hz", type=float, default=float(os.environ.get("MONITOR_HZ", "10")))
    parser.add_argument(
        "--teleop-port",
        default=(
            os.environ.get("LEFT_TELEOP_PORT")
            or os.environ.get("FRANKA_TELEOP_PORT")
            or os.environ.get("TELEOP_PORT")
            or "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB2UWSZ-if00-port0"
        ),
    )
    parser.add_argument(
        "--read-teleop",
        action="store_true",
        help="Also read the homomorphic-arm Dynamixel serial directly. Avoid this if 4_run_env already owns the serial port.",
    )
    parser.add_argument(
        "--event-delta",
        type=float,
        default=0.002,
        help="Minimum continuous closedness delta that triggers a change event.",
    )
    parser.add_argument(
        "--close-threshold",
        type=float,
        default=float(os.environ.get("GRIPPER_CLOSE_THRESHOLD", "0.5")),
        help="Continuous closedness threshold for the close-needed signal.",
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=int(os.environ.get("MONITOR_ROWS", "10")),
        help="Number of recent samples displayed in dashboard mode.",
    )
    parser.add_argument(
        "--no-dashboard",
        action="store_true",
        help="Disable dynamic refresh and print one line per sample.",
    )
    return parser.parse_args()


def scalar(value: Any, default: float = float("nan")) -> float:
    if value is None:
        return default
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        if not value:
            return default
        value = value[0]
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def fmt(value: float, digits: int = 4) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def now() -> str:
    return _dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def make_teleop_reader(port: str):
    from teleop.agents.teleop_agent import TeleopAgent

    agent = TeleopAgent(port=port)

    def read_closedness() -> float:
        return float(np.asarray(agent.act({}), dtype=float)[-1])

    return read_closedness


def clear_screen() -> None:
    print("\033[2J\033[H", end="")


def render_dashboard(
    args: argparse.Namespace,
    rows: collections.deque[str],
    status_lines: collections.deque[str],
    header: str,
    teleop_status: str,
) -> None:
    clear_screen()
    print("FR3 Left Gripper Continuous Signal Monitor")
    print("=" * 104)
    print(
        f"robot_node=tcp://{args.host}:{args.port}  hz={args.hz:g}  "
        f"close_threshold={args.close_threshold:.3f}  rows={args.rows}  "
        f"teleop_direct={teleop_status}"
    )
    print(
        "close rule: needs_close = 1 when cmd_closed_raw >= close_threshold "
        "(0=open, 1=closed)"
    )
    if status_lines:
        print("-" * 104)
        for line in status_lines:
            print(line)
    print("-" * 104)
    print(header)
    print("-" * 104)
    if rows:
        for line in rows:
            print(line)
    else:
        print("(waiting for samples...)")
    print("-" * 104)
    print("Ctrl-C to stop. Use --no-dashboard for scroll log output.")
    print(flush=True)


def add_status(status_lines: collections.deque[str], text: str) -> None:
    status_lines.append(f"[{now()}] {text}")


def main() -> int:
    args = parse_args()
    if args.hz <= 0:
        raise ValueError("--hz must be positive")
    if not 0.0 <= args.close_threshold <= 1.0:
        raise ValueError("--close-threshold must be in [0, 1]")
    if args.rows <= 0:
        raise ValueError("--rows must be positive")

    dashboard = not args.no_dashboard
    header = (
        "time          robot_open robot_closed width_m  target_closed needs_close "
        "target_m source             signal_age event"
    )
    rows: collections.deque[str] = collections.deque(maxlen=args.rows)
    status_lines: collections.deque[str] = collections.deque(maxlen=5)
    teleop_status = "disabled"

    def redraw() -> None:
        if dashboard:
            render_dashboard(args, rows, status_lines, header, teleop_status)

    def emit_status(text: str) -> None:
        if dashboard:
            add_status(status_lines, text)
            redraw()
        else:
            print(f"[{now()}] {text}", flush=True)

    teleop_reader = None
    direct_teleop_ok = False
    if args.read_teleop:
        try:
            teleop_reader = make_teleop_reader(args.teleop_port)
            teleop_status = f"connected:{args.teleop_port}"
            emit_status(f"EVENT direct teleop serial connected: {args.teleop_port}")
        except Exception as exc:
            teleop_status = f"unavailable:{args.teleop_port}"
            emit_status(f"WARN direct teleop serial not available: {args.teleop_port} ({exc})")

    emit_status(
        f"Monitoring left gripper via robot node tcp://{args.host}:{args.port} "
        f"at {args.hz:g} Hz"
    )
    emit_status(
        f"Close-needed threshold: cmd_closed_raw >= {args.close_threshold:.3f}. "
        "cmd_closed_raw is the continuous homomorphic-arm gripper command after it reaches the robot node."
    )
    if not dashboard:
        print(header, flush=True)

    last_cmd_raw = None
    last_needs_close = None
    last_direct = None
    last_direct_needs_close = None
    first_robot_sample = True
    period = 1.0 / args.hz

    with RobotZMQClient(args.host, args.port, timeout_ms=2000) as robot:
        while True:
            loop_t0 = time.monotonic()
            event_parts = []

            try:
                joint_state = np.asarray(robot.get_joint_state(), dtype=float)
                obs = robot.get_observations()
            except Exception as exc:
                emit_status(f"WAIT robot node unavailable: {exc}")
                time.sleep(min(1.0, period))
                continue

            if joint_state.size == 0:
                emit_status("WAIT robot joint_state is empty")
                time.sleep(period)
                continue

            robot_open = float(np.clip(joint_state[-1], 0.0, 1.0))
            robot_closed = 1.0 - robot_open
            width_m = scalar(obs.get("gripper_width"), robot_open * MAX_GRIPPER_WIDTH)
            cmd_raw = scalar(obs.get("gripper_closedness"), scalar(obs.get("gripper_command_raw")))
            target_width = scalar(obs.get("gripper_target_width"))
            source = str(obs.get("gripper_command_source", ""))
            cmd_ts = scalar(obs.get("gripper_command_timestamp"))
            signal_age = time.time() - cmd_ts if math.isfinite(cmd_ts) and cmd_ts > 0 else float("nan")
            needs_close = (
                math.isfinite(cmd_raw) and cmd_raw >= args.close_threshold
            )

            if first_robot_sample:
                first_robot_sample = False
                event_parts.append("robot_feedback_ready")
                if source == "command_joint_state":
                    state = "close" if needs_close else "open"
                    event_parts.append(f"initial_{state}_signal")

            if source == "command_joint_state":
                if last_needs_close is not None and needs_close != last_needs_close:
                    event = "close_signal_on" if needs_close else "close_signal_off"
                    event_parts.append(f"{event}@target_closed={fmt(cmd_raw)}")
                if (
                    last_cmd_raw is not None
                    and math.isfinite(cmd_raw)
                    and abs(cmd_raw - last_cmd_raw) >= args.event_delta
                ):
                    direction = "closing" if cmd_raw > last_cmd_raw else "opening"
                    event_parts.append(f"cmd_{direction}")

            direct_value = float("nan")
            direct_needs_close = False
            if teleop_reader is not None:
                try:
                    direct_value = float(np.clip(teleop_reader(), 0.0, 1.0))
                    direct_needs_close = direct_value >= args.close_threshold
                    if not direct_teleop_ok:
                        direct_teleop_ok = True
                        teleop_status = f"connected:{args.teleop_port}"
                        event_parts.append("direct_teleop_signal_ready")
                    if (
                        last_direct_needs_close is not None
                        and direct_needs_close != last_direct_needs_close
                    ):
                        event = "direct_close_signal_on" if direct_needs_close else "direct_close_signal_off"
                        event_parts.append(f"{event}@direct_closed={fmt(direct_value)}")
                    if last_direct is not None and abs(direct_value - last_direct) >= args.event_delta:
                        event_parts.append("direct_teleop_changed")
                    last_direct = direct_value
                    last_direct_needs_close = direct_needs_close
                except Exception as exc:
                    if direct_teleop_ok:
                        event_parts.append(f"direct_teleop_lost:{type(exc).__name__}")
                    teleop_status = f"lost:{args.teleop_port}"
                    direct_teleop_ok = False

            if math.isfinite(cmd_raw):
                last_cmd_raw = cmd_raw
                last_needs_close = needs_close

            event = ",".join(event_parts)
            line = (
                f"{now()}  "
                f"{fmt(robot_open)}     "
                f"{fmt(robot_closed)}       "
                f"{fmt(width_m, 5)}  "
                f"{fmt(cmd_raw)}      "
                f"{int(needs_close):>11d} "
                f"{fmt(target_width, 5)}  "
                f"{source[:18]:18s} "
                f"{fmt(signal_age, 3):>9s}  "
                f"{event}"
            )
            if teleop_reader is not None:
                line += f"  direct_closed={fmt(direct_value)}"
            if dashboard:
                rows.append(line)
                if event:
                    add_status(status_lines, f"EVENT {event}")
                redraw()
            else:
                print(line, flush=True)

            elapsed = time.monotonic() - loop_t0
            if elapsed < period:
                time.sleep(period - elapsed)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print(f"\n[{now()}] Monitor stopped by user.", flush=True)
        raise SystemExit(130)
PY
