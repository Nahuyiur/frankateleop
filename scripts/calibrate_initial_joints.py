#!/usr/bin/env python3
"""Save and validate calibrated FR3 initial joint poses.

This tool is intentionally read-only with respect to the robot. It only reads
the current joint state from the existing ZMQ robot node and writes a JSON
calibration file. It never sends motion commands.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "config" / "initial_joints.json"
SCHEMA_VERSION = "initial_joints_v1"
JOINT_LIMITS_LOWER = (-2.64, -1.68, -2.80, -2.94, -2.70, 0.64, -2.91)
JOINT_LIMITS_UPPER = (2.64, 1.68, 2.80, -0.25, 2.70, 4.41, 2.91)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="Path to the initial-joints JSON file.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    save = subparsers.add_parser("save", help="Read current robot joints and save them.")
    add_arm_endpoint_args(save)
    save.add_argument("--name", default="", help="Optional calibration name.")

    check = subparsers.add_parser("check", help="Compare current joints with saved config.")
    add_arm_endpoint_args(check)
    check.add_argument(
        "--tolerance-rad",
        type=float,
        default=0.03,
        help="Maximum allowed absolute joint error.",
    )

    subparsers.add_parser("validate", help="Validate the config file only.")
    subparsers.add_parser("show", help="Print the config file.")
    return parser.parse_args()


def add_arm_endpoint_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--arm", choices=("left", "right", "single"), required=True)
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Robot ZMQ host. For right-arm direct mode this is usually 192.168.1.131.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="Robot ZMQ port. Defaults to 6002 for left, 6001 for right/single.",
    )


def default_port(arm: str) -> int:
    return 6002 if arm == "left" else 6001


def read_current_joints(host: str, port: int) -> list[float]:
    sys.path.insert(0, str(REPO_ROOT))
    from franka_capture.core.robot_zmq_client import RobotZMQClient

    with RobotZMQClient(host, port, timeout_ms=3000) as robot:
        observations = robot.get_observations()
        joints = observations.get("joint_positions")
        if joints is None:
            joints = robot.get_joint_state()
    return validate_joints(list(joints[:7]), label=f"tcp://{host}:{port}")


def validate_joints(values: Any, *, label: str) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != 7:
        raise ValueError(f"{label} must contain exactly 7 joints")
    joints = [float(value) for value in values]
    for index, joint in enumerate(joints):
        if not math.isfinite(joint):
            raise ValueError(f"{label}[{index}] is not finite: {joint}")
        lower = JOINT_LIMITS_LOWER[index]
        upper = JOINT_LIMITS_UPPER[index]
        if joint < lower or joint > upper:
            raise ValueError(
                f"{label}[{index}]={joint:.6f} outside configured FR3 limit "
                f"[{lower:.6f}, {upper:.6f}]"
            )
    return joints


def load_config(path: Path, *, allow_missing: bool = False) -> dict[str, Any]:
    if not path.exists():
        if allow_missing:
            return {"schema_version": SCHEMA_VERSION}
        raise FileNotFoundError(path)
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported schema in {path}: {payload.get('schema_version')!r}"
        )
    validate_payload(payload, require_any=False)
    return payload


def validate_payload(payload: dict[str, Any], *, require_any: bool = True) -> None:
    seen = 0
    for arm in ("left", "right", "single"):
        entry = payload.get(arm)
        if entry is None:
            continue
        seen += 1
        joints = entry.get("joints") if isinstance(entry, dict) else entry
        validate_joints(joints, label=f"{arm}.joints")
    if require_any and seen == 0:
        raise ValueError("No left/right/single joints found in config")


def save_config(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")


def timestamp_payload() -> tuple[float, str]:
    now = time.time()
    return now, datetime.fromtimestamp(now).isoformat(timespec="seconds")


def command_save(args: argparse.Namespace) -> None:
    config_path = Path(args.config).expanduser()
    port = args.port if args.port is not None else default_port(args.arm)
    joints = read_current_joints(args.host, port)
    payload = load_config(config_path, allow_missing=True)
    now_unix, now_iso = timestamp_payload()
    payload[args.arm] = {
        "name": args.name or f"{args.arm}_default",
        "joints": joints,
        "source_endpoint": f"tcp://{args.host}:{port}",
        "updated_at_unix": now_unix,
        "updated_at": now_iso,
    }
    payload["updated_at_unix"] = now_unix
    payload["updated_at"] = now_iso
    save_config(config_path, payload)
    print(f"Saved {args.arm} initial joints to {config_path}")
    print(json.dumps(payload[args.arm], indent=2, ensure_ascii=False))


def command_check(args: argparse.Namespace) -> None:
    config_path = Path(args.config).expanduser()
    payload = load_config(config_path)
    entry = payload.get(args.arm)
    if entry is None:
        raise KeyError(f"No {args.arm} entry in {config_path}")
    target = validate_joints(entry["joints"], label=f"{args.arm}.joints")
    port = args.port if args.port is not None else default_port(args.arm)
    current = read_current_joints(args.host, port)
    deltas = [abs(now - expected) for now, expected in zip(current, target)]
    max_delta = max(deltas)
    print(f"Current {args.arm} endpoint: tcp://{args.host}:{port}")
    print(f"Target joints:  {format_joints(target)}")
    print(f"Current joints: {format_joints(current)}")
    print(f"Abs delta:      {format_joints(deltas)}")
    print(f"Max delta: {max_delta:.6f} rad")
    if max_delta > args.tolerance_rad:
        raise SystemExit(
            f"Current joints differ from calibration by {max_delta:.6f} rad "
            f"(tolerance {args.tolerance_rad:.6f})"
        )


def command_validate(args: argparse.Namespace) -> None:
    config_path = Path(args.config).expanduser()
    payload = load_config(config_path)
    validate_payload(payload)
    print(f"Validated {config_path}")


def command_show(args: argparse.Namespace) -> None:
    config_path = Path(args.config).expanduser()
    payload = load_config(config_path)
    print(json.dumps(payload, indent=2, ensure_ascii=False))


def format_joints(values: list[float]) -> str:
    return "[" + ", ".join(f"{value:.6f}" for value in values) + "]"


def main() -> None:
    args = parse_args()
    try:
        if args.command == "save":
            command_save(args)
        elif args.command == "check":
            command_check(args)
        elif args.command == "validate":
            command_validate(args)
        elif args.command == "show":
            command_show(args)
        else:
            raise ValueError(f"Unsupported command: {args.command}")
    except SystemExit:
        raise
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
