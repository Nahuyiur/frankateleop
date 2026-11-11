#!/usr/bin/env python3
"""Record the current left Franka joint pose and print soft limits."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "teleop"))

from polymetis import RobotInterface
from teleop.robots.fr3 import EE_JOINT_LIMITS_LOWER, EE_JOINT_LIMITS_UPPER


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="left robot server host")
    parser.add_argument("--port", type=int, default=50052, help="left robot server port")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/left_current_joint_pose.json"),
        help="where to save the joint pose JSON",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    robot = RobotInterface(ip_address=args.host, port=args.port, enforce_version=False)
    joints = [float(value) for value in robot.get_joint_positions()]
    payload = {
        "timestamp_unix": time.time(),
        "host": args.host,
        "port": args.port,
        "joint_positions": joints,
    }
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + chr(10), encoding="utf-8")

    print(f"saved current left joint pose to {args.output}")
    for index, (value, lower, upper) in enumerate(
        zip(joints, EE_JOINT_LIMITS_LOWER, EE_JOINT_LIMITS_UPPER),
        start=1,
    ):
        print(
            f"J{index}: current={value:.6f} soft_lower={float(lower):.6f} soft_upper={float(upper):.6f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
