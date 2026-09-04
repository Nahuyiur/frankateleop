#!/usr/bin/env python3
"""Validate the robot node mode and ensure a Cartesian policy is running."""

from __future__ import annotations

import argparse
from controller_health import ControllerMonitor
from replay_delta_eef import RobotClient


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=50051)
    parser.add_argument("--node-port", type=int, default=6001)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    args = parser.parse_args()

    node = RobotClient(args.host, args.node_port, args.timeout_ms)
    try:
        mode = str(node.request("get_control_mode"))
    finally:
        node.close()
    if mode != "ee":
        raise RuntimeError(f"Robot node must use control_mode='ee', got {mode!r}")

    monitor = ControllerMonitor(args.host, args.robot_port)
    restarted = monitor.ensure_running()
    print(
        "EE controller ready: "
        f"mode={mode}, policy_running={monitor.is_running()}, restarted={restarted}"
    )


if __name__ == "__main__":
    main()
