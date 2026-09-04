"""Read-only preflight or explicit safe-hover validation for Stack-Ring mapping.

Default behavior connects to the robot node, reads its mode/current EEF pose,
builds the hover path, and sends no commands.  Execution requires both
``--execute`` and the exact confirmation token ``STACK_RING_HOVER``.  This tool
never closes the gripper and never descends to the mapped grasp/release Z.
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
import zmq


CONFIRM_TOKEN = "STACK_RING_HOVER"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping-report", required=True)
    parser.add_argument("--target", choices=("pick", "place"), required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6002)
    parser.add_argument("--timeout-ms", type=int, default=2000)
    parser.add_argument("--clearance-m", type=float, default=0.12)
    parser.add_argument("--minimum-execute-clearance-m", type=float, default=0.08)
    parser.add_argument("--minimum-transit-z", type=float, default=0.55)
    parser.add_argument("--step-m", type=float, default=0.005)
    parser.add_argument("--rotation-step-rad", type=float, default=0.04)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--max-planar-travel-m", type=float, default=0.40)
    parser.add_argument("--use-anchor-orientation", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--skip-robot-check", action="store_true")
    parser.add_argument("--current-pose", default=None, help="Offline pose x,y,z,r,p,y")
    parser.add_argument("--output-plan", default=None)
    return parser.parse_args()


class RobotClient:
    def __init__(self, host: str, port: int, timeout_ms: int) -> None:
        self.host = host
        self.port = int(port)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
        self.socket.setsockopt(zmq.SNDTIMEO, int(timeout_ms))
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(f"tcp://{host}:{port}")

    def request(self, method: str, args: Optional[Dict[str, Any]] = None) -> Any:
        try:
            self.socket.send(pickle.dumps({"method": method, "args": args or {}}))
            result = pickle.loads(self.socket.recv())
        except zmq.Again as exc:
            raise TimeoutError(f"Robot node timeout at tcp://{self.host}:{self.port}") from exc
        if isinstance(result, dict) and "error" in result:
            raise RuntimeError(f"Robot node error: {result['error']}")
        return result

    def close(self) -> None:
        self.socket.close(0)
        self.context.term()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


def parse_pose(value: str) -> np.ndarray:
    parts = [float(item.strip()) for item in value.split(",")]
    pose = np.asarray(parts, dtype=np.float64)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError("--current-pose must contain six finite comma-separated values")
    return pose


def wrap_delta(delta: np.ndarray) -> np.ndarray:
    return (np.asarray(delta) + np.pi) % (2.0 * np.pi) - np.pi


def interpolate(a: np.ndarray, b: np.ndarray, xyz_step: float, rot_step: float) -> np.ndarray:
    xyz_distance = float(np.linalg.norm(b[:3] - a[:3]))
    rotation_delta = wrap_delta(b[3:] - a[3:])
    rotation_distance = float(np.linalg.norm(rotation_delta))
    count = max(1, int(math.ceil(xyz_distance / xyz_step)), int(math.ceil(rotation_distance / rot_step)))
    values = []
    for index in range(1, count + 1):
        alpha = index / float(count)
        pose = a.copy()
        pose[:3] = a[:3] + alpha * (b[:3] - a[:3])
        pose[3:] = a[3:] + alpha * rotation_delta
        values.append(pose)
    return np.asarray(values, dtype=np.float64)


def build_hover_plan(
    current: np.ndarray,
    mapped_pose: np.ndarray,
    clearance: float,
    minimum_transit_z: float,
    xyz_step: float,
    rotation_step: float,
    use_anchor_orientation: bool,
) -> Tuple[np.ndarray, list]:
    hover = mapped_pose.copy()
    hover[2] += float(clearance)
    if not use_anchor_orientation:
        hover[3:] = current[3:]
    safe_z = max(float(current[2]), float(hover[2]), float(minimum_transit_z))
    lift = current.copy(); lift[2] = safe_z
    transit = lift.copy(); transit[:2] = hover[:2]
    rotate = transit.copy(); rotate[3:] = hover[3:]
    phases = []
    segments = []
    start = current
    for name, target in (("vertical_lift", lift), ("planar_transit", transit), ("orientation", rotate), ("hover_descent", hover)):
        segment = interpolate(start, target, xyz_step, rotation_step)
        phases.append({"name": name, "frames": int(len(segment)), "target_pose": target.astype(float).tolist()})
        segments.append(segment)
        start = target
    return np.concatenate(segments, axis=0), phases


def in_workspace(pose: np.ndarray, workspace: Sequence[Sequence[float]]) -> bool:
    return all(float(bounds[0]) <= float(value) <= float(bounds[1]) for value, bounds in zip(pose[:3], workspace))


def main() -> int:
    args = parse_args()
    if args.execute and args.skip_robot_check:
        raise ValueError("--execute cannot be combined with --skip-robot-check")
    if args.execute and args.confirm != CONFIRM_TOKEN:
        raise ValueError(f"Execution requires --confirm {CONFIRM_TOKEN}")
    if args.execute and args.clearance_m < args.minimum_execute_clearance_m:
        raise ValueError(
            f"Execution clearance {args.clearance_m:.3f} m is below minimum "
            f"{args.minimum_execute_clearance_m:.3f} m"
        )
    if args.step_m <= 0 or args.rotation_step_rad <= 0 or args.hz <= 0:
        raise ValueError("Step sizes and --hz must be positive")

    report_path = Path(args.mapping_report).expanduser().resolve()
    report = json.loads(report_path.read_text())
    if report.get("schema") != "stack_ring_robot_mapping_report_v1":
        raise ValueError("Unsupported mapping report schema")
    if not report.get("hard_checks", {}).get("passed"):
        raise RuntimeError("Mapping hard checks did not pass")
    if report.get("approved_execution_scope") != "hover_only":
        raise RuntimeError("Mapping report is not approved for hover-only validation")
    mapped_pose = np.asarray(report[f"target_{args.target}_pose6"], dtype=np.float64)
    workspace = report["workspace_xyz"]

    mode = "offline"
    client = None
    dofs = None
    try:
        if args.skip_robot_check:
            if not args.current_pose:
                raise ValueError("--skip-robot-check requires --current-pose")
            current = parse_pose(args.current_pose)
        else:
            client = RobotClient(args.host, args.port, args.timeout_ms)
            mode = str(client.request("get_control_mode"))
            observations = client.request("get_observations")
            if not isinstance(observations, Mapping):
                raise RuntimeError("Robot observations are not a mapping")
            current = np.asarray(observations.get("ee_pose_euler"), dtype=np.float64).reshape(6)
            dofs = int(client.request("num_dofs"))
            # Some frankateleop nodes report the 7 arm joints only, while the
            # current 170 node includes the gripper as an eighth controllable
            # dimension. Cartesian command_ee_pose is valid for both schemas.
            if dofs not in (7, 8):
                raise RuntimeError(f"Expected Franka node DOFs in {{7, 8}}, got {dofs}")

        plan, phases = build_hover_plan(
            current, mapped_pose, args.clearance_m, args.minimum_transit_z,
            args.step_m, args.rotation_step_rad, args.use_anchor_orientation,
        )
        if not np.isfinite(plan).all():
            raise ValueError("Hover plan contains NaN or Inf")
        planar_travel = float(np.linalg.norm(plan[-1, :2] - current[:2]))
        if planar_travel > args.max_planar_travel_m:
            raise ValueError(f"Planar travel {planar_travel:.4f} m exceeds {args.max_planar_travel_m:.4f}")
        if not all(in_workspace(pose, workspace) for pose in plan):
            raise ValueError("Hover plan leaves the calibrated workspace")
        plan_payload = {
            "schema": "stack_ring_hover_plan_v1",
            "mapping_report": str(report_path),
            "target": args.target,
            "robot_mode": mode,
            "robot_dofs": dofs,
            "current_pose": current.astype(float).tolist(),
            "mapped_anchor_pose": mapped_pose.astype(float).tolist(),
            "hover_pose": plan[-1].astype(float).tolist(),
            "clearance_m": float(args.clearance_m),
            "preserve_current_orientation": not args.use_anchor_orientation,
            "plan_frames": int(len(plan)),
            "phases": phases,
            "planar_travel_m": planar_travel,
            "commands_sent": False,
        }
        output_plan = Path(args.output_plan).expanduser() if args.output_plan else report_path.parent / f"hover_{args.target}_plan.json"
        output_plan.write_text(json.dumps(plan_payload, indent=2, sort_keys=True) + "\n")
        print(json.dumps(plan_payload, indent=2))
        if not args.execute:
            print("READ-ONLY PREFLIGHT: no robot command was sent.")
            return 0
        if mode != "ee":
            raise RuntimeError(f"Execution requires robot control mode 'ee', got {mode!r}")
        assert client is not None
        period = 1.0 / float(args.hz)
        for index, pose in enumerate(plan):
            started = time.monotonic()
            client.request("command_ee_pose", {
                "pose_6d": pose,
                "gripper_width": 0.08,
                "gripper_speed": 0.05,
                "gripper_force": 5.0,
                "update_gripper": False,
            })
            if index == 0 or index == len(plan) - 1 or index % 20 == 0:
                print(f"Sent hover waypoint {index + 1}/{len(plan)}")
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
        plan_payload["commands_sent"] = True
        plan_payload["completed_at_unix"] = time.time()
        output_plan.write_text(json.dumps(plan_payload, indent=2, sort_keys=True) + "\n")
        print("HOVER VALIDATION COMPLETE. Gripper was not actuated.")
        return 0
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    raise SystemExit(main())
