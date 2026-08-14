#!/usr/bin/env python3
"""Safely replay canonical DROID body-frame delta EEF actions on a local FR3."""

from __future__ import annotations

import argparse
import json
import pickle
import time
from pathlib import Path
from typing import Any

import numpy as np
import zmq
from polymetis import GripperInterface
from scipy.spatial.transform import Rotation

from controller_health import ControllerMonitor


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RETURN_HOME_JOINTS = REPO_ROOT / "config/initial_joints.json"


def pose_matrix(pose: np.ndarray, rotation: str) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, 3] = pose[:3]
    converter = Rotation.from_rotvec if rotation == "rotvec" else Rotation.from_euler
    matrix[:3, :3] = converter("xyz", pose[3:]).as_matrix() if rotation == "euler" else converter(pose[3:]).as_matrix()
    return matrix


def matrix_euler(matrix: np.ndarray) -> np.ndarray:
    return np.r_[matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")]


def rotation_distance(left: np.ndarray, right: np.ndarray) -> float:
    return float(Rotation.from_matrix(left[:3, :3].T @ right[:3, :3]).magnitude())


def integrate(anchor_pose_euler: np.ndarray, delta_pose: np.ndarray) -> np.ndarray:
    targets = np.empty((len(delta_pose) + 1, 4, 4), dtype=np.float64)
    targets[0] = pose_matrix(anchor_pose_euler, "euler")
    for index, delta in enumerate(delta_pose):
        targets[index + 1] = targets[index] @ pose_matrix(delta, "rotvec")
    return targets


class RobotClient:
    def __init__(self, host: str, port: int, timeout_ms: int) -> None:
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(f"tcp://{host}:{port}")

    def request(
        self,
        method: str,
        args: dict[str, Any] | None = None,
        timeout_ms: int | None = None,
    ) -> Any:
        old_receive_timeout = self._socket.getsockopt(zmq.RCVTIMEO)
        old_send_timeout = self._socket.getsockopt(zmq.SNDTIMEO)
        if timeout_ms is not None:
            self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
            self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        try:
            self._socket.send(pickle.dumps({"method": method, "args": args or {}}))
            result = pickle.loads(self._socket.recv())
        except zmq.Again as exc:
            raise TimeoutError("Robot node request timed out") from exc
        finally:
            if timeout_ms is not None:
                self._socket.setsockopt(zmq.RCVTIMEO, old_receive_timeout)
                self._socket.setsockopt(zmq.SNDTIMEO, old_send_timeout)
        if isinstance(result, dict) and "error" in result:
            raise RuntimeError(result["error"])
        return result

    def close(self) -> None:
        self._socket.close(0)
        self._context.term()


def current_pose(client: RobotClient) -> np.ndarray:
    observations = client.request("get_observations")
    pose = np.asarray(observations["ee_pose_euler"], dtype=np.float64)
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise RuntimeError(f"Invalid live ee_pose_euler: {pose}")
    return pose


def source_start_joints(data: dict[str, np.ndarray]) -> np.ndarray:
    joints = np.asarray(data["joint_position"], dtype=np.float64)
    mask = np.asarray(data["joint_position_mask"], dtype=bool)
    if joints.ndim != 2 or joints.shape[1] < 7 or mask.shape != joints.shape:
        raise RuntimeError(f"Invalid source joint arrays: {joints.shape}, {mask.shape}")
    if not np.all(mask[0, :7]) or not np.all(np.isfinite(joints[0, :7])):
        raise RuntimeError("DROID frame 0 does not contain seven valid arm joints")
    return joints[0, :7].copy()


def source_end_joints(data: dict[str, np.ndarray]) -> np.ndarray:
    joints = np.asarray(data["joint_position"], dtype=np.float64)
    mask = np.asarray(data["joint_position_mask"], dtype=bool)
    if joints.ndim != 2 or joints.shape[1] < 7 or mask.shape != joints.shape:
        raise RuntimeError(f"Invalid source joint arrays: {joints.shape}, {mask.shape}")
    if not np.all(mask[-1, :7]) or not np.all(np.isfinite(joints[-1, :7])):
        raise RuntimeError("DROID final frame does not contain seven valid arm joints")
    return joints[-1, :7].copy()


def config_home_joints(path: Path, side: str) -> np.ndarray:
    payload = json.loads(path.read_text())
    if payload.get("schema_version") != "initial_joints_v1":
        raise RuntimeError(f"Unsupported config home schema: {path}")
    entry = payload.get(side)
    if not isinstance(entry, dict) or "joints" not in entry:
        raise RuntimeError(f"No {side!r} home joints in {path}")
    joints = np.asarray(entry["joints"], dtype=np.float64)
    if joints.shape != (7,) or not np.all(np.isfinite(joints)):
        raise RuntimeError(f"Invalid {side!r} config home joints: {joints}")
    return joints


def gripper_max_width(gripper: GripperInterface) -> float:
    width = float(getattr(gripper.metadata, "max_width", 0.0))
    if not np.isfinite(width) or width <= 0:
        width = 0.085
    return width


def gripper_targets(
    data: dict[str, np.ndarray], max_width: float, event_delta: float
) -> tuple[np.ndarray, set[int]]:
    opening = np.asarray(data["gripper_opening"], dtype=np.float64).reshape(-1)
    mask = np.asarray(data["gripper_opening_mask"], dtype=bool).reshape(len(opening), -1)
    valid = np.all(mask, axis=1)
    if len(opening) != len(data["delta_pose"]):
        raise RuntimeError("Gripper and EEF trajectory lengths differ")
    if not valid[0] or not np.all(np.isfinite(opening[valid])):
        raise RuntimeError("Invalid gripper opening trajectory")
    for index in range(1, len(opening)):
        if not valid[index]:
            opening[index] = opening[index - 1]
    widths = max_width * np.clip(opening, 0.0, 1.0)
    events: set[int] = set()
    last_width = widths[0]
    for index in range(1, len(widths)):
        if abs(widths[index] - last_width) >= event_delta:
            events.add(index)
            last_width = widths[index]
    if abs(widths[-1] - last_width) > 1e-4:
        events.add(len(widths) - 1)
    return widths, events


def load_trajectory(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    required = {
        "episode_index", "delta_pose", "delta_pose_mask", "delta_dt",
        "delta_dt_mask", "gripper_opening", "gripper_opening_mask",
        "joint_position", "joint_position_mask", "state_pose",
    }
    missing = required - data.keys()
    if missing:
        raise RuntimeError(f"Missing trajectory fields: {sorted(missing)}")
    delta = data["delta_pose"]
    mask = data["delta_pose_mask"].astype(bool)
    if delta.ndim != 2 or delta.shape[1] != 6 or len(delta) < 2:
        raise RuntimeError(f"Invalid delta_pose shape: {delta.shape}")
    if not np.all(mask[:-1]) or np.any(mask[-1]):
        raise RuntimeError("Expected all transitions valid and final delta masked")
    if not np.all(np.isfinite(delta[:-1])):
        raise RuntimeError("Non-finite delta action")
    return data


def preflight(data: dict[str, np.ndarray], targets: np.ndarray, args: argparse.Namespace) -> dict[str, float]:
    delta = data["delta_pose"][:-1]
    step_translation = np.linalg.norm(delta[:, :3], axis=1)
    step_rotation = np.linalg.norm(delta[:, 3:], axis=1)
    anchor = targets[0]
    displacement = np.linalg.norm(targets[:, :3, 3] - anchor[:3, 3], axis=1)
    xyz = targets[:, :3, 3]
    metrics = {
        "max_step_translation_m": float(step_translation.max()),
        "max_step_rotation_rad": float(step_rotation.max()),
        "translation_path_m": float(step_translation.sum()),
        "rotation_path_rad": float(step_rotation.sum()),
        "max_anchor_displacement_m": float(displacement.max()),
    }
    limits = {
        "max_step_translation_m": args.max_step_translation,
        "max_step_rotation_rad": args.max_step_rotation,
        "translation_path_m": args.max_translation_path,
        "rotation_path_rad": args.max_rotation_path,
        "max_anchor_displacement_m": args.max_anchor_displacement,
    }
    violations = [f"{name}={metrics[name]:.6f} > {limit:.6f}" for name, limit in limits.items() if metrics[name] > limit]
    workspace = np.asarray([args.workspace_x, args.workspace_y, args.workspace_z])
    axes = "xyz"
    for axis in range(3):
        low, high = workspace[axis]
        if xyz[:, axis].min() < low or xyz[:, axis].max() > high:
            violations.append(f"workspace {axes[axis]} range [{xyz[:, axis].min():.3f}, {xyz[:, axis].max():.3f}] outside [{low:.3f}, {high:.3f}]")
    if violations:
        raise RuntimeError("Safety preflight failed:\n  " + "\n  ".join(violations))
    return metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6001)
    parser.add_argument("--robot-port", type=int, default=50051)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    parser.add_argument("--start-pose-timeout-ms", type=int, default=30000)
    parser.add_argument("--anchor-pose", type=float, nargs=6, metavar=("X", "Y", "Z", "RX", "RY", "RZ"), help="Offline anchor for file-only dry-run")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-episode", type=int)
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--max-start-joint-delta", type=float, default=1.0)
    parser.add_argument("--max-start-joint-error", type=float, default=0.05)
    parser.add_argument("--start-pose-attempts", type=int, default=1)
    parser.add_argument("--max-source-fk-position-error", type=float, default=0.02)
    parser.add_argument("--max-source-fk-rotation-error", type=float, default=0.10)
    parser.add_argument(
        "--return-home-joints-file", type=Path, default=DEFAULT_RETURN_HOME_JOINTS
    )
    parser.add_argument("--return-home-side", choices=("left", "right"), default="left")
    parser.add_argument("--return-home-timeout-ms", type=int, default=30000)
    parser.add_argument("--max-return-home-joint-delta", type=float, default=1.2)
    parser.add_argument("--max-return-home-joint-error", type=float, default=0.05)
    parser.add_argument("--gripper-port", type=int, default=50052)
    parser.add_argument("--gripper-event-delta", type=float, default=0.005)
    parser.add_argument("--gripper-speed", type=float, default=0.05)
    parser.add_argument("--gripper-force", type=float, default=20.0)
    parser.add_argument("--max-step-translation", type=float, default=0.010)
    parser.add_argument("--max-step-rotation", type=float, default=0.050)
    parser.add_argument("--max-translation-path", type=float, default=0.20)
    parser.add_argument("--max-rotation-path", type=float, default=1.0)
    parser.add_argument("--max-anchor-displacement", type=float, default=0.15)
    parser.add_argument("--workspace-x", type=float, nargs=2, default=(0.20, 0.75))
    parser.add_argument("--workspace-y", type=float, nargs=2, default=(-0.45, 0.45))
    parser.add_argument("--workspace-z", type=float, nargs=2, default=(0.15, 0.80))
    parser.add_argument("--tracking-check-every", type=int, default=5)
    parser.add_argument("--max-tracking-position", type=float, default=0.06)
    parser.add_argument("--max-tracking-rotation", type=float, default=0.50)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.speed <= 0 or args.speed > 1:
        raise RuntimeError("--speed must be in (0, 1] for this first replay")
    if args.start_pose_attempts < 1:
        raise RuntimeError("--start-pose-attempts must be at least 1")
    if args.gripper_event_delta <= 0:
        raise RuntimeError("--gripper-event-delta must be positive")
    data = load_trajectory(args.trajectory)
    episode = int(data["episode_index"])
    if args.execute and args.confirm_episode != episode:
        raise RuntimeError(f"Execution requires --confirm-episode {episode}")
    client = None
    controller = None
    gripper_client = None
    start_joints = None
    start_joint_delta = None
    source_fk_position_error = None
    source_fk_rotation_error = None
    controller_running_before_replay = None
    return_home_joints = None
    predicted_return_home_delta = None
    gripper_widths = None
    gripper_events: set[int] = set()
    try:
        if args.anchor_pose is not None:
            if args.execute:
                raise RuntimeError("--anchor-pose cannot be used with --execute")
            anchor = np.asarray(args.anchor_pose, dtype=np.float64)
            mode = "offline"
            max_gripper_width = 0.085
            gripper_widths, gripper_events = gripper_targets(
                data, max_gripper_width, args.gripper_event_delta
            )
        else:
            client = RobotClient(args.host, args.port, args.timeout_ms)
            mode = str(client.request("get_control_mode"))
            if mode != "ee":
                raise RuntimeError(f"Robot node control mode must be 'ee', got {mode!r}")
            controller = ControllerMonitor(args.host, args.robot_port)
            controller_running_before_replay = controller.is_running()
            start_joints = source_start_joints(data)
            return_home_joints = config_home_joints(
                args.return_home_joints_file, args.return_home_side
            )
            predicted_return_home_delta = float(
                np.max(np.abs(source_end_joints(data) - return_home_joints))
            )
            if predicted_return_home_delta > args.max_return_home_joint_delta:
                raise RuntimeError(
                    f"Predicted return-home approach too large: "
                    f"{predicted_return_home_delta:.3f} rad > "
                    f"{args.max_return_home_joint_delta:.3f} rad"
                )
            current_joints = np.asarray(
                client.request("get_joint_state"), dtype=np.float64
            )[:7]
            start_joint_delta = float(np.max(np.abs(current_joints - start_joints)))
            if start_joint_delta > args.max_start_joint_delta:
                raise RuntimeError(
                    f"Source-start approach too large: {start_joint_delta:.3f} rad > "
                    f"{args.max_start_joint_delta:.3f} rad"
                )
            anchor = controller.forward_kinematics_euler(start_joints)
            fk_matrix = pose_matrix(anchor, "euler")
            source_matrix = pose_matrix(
                np.asarray(data["state_pose"][0], dtype=np.float64), "rotvec"
            )
            source_fk_position_error = float(
                np.linalg.norm(fk_matrix[:3, 3] - source_matrix[:3, 3])
            )
            source_fk_rotation_error = rotation_distance(fk_matrix, source_matrix)
            if source_fk_position_error > args.max_source_fk_position_error:
                raise RuntimeError(
                    f"Source joint/FK position mismatch: "
                    f"{source_fk_position_error:.4f} m"
                )
            if source_fk_rotation_error > args.max_source_fk_rotation_error:
                raise RuntimeError(
                    f"Source joint/FK rotation mismatch: "
                    f"{source_fk_rotation_error:.4f} rad"
                )
            gripper_client = GripperInterface(
                ip_address=args.host, port=args.gripper_port
            )
            max_gripper_width = gripper_max_width(gripper_client)
            gripper_widths, gripper_events = gripper_targets(
                data, max_gripper_width, args.gripper_event_delta
            )

        delta = data["delta_pose"][:-1]
        targets = integrate(anchor, delta)
        metrics = preflight(data, targets, args)
        summary = {
            "episode_index": episode,
            "frames": len(targets),
            "anchor_pose_euler": anchor.tolist(),
            "final_pose_euler": matrix_euler(targets[-1]).tolist(),
            "control_mode": mode,
            "controller_running_before_replay": controller_running_before_replay,
            "return_to_droid_start_before_replay": client is not None,
            "source_start_joint_position": start_joints.tolist() if start_joints is not None else None,
            "max_start_joint_delta_rad": start_joint_delta,
            "source_joint_fk_position_error_m": source_fk_position_error,
            "source_joint_fk_rotation_error_rad": source_fk_rotation_error,
            "return_to_config_home_after_success": client is not None,
            "return_home_joint_position": (
                return_home_joints.tolist() if return_home_joints is not None else None
            ),
            "predicted_return_home_joint_delta_rad": predicted_return_home_delta,
            "gripper_replay": True,
            "gripper_max_width_m": max_gripper_width,
            "gripper_initial_width_m": float(gripper_widths[0]),
            "gripper_final_width_m": float(gripper_widths[-1]),
            "gripper_event_count": len(gripper_events),
            "execute": args.execute,
            "speed": args.speed,
            **metrics,
        }
        print(json.dumps(summary, indent=2))
        if not args.execute:
            print(
                "DRY RUN PASSED: execute will return to the DROID frame-0 "
                "joint position first; no robot commands were sent"
            )
            return

        assert (
            client is not None
            and controller is not None
            and start_joints is not None
            and return_home_joints is not None
        )
        start_error = float("inf")
        for attempt in range(1, args.start_pose_attempts + 1):
            print(
                f"SOURCE START {attempt}/{args.start_pose_attempts}: moving to "
                "DROID frame-0 joint position"
            )
            client.request(
                "move_to_joint_positions",
                {
                    "joint_positions": start_joints,
                    "gripper_width": float(gripper_widths[0]),
                    "gripper_speed": args.gripper_speed,
                    "gripper_force": args.gripper_force,
                    "restart_controller": True,
                },
                timeout_ms=args.start_pose_timeout_ms,
            )
            actual_joints = np.asarray(
                client.request("get_joint_state"), dtype=np.float64
            )[:7]
            start_error = float(np.max(np.abs(actual_joints - start_joints)))
            if start_error <= args.max_start_joint_error:
                break
            print(
                f"Source-start residual {start_error:.6f} rad; retrying."
            )
        if start_error > args.max_start_joint_error:
            raise RuntimeError(
                f"Source-start joint error too large: {start_error:.4f} rad > "
                f"{args.max_start_joint_error:.4f} rad"
            )
        controller.ensure_running()
        anchor = current_pose(client)
        targets = integrate(anchor, delta)
        preflight(data, targets, args)
        print(
            f"SOURCE START COMPLETE: max_joint_error={start_error:.6f} rad, "
            f"anchor_pose_euler={anchor.tolist()}"
        )

        dt = np.asarray(data["delta_dt"][:-1], dtype=np.float64)
        if not np.all((dt > 0) & np.isfinite(dt)):
            raise RuntimeError("Invalid delta_dt")
        period_start = time.monotonic()
        for index, target in enumerate(targets):
            if index % args.tracking_check_every == 0 and not controller.is_running():
                raise RuntimeError(
                    f"Cartesian controller stopped before frame {index}; replay aborted"
                )
            if index in gripper_events:
                gripper_client.goto(
                    width=float(gripper_widths[index]),
                    speed=args.gripper_speed,
                    force=args.gripper_force,
                    blocking=False,
                )
            client.request("command_ee_pose", {
                "pose_6d": matrix_euler(target),
                "gripper_width": float(gripper_widths[index]),
                "gripper_speed": args.gripper_speed,
                "gripper_force": args.gripper_force,
                "update_gripper": False,
            })
            if index and index % args.tracking_check_every == 0:
                actual = pose_matrix(current_pose(client), "euler")
                position_error = np.linalg.norm(actual[:3, 3] - target[:3, 3])
                angle_error = rotation_distance(actual, target)
                if position_error > args.max_tracking_position or angle_error > args.max_tracking_rotation:
                    raise RuntimeError(f"Tracking error at frame {index}: {position_error:.3f} m, {angle_error:.3f} rad")
            if index < len(targets) - 1:
                period_start += float(dt[index] / args.speed)
                time.sleep(max(0.0, period_start - time.monotonic()))
        current_joints = np.asarray(
            client.request("get_joint_state"), dtype=np.float64
        )[:7]
        return_delta = float(np.max(np.abs(current_joints - return_home_joints)))
        if return_delta > args.max_return_home_joint_delta:
            raise RuntimeError(
                f"Return-home approach too large: {return_delta:.3f} rad > "
                f"{args.max_return_home_joint_delta:.3f} rad"
            )
        print(
            f"RETURN HOME: moving to {args.return_home_side} config joints "
            f"from {args.return_home_joints_file}"
        )
        client.request(
            "move_to_joint_positions",
            {
                "joint_positions": return_home_joints,
                "restart_controller": True,
            },
            timeout_ms=args.return_home_timeout_ms,
        )
        actual_home_joints = np.asarray(
            client.request("get_joint_state"), dtype=np.float64
        )[:7]
        return_home_error = float(
            np.max(np.abs(actual_home_joints - return_home_joints))
        )
        if return_home_error > args.max_return_home_joint_error:
            raise RuntimeError(
                f"Return-home joint error too large: {return_home_error:.4f} rad > "
                f"{args.max_return_home_joint_error:.4f} rad"
            )
        controller.ensure_running()
        print(
            f"EXECUTION COMPLETE: episode {episode}, {len(targets)} frames, "
            f"gripper_events={len(gripper_events)}, "
            f"return_home_error={return_home_error:.6f} rad"
        )
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    main()
