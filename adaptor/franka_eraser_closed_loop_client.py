#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import pickle
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import zmq

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from franka_capture.cameras.realsense import RealSenseCapture
from franka_capture.config.fr3_single import DEFAULT_CAMERAS, DEFAULT_ROBOT


MAX_GRIPPER_WIDTH = 0.09
DONE_STATUSES = {"done", "completed", "complete"}
DEFAULT_POLICY_START_POSE_FILE = Path("adaptor/franka_eraser_policy_start_pose.json")


def _encode_frame(frame: np.ndarray, seed: int | None = None, route: str = "predict") -> str:
    return json.dumps(
        {
            "route": route,
            "shape": list(frame.shape),
            "dtype": str(frame.dtype),
            "frame_b64": base64.b64encode(np.ascontiguousarray(frame).tobytes()).decode("ascii"),
            "seed": seed,
        }
    )


def normalize_ws_timeout(value: float | None) -> float | None:
    if value is None:
        return None
    return None if value <= 0 else float(value)


def safe_camera_read(camera: RealSenseCapture, *, max_retries: int = 5, retry_sleep: float = 0.15):
    last_error = None
    for _ in range(max_retries):
        try:
            return camera.read()
        except RuntimeError as exc:
            last_error = exc
            time.sleep(retry_sleep)
    raise RuntimeError(f"Camera {camera.name} failed to return a frame: {last_error}")


def prepare_policy_frame(rgb: np.ndarray, resize_to: int | None) -> np.ndarray:
    frame = np.asarray(rgb)
    if frame.dtype != np.uint8:
        frame = np.clip(frame, 0, 255).astype(np.uint8)
    if resize_to is not None:
        if resize_to <= 0:
            raise ValueError("--resize-to must be positive")
        frame = cv2.resize(frame, (resize_to, resize_to), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(frame)


def load_prime_frame(path: Path, resize_to: int | None) -> np.ndarray:
    suffix = path.suffix.lower()
    if suffix in {".mp4", ".mov", ".avi", ".mkv"}:
        cap = cv2.VideoCapture(str(path))
        try:
            ok, frame_bgr = cap.read()
        finally:
            cap.release()
        if not ok or frame_bgr is None:
            raise RuntimeError(f"Failed to read first frame from video: {path}")
    else:
        frame_bgr = cv2.imread(str(path))
        if frame_bgr is None:
            raise RuntimeError(f"Failed to read prime image: {path}")
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    return prepare_policy_frame(frame_rgb, resize_to)


def normalize_action_chunk(payload: Any) -> np.ndarray:
    array = np.asarray(payload, dtype=np.float32)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if array.ndim != 2:
        raise RuntimeError(f"Invalid action shape: {array.shape}; expected (N, 7) or (N, 14)")

    if array.shape[1] == 7:
        chunk = array
    elif array.shape[1] == 14:
        chunk = np.concatenate([array[:, :6], array[:, -1:]], axis=1)
    else:
        raise RuntimeError(f"Invalid action shape: {array.shape}; expected (N, 7) or (N, 14)")

    if not np.all(np.isfinite(chunk)):
        raise RuntimeError("Action chunk contains NaN or inf")
    chunk = chunk.astype(np.float32, copy=True)
    chunk[:, 6] = np.clip(chunk[:, 6], 0.0, MAX_GRIPPER_WIDTH)
    return chunk


def extract_action_chunk(response: dict[str, Any]) -> np.ndarray | None:
    if "action" in response:
        return normalize_action_chunk(response["action"])
    if "actions" in response:
        return normalize_action_chunk(response["actions"])
    return None


def response_indicates_done(response: dict[str, Any]) -> bool:
    status = str(response.get("status", "")).lower()
    if response.get("done") is True:
        return True
    if response.get("has_more") is False or response.get("more") is False:
        return True
    return status in DONE_STATUSES


def angle_delta(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (a - b + np.pi) % (2 * np.pi) - np.pi


def resolve_project_path(path_value: str | Path) -> Path:
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def format_pose(pose: np.ndarray) -> str:
    return np.array2string(np.asarray(pose, dtype=np.float32), precision=4)


def ee_pose_delta(current_pose: np.ndarray, expected_pose: np.ndarray) -> tuple[float, float]:
    current = np.asarray(current_pose, dtype=np.float32)
    expected = np.asarray(expected_pose, dtype=np.float32)
    if current.shape != (6,) or expected.shape != (6,):
        raise RuntimeError(
            f"Expected EE poses with shape (6,), got current={current.shape}, expected={expected.shape}"
        )
    pos_delta = float(np.linalg.norm(current[:3] - expected[:3]))
    rot_delta = float(np.max(np.abs(angle_delta(current[3:6], expected[3:6]))))
    return pos_delta, rot_delta


def load_policy_start_pose(path: Path) -> tuple[np.ndarray, float, dict[str, Any]]:
    if not path.exists():
        raise RuntimeError(
            "Missing policy start pose file; refusing to execute absolute EE policy.\n"
            f"Expected file: {path}\n"
            "Capture it after placing the robot at the eraser task start:\n"
            "  bash 13_run_eraser_closed_loop.sh --capture-policy-start"
        )

    payload = json.loads(path.read_text(encoding="utf-8"))
    pose = np.asarray(payload.get("ee_pose_euler"), dtype=np.float32)
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise RuntimeError(f"Invalid ee_pose_euler in policy start file {path}: shape={pose.shape}")

    gripper_width = float(payload.get("gripper_width", 0.0))
    if not np.isfinite(gripper_width):
        raise RuntimeError(f"Invalid gripper_width in policy start file {path}: {gripper_width}")
    gripper_width = float(np.clip(gripper_width, 0.0, MAX_GRIPPER_WIDTH))
    return pose, gripper_width, payload


def save_policy_start_pose(path: Path, pose: np.ndarray, gripper_width: float) -> None:
    pose = np.asarray(pose, dtype=np.float32)
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise RuntimeError(f"Cannot save invalid policy start EE pose: shape={pose.shape}")

    payload = {
        "format_version": 1,
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "ee_pose_euler": pose.astype(float).tolist(),
        "gripper_width": float(np.clip(gripper_width, 0.0, MAX_GRIPPER_WIDTH)),
        "description": "Task-specific absolute EE policy start for the FR3 eraser cloud policy.",
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def check_policy_start_pose(
    robot: "RobotZMQExecutionClient",
    path: Path,
    *,
    pos_tol: float,
    rot_tol: float,
) -> tuple[np.ndarray, float]:
    expected_pose, expected_width, _ = load_policy_start_pose(path)
    current_pose = robot.get_ee_pose_euler()
    pos_delta, rot_delta = ee_pose_delta(current_pose, expected_pose)

    print("Policy start check:")
    print(f"  file: {path}")
    print(f"  expected EE pose: {format_pose(expected_pose)}")
    print(f"  current  EE pose: {format_pose(current_pose)}")
    print(
        f"  delta: pos={pos_delta:.4f}m / tol={pos_tol:.4f}m, "
        f"rot={rot_delta:.4f}rad / tol={rot_tol:.4f}rad"
    )

    if pos_delta > pos_tol or rot_delta > rot_tol:
        raise RuntimeError(
            "Current EE pose is not close to the saved policy start; "
            "aborting before camera/cloud request.\n"
            f"Expected EE pose: {format_pose(expected_pose)}\n"
            f"Current  EE pose: {format_pose(current_pose)}\n"
            f"Delta: pos={pos_delta:.4f}m (tol {pos_tol:.4f}m), "
            f"rot={rot_delta:.4f}rad (tol {rot_tol:.4f}rad)\n"
            "Move the robot back to the eraser policy start, or recapture it with:\n"
            "  bash 13_run_eraser_closed_loop.sh --capture-policy-start"
        )
    return expected_pose, expected_width


def interpolated_pose(start_pose: np.ndarray, target_pose: np.ndarray, alpha: float) -> np.ndarray:
    start = np.asarray(start_pose, dtype=np.float32)
    target = np.asarray(target_pose, dtype=np.float32)
    pose = start.copy()
    pose[:3] = start[:3] + alpha * (target[:3] - start[:3])
    pose[3:6] = start[3:6] + alpha * angle_delta(target[3:6], start[3:6])
    return pose


def move_to_policy_start_pose(
    robot: "RobotZMQExecutionClient",
    target_pose: np.ndarray,
    target_gripper_width: float,
    *,
    label: str,
    fps: int,
    pos_speed: float,
    rot_speed: float,
    max_pos_delta: float,
    max_rot_delta: float,
    settle_time: float,
    gripper_speed: float,
    gripper_force: float,
    move_gripper: bool,
) -> None:
    if fps <= 0:
        raise ValueError("--move-to-policy-start-fps must be positive")
    if pos_speed <= 0 or rot_speed <= 0:
        raise ValueError("--move-to-policy-start-speed and --move-to-policy-start-rot-speed must be positive")
    if max_pos_delta <= 0 or max_rot_delta <= 0:
        raise ValueError("--move-to-policy-start-max-pos-delta and --move-to-policy-start-max-rot-delta must be positive")

    current_pose = robot.get_ee_pose_euler()
    pos_delta, rot_delta = ee_pose_delta(current_pose, target_pose)
    print(
        f"Move-to-{label} requested: "
        f"pos_delta={pos_delta:.4f}m, rot_delta={rot_delta:.4f}rad"
    )
    if pos_delta > max_pos_delta or rot_delta > max_rot_delta:
        raise RuntimeError(
            f"{label} is too far for automatic slow move.\n"
            f"Current EE pose: {format_pose(current_pose)}\n"
            f"Target  EE pose: {format_pose(target_pose)}\n"
            f"Delta: pos={pos_delta:.4f}m (max {max_pos_delta:.4f}m), "
            f"rot={rot_delta:.4f}rad (max {max_rot_delta:.4f}rad)\n"
            "Move the robot closer manually, or increase the limit only after checking the path is safe."
        )

    duration = max(pos_delta / pos_speed, rot_delta / rot_speed, 1.0 / float(fps))
    steps = max(1, int(np.ceil(duration * fps)))
    print(
        f"Slow moving to {label} over {steps} steps "
        f"({duration:.1f}s at {fps}Hz)."
    )

    for step in range(1, steps + 1):
        loop_start = time.monotonic()
        target_step_pose = interpolated_pose(current_pose, target_pose, step / float(steps))
        robot.command_ee_pose(
            target_step_pose,
            target_gripper_width,
            gripper_speed=gripper_speed,
            gripper_force=gripper_force,
            update_gripper=move_gripper and step == steps,
        )
        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, (1.0 / float(fps)) - elapsed))
        if step == 1 or step == steps or step % max(1, fps * 2) == 0:
            print(f"Move-to-{label} step {step}/{steps}")

    if settle_time > 0:
        time.sleep(settle_time)

    final_pose = robot.get_ee_pose_euler()
    final_pos_delta, final_rot_delta = ee_pose_delta(final_pose, target_pose)
    print(
        f"Arrived near {label}: "
        f"pos_delta={final_pos_delta:.4f}m, rot_delta={final_rot_delta:.4f}rad"
    )


class RobotZMQExecutionClient:
    def __init__(
        self,
        host: str = DEFAULT_ROBOT.host,
        port: int = DEFAULT_ROBOT.port,
        timeout_ms: int = DEFAULT_ROBOT.timeout_ms,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(f"tcp://{host}:{port}")

    def _request(self, method: str, args: dict[str, Any] | None = None) -> Any:
        request = {"method": method, "args": args or {}}
        try:
            self._socket.send(pickle.dumps(request))
            result = pickle.loads(self._socket.recv())
        except zmq.Again as exc:
            raise TimeoutError(
                f"Timed out waiting for robot node at tcp://{self.host}:{self.port}"
            ) from exc

        if isinstance(result, dict) and "error" in result:
            raise RuntimeError(f"Robot node returned error: {result['error']}")
        return result

    def get_joint_state(self) -> np.ndarray:
        return np.asarray(self._request("get_joint_state"), dtype=np.float32)

    def get_control_mode(self) -> str:
        return str(self._request("get_control_mode"))

    def get_observations(self) -> dict[str, Any]:
        return self._request("get_observations")

    def get_ee_pose_euler(self) -> np.ndarray:
        observations = self.get_observations()
        if "ee_pose_euler" not in observations:
            raise RuntimeError("Robot node does not expose ee_pose_euler")
        pose = np.asarray(observations["ee_pose_euler"], dtype=np.float32)
        if pose.shape != (6,):
            raise RuntimeError(f"Invalid ee_pose_euler shape: {pose.shape}")
        return pose

    def get_gripper_width(self) -> float:
        joint_state = self.get_joint_state()
        if joint_state.shape[0] < 8:
            raise RuntimeError(f"Expected joint state length >= 8, got {joint_state.shape}")
        return float(np.clip(joint_state[-1], 0.0, 1.0) * MAX_GRIPPER_WIDTH)

    def command_ee_pose(
        self,
        pose_6d: np.ndarray,
        gripper_width: float,
        *,
        gripper_speed: float,
        gripper_force: float,
        update_gripper: bool,
    ) -> None:
        self._request(
            "command_ee_pose",
            {
                "pose_6d": np.asarray(pose_6d, dtype=np.float32),
                "gripper_width": float(gripper_width),
                "gripper_speed": float(gripper_speed),
                "gripper_force": float(gripper_force),
                "update_gripper": bool(update_gripper),
            },
        )

    def close(self) -> None:
        self._socket.close(0)
        self._context.term()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


def validate_action_chunk(
    actions: np.ndarray,
    current_pose: np.ndarray,
    *,
    max_start_pos_delta: float,
    max_start_rot_delta: float,
    max_step_pos_delta: float,
    max_step_rot_delta: float,
) -> None:
    if actions.ndim != 2 or actions.shape[1] != 7:
        raise RuntimeError(f"Invalid normalized action shape: {actions.shape}")

    start_pos_delta = float(np.linalg.norm(actions[0, :3] - current_pose[:3]))
    start_rot_delta = float(np.max(np.abs(angle_delta(actions[0, 3:6], current_pose[3:6]))))
    if start_pos_delta > max_start_pos_delta:
        raise RuntimeError(
            f"First EE target is too far from current pose: "
            f"pos_delta={start_pos_delta:.4f} > {max_start_pos_delta:.4f}"
        )
    if start_rot_delta > max_start_rot_delta:
        raise RuntimeError(
            f"First EE target rotation is too far from current pose: "
            f"rot_delta={start_rot_delta:.4f} > {max_start_rot_delta:.4f}"
        )

    if len(actions) <= 1:
        return

    step_pos_delta = np.linalg.norm(np.diff(actions[:, :3], axis=0), axis=1)
    step_rot_delta = np.max(np.abs(angle_delta(actions[1:, 3:6], actions[:-1, 3:6])), axis=1)
    max_pos = float(step_pos_delta.max())
    max_rot = float(step_rot_delta.max())
    if max_pos > max_step_pos_delta:
        raise RuntimeError(
            f"Action chunk has a large EE position jump: "
            f"max_step_pos_delta={max_pos:.4f} > {max_step_pos_delta:.4f}"
        )
    if max_rot > max_step_rot_delta:
        raise RuntimeError(
            f"Action chunk has a large EE rotation jump: "
            f"max_step_rot_delta={max_rot:.4f} > {max_step_rot_delta:.4f}"
        )


def summarize_action_chunk(actions: np.ndarray) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "shape": list(actions.shape),
    }
    if actions.ndim != 2 or actions.shape[1] != 7 or actions.shape[0] == 0:
        return summary

    summary["first"] = actions[0].astype(float).tolist()
    summary["last"] = actions[-1].astype(float).tolist()
    summary["min"] = np.min(actions, axis=0).astype(float).tolist()
    summary["max"] = np.max(actions, axis=0).astype(float).tolist()
    if actions.shape[0] > 1:
        step_pos_delta = np.linalg.norm(np.diff(actions[:, :3], axis=0), axis=1)
        step_rot_delta = np.max(
            np.abs(angle_delta(actions[1:, 3:6], actions[:-1, 3:6])),
            axis=1,
        )
        summary["max_step_pos_delta"] = float(step_pos_delta.max())
        summary["max_step_rot_delta"] = float(step_rot_delta.max())
        summary["max_step_pos_index"] = int(step_pos_delta.argmax() + 1)
        summary["max_step_rot_index"] = int(step_rot_delta.argmax() + 1)
    return summary


def write_run_artifacts(
    output_dir: Path,
    raw_responses: list[dict[str, Any]],
    action_summaries: list[dict[str, Any]],
    received_chunks: list[np.ndarray],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "cloud_policy_responses.json").write_text(
        json.dumps(raw_responses, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    (output_dir / "action_summaries.json").write_text(
        json.dumps(action_summaries, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    save_actions(output_dir, received_chunks)


def execute_action_chunk(
    robot: RobotZMQExecutionClient | None,
    actions: np.ndarray,
    *,
    execute: bool,
    fps: int,
    chunk_index: int,
    gripper_speed: float,
    gripper_force: float,
    close_threshold: float,
    continuous_gripper: bool,
    max_start_pos_delta: float,
    max_start_rot_delta: float,
    max_step_pos_delta: float,
    max_step_rot_delta: float,
) -> None:
    if fps <= 0:
        raise ValueError("--fps must be positive")

    current_pose = robot.get_ee_pose_euler() if robot is not None else actions[0, :6]
    start_pos_delta = float(np.linalg.norm(actions[0, :3] - current_pose[:3]))
    start_rot_delta = float(np.max(np.abs(angle_delta(actions[0, 3:6], current_pose[3:6]))))
    print(
        "First target delta from current EE: "
        f"pos={start_pos_delta:.4f}m, rot={start_rot_delta:.4f}rad"
    )
    validate_action_chunk(
        actions,
        current_pose,
        max_start_pos_delta=max_start_pos_delta,
        max_start_rot_delta=max_start_rot_delta,
        max_step_pos_delta=max_step_pos_delta,
        max_step_rot_delta=max_step_rot_delta,
    )

    if robot is None:
        print(f"Chunk {chunk_index}: robot check skipped, no command will be sent.")
        return

    current_width = robot.get_gripper_width()
    gripper_state = "close" if current_width < close_threshold else "open"
    mode = "EXECUTE" if execute else "DRY-RUN"
    print(
        f"Chunk {chunk_index}: {mode}, actions={actions.shape[0]}, "
        f"current_width={current_width:.4f}m, gripper_state={gripper_state}"
    )

    if not execute:
        return

    for action_index, target in enumerate(actions, start=1):
        loop_start = time.monotonic()
        pose_6d = target[:6]
        gripper_width = float(np.clip(target[6], 0.0, MAX_GRIPPER_WIDTH))
        target_gripper_state = "close" if gripper_width < close_threshold else "open"
        update_gripper = continuous_gripper or target_gripper_state != gripper_state

        robot.command_ee_pose(
            pose_6d,
            gripper_width,
            gripper_speed=gripper_speed,
            gripper_force=gripper_force,
            update_gripper=update_gripper,
        )
        if update_gripper:
            gripper_state = target_gripper_state

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, (1.0 / float(fps)) - elapsed))
        if action_index == 1 or action_index == actions.shape[0] or action_index % 10 == 0:
            print(f"Executed chunk {chunk_index} action {action_index}/{actions.shape[0]}")


def maybe_bridge_to_first_action(
    robot: RobotZMQExecutionClient | None,
    actions: np.ndarray,
    *,
    enabled: bool,
    max_start_pos_delta: float,
    max_start_rot_delta: float,
    fps: int,
    pos_speed: float,
    rot_speed: float,
    max_pos_delta: float,
    max_rot_delta: float,
    settle_time: float,
    gripper_speed: float,
    gripper_force: float,
    move_gripper: bool,
) -> bool:
    if not enabled:
        return False
    if robot is None:
        return False

    current_pose = robot.get_ee_pose_euler()
    first_target_pose = actions[0, :6]
    pos_delta, rot_delta = ee_pose_delta(current_pose, first_target_pose)
    if pos_delta <= max_start_pos_delta and rot_delta <= max_start_rot_delta:
        return False

    print(
        "Startup prealign target is outside normal execution threshold; "
        "bridging slowly and discarding this stale chunk. "
        f"pos_delta={pos_delta:.4f}m, rot_delta={rot_delta:.4f}rad"
    )
    move_to_policy_start_pose(
        robot,
        first_target_pose,
        float(np.clip(actions[0, 6], 0.0, MAX_GRIPPER_WIDTH)),
        label="startup prealign target",
        fps=fps,
        pos_speed=pos_speed,
        rot_speed=rot_speed,
        max_pos_delta=max_pos_delta,
        max_rot_delta=max_rot_delta,
        settle_time=settle_time,
        gripper_speed=gripper_speed,
        gripper_force=gripper_force,
        move_gripper=move_gripper,
    )
    print("Discarded startup prealign chunk; a fresh camera frame will be sent before executing policy actions.")
    return True


async def maybe_reset_remote_policy(ws, *, enabled: bool) -> None:
    if not enabled:
        return
    await ws.send(json.dumps({"route": "/reset"}))
    try:
        message = await asyncio.wait_for(ws.recv(), timeout=10.0)
        print(f"Remote policy reset response: {message[:160]}")
    except asyncio.TimeoutError:
        print("Remote policy reset timed out; continuing.")


async def send_priming_frame(
    ws,
    frame_rgb: np.ndarray,
    *,
    route: str,
    seed: int,
    output_dir: Path,
) -> None:
    await ws.send(_encode_frame(frame_rgb, seed=seed, route=route))
    message = await ws.recv()
    try:
        response = json.loads(message)
    except json.JSONDecodeError:
        response = {"raw": message}
    (output_dir / "prime_response.json").write_text(
        json.dumps(response, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print("Sent priming frame and ignored its returned action.")


async def receive_policy_chunk(
    ws,
    *,
    recv_timeout: float,
    response_log: list[dict[str, Any]],
) -> tuple[dict[str, Any], np.ndarray | None]:
    while True:
        message = await asyncio.wait_for(ws.recv(), timeout=recv_timeout)
        try:
            response = json.loads(message)
        except json.JSONDecodeError:
            response = {"raw": message}
        response_log.append(response)

        chunk = extract_action_chunk(response) if isinstance(response, dict) else None
        if chunk is not None or (isinstance(response, dict) and response_indicates_done(response)):
            return response, chunk


def save_actions(output_dir: Path, chunks: list[np.ndarray]) -> None:
    if not chunks:
        return
    try:
        actions = np.stack(chunks, axis=0)
        np.save(output_dir / "cloud_actions.npy", actions)
    except ValueError:
        np.save(output_dir / "cloud_actions.npy", np.asarray(chunks, dtype=object), allow_pickle=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Closed-loop cloud policy client for FR3 eraser task.")
    parser.add_argument("--uri", default="ws://127.0.0.1:8765")
    parser.add_argument("--route", default="predict")
    parser.add_argument("--camera-name", default="right")
    parser.add_argument("--resize-to", type=int, default=None, help="Optional square resize before sending to cloud, e.g. 224.")
    parser.add_argument("--prime-frame-path", default=None)
    parser.add_argument("--prime-route", default=None)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--output-root", default="output/franka_eraser_closed_loop")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--max-chunks", type=int, default=None)
    parser.add_argument("--execute", action="store_true", help="Actually send EE commands to the robot node.")
    parser.add_argument(
        "--capture-policy-start",
        action="store_true",
        help="Save the current EE pose/gripper as the eraser policy start pose, then exit.",
    )
    parser.add_argument(
        "--policy-start-pose-file",
        default=str(DEFAULT_POLICY_START_POSE_FILE),
        help="Task-specific absolute EE policy start JSON. Relative paths are resolved from the repo root.",
    )
    parser.add_argument("--policy-start-pos-tol", type=float, default=0.03)
    parser.add_argument("--policy-start-rot-tol", type=float, default=0.25)
    parser.add_argument(
        "--move-to-policy-start",
        action="store_true",
        help="Before camera/cloud request, slowly move to the saved policy start pose, then re-check.",
    )
    parser.add_argument("--move-to-policy-start-fps", type=int, default=5)
    parser.add_argument("--move-to-policy-start-speed", type=float, default=0.03)
    parser.add_argument("--move-to-policy-start-rot-speed", type=float, default=0.25)
    parser.add_argument("--move-to-policy-start-max-pos-delta", type=float, default=0.25)
    parser.add_argument("--move-to-policy-start-max-rot-delta", type=float, default=1.2)
    parser.add_argument("--move-to-policy-start-settle", type=float, default=0.5)
    parser.add_argument(
        "--move-to-policy-start-gripper",
        action="store_true",
        help="Also move gripper to the saved policy-start width at the final pre-move step.",
    )
    parser.add_argument("--skip-robot-check", action="store_true", help="Do not connect to robot node; camera/cloud only.")
    parser.add_argument("--skip-policy-reset", action="store_true")
    parser.add_argument("--skip-first-chunk", action="store_true")
    parser.add_argument("--camera-warmup", type=float, default=1.5)
    parser.add_argument("--camera-flush-frames", type=int, default=10)
    parser.add_argument("--recv-timeout", type=float, default=180.0)
    parser.add_argument("--ws-ping-interval", type=float, default=0.0)
    parser.add_argument("--ws-ping-timeout", type=float, default=0.0)
    parser.add_argument("--ws-close-timeout", type=float, default=10.0)
    parser.add_argument("--robot-host", default=DEFAULT_ROBOT.host)
    parser.add_argument("--robot-port", type=int, default=DEFAULT_ROBOT.port)
    parser.add_argument("--robot-timeout-ms", type=int, default=DEFAULT_ROBOT.timeout_ms)
    parser.add_argument("--gripper-speed", type=float, default=0.05)
    parser.add_argument("--gripper-force", type=float, default=40.0)
    parser.add_argument("--close-threshold", type=float, default=0.04)
    parser.add_argument("--continuous-gripper", action="store_true")
    parser.add_argument("--max-start-pos-delta", type=float, default=0.08)
    parser.add_argument("--max-start-rot-delta", type=float, default=0.5)
    parser.add_argument("--max-step-pos-delta", type=float, default=0.08)
    parser.add_argument("--max-step-rot-delta", type=float, default=0.5)
    parser.add_argument(
        "--bridge-to-first-action",
        action="store_true",
        help=(
            "If the first returned action is outside the normal start threshold, "
            "slowly move to that first target, discard the chunk, then request a fresh chunk."
        ),
    )
    parser.add_argument("--bridge-to-first-action-fps", type=int, default=5)
    parser.add_argument("--bridge-to-first-action-speed", type=float, default=0.03)
    parser.add_argument("--bridge-to-first-action-rot-speed", type=float, default=0.25)
    parser.add_argument("--bridge-to-first-action-max-pos-delta", type=float, default=0.25)
    parser.add_argument("--bridge-to-first-action-max-rot-delta", type=float, default=1.2)
    parser.add_argument("--bridge-to-first-action-settle", type=float, default=0.5)
    parser.add_argument(
        "--max-startup-bridge-attempts",
        type=int,
        default=2,
        help="Maximum startup prealign bridge chunks before official execution starts.",
    )
    parser.add_argument(
        "--bridge-to-first-action-gripper",
        action="store_true",
        help="Also move gripper to the first action width during the bridge move.",
    )
    return parser


async def main() -> int:
    args = build_parser().parse_args()
    requires_robot = args.execute or args.capture_policy_start
    policy_start_path = resolve_project_path(args.policy_start_pose_file)

    if args.capture_policy_start and args.execute:
        raise ValueError("--capture-policy-start captures the current pose and exits; do not combine it with --execute")
    if requires_robot and args.skip_robot_check:
        raise ValueError("--skip-robot-check cannot be used with --execute or --capture-policy-start")
    if args.max_chunks is not None and args.max_chunks <= 0:
        raise ValueError("--max-chunks must be positive")
    if args.policy_start_pos_tol <= 0 or args.policy_start_rot_tol <= 0:
        raise ValueError("--policy-start-pos-tol and --policy-start-rot-tol must be positive")
    if args.move_to_policy_start and not args.execute:
        raise ValueError("--move-to-policy-start requires --execute")
    if args.move_to_policy_start_settle < 0:
        raise ValueError("--move-to-policy-start-settle must be non-negative")
    if args.bridge_to_first_action and not args.execute:
        raise ValueError("--bridge-to-first-action requires --execute")
    if args.bridge_to_first_action_settle < 0:
        raise ValueError("--bridge-to-first-action-settle must be non-negative")
    if args.max_startup_bridge_attempts < 0:
        raise ValueError("--max-startup-bridge-attempts must be non-negative")

    if not args.capture_policy_start and args.camera_name not in DEFAULT_CAMERAS:
        raise RuntimeError(
            f"Unknown camera '{args.camera_name}'. Available cameras: {sorted(DEFAULT_CAMERAS)}"
        )

    robot: RobotZMQExecutionClient | None = None
    if not args.skip_robot_check:
        try:
            robot = RobotZMQExecutionClient(args.robot_host, args.robot_port, args.robot_timeout_ms)
            print(f"Robot node: tcp://{args.robot_host}:{args.robot_port}")
            print(f"Current EE pose: {np.array2string(robot.get_ee_pose_euler(), precision=4)}")
            print(f"Current gripper width: {robot.get_gripper_width():.4f} m")
            if requires_robot:
                control_mode = robot.get_control_mode()
                print(f"Robot control mode: {control_mode}")
                if control_mode != "ee":
                    raise RuntimeError(
                        f"Robot node is running control_mode={control_mode!r}; "
                        "eraser policy execution/capture requires control_mode='ee'. "
                        "Stop the existing node and start it with:\n"
                        "  bash 3_launch_node.sh --control-mode ee"
                    )
        except (TimeoutError, RuntimeError) as exc:
            if robot is not None:
                robot.close()
                robot = None
            if requires_robot:
                raise RuntimeError(
                    "Robot node is required for --execute or --capture-policy-start. Start it with:\n"
                    "  bash 3_launch_node.sh --control-mode ee\n"
                    f"Original error: {exc}"
                ) from exc
            print(
                "Warning: robot node check failed; continuing camera/cloud dry-run only. "
                f"Original error: {exc}"
            )
    elif requires_robot:
        raise ValueError("Internal error: execute cannot run without robot")

    if args.capture_policy_start:
        if robot is None:
            raise RuntimeError("Internal error: capture requested without a robot connection")
        pose = robot.get_ee_pose_euler()
        gripper_width = robot.get_gripper_width()
        save_policy_start_pose(policy_start_path, pose, gripper_width)
        print(f"Saved eraser policy start pose to: {policy_start_path}")
        print(f"  EE pose: {format_pose(pose)}")
        print(f"  gripper width: {gripper_width:.4f} m")
        robot.close()
        return 0

    if args.execute:
        if robot is None:
            raise RuntimeError("Internal error: execute requested without a robot connection")
        if args.move_to_policy_start:
            expected_pose, expected_width, _ = load_policy_start_pose(policy_start_path)
            move_to_policy_start_pose(
                robot,
                expected_pose,
                expected_width,
                label="policy start",
                fps=args.move_to_policy_start_fps,
                pos_speed=args.move_to_policy_start_speed,
                rot_speed=args.move_to_policy_start_rot_speed,
                max_pos_delta=args.move_to_policy_start_max_pos_delta,
                max_rot_delta=args.move_to_policy_start_max_rot_delta,
                settle_time=args.move_to_policy_start_settle,
                gripper_speed=args.gripper_speed,
                gripper_force=args.gripper_force,
                move_gripper=args.move_to_policy_start_gripper,
            )
        check_policy_start_pose(
            robot,
            policy_start_path,
            pos_tol=args.policy_start_pos_tol,
            rot_tol=args.policy_start_rot_tol,
        )

    try:
        import websockets
    except ImportError as exc:
        raise RuntimeError(
            "Missing dependency 'websockets'. Install it in the runtime environment, "
            "for example: conda activate franka_capture && python -m pip install websockets"
        ) from exc

    run_id = args.run_id or time.strftime("franka_eraser_%Y%m%d_%H%M%S")
    output_dir = Path(args.output_root).expanduser() / run_id
    output_dir.mkdir(parents=True, exist_ok=True)

    camera_config = DEFAULT_CAMERAS[args.camera_name].to_dict()
    camera = RealSenseCapture(**camera_config)
    raw_responses: list[dict[str, Any]] = []
    received_chunks: list[np.ndarray] = []
    action_summaries: list[dict[str, Any]] = []

    try:
        print(f"Opening camera {args.camera_name}; warming up for {args.camera_warmup:.1f}s")
        time.sleep(max(0.0, args.camera_warmup))
        for _ in range(max(0, args.camera_flush_frames)):
            safe_camera_read(camera, max_retries=2, retry_sleep=0.05)

        first_rgb, _ = safe_camera_read(camera)
        cv2.imwrite(str(output_dir / "camera_initial.png"), first_rgb[:, :, ::-1])
        first_policy_rgb = prepare_policy_frame(first_rgb, args.resize_to)

        async with websockets.connect(
            args.uri,
            max_size=None,
            ping_interval=normalize_ws_timeout(args.ws_ping_interval),
            ping_timeout=normalize_ws_timeout(args.ws_ping_timeout),
            close_timeout=normalize_ws_timeout(args.ws_close_timeout),
        ) as ws:
            await maybe_reset_remote_policy(ws, enabled=not args.skip_policy_reset)

            if args.prime_frame_path:
                prime_rgb = load_prime_frame(Path(args.prime_frame_path).expanduser(), args.resize_to)
                await send_priming_frame(
                    ws,
                    prime_rgb,
                    route=args.prime_route or args.route,
                    seed=args.seed,
                    output_dir=output_dir,
                )

            await ws.send(_encode_frame(first_policy_rgb, seed=args.seed, route=args.route))
            print("Sent initial frame to cloud policy.")

            chunk_index = 0
            accepted_chunk_count = 0
            startup_bridge_count = 0
            bridge_disabled_logged = False
            while True:
                response, chunk = await receive_policy_chunk(
                    ws,
                    recv_timeout=args.recv_timeout,
                    response_log=raw_responses,
                )

                if chunk is not None:
                    chunk_index += 1
                    received_chunks.append(chunk)
                    summary = summarize_action_chunk(chunk)
                    summary["chunk_index"] = chunk_index
                    action_summaries.append(summary)
                    print(f"Received action chunk #{chunk_index}: shape={tuple(chunk.shape)}")
                    if "max_step_pos_delta" in summary:
                        print(
                            "Action summary: "
                            f"max_step_pos_delta={summary['max_step_pos_delta']:.4f}, "
                            f"max_step_rot_delta={summary['max_step_rot_delta']:.4f}"
                        )
                    if args.skip_first_chunk and chunk_index == 1:
                        summary["status"] = "skipped"
                        write_run_artifacts(output_dir, raw_responses, action_summaries, received_chunks)
                        print("Skipping first chunk by request.")
                    elif (
                        args.bridge_to_first_action
                        and accepted_chunk_count == 0
                        and startup_bridge_count < args.max_startup_bridge_attempts
                        and maybe_bridge_to_first_action(
                            robot,
                            chunk,
                            enabled=True,
                            max_start_pos_delta=args.max_start_pos_delta,
                            max_start_rot_delta=args.max_start_rot_delta,
                            fps=args.bridge_to_first_action_fps,
                            pos_speed=args.bridge_to_first_action_speed,
                            rot_speed=args.bridge_to_first_action_rot_speed,
                            max_pos_delta=args.bridge_to_first_action_max_pos_delta,
                            max_rot_delta=args.bridge_to_first_action_max_rot_delta,
                            settle_time=args.bridge_to_first_action_settle,
                            gripper_speed=args.gripper_speed,
                            gripper_force=args.gripper_force,
                            move_gripper=args.bridge_to_first_action_gripper,
                        )
                    ):
                        startup_bridge_count += 1
                        summary["status"] = "startup_bridge_discarded"
                        summary["startup_bridge_attempt"] = startup_bridge_count
                        write_run_artifacts(output_dir, raw_responses, action_summaries, received_chunks)
                        print(
                            f"Startup prealign chunk #{startup_bridge_count}/"
                            f"{args.max_startup_bridge_attempts} discarded."
                        )
                    else:
                        if (
                            args.bridge_to_first_action
                            and accepted_chunk_count > 0
                            and not bridge_disabled_logged
                        ):
                            print("Execution has started; bridge disabled for subsequent chunks.")
                            bridge_disabled_logged = True
                        elif (
                            args.bridge_to_first_action
                            and accepted_chunk_count == 0
                            and startup_bridge_count >= args.max_startup_bridge_attempts
                        ):
                            print(
                                "Startup bridge attempt limit reached; next chunk must pass normal "
                                "execution safety checks."
                            )
                        summary["status"] = "execute_attempted" if args.execute else "dry_run"
                        write_run_artifacts(output_dir, raw_responses, action_summaries, received_chunks)
                        execute_action_chunk(
                            robot,
                            chunk,
                            gripper_speed=args.gripper_speed,
                            gripper_force=args.gripper_force,
                            execute=args.execute,
                            fps=args.fps,
                            chunk_index=chunk_index,
                            close_threshold=args.close_threshold,
                            continuous_gripper=args.continuous_gripper,
                            max_start_pos_delta=args.max_start_pos_delta,
                            max_start_rot_delta=args.max_start_rot_delta,
                            max_step_pos_delta=args.max_step_pos_delta,
                            max_step_rot_delta=args.max_step_rot_delta,
                        )
                        summary["status"] = "executed" if args.execute else "dry_run"
                        write_run_artifacts(output_dir, raw_responses, action_summaries, received_chunks)
                        accepted_chunk_count += 1

                if response_indicates_done(response):
                    print("Cloud policy indicated done.")
                    break
                if args.max_chunks is not None and accepted_chunk_count >= args.max_chunks:
                    print(f"Reached --max-chunks={args.max_chunks}; stopping closed loop.")
                    break

                live_rgb, _ = safe_camera_read(camera, max_retries=5, retry_sleep=0.1)
                live_policy_rgb = prepare_policy_frame(live_rgb, args.resize_to)
                next_seed = args.seed + chunk_index
                await ws.send(_encode_frame(live_policy_rgb, seed=next_seed, route=args.route))
                print(f"Sent live frame after chunk {chunk_index}; waiting for next chunk.")

    finally:
        camera.close()
        if robot is not None:
            robot.close()

    write_run_artifacts(output_dir, raw_responses, action_summaries, received_chunks)
    if not received_chunks:
        raise RuntimeError("Cloud policy did not return any action chunks")

    print(f"Saved closed-loop run artifacts to: {output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        print("\nInterrupted.")
        raise SystemExit(130)
