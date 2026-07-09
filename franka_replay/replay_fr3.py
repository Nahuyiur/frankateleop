"""Replay a recorded single-arm FR3 joint trajectory through robot node ZMQ."""

import argparse
import gzip
import json
import pickle
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import zmq

from franka_capture.gripper_fields import (
    GRIPPER_CLOSED_THRESHOLD,
    MAX_GRIPPER_WIDTH,
    frame_gripper_target_width,
)

GRIPPER_BINARY_THRESHOLD = GRIPPER_CLOSED_THRESHOLD


def gripper_width_to_command(width: float) -> float:
    return float(np.clip(1.0 - (float(width) / MAX_GRIPPER_WIDTH), 0.0, 1.0))


def gripper_command_to_width(command: float) -> float:
    return float(MAX_GRIPPER_WIDTH * (1.0 - np.clip(float(command), 0.0, 1.0)))


class RobotZMQReplayClient:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 6001,
        timeout_ms: int = 2000,
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

    def _request(self, method: str, args: Optional[Dict[str, Any]] = None) -> Any:
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

    def num_dofs(self) -> int:
        return int(self._request("num_dofs"))

    def get_joint_state(self) -> np.ndarray:
        return np.asarray(self._request("get_joint_state"), dtype=float)

    def command_joint_state(
        self,
        joint_state: np.ndarray,
        gripper_speed: float,
        gripper_force: float,
        update_gripper: bool = True,
    ) -> None:
        self._request(
            "command_joint_state",
            {
                "joint_state": joint_state,
                "gripper_speed": gripper_speed,
                "gripper_force": gripper_force,
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


def _add_polymetis_build_to_path() -> None:
    build_dir = Path(__file__).resolve().parents[1] / "polymetis" / "polymetis" / "build"
    if build_dir.exists() and str(build_dir) not in sys.path:
        sys.path.insert(0, str(build_dir))


class GripperDirectClient:
    """Small gRPC client that writes width-move commands to the gripper server."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 50052,
        timeout_ms: int = 2000,
    ) -> None:
        _add_polymetis_build_to_path()
        try:
            import grpc
            import polymetis_pb2
            import polymetis_pb2_grpc
        except ImportError as exc:
            raise RuntimeError(
                "Direct gripper replay needs grpc/protobuf from the polymetis environment. "
                "Run through 7_replay_fr3.sh so conda activates polymetis."
            ) from exc

        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self._grpc = grpc
        self._pb2 = polymetis_pb2
        self._channel = grpc.insecure_channel(f"{host}:{port}")
        try:
            grpc.channel_ready_future(self._channel).result(timeout=timeout_ms / 1000.0)
        except grpc.FutureTimeoutError as exc:
            raise TimeoutError(
                f"Timed out waiting for gripper server at {host}:{port}. "
                "Start 2_launch_gripper.sh before executing replay."
            ) from exc

        self._stub = polymetis_pb2_grpc.GripperServerStub(self._channel)

    def get_width(self) -> float:
        state = self._stub.GetState(self._pb2.Empty(), timeout=self.timeout_ms / 1000.0)
        return float(state.width)

    def goto_width(self, width: float, speed: float, force: float) -> None:
        width = float(np.clip(width, 0.0, MAX_GRIPPER_WIDTH))
        cmd = self._pb2.GripperCommand(
            width=width,
            speed=float(speed),
            force=float(force),
            grasp=False,
        )
        cmd.timestamp.GetCurrentTime()
        try:
            self._stub.Goto(cmd, timeout=self.timeout_ms / 1000.0)
        except self._grpc.RpcError as exc:
            raise RuntimeError(f"Failed to command gripper width {width:.6f}: {exc}") from exc

    def close(self) -> None:
        self._channel.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "episode",
        help=(
            "Episode directory, metadata.json, or .pkl.gz file. Supports old "
            "task/index and current task/Quality/index layouts."
        ),
    )
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--timeout-ms", type=int, default=2000)
    parser.add_argument(
        "--arm",
        choices=("auto", "left", "right"),
        default="auto",
        help="Single-arm side. auto uses metadata arm_side when available.",
    )
    parser.add_argument(
        "--latest",
        action="store_true",
        help="If the input is a task or quality directory containing multiple episodes, replay the newest one.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send the recorded trajectory to the robot node.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Replay speed multiplier. 0.5 is half speed; 1.0 is original timing.",
    )
    parser.add_argument(
        "--max-start-delta",
        type=float,
        default=0.25,
        help=(
            "If current joint state is farther from frame 0 than this many "
            "radians, abort unless --execute --approach-start is enabled."
        ),
    )
    parser.add_argument(
        "--approach-start",
        action="store_true",
        help=(
            "If --execute starts farther than --max-start-delta, slowly move to "
            "frame 0 first, then start replay."
        ),
    )
    parser.add_argument(
        "--approach-start-max-delta",
        type=float,
        default=0.75,
        help="Refuse auto-approach if any joint is farther than this many radians.",
    )
    parser.add_argument(
        "--approach-start-step-delta",
        type=float,
        default=0.02,
        help="Maximum per-step joint delta during auto-approach to frame 0.",
    )
    parser.add_argument(
        "--approach-start-hz",
        type=float,
        default=5.0,
        help="Command rate for auto-approach interpolation.",
    )
    parser.add_argument(
        "--gripper-speed",
        type=float,
        default=0.1,
        help="Gripper goto speed used during replay.",
    )
    parser.add_argument(
        "--gripper-force",
        type=float,
        default=10.0,
        help="Gripper goto force used during replay.",
    )
    parser.add_argument(
        "--gripper-host",
        default=None,
        help="Host for the gripper gRPC server launched by 2_launch_gripper.sh.",
    )
    parser.add_argument(
        "--gripper-port",
        type=int,
        default=None,
        help="Port for the gripper gRPC server launched by 2_launch_gripper.sh.",
    )
    parser.add_argument(
        "--gripper-event-delta",
        type=float,
        default=0.01,
        help="Emit a direct gripper command when recorded target width changes by at least this many meters.",
    )
    parser.add_argument(
        "--gripper-replay-mode",
        choices=("event", "continuous"),
        default="event",
        help="event sends direct gripper commands only at width-change events; continuous sends sampled width commands.",
    )
    parser.add_argument(
        "--gripper-command-hz",
        type=float,
        default=15.0,
        help="Direct gripper command rate used by --gripper-replay-mode continuous.",
    )
    parser.add_argument(
        "--gripper-hold-sec",
        type=float,
        default=0.0,
        help="Pause the joint replay timeline after each direct gripper event.",
    )
    parser.add_argument(
        "--skip-robot-check",
        action="store_true",
        help="Only inspect the recorded file without connecting to the robot node. Not allowed with --execute.",
    )
    return parser.parse_args()


def resolve_episode_file(path_like: str, *, latest: bool = False) -> Path:
    path = Path(path_like).expanduser()
    if path.is_file():
        if path.name.endswith(".pkl.gz"):
            return path
        if path.name in {"metadata.json", "keyframes.json", "instruction.txt"}:
            return resolve_episode_file(str(path.parent), latest=latest)
        raise ValueError(f"Replay input file must end with .pkl.gz: {path}")
    if not path.is_dir():
        raise FileNotFoundError(f"Episode path does not exist: {path}")

    direct = _resolve_episode_dir(path)
    if direct is not None:
        return direct

    candidates = _discover_episode_files(path)
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise FileNotFoundError(
            f"No .pkl.gz file found under replay input: {path}. "
            "Pass an episode directory such as task/High_Quality/0, metadata.json, "
            "or use --latest on a task/quality directory."
        )
    if latest:
        return max(candidates, key=lambda candidate: candidate.stat().st_mtime)
    raise RuntimeError(
        f"Multiple .pkl.gz files found under {path}; pass the exact episode path "
        "or add --latest. Candidates include:\n"
        + "\n".join(f"  {candidate}" for candidate in candidates[:12])
    )


def _resolve_episode_dir(path: Path) -> Optional[Path]:
    preferred = path / f"{path.name}.pkl.gz"
    if preferred.exists():
        return preferred
    candidates = sorted(path.glob("*.pkl.gz"))
    if len(candidates) == 1:
        return candidates[0]
    return None


def _discover_episode_files(path: Path) -> List[Path]:
    candidates: Dict[Path, None] = {}
    for pattern in ("*.pkl.gz", "*/*.pkl.gz", "*/*/*.pkl.gz"):
        for candidate in path.glob(pattern):
            if candidate.is_file():
                candidates[candidate] = None
    return sorted(candidates)


def load_episode(path: Path):
    with gzip.open(path, "rb") as f:
        payload = pickle.load(f)

    if not isinstance(payload, dict):
        raise ValueError(f"Invalid episode payload in {path}: expected dict")

    frames = payload.get("data")
    if frames is None:
        frames = payload.get("frames")
    if frames is None:
        raise ValueError(f"Invalid episode payload in {path}: expected data or frames")
    if not frames:
        raise ValueError(f"Episode has no frames: {path}")
    return payload, frames


def load_episode_metadata(episode_path: Path) -> Dict[str, Any]:
    metadata_path = episode_path.parent / "metadata.json"
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return metadata if isinstance(metadata, dict) else {}


def infer_episode_kind(frames, metadata: Dict[str, Any]) -> str:
    first_frame = frames[0] if isinstance(frames[0], dict) else {}
    schema = str(metadata.get("schema_version", "") or first_frame.get("schema_version", ""))
    if schema.startswith("franka_dual"):
        return "dual"
    for frame in frames[:20]:
        if not isinstance(frame, dict):
            continue
        if "left_joint" in frame and "right_joint" in frame:
            return "dual"
        if "joint" in frame:
            return "single"
    raise ValueError("Could not infer replay kind from episode frame fields")


def infer_single_arm_side(args, metadata: Dict[str, Any]) -> str:
    if args.arm != "auto":
        return args.arm
    arm_side = str(metadata.get("arm_side", "") or "").strip().lower()
    if arm_side in {"left", "right"}:
        return arm_side
    return "left"


def apply_single_endpoint_defaults(args, metadata: Dict[str, Any], arm_side: str) -> None:
    robot_metadata = metadata.get("robot") if isinstance(metadata.get("robot"), dict) else {}
    metadata_host = robot_metadata.get("host")
    metadata_port = robot_metadata.get("port")

    if args.host is None:
        args.host = str(metadata_host or "127.0.0.1")
    if args.port is None:
        args.port = int(metadata_port or 6001)

    if args.gripper_host is None:
        args.gripper_host = args.host if arm_side == "right" else "127.0.0.1"
    if args.gripper_port is None:
        args.gripper_port = 50053 if arm_side == "right" else 50052


def extract_trajectory(frames):
    joints = np.asarray([frame["joint"] for frame in frames], dtype=float)
    gripper_commands, gripper_widths = _extract_gripper_command_and_width(frames)
    timestamps = np.asarray([frame["timestamp"] for frame in frames], dtype=float)

    if joints.ndim != 2 or joints.shape[1] != 7:
        raise ValueError(f"Expected joint trajectory shape (N, 7), got {joints.shape}")
    if gripper_widths.ndim != 1 or gripper_widths.shape[0] != joints.shape[0]:
        raise ValueError("Invalid gripper trajectory shape")
    if timestamps.ndim != 1 or timestamps.shape[0] != joints.shape[0]:
        raise ValueError("Invalid timestamp trajectory shape")
    if np.any(np.diff(timestamps) < 0):
        raise ValueError("Episode timestamps must be monotonically nondecreasing")

    commands = np.concatenate([joints, gripper_commands[:, None]], axis=1)
    return joints, gripper_widths, timestamps, commands


def _extract_gripper_command_and_width(frames) -> Tuple[np.ndarray, np.ndarray]:
    commands = []
    target_widths = []
    for frame in frames:
        command, target_width = _frame_gripper_command_and_width(frame)
        commands.append(command)
        target_widths.append(target_width)
    return (
        np.asarray(commands, dtype=float),
        np.asarray(target_widths, dtype=float),
    )


def _frame_gripper_command_and_width(frame: Dict[str, Any]) -> Tuple[float, float]:
    target_width = frame_gripper_target_width(frame)
    return gripper_width_to_command(target_width), _validate_gripper_width(target_width)


def _validate_gripper_width(width: float) -> float:
    width = float(width)
    if width < -1e-4 or width > MAX_GRIPPER_WIDTH + 1e-3:
        raise ValueError(
            "Recorded gripper target width is outside the Franka Hand range: "
            f"{width:.6f} m"
        )
    return float(np.clip(width, 0.0, MAX_GRIPPER_WIDTH))


def extract_gripper_events(
    gripper_widths: np.ndarray,
    timestamps: np.ndarray,
    event_delta: float,
) -> List[Tuple[int, float, float]]:
    if event_delta < 0:
        raise ValueError("--gripper-event-delta must be nonnegative")

    events = [(0, float(timestamps[0]), float(gripper_widths[0]))]
    width_steps = np.abs(np.diff(gripper_widths))
    noise_floor = max(1e-5, event_delta * 0.02)
    change_indices = np.flatnonzero(width_steps > noise_floor) + 1
    if len(change_indices) == 0:
        return events

    if event_delta == 0:
        for idx in change_indices:
            events.append((int(idx), float(timestamps[idx]), float(gripper_widths[idx])))
        return events

    def change_direction(frame_idx: int) -> int:
        delta = float(gripper_widths[frame_idx] - gripper_widths[frame_idx - 1])
        return 1 if delta > 0 else -1

    last_target_width = float(gripper_widths[0])
    group_start = int(change_indices[0])
    group_end = group_start
    group_direction = change_direction(group_start)
    previous_change_idx = group_start
    max_gap_s = 1.0

    def close_group(send_idx: int, target_idx: int) -> None:
        nonlocal last_target_width
        target_width = float(gripper_widths[target_idx])
        if abs(target_width - last_target_width) >= event_delta:
            events.append((send_idx, float(timestamps[send_idx]), target_width))
            last_target_width = target_width

    for raw_idx in change_indices[1:]:
        idx = int(raw_idx)
        direction = change_direction(idx)
        gap_s = float(timestamps[idx] - timestamps[previous_change_idx])
        if gap_s <= max_gap_s and direction == group_direction:
            group_end = idx
            previous_change_idx = idx
            continue

        close_group(group_start, group_end)
        group_start = idx
        group_end = idx
        group_direction = direction
        previous_change_idx = idx

    close_group(group_start, group_end)
    return events


def print_episode_summary(
    episode_path,
    payload,
    metadata,
    arm_side,
    joints,
    gripper_widths,
    timestamps,
    gripper_events,
):
    duration = float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0
    avg_fps = float((len(timestamps) - 1) / duration) if duration > 0 else 0.0

    print(f"Episode: {episode_path}")
    print(f"Metadata schema: {metadata.get('schema_version', '<missing>')}")
    print(f"Inferred arm side: {arm_side}")
    print(f"Frames: {len(timestamps)}")
    print(f"Duration: {duration:.3f} s")
    print(f"Average capture FPS: {avg_fps:.3f}")
    print(f"Keyframes: {payload.get('keyframes', [])}")
    print(f"Joint first: {np.array2string(joints[0], precision=4)}")
    print(f"Joint last:  {np.array2string(joints[-1], precision=4)}")
    print(f"Joint range: {np.array2string(joints.max(axis=0) - joints.min(axis=0), precision=4)}")
    print(
        "Gripper target width first/last/range: "
        f"{gripper_widths[0]:.6f} / {gripper_widths[-1]:.6f} / "
        f"{float(gripper_widths.max() - gripper_widths.min()):.6f}"
    )
    print(f"Gripper direct events: {len(gripper_events)}")
    for event_idx, (frame_idx, timestamp, width) in enumerate(gripper_events):
        if event_idx >= 12:
            remaining = len(gripper_events) - event_idx
            print(f"  ... {remaining} more events")
            break
        rel_time = float(timestamp - timestamps[0])
        print(f"  frame={frame_idx:04d}  t={rel_time:7.3f}s  target_width={width:.6f} m")


def check_robot_start(client, commands):
    num_dofs = client.num_dofs()
    if num_dofs != 8:
        raise RuntimeError(f"Expected robot node with 8 DOFs, got {num_dofs}")

    current = client.get_joint_state()
    if current.shape[0] != 8:
        raise RuntimeError(f"Expected current joint state length 8, got {current.shape}")

    target = commands[0]
    joint_delta = np.abs(current[:7] - target[:7])
    max_delta = float(joint_delta.max())
    current_width = float(current[-1] * MAX_GRIPPER_WIDTH)
    target_width = float((1.0 - target[-1]) * MAX_GRIPPER_WIDTH)

    print(f"Robot node: tcp://{client.host}:{client.port}")
    print(f"Current joints: {np.array2string(current[:7], precision=4)}")
    print(f"Target frame 0 joints: {np.array2string(target[:7], precision=4)}")
    print(f"Start joint max delta: {max_delta:.6f} rad")
    print(f"Current/target gripper width: {current_width:.6f} / {target_width:.6f}")

    return {
        "current": current,
        "target": target,
        "joint_delta": joint_delta,
        "max_delta": max_delta,
        "current_width": current_width,
        "target_width": target_width,
    }


def check_start_delta_policy(args, start_info) -> bool:
    max_delta = float(start_info["max_delta"])
    if max_delta <= args.max_start_delta:
        return False

    if not args.approach_start:
        hint = (
            "Pass --approach-start to slowly move to frame 0 first."
            if args.execute
            else "Move the robot close to frame 0 first, or enable --approach-start."
        )
        raise RuntimeError(
            f"Start joint max delta {max_delta:.6f} exceeds --max-start-delta "
            f"{args.max_start_delta:.6f}. {hint}"
        )

    if max_delta > args.approach_start_max_delta:
        raise RuntimeError(
            f"Start joint max delta {max_delta:.6f} exceeds "
            f"--approach-start-max-delta {args.approach_start_max_delta:.6f}. "
            "Move the robot closer to frame 0 first."
        )

    if not args.execute:
        print(
            "Start joint max delta exceeds --max-start-delta, but is within "
            "--approach-start-max-delta. Dry-run passes because --execute "
            "--approach-start would slowly move to frame 0 before replay."
        )
        return False

    return True


def approach_start_frame(
    client,
    start_info,
    step_delta,
    hz,
    gripper_speed,
    gripper_force,
):
    if step_delta <= 0:
        raise ValueError("--approach-start-step-delta must be positive")
    if hz <= 0:
        raise ValueError("--approach-start-hz must be positive")
    if gripper_speed <= 0:
        raise ValueError("--gripper-speed must be positive")
    if gripper_force <= 0:
        raise ValueError("--gripper-force must be positive")

    current_joint = np.asarray(start_info["current"][:7], dtype=float)
    target_joint = np.asarray(start_info["target"][:7], dtype=float)
    active_width = float(start_info["current_width"])
    delta = target_joint - current_joint
    max_delta = float(np.max(np.abs(delta)))
    steps = max(1, int(np.ceil(max_delta / float(step_delta))))
    period_sec = 1.0 / float(hz)

    print(
        "Approaching replay frame 0 before starting timeline: "
        f"max_delta={max_delta:.6f} rad, steps={steps}, "
        f"step_delta<={step_delta:.6f} rad, hz={hz:.3f}, "
        f"held_gripper_width={active_width:.6f} m"
    )
    started_at = time.monotonic()
    for step_idx in range(1, steps + 1):
        action_t0 = time.monotonic()
        alpha = step_idx / steps
        joint = current_joint + alpha * delta
        command = np.concatenate(
            [joint, np.asarray([gripper_width_to_command(active_width)], dtype=float)]
        )
        client.command_joint_state(command, gripper_speed, gripper_force)
        sleep_sec = max(0.0, period_sec - (time.monotonic() - action_t0))
        if sleep_sec:
            time.sleep(sleep_sec)
    print(f"Approach finished in {time.monotonic() - started_at:.3f}s.")


def replay_trajectory(
    client,
    gripper_client,
    timestamps,
    commands,
    gripper_events,
    speed,
    gripper_speed,
    gripper_force,
    gripper_hold_sec,
    gripper_replay_mode,
    gripper_command_hz,
):
    if speed <= 0:
        raise ValueError("--speed must be positive")
    if gripper_speed <= 0:
        raise ValueError("--gripper-speed must be positive")
    if gripper_force <= 0:
        raise ValueError("--gripper-force must be positive")
    if gripper_hold_sec < 0:
        raise ValueError("--gripper-hold-sec must be nonnegative")
    if gripper_replay_mode not in {"event", "continuous"}:
        raise ValueError("--gripper-replay-mode must be event or continuous")
    if gripper_replay_mode == "continuous" and gripper_command_hz <= 0:
        raise ValueError("--gripper-command-hz must be positive in continuous mode")

    gripper_events_by_frame = {frame_idx: width for frame_idx, _, width in gripper_events}
    active_gripper_width = (
        float(gripper_events[0][2])
        if gripper_events
        else float((1.0 - commands[0, -1]) * MAX_GRIPPER_WIDTH)
    )
    start_wall = time.monotonic()
    start_episode = float(timestamps[0])
    timeline_delay = 0.0
    total = len(commands)
    gripper_period = 1.0 / float(gripper_command_hz)
    next_gripper_elapsed = 0.0

    print(
        "Executing replay. "
        f"replay_speed={speed}, gripper_speed={gripper_speed}, "
        f"gripper_force={gripper_force}, gripper_mode={gripper_replay_mode}, "
        f"gripper_command_hz={gripper_command_hz}, gripper_hold_sec={gripper_hold_sec}. "
        "Press Ctrl+C to interrupt."
    )
    for idx, command in enumerate(commands):
        target_elapsed = (float(timestamps[idx]) - start_episode) / speed
        while True:
            remaining = start_wall + target_elapsed + timeline_delay - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.01))

        joint_command = np.asarray(command, dtype=float).copy()
        target_width = gripper_command_to_width(joint_command[-1])
        if gripper_replay_mode == "continuous":
            active_gripper_width = target_width
        joint_command[-1] = gripper_width_to_command(active_gripper_width)
        client.command_joint_state(
            joint_command,
            gripper_speed,
            gripper_force,
            update_gripper=False,
        )
        if gripper_replay_mode == "continuous":
            if idx == 0 or idx == total - 1 or target_elapsed + 1e-9 >= next_gripper_elapsed:
                gripper_client.goto_width(target_width, gripper_speed, gripper_force)
                active_gripper_width = target_width
                while next_gripper_elapsed <= target_elapsed + 1e-9:
                    next_gripper_elapsed += gripper_period
        else:
            gripper_width = gripper_events_by_frame.get(idx)
            if gripper_width is not None:
                gripper_client.goto_width(gripper_width, gripper_speed, gripper_force)
                active_gripper_width = gripper_width
                print(
                    f"Sent gripper event at frame {idx + 1}/{total}: "
                    f"target_width={gripper_width:.6f} m"
                )
                if gripper_hold_sec > 0:
                    print(
                        f"Holding joint replay for {gripper_hold_sec:.3f}s "
                        "after gripper event"
                    )
                    time.sleep(gripper_hold_sec)
                    timeline_delay += gripper_hold_sec
        if idx == 0 or idx == total - 1 or idx % 50 == 0:
            print(f"Sent frame {idx + 1}/{total}")


def main():
    args = parse_args()
    if args.execute and args.skip_robot_check:
        raise ValueError("--skip-robot-check cannot be used with --execute")
    if args.approach_start_max_delta <= 0:
        raise ValueError("--approach-start-max-delta must be positive")
    if args.approach_start_step_delta <= 0:
        raise ValueError("--approach-start-step-delta must be positive")
    if args.approach_start_hz <= 0:
        raise ValueError("--approach-start-hz must be positive")

    try:
        episode_path = resolve_episode_file(args.episode, latest=args.latest)
        payload, frames = load_episode(episode_path)
        metadata = load_episode_metadata(episode_path)
        episode_kind = infer_episode_kind(frames, metadata)
        if episode_kind != "single":
            raise ValueError(
                f"Episode appears to be {episode_kind}; use 16_replay_bi_arm_pipeline.sh "
                "or python -m franka_replay.replay_fr3_dual for dual-arm replay."
            )
        arm_side = infer_single_arm_side(args, metadata)
        apply_single_endpoint_defaults(args, metadata, arm_side)
        joints, gripper_widths, timestamps, commands = extract_trajectory(frames)
        gripper_events = extract_gripper_events(
            gripper_widths,
            timestamps,
            args.gripper_event_delta,
        )
        print_episode_summary(
            episode_path,
            payload,
            metadata,
            arm_side,
            joints,
            gripper_widths,
            timestamps,
            gripper_events,
        )
    except (FileNotFoundError, RuntimeError, ValueError, KeyError) as exc:
        print(f"Replay check failed: {exc}")
        print(
            "No command was sent. Pass an exact episode directory, metadata.json, "
            "or .pkl.gz file; use --latest only when intentionally selecting the "
            "newest episode."
        )
        raise SystemExit(1)

    if args.skip_robot_check:
        print("Robot check skipped. No command was sent.")
        return

    try:
        with RobotZMQReplayClient(args.host, args.port, args.timeout_ms) as client:
            start_info = check_robot_start(client, commands)
            should_approach = check_start_delta_policy(args, start_info)
            if should_approach:
                approach_start_frame(
                    client,
                    start_info,
                    args.approach_start_step_delta,
                    args.approach_start_hz,
                    args.gripper_speed,
                    args.gripper_force,
                )
            if not args.execute:
                print("Dry-run only. Add --execute to send this trajectory to the robot.")
                return

            with GripperDirectClient(
                args.gripper_host,
                args.gripper_port,
                args.timeout_ms,
            ) as gripper_client:
                current_gripper_width = gripper_client.get_width()
                print(
                    f"Direct gripper server: {args.gripper_host}:{args.gripper_port}, "
                    f"current_width={current_gripper_width:.6f} m"
                )
                replay_trajectory(
                    client,
                    gripper_client,
                    timestamps,
                    commands,
                    gripper_events,
                    args.speed,
                    args.gripper_speed,
                    args.gripper_force,
                    args.gripper_hold_sec,
                    args.gripper_replay_mode,
                    args.gripper_command_hz,
                )
            print("Replay finished.")
    except KeyboardInterrupt:
        print("\nReplay interrupted by user.")
        raise SystemExit(130)
    except (TimeoutError, RuntimeError, ValueError, KeyError, FileNotFoundError) as exc:
        print(f"Replay check failed: {exc}")
        if not args.execute:
            print(
                "No command was sent. Start 3_launch_node.sh for robot checks, "
                "or use --skip-robot-check to inspect only the file."
            )
        else:
            print("Replay aborted.")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
