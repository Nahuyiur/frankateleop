"""Replay a recorded single-arm FR3 joint trajectory through robot node ZMQ."""

import argparse
import gzip
import pickle
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import zmq

MAX_GRIPPER_WIDTH = 0.09


def gripper_width_to_command(width: float) -> float:
    return float(np.clip(1.0 - (float(width) / MAX_GRIPPER_WIDTH), 0.0, 1.0))


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
    ) -> None:
        self._request(
            "command_joint_state",
            {
                "joint_state": joint_state,
                "gripper_speed": gripper_speed,
                "gripper_force": gripper_force,
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
        help="Episode directory such as /home/pnp/Desktop/franka_record_data/pick_block/3, or a .pkl.gz file.",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6001)
    parser.add_argument("--timeout-ms", type=int, default=2000)
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
        default="127.0.0.1",
        help="Host for the gripper gRPC server launched by 2_launch_gripper.sh.",
    )
    parser.add_argument(
        "--gripper-port",
        type=int,
        default=50052,
        help="Port for the gripper gRPC server launched by 2_launch_gripper.sh.",
    )
    parser.add_argument(
        "--gripper-event-delta",
        type=float,
        default=0.01,
        help="Emit a direct gripper command when recorded width changes by at least this many meters.",
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


def resolve_episode_file(path_like: str) -> Path:
    path = Path(path_like).expanduser()
    if path.is_file():
        if path.name.endswith(".pkl.gz"):
            return path
        raise ValueError(f"Replay input file must end with .pkl.gz: {path}")
    if not path.is_dir():
        raise FileNotFoundError(f"Episode path does not exist: {path}")

    preferred = path / f"{path.name}.pkl.gz"
    if preferred.exists():
        return preferred

    candidates = sorted(path.glob("*.pkl.gz"))
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise FileNotFoundError(f"No .pkl.gz file found in episode directory: {path}")
    raise RuntimeError(
        f"Multiple .pkl.gz files found in {path}; pass the exact file path instead."
    )


def load_episode(path: Path):
    with gzip.open(path, "rb") as f:
        payload = pickle.load(f)

    if not isinstance(payload, dict) or "data" not in payload:
        raise ValueError(f"Invalid episode payload in {path}: expected dict with data")

    frames = payload["data"]
    if not frames:
        raise ValueError(f"Episode has no frames: {path}")
    return payload, frames


def extract_trajectory(frames):
    joints = np.asarray([frame["joint"] for frame in frames], dtype=float)
    gripper_widths = np.asarray([frame["gripper"] for frame in frames], dtype=float)
    timestamps = np.asarray([frame["timestamp"] for frame in frames], dtype=float)

    if joints.ndim != 2 or joints.shape[1] != 7:
        raise ValueError(f"Expected joint trajectory shape (N, 7), got {joints.shape}")
    if gripper_widths.ndim != 1 or gripper_widths.shape[0] != joints.shape[0]:
        raise ValueError("Invalid gripper trajectory shape")
    if timestamps.ndim != 1 or timestamps.shape[0] != joints.shape[0]:
        raise ValueError("Invalid timestamp trajectory shape")
    if np.any(np.diff(timestamps) < 0):
        raise ValueError("Episode timestamps must be monotonically nondecreasing")

    gripper_min = float(gripper_widths.min())
    gripper_max = float(gripper_widths.max())
    if gripper_min < -1e-4 or gripper_max > MAX_GRIPPER_WIDTH + 1e-3:
        raise ValueError(
            "Recorded gripper values do not look like real gripper widths in meters: "
            f"min={gripper_min:.6f}, max={gripper_max:.6f}. "
            "Use an episode recorded after the gripper-width fix."
        )

    gripper_commands = np.asarray(
        [gripper_width_to_command(width) for width in gripper_widths],
        dtype=float,
    )
    commands = np.concatenate([joints, gripper_commands[:, None]], axis=1)
    return joints, gripper_widths, timestamps, commands


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
    joints,
    gripper_widths,
    timestamps,
    gripper_events,
):
    duration = float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0
    avg_fps = float((len(timestamps) - 1) / duration) if duration > 0 else 0.0

    print(f"Episode: {episode_path}")
    print(f"Frames: {len(timestamps)}")
    print(f"Duration: {duration:.3f} s")
    print(f"Average capture FPS: {avg_fps:.3f}")
    print(f"Keyframes: {payload.get('keyframes', [])}")
    print(f"Joint first: {np.array2string(joints[0], precision=4)}")
    print(f"Joint last:  {np.array2string(joints[-1], precision=4)}")
    print(f"Joint range: {np.array2string(joints.max(axis=0) - joints.min(axis=0), precision=4)}")
    print(
        "Gripper width first/last/range: "
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
        print(f"  frame={frame_idx:04d}  t={rel_time:7.3f}s  width={width:.6f} m")


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
):
    if speed <= 0:
        raise ValueError("--speed must be positive")
    if gripper_speed <= 0:
        raise ValueError("--gripper-speed must be positive")
    if gripper_force <= 0:
        raise ValueError("--gripper-force must be positive")
    if gripper_hold_sec < 0:
        raise ValueError("--gripper-hold-sec must be nonnegative")

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

    print(
        "Executing replay. "
        f"replay_speed={speed}, gripper_speed={gripper_speed}, "
        f"gripper_force={gripper_force}, gripper_hold_sec={gripper_hold_sec}. "
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
        joint_command[-1] = gripper_width_to_command(active_gripper_width)
        client.command_joint_state(joint_command, gripper_speed, gripper_force)
        gripper_width = gripper_events_by_frame.get(idx)
        if gripper_width is not None:
            gripper_client.goto_width(gripper_width, gripper_speed, gripper_force)
            active_gripper_width = gripper_width
            print(
                f"Sent gripper event at frame {idx + 1}/{total}: "
                f"width={gripper_width:.6f} m"
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

    episode_path = resolve_episode_file(args.episode)
    payload, frames = load_episode(episode_path)
    joints, gripper_widths, timestamps, commands = extract_trajectory(frames)
    gripper_events = extract_gripper_events(
        gripper_widths,
        timestamps,
        args.gripper_event_delta,
    )
    print_episode_summary(
        episode_path,
        payload,
        joints,
        gripper_widths,
        timestamps,
        gripper_events,
    )

    if args.skip_robot_check:
        print("Robot check skipped. No command was sent.")
        return

    try:
        with RobotZMQReplayClient(args.host, args.port, args.timeout_ms) as client:
            start_info = check_robot_start(client, commands)
            if start_info["max_delta"] > args.max_start_delta:
                if not args.execute:
                    raise RuntimeError(
                        f"Start joint max delta {start_info['max_delta']:.6f} "
                        f"exceeds --max-start-delta {args.max_start_delta:.6f}. "
                        "Move the robot close to frame 0 first, or run with "
                        "--execute --approach-start to auto-approach."
                    )
                if not args.approach_start:
                    raise RuntimeError(
                        f"Start joint max delta {start_info['max_delta']:.6f} "
                        f"exceeds --max-start-delta {args.max_start_delta:.6f}. "
                        "Pass --approach-start to slowly move to frame 0 first."
                    )
                if start_info["max_delta"] > args.approach_start_max_delta:
                    raise RuntimeError(
                        f"Start joint max delta {start_info['max_delta']:.6f} exceeds "
                        f"--approach-start-max-delta "
                        f"{args.approach_start_max_delta:.6f}. "
                        "Move the robot closer to frame 0 first."
                    )
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
                )
            print("Replay finished.")
    except KeyboardInterrupt:
        print("\nReplay interrupted by user.")
        raise SystemExit(130)
    except (TimeoutError, RuntimeError, ValueError) as exc:
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
