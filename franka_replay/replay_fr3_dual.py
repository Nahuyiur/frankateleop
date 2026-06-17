"""Replay a recorded dual-arm FR3 joint trajectory through two robot nodes."""

import argparse
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

from franka_capture.gripper_fields import frame_gripper_target_width

from .replay_fr3 import (
    MAX_GRIPPER_WIDTH,
    GripperDirectClient,
    RobotZMQReplayClient,
    extract_gripper_events,
    gripper_command_to_width,
    gripper_width_to_command,
    load_episode,
    resolve_episode_file,
)


@dataclass
class ArmTrajectory:
    name: str
    joints: np.ndarray
    gripper_widths: np.ndarray
    commands: np.ndarray
    gripper_events: List[Tuple[int, float, float]]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "episode",
        help="Dual-arm episode directory, or the exact .pkl.gz file.",
    )
    parser.add_argument("--left-host", default="127.0.0.1")
    parser.add_argument("--left-port", type=int, default=6002)
    parser.add_argument("--right-host", default="127.0.0.1")
    parser.add_argument("--right-port", type=int, default=16001)
    parser.add_argument("--timeout-ms", type=int, default=2000)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send the recorded trajectory to both robot nodes.",
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
            "If either current joint state is farther from frame 0 than this many "
            "radians, abort unless --execute --approach-start is enabled."
        ),
    )
    parser.add_argument(
        "--approach-start",
        action="store_true",
        help=(
            "If --execute starts farther than --max-start-delta, slowly move both "
            "arms to frame 0 before starting replay."
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
        "--left-gripper-host",
        default="127.0.0.1",
        help="Host for the local left gripper gRPC server.",
    )
    parser.add_argument(
        "--left-gripper-port",
        type=int,
        default=50054,
        help="Port for the local left gripper gRPC server.",
    )
    parser.add_argument(
        "--right-gripper-host",
        default="127.0.0.1",
        help="Host for the right gripper gRPC server, usually an SSH tunnel.",
    )
    parser.add_argument(
        "--right-gripper-port",
        type=int,
        default=15053,
        help="Local port for the tunneled right gripper gRPC server.",
    )
    parser.add_argument(
        "--gripper-event-delta",
        type=float,
        default=0.01,
        help="Emit a direct gripper command when target width changes by this many meters.",
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
        help="Pause the joint replay timeline after any direct gripper event.",
    )
    parser.add_argument(
        "--skip-robot-check",
        action="store_true",
        help="Only inspect the recorded file without connecting to robot nodes. Not allowed with --execute.",
    )
    return parser.parse_args()


def extract_dual_trajectories(frames, timestamps, event_delta) -> Dict[str, ArmTrajectory]:
    trajectories = {}
    for name in ("left", "right"):
        joints, gripper_widths, commands = extract_arm_trajectory(frames, name)
        gripper_events = extract_gripper_events(
            gripper_widths,
            timestamps,
            event_delta,
        )
        trajectories[name] = ArmTrajectory(
            name=name,
            joints=joints,
            gripper_widths=gripper_widths,
            commands=commands,
            gripper_events=gripper_events,
        )
    return trajectories


def extract_arm_trajectory(frames, arm: str):
    joint_key = f"{arm}_joint"
    joints = np.asarray([frame[joint_key] for frame in frames], dtype=float)
    gripper_commands, gripper_widths = _extract_arm_gripper_command_and_width(frames, arm)

    if joints.ndim != 2 or joints.shape[1] != 7:
        raise ValueError(f"Expected {arm} joint trajectory shape (N, 7), got {joints.shape}")
    if gripper_widths.ndim != 1 or gripper_widths.shape[0] != joints.shape[0]:
        raise ValueError(f"Invalid {arm} gripper trajectory shape")

    commands = np.concatenate([joints, gripper_commands[:, None]], axis=1)
    return joints, gripper_widths, commands


def _extract_arm_gripper_command_and_width(frames, arm: str) -> Tuple[np.ndarray, np.ndarray]:
    commands = []
    target_widths = []
    for frame in frames:
        command, target_width = _frame_arm_gripper_command_and_width(frame, arm)
        commands.append(command)
        target_widths.append(target_width)
    return (
        np.asarray(commands, dtype=float),
        np.asarray(target_widths, dtype=float),
    )


def _frame_arm_gripper_command_and_width(frame: Dict[str, Any], arm: str) -> Tuple[float, float]:
    target_width = frame_gripper_target_width(frame, prefix=arm)
    return gripper_width_to_command(target_width), _validate_gripper_width(target_width)


def _validate_gripper_width(width: float) -> float:
    width = float(width)
    if width < -1e-4 or width > MAX_GRIPPER_WIDTH + 1e-3:
        raise ValueError(
            "Recorded gripper target width is outside the Franka Hand range: "
            f"{width:.6f} m"
        )
    return float(np.clip(width, 0.0, MAX_GRIPPER_WIDTH))


def extract_timestamps(frames) -> np.ndarray:
    timestamps = np.asarray([frame["timestamp"] for frame in frames], dtype=float)
    if timestamps.ndim != 1 or timestamps.shape[0] != len(frames):
        raise ValueError("Invalid timestamp trajectory shape")
    if np.any(np.diff(timestamps) < 0):
        raise ValueError("Episode timestamps must be monotonically nondecreasing")
    return timestamps


def print_episode_summary(
    episode_path: Path,
    payload,
    frames,
    timestamps,
    trajectories: Dict[str, ArmTrajectory],
):
    duration = float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0
    avg_fps = float((len(timestamps) - 1) / duration) if duration > 0 else 0.0
    schema = frames[0].get("schema_version", "<missing>")

    print(f"Episode: {episode_path}")
    print(f"Schema: {schema}")
    print(f"Frames: {len(timestamps)}")
    print(f"Duration: {duration:.3f} s")
    print(f"Average capture FPS: {avg_fps:.3f}")
    print(f"Keyframes: {payload.get('keyframes', [])}")
    for name in ("left", "right"):
        traj = trajectories[name]
        print(f"{name.capitalize()} joint first: {np.array2string(traj.joints[0], precision=4)}")
        print(f"{name.capitalize()} joint last:  {np.array2string(traj.joints[-1], precision=4)}")
        print(
            f"{name.capitalize()} joint range: "
            f"{np.array2string(traj.joints.max(axis=0) - traj.joints.min(axis=0), precision=4)}"
        )
        print(
            f"{name.capitalize()} gripper target width first/last/range: "
            f"{traj.gripper_widths[0]:.6f} / {traj.gripper_widths[-1]:.6f} / "
            f"{float(traj.gripper_widths.max() - traj.gripper_widths.min()):.6f}"
        )
        print(f"{name.capitalize()} gripper direct events: {len(traj.gripper_events)}")
        for event_idx, (frame_idx, timestamp, width) in enumerate(traj.gripper_events):
            if event_idx >= 8:
                remaining = len(traj.gripper_events) - event_idx
                print(f"  ... {remaining} more {name} events")
                break
            rel_time = float(timestamp - timestamps[0])
            print(
                f"  {name} frame={frame_idx:04d}  "
                f"t={rel_time:7.3f}s  target_width={width:.6f} m"
            )


def check_robot_start(name: str, client, commands):
    num_dofs = client.num_dofs()
    if num_dofs != 8:
        raise RuntimeError(f"Expected {name} robot node with 8 DOFs, got {num_dofs}")

    current = client.get_joint_state()
    if current.shape[0] != 8:
        raise RuntimeError(f"Expected {name} current joint state length 8, got {current.shape}")

    target = commands[0]
    joint_delta = np.abs(current[:7] - target[:7])
    max_delta = float(joint_delta.max())
    current_width = float(current[-1] * MAX_GRIPPER_WIDTH)
    target_width = float((1.0 - target[-1]) * MAX_GRIPPER_WIDTH)

    print(f"{name.capitalize()} robot node: tcp://{client.host}:{client.port}")
    print(f"{name.capitalize()} current joints: {np.array2string(current[:7], precision=4)}")
    print(f"{name.capitalize()} target frame 0 joints: {np.array2string(target[:7], precision=4)}")
    print(f"{name.capitalize()} start joint max delta: {max_delta:.6f} rad")
    print(f"{name.capitalize()} current/target gripper width: {current_width:.6f} / {target_width:.6f}")

    return {
        "current": current,
        "target": target,
        "joint_delta": joint_delta,
        "max_delta": max_delta,
        "current_width": current_width,
        "target_width": target_width,
    }


def check_start_delta_policy(args, start_infos):
    max_delta = max(float(info["max_delta"]) for info in start_infos.values())
    if max_delta <= args.max_start_delta:
        return False

    if not args.execute:
        raise RuntimeError(
            f"Start joint max delta {max_delta:.6f} exceeds --max-start-delta "
            f"{args.max_start_delta:.6f}. Move both robots close to frame 0 first, "
            "or run with --execute --approach-start to auto-approach."
        )
    if not args.approach_start:
        raise RuntimeError(
            f"Start joint max delta {max_delta:.6f} exceeds --max-start-delta "
            f"{args.max_start_delta:.6f}. Pass --approach-start to slowly move "
            "both robots to frame 0 first."
        )
    if max_delta > args.approach_start_max_delta:
        raise RuntimeError(
            f"Start joint max delta {max_delta:.6f} exceeds "
            f"--approach-start-max-delta {args.approach_start_max_delta:.6f}. "
            "Move both robots closer to frame 0 first."
        )
    return True


def _send_joint_pair(
    executor,
    left_client,
    right_client,
    left_command,
    right_command,
    gripper_speed,
    gripper_force,
    update_gripper=True,
) -> None:
    futures = [
        executor.submit(
            left_client.command_joint_state,
            left_command,
            gripper_speed,
            gripper_force,
            update_gripper,
        ),
        executor.submit(
            right_client.command_joint_state,
            right_command,
            gripper_speed,
            gripper_force,
            update_gripper,
        ),
    ]
    for future in futures:
        future.result()


def approach_start_frames(
    left_client,
    right_client,
    left_info,
    right_info,
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

    left_current_joint = np.asarray(left_info["current"][:7], dtype=float)
    right_current_joint = np.asarray(right_info["current"][:7], dtype=float)
    left_target_joint = np.asarray(left_info["target"][:7], dtype=float)
    right_target_joint = np.asarray(right_info["target"][:7], dtype=float)
    left_delta = left_target_joint - left_current_joint
    right_delta = right_target_joint - right_current_joint
    max_delta = max(
        float(np.max(np.abs(left_delta))),
        float(np.max(np.abs(right_delta))),
    )
    steps = max(1, int(np.ceil(max_delta / float(step_delta))))
    period_sec = 1.0 / float(hz)
    left_width = float(left_info["current_width"])
    right_width = float(right_info["current_width"])

    print(
        "Approaching replay frame 0 before starting dual-arm timeline: "
        f"max_delta={max_delta:.6f} rad, steps={steps}, "
        f"step_delta<={step_delta:.6f} rad, hz={hz:.3f}"
    )
    started_at = time.monotonic()
    with ThreadPoolExecutor(max_workers=2) as executor:
        for step_idx in range(1, steps + 1):
            action_t0 = time.monotonic()
            alpha = step_idx / steps
            left_command = np.concatenate(
                [
                    left_current_joint + alpha * left_delta,
                    np.asarray([gripper_width_to_command(left_width)], dtype=float),
                ]
            )
            right_command = np.concatenate(
                [
                    right_current_joint + alpha * right_delta,
                    np.asarray([gripper_width_to_command(right_width)], dtype=float),
                ]
            )
            _send_joint_pair(
                executor,
                left_client,
                right_client,
                left_command,
                right_command,
                gripper_speed,
                gripper_force,
                update_gripper=False,
            )
            sleep_sec = max(0.0, period_sec - (time.monotonic() - action_t0))
            if sleep_sec:
                time.sleep(sleep_sec)
    print(f"Approach finished in {time.monotonic() - started_at:.3f}s.")


def replay_dual_trajectory(
    left_client,
    right_client,
    left_gripper_client,
    right_gripper_client,
    timestamps,
    trajectories: Dict[str, ArmTrajectory],
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

    left_events_by_frame = {
        frame_idx: width for frame_idx, _, width in trajectories["left"].gripper_events
    }
    right_events_by_frame = {
        frame_idx: width for frame_idx, _, width in trajectories["right"].gripper_events
    }
    left_active_width = _initial_active_width(trajectories["left"])
    right_active_width = _initial_active_width(trajectories["right"])
    start_wall = time.monotonic()
    start_episode = float(timestamps[0])
    timeline_delay = 0.0
    total = len(timestamps)
    gripper_period = 1.0 / float(gripper_command_hz)
    next_gripper_elapsed = 0.0

    print(
        "Executing dual-arm replay. "
        f"replay_speed={speed}, gripper_speed={gripper_speed}, "
        f"gripper_force={gripper_force}, gripper_mode={gripper_replay_mode}, "
        f"gripper_command_hz={gripper_command_hz}, gripper_hold_sec={gripper_hold_sec}. "
        "Press Ctrl+C to interrupt."
    )
    with ThreadPoolExecutor(max_workers=2) as executor:
        for idx in range(total):
            target_elapsed = (float(timestamps[idx]) - start_episode) / speed
            while True:
                remaining = start_wall + target_elapsed + timeline_delay - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 0.01))

            left_command = np.asarray(trajectories["left"].commands[idx], dtype=float).copy()
            right_command = np.asarray(trajectories["right"].commands[idx], dtype=float).copy()
            left_target_width = gripper_command_to_width(left_command[-1])
            right_target_width = gripper_command_to_width(right_command[-1])
            if gripper_replay_mode == "continuous":
                left_active_width = left_target_width
                right_active_width = right_target_width
            left_command[-1] = gripper_width_to_command(left_active_width)
            right_command[-1] = gripper_width_to_command(right_active_width)
            _send_joint_pair(
                executor,
                left_client,
                right_client,
                left_command,
                right_command,
                gripper_speed,
                gripper_force,
            )

            if gripper_replay_mode == "continuous":
                if idx == 0 or idx == total - 1 or target_elapsed + 1e-9 >= next_gripper_elapsed:
                    futures = [
                        executor.submit(
                            left_gripper_client.goto_width,
                            left_target_width,
                            gripper_speed,
                            gripper_force,
                        ),
                        executor.submit(
                            right_gripper_client.goto_width,
                            right_target_width,
                            gripper_speed,
                            gripper_force,
                        ),
                    ]
                    for future in futures:
                        future.result()
                    left_active_width = left_target_width
                    right_active_width = right_target_width
                    while next_gripper_elapsed <= target_elapsed + 1e-9:
                        next_gripper_elapsed += gripper_period
                if idx == 0 or idx == total - 1 or idx % 50 == 0:
                    print(f"Sent dual-arm frame {idx + 1}/{total}")
                continue

            gripper_futures = []
            left_width = left_events_by_frame.get(idx)
            right_width = right_events_by_frame.get(idx)
            if left_width is not None:
                gripper_futures.append(
                    executor.submit(
                        left_gripper_client.goto_width,
                        left_width,
                        gripper_speed,
                        gripper_force,
                    )
                )
            if right_width is not None:
                gripper_futures.append(
                    executor.submit(
                        right_gripper_client.goto_width,
                        right_width,
                        gripper_speed,
                        gripper_force,
                    )
                )
            for future in gripper_futures:
                future.result()

            had_gripper_event = False
            if left_width is not None:
                left_active_width = left_width
                had_gripper_event = True
                print(
                    f"Sent left gripper event at frame {idx + 1}/{total}: "
                    f"target_width={left_width:.6f} m"
                )
            if right_width is not None:
                right_active_width = right_width
                had_gripper_event = True
                print(
                    f"Sent right gripper event at frame {idx + 1}/{total}: "
                    f"target_width={right_width:.6f} m"
                )
            if had_gripper_event and gripper_hold_sec > 0:
                print(
                    f"Holding dual-arm joint replay for {gripper_hold_sec:.3f}s "
                    "after gripper event"
                )
                time.sleep(gripper_hold_sec)
                timeline_delay += gripper_hold_sec

            if idx == 0 or idx == total - 1 or idx % 50 == 0:
                print(f"Sent dual-arm frame {idx + 1}/{total}")


def _initial_active_width(trajectory: ArmTrajectory) -> float:
    if trajectory.gripper_events:
        return float(trajectory.gripper_events[0][2])
    return float((1.0 - trajectory.commands[0, -1]) * MAX_GRIPPER_WIDTH)


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
    timestamps = extract_timestamps(frames)
    trajectories = extract_dual_trajectories(
        frames,
        timestamps,
        args.gripper_event_delta,
    )
    print_episode_summary(
        episode_path,
        payload,
        frames,
        timestamps,
        trajectories,
    )

    if args.skip_robot_check:
        print("Robot check skipped. No command was sent.")
        return

    try:
        with RobotZMQReplayClient(args.left_host, args.left_port, args.timeout_ms) as left_client:
            with RobotZMQReplayClient(args.right_host, args.right_port, args.timeout_ms) as right_client:
                left_info = check_robot_start("left", left_client, trajectories["left"].commands)
                right_info = check_robot_start("right", right_client, trajectories["right"].commands)
                should_approach = check_start_delta_policy(
                    args,
                    {"left": left_info, "right": right_info},
                )
                if should_approach:
                    approach_start_frames(
                        left_client,
                        right_client,
                        left_info,
                        right_info,
                        args.approach_start_step_delta,
                        args.approach_start_hz,
                        args.gripper_speed,
                        args.gripper_force,
                    )
                if not args.execute:
                    print("Dry-run only. Add --execute to send this trajectory to both robots.")
                    return

                with GripperDirectClient(
                    args.left_gripper_host,
                    args.left_gripper_port,
                    args.timeout_ms,
                ) as left_gripper_client:
                    with GripperDirectClient(
                        args.right_gripper_host,
                        args.right_gripper_port,
                        args.timeout_ms,
                    ) as right_gripper_client:
                        print(
                            f"Left direct gripper server: "
                            f"{args.left_gripper_host}:{args.left_gripper_port}, "
                            f"current_width={left_gripper_client.get_width():.6f} m"
                        )
                        print(
                            f"Right direct gripper server: "
                            f"{args.right_gripper_host}:{args.right_gripper_port}, "
                            f"current_width={right_gripper_client.get_width():.6f} m"
                        )
                        replay_dual_trajectory(
                            left_client,
                            right_client,
                            left_gripper_client,
                            right_gripper_client,
                            timestamps,
                            trajectories,
                            args.speed,
                        args.gripper_speed,
                        args.gripper_force,
                        args.gripper_hold_sec,
                        args.gripper_replay_mode,
                        args.gripper_command_hz,
                    )
                print("Dual-arm replay finished.")
    except KeyboardInterrupt:
        print("\nDual-arm replay interrupted by user.")
        raise SystemExit(130)
    except (TimeoutError, RuntimeError, ValueError, KeyError) as exc:
        print(f"Dual-arm replay check failed: {exc}")
        if not args.execute:
            print(
                "No command was sent. Start both robot nodes for robot checks, "
                "or use --skip-robot-check to inspect only the file."
            )
        else:
            print("Dual-arm replay aborted.")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
