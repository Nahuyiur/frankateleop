"""Record synchronized dual-arm FR3 observations with configured cameras."""

import argparse
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path
from typing import Any, Dict, Optional

from franka_capture.cameras.realsense import create_realsense_cameras
from franka_capture.config.fr3_dual import (
    DEFAULT_CAMERAS,
    DEFAULT_LEFT_ROBOT,
    DEFAULT_RECORDING,
    DEFAULT_RIGHT_ROBOT,
    DUAL_SCHEMA_VERSION,
)
from franka_capture.core.robot_zmq_client import RobotZMQClient
from franka_capture.recording.episode_writer import EpisodeWriter
from franka_capture.recording.preview import concatenate_rgb_images, show_rgb_preview

MAX_GRIPPER_WIDTH = 0.09
GRIPPER_BINARY_THRESHOLD = 0.5
FIXED_RECORDING_FPS = 30


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--index",
        type=int,
        default=None,
        help="Optional starting episode index. Defaults to max existing index + 1.",
    )
    parser.add_argument("--task", default="test")
    parser.add_argument("--output-root", default=DEFAULT_RECORDING.output_root)
    parser.add_argument("--left-host", default=DEFAULT_LEFT_ROBOT.host)
    parser.add_argument("--left-port", type=int, default=DEFAULT_LEFT_ROBOT.port)
    parser.add_argument("--right-host", default=DEFAULT_RIGHT_ROBOT.host)
    parser.add_argument("--right-port", type=int, default=DEFAULT_RIGHT_ROBOT.port)
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=max(DEFAULT_LEFT_ROBOT.timeout_ms, DEFAULT_RIGHT_ROBOT.timeout_ms),
    )
    return parser.parse_args()


def _as_saved_value(value: Any):
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


def _as_scalar(value: Any) -> float:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        if not value:
            raise ValueError("Expected scalar value, got empty sequence")
        value = value[0]
    return float(value)


def _as_binary_gripper_command(value: Any) -> float:
    return 1.0 if _as_scalar(value) >= GRIPPER_BINARY_THRESHOLD else 0.0


def _next_episode_index(output_root: str, task: str, start_index: Optional[int]) -> int:
    task_dir = Path(output_root).expanduser() / task
    existing_indices = []
    if task_dir.exists():
        for child in task_dir.iterdir():
            if child.is_dir() and child.name.isdigit():
                existing_indices.append(int(child.name))

    next_index = max(existing_indices, default=-1) + 1
    if start_index is not None:
        next_index = max(next_index, start_index)

    while (task_dir / str(next_index)).exists():
        next_index += 1
    return next_index


def _fixed_fps_camera_configs():
    return {
        name: replace(config, fps=FIXED_RECORDING_FPS)
        for name, config in DEFAULT_CAMERAS.items()
    }


def _extract_arm_state(
    robot_observations: Dict[str, Any],
    joint_state: Any,
) -> Dict[str, Any]:
    if "ee_pose_euler" not in robot_observations:
        raise RuntimeError(
            "Robot node does not expose ee_pose_euler. "
            "Restart 3_launch_node.sh after updating teleop/teleop/robots/fr3.py."
        )
    if "gripper_command" not in robot_observations:
        raise RuntimeError(
            "Robot node does not expose gripper_command. "
            "Restart 3_launch_node.sh after updating teleop/teleop/robots/fr3.py."
        )

    gripper_command_value = _as_scalar(robot_observations["gripper_command"])
    gripper_command = _as_binary_gripper_command(gripper_command_value)
    gripper_command_raw = _as_scalar(
        robot_observations.get("gripper_command_raw", gripper_command_value)
    )
    gripper_target_width = _as_scalar(
        robot_observations.get(
            "gripper_target_width",
            MAX_GRIPPER_WIDTH * (1.0 - gripper_command_raw),
        )
    )
    gripper_command_timestamp = _as_scalar(
        robot_observations.get("gripper_command_timestamp", time.time())
    )
    gripper_width = _as_scalar(
        robot_observations.get(
            "gripper_width",
            joint_state[-1] * MAX_GRIPPER_WIDTH,
        )
    )
    return {
        "pose": _as_saved_value(robot_observations["ee_pose_euler"]),
        "joint": _as_saved_value(joint_state[:7]),
        "gripper": gripper_command,
        "gripper_width": gripper_width,
        "gripper_command_raw": gripper_command_raw,
        "gripper_target_width": gripper_target_width,
        "gripper_command_timestamp": gripper_command_timestamp,
        "gripper_command_source": robot_observations.get("gripper_command_source", ""),
    }


def _fetch_arm_state(robot: RobotZMQClient) -> Dict[str, Any]:
    start_wall = time.time()
    start_monotonic = time.monotonic()
    robot_observations = robot.get_observations()
    joint_state = robot.get_joint_state()
    end_monotonic = time.monotonic()
    end_wall = time.time()

    state = _extract_arm_state(robot_observations, joint_state)
    state.update(
        {
            "robot_read_start_timestamp": start_wall,
            "robot_read_end_timestamp": end_wall,
            "robot_read_start_monotonic": start_monotonic,
            "robot_read_end_monotonic": end_monotonic,
            "robot_read_duration_ms": (end_monotonic - start_monotonic) * 1000.0,
        }
    )
    return state


def _add_prefixed_fields(
    frame: Dict[str, Any],
    prefix: str,
    values: Dict[str, Any],
) -> None:
    for key, value in values.items():
        frame[f"{prefix}_{key}"] = value


def main() -> None:
    args = parse_args()
    import cv2

    cameras = {}
    left_robot = None
    right_robot = None
    writer = None
    record_flag = False
    frame_index = 0
    next_index = _next_episode_index(args.output_root, args.task, args.index)
    camera_metadata: Dict[str, Any] = {}
    executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="dual-arm-read")

    def base_metadata():
        return {
            "source": "franka_capture.scripts.record_fr3_dual",
            "schema_version": DUAL_SCHEMA_VERSION,
            "started_at_unix": time.time(),
            "robots": {
                "left": {
                    "host": args.left_host,
                    "port": args.left_port,
                    "timeout_ms": args.timeout_ms,
                },
                "right": {
                    "host": args.right_host,
                    "port": args.right_port,
                    "timeout_ms": args.timeout_ms,
                },
            },
            "cameras": camera_metadata,
            "gripper_semantics": "binary_closedness_command",
            "gripper_values": {"0": "open", "1": "closed"},
            "gripper_command_threshold": GRIPPER_BINARY_THRESHOLD,
        }

    def start_episode() -> None:
        nonlocal writer, record_flag, next_index, frame_index
        if writer is None:
            frame_index = 0
            writer = EpisodeWriter(
                output_root=args.output_root,
                task=args.task,
                index=next_index,
                camera_names=camera_names,
                video_fps=FIXED_RECORDING_FPS,
                metadata=base_metadata(),
            )
            print(f"Start dual-arm recording episode {next_index}: {writer.output_dir}")
            next_index += 1
        else:
            print(f"Resume dual-arm recording episode {writer.index}: {writer.output_dir}")
        record_flag = True

    def save_episode(quiet: bool = False) -> None:
        nonlocal writer, record_flag
        if writer is None:
            if not quiet:
                print("No active episode to save")
            return
        record_flag = False
        writer.update_metadata({"ended_at_unix": time.time()})
        output_dir = writer.finish()
        print(f"Saved {len(writer.frames)} dual-arm frames to {output_dir}")
        writer = None

    def discard_episode() -> None:
        nonlocal writer, record_flag, next_index, frame_index
        if writer is None:
            print("No active episode to discard")
            return
        record_flag = False
        discarded_index = writer.index
        output_dir = writer.discard()
        next_index = min(next_index, discarded_index)
        frame_index = 0
        print(f"Discarded episode {discarded_index}: removed {output_dir}")
        print(f"Next episode index reset to {next_index}. Press s to record again.")
        writer = None

    try:
        camera_configs = _fixed_fps_camera_configs()
        cameras = create_realsense_cameras(camera_configs)
        camera_names = list(cameras.keys())
        camera_metadata = {name: camera.metadata() for name, camera in cameras.items()}

        left_robot = RobotZMQClient(
            args.left_host,
            args.left_port,
            timeout_ms=args.timeout_ms,
        )
        right_robot = RobotZMQClient(
            args.right_host,
            args.right_port,
            timeout_ms=args.timeout_ms,
        )

        print(f"Left robot node: tcp://{args.left_host}:{args.left_port}")
        print(f"Right robot node: tcp://{args.right_host}:{args.right_port}")
        print(f"Left robot DOFs: {left_robot.num_dofs()}")
        print(f"Right robot DOFs: {right_robot.num_dofs()}")
        print(f"Connected cameras: {camera_names}")
        print(f"Schema: {DUAL_SCHEMA_VERSION}")
        print(f"Camera FPS: {FIXED_RECORDING_FPS}")
        print(f"Video FPS: {FIXED_RECORDING_FPS}")
        print(f"Task: {args.task}")
        print(f"Next episode index: {next_index}")
        print(
            "Click the RGB window first. "
            "s=start/resume, w=pause, e=end/save episode, "
            "d=discard episode, k=keyframe, q=quit/save."
        )

        while True:
            rgb_frames = {}
            for name, camera in cameras.items():
                rgb, _ = camera.read()
                rgb_frames[name] = rgb

            preview = concatenate_rgb_images(
                [rgb_frames[name] for name in camera_names],
                line_width=DEFAULT_RECORDING.preview_line_width,
            )
            key = show_rgb_preview("RGB", preview)

            if key in (ord("q"), ord("Q")):
                save_episode(quiet=True)
                break
            if key in (ord("s"), ord("S")):
                start_episode()
            if key in (ord("w"), ord("W")):
                record_flag = False
                print("Pause recording")
            if key in (ord("e"), ord("E")):
                save_episode()
            if key in (ord("d"), ord("D")):
                discard_episode()
            if key in (ord("k"), ord("K")):
                if not record_flag:
                    print("Keyframe ignored: recording is paused. Press s first.")
                else:
                    if writer is None:
                        start_episode()
                    keyframe = writer.add_keyframe()
                    print(f"Episode {writer.index} keyframe {keyframe} added")

            if not record_flag:
                continue
            if writer is None:
                start_episode()

            loop_start_monotonic = time.monotonic()
            loop_start_timestamp = time.time()
            left_future = executor.submit(_fetch_arm_state, left_robot)
            right_future = executor.submit(_fetch_arm_state, right_robot)
            left_state = left_future.result()
            right_state = right_future.result()
            loop_end_monotonic = time.monotonic()
            timestamp = time.time()

            frame = {
                "schema_version": DUAL_SCHEMA_VERSION,
                "frame_index": frame_index,
                "timestamp": timestamp,
                "loop_start_timestamp": loop_start_timestamp,
                "loop_end_timestamp": timestamp,
                "loop_start_monotonic": loop_start_monotonic,
                "loop_end_monotonic": loop_end_monotonic,
                "loop_duration_ms": (loop_end_monotonic - loop_start_monotonic) * 1000.0,
            }
            _add_prefixed_fields(frame, "left", left_state)
            _add_prefixed_fields(frame, "right", right_state)

            for name in camera_names:
                frame[f"{name}_image"] = rgb_frames[name][:, :, ::-1].copy()

            writer.append(frame, rgb_frames)
            frame_index += 1
    finally:
        executor.shutdown(wait=False)
        if writer is not None:
            writer.close()
        cv2.destroyAllWindows()
        for camera in cameras.values():
            camera.close()
        if left_robot is not None:
            left_robot.close()
        if right_robot is not None:
            right_robot.close()


if __name__ == "__main__":
    main()
