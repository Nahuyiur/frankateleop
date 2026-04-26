"""Record FR3 robot-node observations with configured RealSense cameras."""

import argparse
import time
from pathlib import Path
from typing import Any, Optional

from franka_capture.cameras.realsense import create_realsense_cameras
from franka_capture.config.fr3_single import (
    DEFAULT_CAMERAS,
    DEFAULT_RECORDING,
    DEFAULT_ROBOT,
)
from franka_capture.core.robot_zmq_client import RobotZMQClient
from franka_capture.recording.episode_writer import EpisodeWriter
from franka_capture.recording.preview import concatenate_rgb_images, show_rgb_preview

MAX_GRIPPER_WIDTH = 0.09


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
    parser.add_argument("--host", default=DEFAULT_ROBOT.host)
    parser.add_argument("--port", type=int, default=DEFAULT_ROBOT.port)
    parser.add_argument("--timeout-ms", type=int, default=DEFAULT_ROBOT.timeout_ms)
    parser.add_argument("--video-fps", type=int, default=DEFAULT_RECORDING.video_fps)
    return parser.parse_args()


def _as_saved_value(value: Any):
    if hasattr(value, "tolist"):
        return value.tolist()
    return value


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


def main() -> None:
    args = parse_args()
    import cv2

    cameras = {}
    robot = None
    writer = None
    record_flag = False
    next_index = _next_episode_index(args.output_root, args.task, args.index)

    def start_episode() -> None:
        nonlocal writer, record_flag, next_index
        if writer is None:
            writer = EpisodeWriter(
                output_root=args.output_root,
                task=args.task,
                index=next_index,
                camera_names=camera_names,
                video_fps=args.video_fps,
            )
            print(f"Start recording episode {next_index}: {writer.output_dir}")
            next_index += 1
        else:
            print(f"Resume recording episode {writer.index}: {writer.output_dir}")
        record_flag = True

    def save_episode(quiet: bool = False) -> None:
        nonlocal writer, record_flag
        if writer is None:
            if not quiet:
                print("No active episode to save")
            return
        record_flag = False
        output_dir = writer.finish()
        print(f"Saved {len(writer.frames)} frames to {output_dir}")
        writer = None

    def discard_episode() -> None:
        nonlocal writer, record_flag, next_index
        if writer is None:
            print("No active episode to discard")
            return
        record_flag = False
        discarded_index = writer.index
        output_dir = writer.discard()
        next_index = min(next_index, discarded_index)
        print(f"Discarded episode {discarded_index}: removed {output_dir}")
        print(f"Next episode index reset to {next_index}. Press s to record again.")
        writer = None

    try:
        cameras = create_realsense_cameras(DEFAULT_CAMERAS)
        camera_names = list(cameras.keys())
        robot = RobotZMQClient(args.host, args.port, timeout_ms=args.timeout_ms)

        print(f"Robot node: tcp://{args.host}:{args.port}")
        print(f"Robot DOFs: {robot.num_dofs()}")
        print(f"Connected cameras: {camera_names}")
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
            if key in (ord("k"), ord("K")) and record_flag:
                if writer is None:
                    start_episode()
                keyframe = writer.add_keyframe()
                print(f"Episode {writer.index} keyframe {keyframe} added")

            if not record_flag:
                continue
            if writer is None:
                start_episode()

            robot_observations = robot.get_observations()
            joint_state = robot.get_joint_state()
            if "ee_pose_euler" not in robot_observations:
                raise RuntimeError(
                    "Robot node does not expose ee_pose_euler. "
                    "Restart 3_launch_node.sh after updating teleop/teleop/robots/fr3.py."
                )
            pose = robot_observations["ee_pose_euler"]
            frame = {
                "pose": _as_saved_value(pose),
                "joint": _as_saved_value(joint_state[:7]),
                "gripper": float(joint_state[-1] * MAX_GRIPPER_WIDTH),
                "timestamp": time.time(),
            }
            for name in camera_names:
                frame[f"{name}_image"] = rgb_frames[name][:, :, ::-1].copy()

            writer.append(frame, rgb_frames)
    finally:
        if writer is not None:
            writer.close()
        cv2.destroyAllWindows()
        for camera in cameras.values():
            camera.close()
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
