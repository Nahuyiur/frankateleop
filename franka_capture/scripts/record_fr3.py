"""Record FR3 robot-node observations with configured RealSense cameras."""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Optional

import numpy as np

from franka_capture.cameras.realsense import create_realsense_cameras
from franka_capture.config.fr3_single import (
    DEFAULT_CAMERAS,
    DEFAULT_RECORDING,
    DEFAULT_ROBOT,
    SINGLE_SCHEMA_VERSION,
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
    parser.add_argument("--host", default=DEFAULT_ROBOT.host)
    parser.add_argument("--port", type=int, default=DEFAULT_ROBOT.port)
    parser.add_argument("--timeout-ms", type=int, default=DEFAULT_ROBOT.timeout_ms)
    parser.add_argument(
        "--enable-depth",
        action="store_true",
        help="Enable aligned depth recording for selected cameras.",
    )
    parser.add_argument(
        "--no-depth",
        action="store_true",
        help="Disable depth recording even if the pipeline enables it by default.",
    )
    parser.add_argument(
        "--depth-cameras",
        default="all",
        help="Comma-separated camera names for depth recording, or 'all'.",
    )
    parser.add_argument(
        "--no-depth-proof",
        action="store_true",
        help="Do not write depth_proof PNG/PLY/summary files after saving.",
    )
    parser.add_argument(
        "--pointcloud-stride",
        type=int,
        default=4,
        help="Pixel stride used for proof PLY point clouds. Raw depth is still saved fully.",
    )
    parser.add_argument(
        "--pointcloud-max-points",
        type=int,
        default=80000,
        help="Maximum points per proof PLY file.",
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


def _resolve_depth_camera_names(spec: str, camera_names: list[str]) -> set[str]:
    spec = (spec or "all").strip()
    if spec in {"all", "*"}:
        return set(camera_names)

    names = {name.strip() for name in spec.split(",") if name.strip()}
    unknown = sorted(names - set(camera_names))
    if unknown:
        raise ValueError(
            f"Unknown depth camera(s): {unknown}. Available cameras: {camera_names}"
        )
    return names


def _fixed_fps_camera_configs(depth_camera_names: set[str]):
    return {
        name: replace(
            config,
            fps=FIXED_RECORDING_FPS,
            depth=(name in depth_camera_names),
            align_depth=True if name in depth_camera_names else config.align_depth,
            read_timeout_ms=3000,
        )
        for name, config in DEFAULT_CAMERAS.items()
    }


def main() -> None:
    args = parse_args()
    import cv2

    cameras = {}
    robot = None
    writer = None
    record_flag = False
    next_index = _next_episode_index(args.output_root, args.task, args.index)
    configured_camera_names = list(DEFAULT_CAMERAS.keys())
    depth_camera_names = set()
    if args.enable_depth and not args.no_depth:
        depth_camera_names = _resolve_depth_camera_names(
            args.depth_cameras,
            configured_camera_names,
        )
    elif not args.no_depth:
        depth_camera_names = {
            name for name, config in DEFAULT_CAMERAS.items() if config.depth
        }
    depth_enabled = bool(depth_camera_names)
    camera_metadata = {}

    def start_episode() -> None:
        nonlocal writer, record_flag, next_index
        if writer is None:
            writer = EpisodeWriter(
                output_root=args.output_root,
                task=args.task,
                index=next_index,
                camera_names=camera_names,
                video_fps=FIXED_RECORDING_FPS,
                metadata={
                    "source": "franka_capture.scripts.record_fr3",
                    "schema_version": SINGLE_SCHEMA_VERSION,
                    "started_at_unix": time.time(),
                    "robot": {
                        "host": args.host,
                        "port": args.port,
                        "timeout_ms": args.timeout_ms,
                    },
                    "gripper_semantics": "binary_closedness_command",
                    "gripper_values": {"0": "open", "1": "closed"},
                    "gripper_command_threshold": GRIPPER_BINARY_THRESHOLD,
                    "cameras": camera_metadata,
                    "depth_recording": {
                        "enabled": depth_enabled,
                        "camera_names": sorted(depth_camera_names),
                        "storage": "per-frame {camera_name}_depth float32 image",
                        "units": "meters",
                        "aligned_to": "color",
                        "pointcloud_derivation": "depth + RGB + camera intrinsics",
                        "depth_proof_dir": None
                        if args.no_depth_proof or not depth_enabled
                        else "depth_proof",
                    },
                },
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
        frame_count = len(writer.frames)
        output_index = writer.index
        frames = writer.frames
        metadata = dict(writer.metadata or {})
        writer.update_metadata({"ended_at_unix": time.time()})
        output_dir = writer.finish()
        if depth_enabled and not args.no_depth_proof:
            try:
                from franka_capture.recording.depth_proof import write_depth_proof

                summary = write_depth_proof(
                    output_dir,
                    output_index,
                    frames,
                    metadata,
                    stride=args.pointcloud_stride,
                    max_points=args.pointcloud_max_points,
                )
                proof_cameras = sorted(summary.get("cameras", {}).keys())
                print(
                    "Depth proof written to "
                    f"{output_dir / 'depth_proof'} for cameras: {proof_cameras}"
                )
            except Exception as exc:
                print(f"WARNING: failed to write depth proof files: {exc}")
        print(f"Saved {frame_count} frames to {output_dir}")
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
        camera_configs = _fixed_fps_camera_configs(depth_camera_names)
        cameras = create_realsense_cameras(camera_configs, allow_missing=True)
        camera_names = list(cameras.keys())
        skipped_depth_cameras = sorted(depth_camera_names - set(camera_names))
        if skipped_depth_cameras:
            print(
                "WARNING: depth disabled for unavailable camera(s): "
                f"{skipped_depth_cameras}"
            )
        depth_camera_names &= set(camera_names)
        depth_enabled = bool(depth_camera_names)
        camera_metadata = {name: camera.metadata() for name, camera in cameras.items()}
        robot = RobotZMQClient(args.host, args.port, timeout_ms=args.timeout_ms)

        print(f"Robot node: tcp://{args.host}:{args.port}")
        print(f"Robot DOFs: {robot.num_dofs()}")
        print(f"Connected cameras: {camera_names}")
        print(
            "Depth recording: "
            f"{'enabled' if depth_enabled else 'disabled'}"
            + (f" ({sorted(depth_camera_names)})" if depth_enabled else "")
        )
        print(f"Camera FPS: {FIXED_RECORDING_FPS}")
        print(f"Video FPS: {FIXED_RECORDING_FPS}")
        print(f"Task: {args.task}")
        print(f"Next episode index: {next_index}")
        print(
            "Click the RGB window first. "
            "s=start/resume, w=pause, e=end/save episode, "
            "d=discard episode, k=keyframe, q=quit/save."
        )
        if not camera_names:
            print("No RealSense cameras connected; recording robot state only.")

        while True:
            rgb_frames = {}
            depth_frames = {}
            for name, camera in cameras.items():
                rgb, depth = camera.read()
                rgb_frames[name] = rgb
                if depth is not None:
                    depth_frames[name] = depth

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

            robot_observations = robot.get_observations()
            joint_state = robot.get_joint_state()
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
            pose = robot_observations["ee_pose_euler"]
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
            frame = {
                "schema_version": SINGLE_SCHEMA_VERSION,
                "pose": _as_saved_value(pose),
                "joint": _as_saved_value(joint_state[:7]),
                "gripper": gripper_command,
                "gripper_width": gripper_width,
                "gripper_command_raw": gripper_command_raw,
                "gripper_target_width": gripper_target_width,
                "gripper_command_timestamp": gripper_command_timestamp,
                "gripper_command_source": robot_observations.get("gripper_command_source", ""),
                "timestamp": time.time(),
            }
            for name in camera_names:
                frame[f"{name}_image"] = rgb_frames[name][:, :, ::-1].copy()
                depth = depth_frames.get(name)
                if depth is not None:
                    if depth.ndim == 3 and depth.shape[2] == 1:
                        depth = depth[:, :, 0]
                    frame[f"{name}_depth"] = np.asarray(depth, dtype=np.float32).copy()

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
