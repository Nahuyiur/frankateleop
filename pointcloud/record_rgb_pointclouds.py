"""Record configured RealSense RGB-D streams without starting robot nodes."""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from franka_capture.cameras.realsense import create_realsense_cameras
from franka_capture.config.fr3_single import DEFAULT_CAMERAS, DEFAULT_RECORDING
from pointcloud.depth_proof import (
    depth_png_relative_path,
    write_depth_proof,
    write_metric_depth_png,
)
from franka_capture.recording.episode_writer import EpisodeWriter
from franka_capture.recording.preview import concatenate_rgb_images, show_rgb_preview

DEFAULT_OUTPUT_ROOT = str(Path.home() / "Desktop" / "Muka_NAS")
DEFAULT_TASK = "rgb_pointcloud"
FIXED_RECORDING_FPS = 30


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--output-root", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--index", type=int, default=None)
    parser.add_argument(
        "--camera-names",
        default="all",
        help="Comma-separated configured camera names to record, or all.",
    )
    parser.add_argument(
        "--depth-cameras",
        default="all",
        help="Comma-separated recorded camera names that should enable depth, or all.",
    )
    parser.add_argument(
        "--no-depth",
        action="store_true",
        help="Record RGB only. 19 can still show RGB, but no PLY can be generated.",
    )
    parser.add_argument(
        "--camera-fps",
        type=int,
        default=FIXED_RECORDING_FPS,
        help="RealSense stream/video FPS. Lower this when multiple depth cameras exceed USB bandwidth.",
    )
    parser.add_argument("--width", type=int, default=640, help="RealSense stream width.")
    parser.add_argument("--height", type=int, default=480, help="RealSense stream height.")
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Start recording immediately instead of waiting for s.",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="If >0, save and quit automatically after this many recording seconds.",
    )
    parser.add_argument(
        "--no-depth-proof",
        action="store_true",
        help="Skip writing depth_proof PNG/PLY files after saving.",
    )
    parser.add_argument("--pointcloud-stride", type=int, default=4)
    parser.add_argument("--pointcloud-max-points", type=int, default=80000)
    return parser.parse_args()


def _next_episode_index(output_root: str, task: str, start_index: Optional[int]) -> int:
    task_dir = Path(output_root).expanduser() / task
    existing_indices = []
    if task_dir.exists():
        for child in task_dir.iterdir():
            if child.is_dir() and child.name.isdigit():
                existing_indices.append(int(child.name))

    next_index = max(existing_indices, default=-1) + 1
    if start_index is not None:
        next_index = max(next_index, int(start_index))

    while (task_dir / str(next_index)).exists():
        next_index += 1
    return next_index


def _require_muka_nas_if_needed(output_root: Path) -> None:
    nas_root = Path.home() / "Desktop" / "Muka_NAS"
    if output_root == nas_root or nas_root in output_root.parents:
        if not nas_root.is_mount():
            raise RuntimeError(f"NAS is not mounted at {nas_root}; mount Muka_NAS before recording.")


def _resolve_names(spec: str, available_names: list[str], label: str) -> list[str]:
    spec = (spec or "all").strip()
    if spec in {"all", "*"}:
        return list(available_names)

    names = [name.strip() for name in spec.split(",") if name.strip()]
    unknown = sorted(set(names) - set(available_names))
    if unknown:
        raise ValueError(f"Unknown {label}: {unknown}. Available: {available_names}")
    return names


def _camera_configs(
    camera_names: list[str],
    depth_camera_names: set[str],
    fps: int,
    dim: tuple[int, int],
) -> Dict[str, Any]:
    configs = {}
    for name in camera_names:
        config = DEFAULT_CAMERAS[name]
        configs[name] = replace(
            config,
            fps=fps,
            dim=dim,
            depth=(name in depth_camera_names),
            align_depth=True if name in depth_camera_names else config.align_depth,
        )
    return configs


def main() -> None:
    args = parse_args()
    import cv2

    available_names = list(DEFAULT_CAMERAS.keys())
    camera_names = _resolve_names(args.camera_names, available_names, "camera name(s)")
    depth_camera_names = set()
    if not args.no_depth:
        depth_camera_names = set(
            _resolve_names(args.depth_cameras, camera_names, "depth camera name(s)")
        )

    output_root_path = Path(args.output_root).expanduser()
    _require_muka_nas_if_needed(output_root_path)
    output_root = str(output_root_path)
    next_index = _next_episode_index(output_root, args.task, args.index)

    cameras = {}
    writer = None
    record_flag = False
    recording_started_monotonic = None
    camera_metadata: Dict[str, Any] = {}

    def start_episode() -> None:
        nonlocal writer, record_flag, next_index, recording_started_monotonic
        if writer is None:
            writer = EpisodeWriter(
                output_root=output_root,
                task=args.task,
                index=next_index,
                camera_names=camera_names,
                video_fps=int(args.camera_fps),
                metadata={
                    "source": "pointcloud.record_rgb_pointclouds",
                    "schema_version": "camera_rgbd_v1",
                    "started_at_unix": time.time(),
                    "camera_only": True,
                    "robot": None,
                    "cameras": camera_metadata,
                    "video_fps": int(args.camera_fps),
                    "rgb_recording": {
                        "enabled": True,
                        "storage": "{camera_name}.mp4",
                        "frame_index": "matches pkl frame_index",
                    },
                    "depth_recording": {
                        "enabled": bool(depth_camera_names),
                        "camera_names": sorted(depth_camera_names),
                        "storage": "per-frame depth/{camera_name}/{frame_index:06d}.png uint16 image",
                        "units": "uint16 depth units",
                        "meters_conversion": "depth_m = depth_uint16 * camera depth_scale",
                        "aligned_to": "color",
                        "pointcloud_derivation": "depth + RGB + camera intrinsics",
                        "depth_proof_dir": None
                        if args.no_depth_proof or not depth_camera_names
                        else "depth_proof",
                    },
                },
            )
            print(f"Start camera-only recording episode {writer.index}: {writer.output_dir}")
            next_index += 1
        else:
            print(f"Resume camera-only recording episode {writer.index}: {writer.output_dir}")
        record_flag = True
        if recording_started_monotonic is None:
            recording_started_monotonic = time.monotonic()

    def save_episode(quiet: bool = False) -> None:
        nonlocal writer, record_flag, recording_started_monotonic
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
        if depth_camera_names and not args.no_depth_proof:
            try:
                summary = write_depth_proof(
                    output_dir,
                    output_index,
                    frames,
                    metadata,
                    stride=args.pointcloud_stride,
                    max_points=args.pointcloud_max_points,
                )
                print(
                    "Depth proof written to "
                    f"{output_dir / 'depth_proof'} for cameras: "
                    f"{sorted(summary.get('cameras', {}).keys())}"
                )
            except Exception as exc:
                print(f"WARNING: failed to write depth proof files: {exc}")
        print(f"Saved {frame_count} RGB-D frames to {output_dir}")
        print(f"View it with: bash 19_view_recorded_rgb_pointclouds.sh {output_dir}")
        writer = None
        recording_started_monotonic = None

    def discard_episode() -> None:
        nonlocal writer, record_flag, next_index, recording_started_monotonic
        if writer is None:
            print("No active episode to discard")
            return
        record_flag = False
        discarded_index = writer.index
        output_dir = writer.discard()
        next_index = min(next_index, discarded_index)
        recording_started_monotonic = None
        writer = None
        print(f"Discarded episode {discarded_index}: removed {output_dir}")

    try:
        camera_configs = _camera_configs(
            camera_names,
            depth_camera_names,
            fps=int(args.camera_fps),
            dim=(int(args.width), int(args.height)),
        )
        print("Selected camera configs:")
        for name, config in camera_configs.items():
            print(
                f"  {name}: serial={config.serial_number}, "
                f"depth={config.depth}, fps={config.fps}, dim={config.dim}"
            )
        try:
            cameras = create_realsense_cameras(camera_configs)
        except Exception:
            print("")
            print("Camera startup failed.")
            print("Try isolating one camera, for example:")
            print("  bash 18_record_rgb_pointclouds.sh --camera-names middle --depth-cameras middle")
            print("If that works, add cameras back one by one to find the USB/camera that times out.")
            raise
        camera_metadata = {name: camera.metadata() for name, camera in cameras.items()}
        print(f"Connected cameras: {camera_names}")
        print(
            "Depth cameras: "
            f"{sorted(depth_camera_names) if depth_camera_names else 'disabled'}"
        )
        print(f"Output root: {output_root}")
        print(f"Task: {args.task}")
        print(f"Next episode index: {next_index}")
        print(
            "Click the RGB window first. "
            "s=start/resume, w=pause, e=save, d=discard, k=keyframe, q=save+quit."
        )

        if args.auto_start:
            start_episode()

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
            key = show_rgb_preview("RGB-D Camera Recorder", preview)

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
                if not record_flag or writer is None:
                    print("Keyframe ignored: recording is paused. Press s first.")
                else:
                    keyframe = writer.add_keyframe()
                    print(f"Episode {writer.index} keyframe {keyframe} added")

            if not record_flag:
                continue
            if writer is None:
                start_episode()

            frame_index = len(writer.frames)
            frame = {
                "schema_version": "camera_rgbd_v1",
                "camera_only": True,
                "frame_index": frame_index,
                "timestamp": time.time(),
            }
            for name in camera_names:
                frame[f"{name}_image_path"] = f"{name}.mp4"
                depth = depth_frames.get(name)
                if depth is not None:
                    if depth.ndim == 3 and depth.shape[2] == 1:
                        depth = depth[:, :, 0]
                    depth = np.asarray(depth, dtype=np.float32)
                    rel_path = depth_png_relative_path(name, frame_index)
                    depth_scale = float(camera_metadata.get(name, {}).get("depth_scale") or 0.001)
                    write_metric_depth_png(writer.output_dir / rel_path, depth, depth_scale)
                    frame[f"{name}_depth_path"] = str(rel_path)
                    frame[f"{name}_depth_scale"] = depth_scale

            writer.append(frame, rgb_frames)

            if (
                args.duration_sec > 0
                and recording_started_monotonic is not None
                and time.monotonic() - recording_started_monotonic >= args.duration_sec
            ):
                save_episode()
                break
    finally:
        if writer is not None:
            writer.close()
        cv2.destroyAllWindows()
        for camera in cameras.values():
            camera.close()
        print("Cameras closed.")


if __name__ == "__main__":
    main()
