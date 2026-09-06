"""Record configured RealSense RGB-D streams without starting robot nodes."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
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


@dataclass
class _FrameSample:
    rgb: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    captured_monotonic_ns: Optional[int] = None
    sequence: int = 0
    error: Optional[str] = None


class _CameraWorker:
    """Continuously read one camera so three USB streams do not block serially."""

    def __init__(self, name: str, camera: Any) -> None:
        self.name = name
        self.camera = camera
        self.sample = _FrameSample()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name=f"rgbd-reader-{name}",
            daemon=True,
        )

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()

    def join(self, timeout: float = 3.0) -> None:
        self.thread.join(timeout=timeout)

    def latest(self) -> _FrameSample:
        with self.lock:
            return _FrameSample(
                rgb=None if self.sample.rgb is None else self.sample.rgb.copy(),
                depth=None if self.sample.depth is None else self.sample.depth.copy(),
                captured_monotonic_ns=self.sample.captured_monotonic_ns,
                sequence=self.sample.sequence,
                error=self.sample.error,
            )

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                rgb, depth = self.camera.read()
                with self.lock:
                    self.sample.rgb = rgb
                    self.sample.depth = depth
                    self.sample.captured_monotonic_ns = time.monotonic_ns()
                    self.sample.sequence += 1
                    self.sample.error = None
            except Exception as exc:
                with self.lock:
                    self.sample.error = str(exc)
                self.stop_event.wait(0.05)


def _retime_constant_rate_videos(
    output_dir: Path,
    camera_names: list[str],
    source_fps: float,
    effective_fps: float,
) -> None:
    """Correct MP4 presentation timestamps without re-encoding image data."""

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required to correct the recorded video timeline")
    scale = float(source_fps) / float(effective_fps)
    completed: list[tuple[Path, Path]] = []
    temporary_paths = [output_dir / f".{name}.retimed.mp4" for name in camera_names]
    try:
        for name, temporary in zip(camera_names, temporary_paths):
            source = output_dir / f"{name}.mp4"
            if temporary.exists():
                temporary.unlink()
            subprocess.run(
                [
                    ffmpeg,
                    "-y",
                    "-loglevel",
                    "error",
                    "-itsscale",
                    f"{scale:.12f}",
                    "-i",
                    str(source),
                    "-map",
                    "0:v:0",
                    "-c",
                    "copy",
                    str(temporary),
                ],
                check=True,
            )
            completed.append((source, temporary))
        for source, temporary in completed:
            temporary.replace(source)
    finally:
        for temporary in temporary_paths:
            if temporary.exists():
                temporary.unlink()


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
    workers: Dict[str, _CameraWorker] = {}
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
        if frame_count >= 2:
            capture_times = [
                float(np.mean(list(frame["camera_capture_monotonic_ns"].values())))
                / 1e9
                for frame in frames
            ]
            capture_span_sec = capture_times[-1] - capture_times[0]
            if capture_span_sec > 0:
                effective_fps = (frame_count - 1) / capture_span_sec
                source_fps = float(writer.video_fps)
                # The encoder starts as CFR at the requested camera rate.  If
                # disk/codec work drops frames, scale MP4 PTS to the measured
                # capture span so human motion does not play back too quickly.
                if abs(effective_fps - source_fps) / source_fps > 0.01:
                    writer.close_videos()
                    _retime_constant_rate_videos(
                        writer.output_dir,
                        camera_names,
                        source_fps,
                        effective_fps,
                    )
                writer.video_fps = float(effective_fps)
                writer.update_metadata(
                    {
                        "requested_camera_fps": int(args.camera_fps),
                        "measured_video_fps": float(effective_fps),
                        "capture_span_sec": float(capture_span_sec),
                        "timing_basis": "mean per-camera capture timestamp; MP4 PTS retimed without re-encoding",
                    }
                )
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
        workers = {
            name: _CameraWorker(name, camera) for name, camera in cameras.items()
        }
        for worker in workers.values():
            worker.start()
        startup_deadline = time.monotonic() + 12.0
        while time.monotonic() < startup_deadline:
            startup_samples = {name: worker.latest() for name, worker in workers.items()}
            if all(sample.rgb is not None for sample in startup_samples.values()):
                break
            time.sleep(0.02)
        else:
            details = {
                name: sample.error or "no frame"
                for name, sample in startup_samples.items()
                if sample.rgb is None
            }
            raise RuntimeError(f"Timed out waiting for camera frames: {details}")
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

        last_sequences = {name: 0 for name in camera_names}
        while True:
            samples = {name: workers[name].latest() for name in camera_names}
            failed = {
                name: sample.error for name, sample in samples.items() if sample.error
            }
            if failed:
                raise RuntimeError(f"Camera reader failed: {failed}")
            # Process each physical frame at most once. Waiting for every stream
            # also prevents a fast camera from being duplicated while a slower
            # camera catches up.
            if not all(
                samples[name].sequence > last_sequences[name] for name in camera_names
            ):
                time.sleep(0.001)
                continue
            last_sequences = {name: samples[name].sequence for name in camera_names}
            rgb_frames = {name: samples[name].rgb for name in camera_names}
            depth_frames = {
                name: samples[name].depth
                for name in camera_names
                if samples[name].depth is not None
            }

            preview_frames = []
            for name in camera_names:
                # Keep recorded frames untouched; labels exist only in the live
                # preview so the operator cannot confuse the three viewpoints.
                source = rgb_frames[name]
                panel_width = 480
                panel_height = max(1, round(source.shape[0] * panel_width / source.shape[1]))
                panel = cv2.resize(
                    source,
                    (panel_width, panel_height),
                    interpolation=cv2.INTER_AREA,
                )
                state = "REC" if record_flag else "READY / PAUSED"
                label = f"{name}  {state}"
                cv2.putText(
                    panel,
                    label,
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 0),
                    4,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    panel,
                    label,
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                preview_frames.append(panel)
            preview = concatenate_rgb_images(
                preview_frames,
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
                "camera_sequence": {
                    name: int(samples[name].sequence) for name in camera_names
                },
                "camera_capture_monotonic_ns": {
                    name: int(samples[name].captured_monotonic_ns)
                    for name in camera_names
                },
                "capture_spread_ms": (
                    max(samples[name].captured_monotonic_ns for name in camera_names)
                    - min(samples[name].captured_monotonic_ns for name in camera_names)
                )
                / 1e6,
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
        for worker in workers.values():
            worker.stop()
        for worker in workers.values():
            worker.join()
        for camera in cameras.values():
            camera.close()
        print("Cameras closed.")


if __name__ == "__main__":
    main()
