"""Validate saved Franka teleop task data under the NAS recording root.

This tool verifies saved frame-level alignment between videos and action data.
It cannot prove RealSense hardware timestamp synchronization because the current
episode format does not store per-camera hardware timestamps.
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import os
import pickle
import re
import sys
import threading
import traceback
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

QUALITY_DIRS = ("High_Quality", "Low_Quality", "Failure")
PREVIEW_STEM = "preview_all"
KNOWN_CAMERA_SETS = {
    "left": ("left_wrist", "left", "middle"),
    "right": ("middle", "right", "right_wrist"),
    "dual": ("left_wrist", "left", "middle", "right", "right_wrist"),
}
SKIP_DIR_PREFIXES = (".", "#")
DEFAULT_ROOT = Path.home() / "Desktop" / "Muka_NAS"
V3_SCHEMA_SUFFIX = "_v3"


@dataclass
class Issue:
    level: str
    code: str
    message: str


@dataclass
class VideoInfo:
    path: str
    frame_count: int
    fps: float
    width: int
    height: int


@dataclass
class EpisodeReport:
    path: str
    status: str = "PASS"
    schema_version: str = ""
    frame_count: int = 0
    camera_names: List[str] = field(default_factory=list)
    duration_sec: float = 0.0
    average_fps: float = 0.0
    issues: List[Issue] = field(default_factory=list)
    videos: Dict[str, VideoInfo] = field(default_factory=dict)

    def add(self, level: str, code: str, message: str) -> None:
        self.issues.append(Issue(level=level, code=code, message=message))
        if level == "ERROR":
            self.status = "FAIL"
        elif self.status == "PASS":
            self.status = "WARN"

    @property
    def error_count(self) -> int:
        return sum(1 for issue in self.issues if issue.level == "ERROR")

    @property
    def warning_count(self) -> int:
        return sum(1 for issue in self.issues if issue.level == "WARN")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "task_name",
        help=(
            "Task folder below --root. Nested task paths such as "
            "chuyao_data_collection/pick_banana are allowed; absolute paths "
            "and '..' are rejected."
        ),
    )
    parser.add_argument("--root", default=str(DEFAULT_ROOT), help="NAS recording root.")
    parser.add_argument(
        "--quality",
        action="append",
        choices=QUALITY_DIRS,
        help="Only validate episodes whose path contains this quality directory. May repeat.",
    )
    parser.add_argument("--limit", type=int, default=0, help="Validate at most this many episodes.")
    parser.add_argument("--min-cameras", type=int, default=3, help="Minimum required camera views.")
    parser.add_argument(
        "--strict-camera-set",
        choices=("none", "auto", "left", "right", "dual"),
        default="auto",
        help="Require an exact known camera set. auto uses schema_version or arm_side when possible.",
    )
    parser.add_argument(
        "--max-video-frame-delta",
        type=int,
        default=0,
        help="Allowed absolute difference between pkl frame count and each mp4 frame count.",
    )
    parser.add_argument(
        "--max-duration-delta-sec",
        type=float,
        default=0.25,
        help="Allowed difference between pkl timestamp duration and mp4 frame-count/fps duration.",
    )
    parser.add_argument(
        "--expected-fps",
        type=float,
        default=30.0,
        help="Fallback FPS when metadata has no video_fps.",
    )
    parser.add_argument(
        "--max-fps-rel-error",
        type=float,
        default=0.10,
        help="Maximum relative difference between timestamp-derived FPS and target FPS.",
    )
    parser.add_argument(
        "--warn-timestamp-gap",
        type=float,
        default=0.05,
        help="Warn when any adjacent timestamp gap exceeds this many seconds.",
    )
    parser.add_argument(
        "--max-timestamp-gap",
        type=float,
        default=0.10,
        help="Maximum allowed gap between adjacent pkl timestamps in seconds.",
    )
    parser.add_argument(
        "--max-timestamp-p95-gap",
        type=float,
        default=0.045,
        help="Fail when p95 adjacent timestamp gap exceeds this many seconds.",
    )
    parser.add_argument("--warn-joint-delta", type=float, default=0.10, help="Warn single-frame joint jump in rad.")
    parser.add_argument("--max-joint-delta", type=float, default=0.20, help="Fail single-frame joint jump in rad.")
    parser.add_argument("--max-joint-velocity", type=float, default=6.0, help="Max joint velocity in rad/s.")
    parser.add_argument("--max-abs-joint", type=float, default=4.0, help="Max absolute joint value in rad.")
    parser.add_argument("--min-gripper-width", type=float, default=-0.0001, help="Minimum valid gripper width in m.")
    parser.add_argument("--max-gripper-width", type=float, default=0.091, help="Maximum valid gripper width in m.")
    parser.add_argument(
        "--max-robot-state-age-ms",
        type=float,
        default=100.0,
        help="Maximum robot_state_age_ms when present.",
    )
    parser.add_argument(
        "--warn-robot-state-age-p95-ms",
        type=float,
        default=50.0,
        help="Warn when p95 robot_state_age_ms exceeds this value.",
    )
    parser.add_argument(
        "--max-robot-read-duration-ms",
        type=float,
        default=50.0,
        help="Maximum robot_read_duration_ms when present.",
    )
    parser.add_argument(
        "--warn-robot-read-duration-p95-ms",
        type=float,
        default=20.0,
        help="Warn when p95 robot_read_duration_ms exceeds this value.",
    )
    parser.add_argument(
        "--max-invalid-robot-ratio",
        type=float,
        default=0.0,
        help="Maximum allowed ratio of frames with robot_state_valid == false.",
    )
    parser.add_argument("--strict", action="store_true", help="Return failure when warnings are present.")
    parser.add_argument("--fail-fast", action="store_true", help="Stop after the first failed episode.")
    parser.add_argument("--json-output", default="", help="Optional path for machine-readable JSON report.")
    parser.add_argument("--verbose", action="store_true", help="Print all warning details for every episode.")
    return parser.parse_args(argv)


def default_episode_validation_args() -> argparse.Namespace:
    """Return the default V-script thresholds for one already-resolved episode."""
    return parse_args(["__single_episode__"])


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        task_root = resolve_task_root(args.root, args.task_name)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if not task_root.is_dir():
        print(f"ERROR: task folder does not exist: {task_root}", file=sys.stderr)
        return 2

    episodes = discover_episodes(task_root, args.quality)
    if args.limit and args.limit > 0:
        episodes = episodes[: args.limit]

    print(f"Task root: {task_root}")
    print(f"Episodes found: {len(episodes)}")
    if args.quality:
        print(f"Quality filter: {', '.join(args.quality)}")
    print(
        "Sync scope: saved frame-level alignment only; camera hardware timestamps "
        "are not present in this data format."
    )

    reports: List[EpisodeReport] = []
    for index, episode_dir in enumerate(episodes, start=1):
        report = validate_episode(episode_dir, args)
        reports.append(report)
        print_episode_report(report, task_root, index, len(episodes), args.verbose)
        if args.fail_fast and report.status == "FAIL":
            break

    summary = summarize_reports(reports)
    print_summary(summary, strict=args.strict)

    if args.json_output:
        write_json_report(Path(args.json_output).expanduser(), task_root, args, reports, summary)

    if summary["fail"] > 0:
        return 1
    if args.strict and summary["warn"] > 0:
        return 1
    return 0


def resolve_task_root(root: str, task_name: str) -> Path:
    root_path = Path(root).expanduser()
    task_path = Path(task_name)
    if task_path.is_absolute():
        raise ValueError("task_name must be relative to --root, not an absolute path")
    invalid_parts = {"", ".", ".."}
    for part in task_path.parts:
        if part in invalid_parts:
            raise ValueError(f"task_name contains invalid path component: {part!r}")
        if part.startswith(".partial"):
            raise ValueError(f"task_name cannot point into partial output: {part!r}")
    return root_path / task_path


def discover_episodes(task_root: Path, qualities: Optional[Sequence[str]]) -> List[Path]:
    quality_set = set(qualities or [])
    episode_dirs = set()
    for current_root, dirnames, filenames in os.walk(task_root):
        dirnames[:] = [name for name in dirnames if not should_skip_dir(name)]
        current = Path(current_root)
        if any(part.startswith(".partial") or should_skip_dir(part) for part in current.relative_to(task_root).parts):
            continue
        if quality_set and not quality_set.intersection(current.parts):
            continue
        if any(name.endswith(".pkl.gz") for name in filenames):
            episode_dirs.add(current)
    return sorted(episode_dirs, key=natural_path_key)


def should_skip_dir(name: str) -> bool:
    return name.startswith(SKIP_DIR_PREFIXES) or name.startswith(".partial")


def natural_path_key(path: Path) -> List[Any]:
    parts: List[Any] = []
    for token in re.split(r"(\d+)", path.as_posix()):
        if token.isdigit():
            parts.append(int(token))
        else:
            parts.append(token.casefold())
    return parts


class ValidationCancelled(RuntimeError):
    pass


def _raise_if_cancelled(cancel_event: Optional[threading.Event]) -> None:
    if cancel_event is not None and cancel_event.is_set():
        raise ValidationCancelled("episode validation cancelled")


def validate_episode(
    episode_dir: Path,
    args: argparse.Namespace,
    cancel_event: Optional[threading.Event] = None,
) -> EpisodeReport:
    _raise_if_cancelled(cancel_event)
    report = EpisodeReport(path=str(episode_dir))
    metadata = load_metadata(episode_dir / "metadata.json", report)
    pkl_path = select_pkl(episode_dir, report)
    if pkl_path is None:
        return report

    frames = load_frames(pkl_path, report)
    _raise_if_cancelled(cancel_event)
    if not frames:
        return report
    report.frame_count = len(frames)
    first_frame = frames[0]
    metadata_schema = str(metadata.get("schema_version") or "")
    frame_schema = str(first_frame.get("schema_version") or "")
    v3_schema = next(
        (schema for schema in (metadata_schema, frame_schema) if is_v3_schema(schema)),
        "",
    )
    report.schema_version = v3_schema or metadata_schema or frame_schema

    validate_metadata_counts(metadata, frames, report)
    if v3_schema:
        validate_v3_contract(episode_dir, frames, metadata, v3_schema, report)
    validate_keyframes(episode_dir / "keyframes.json", frames, report)

    camera_names = resolve_camera_names(metadata, first_frame, episode_dir, report)
    report.camera_names = camera_names
    validate_camera_sets(camera_names, metadata, report.schema_version, args, report)
    validate_frame_camera_fields(frames, camera_names, metadata, report.schema_version, report)
    validate_videos(
        episode_dir,
        camera_names,
        frames,
        metadata,
        report,
        args,
        cancel_event=cancel_event,
    )

    _raise_if_cancelled(cancel_event)
    timestamps = validate_timestamps(frames, metadata, report, args)
    validate_robot_timing(frames, report, args)
    validate_actions(frames, timestamps, report, args)
    return report


def load_metadata(path: Path, report: EpisodeReport) -> Dict[str, Any]:
    if not path.exists():
        report.add("WARN", "metadata_missing", "metadata.json is missing; using pkl/mp4 fallback checks")
        return {}
    try:
        with path.open("r", encoding="utf-8") as f:
            metadata = json.load(f)
    except Exception as exc:
        report.add("ERROR", "metadata_corrupt", f"failed to read metadata.json: {exc}")
        return {}
    if not isinstance(metadata, dict):
        report.add("ERROR", "metadata_type", "metadata.json top-level value is not an object")
        return {}
    return metadata


def select_pkl(episode_dir: Path, report: EpisodeReport) -> Optional[Path]:
    pkl_files = sorted(episode_dir.glob("*.pkl.gz"), key=natural_path_key)
    if not pkl_files:
        report.add("ERROR", "pkl_missing", "no *.pkl.gz file found")
        return None
    if len(pkl_files) > 1:
        report.add(
            "ERROR",
            "pkl_multiple",
            "multiple *.pkl.gz files found: " + ", ".join(path.name for path in pkl_files),
        )
    return pkl_files[0]


def load_frames(pkl_path: Path, report: EpisodeReport) -> List[Dict[str, Any]]:
    try:
        with gzip.open(pkl_path, "rb") as f:
            payload = pickle.load(f)
    except Exception as exc:
        report.add("ERROR", "pkl_corrupt", f"failed to read {pkl_path.name}: {exc}")
        return []
    if isinstance(payload, dict):
        frames = payload.get("data")
        if frames is None:
            frames = payload.get("frames")
    else:
        frames = payload
    if not isinstance(frames, list):
        report.add("ERROR", "pkl_frames_type", "pkl payload does not contain a frame list")
        return []
    if not frames:
        report.add("ERROR", "empty_episode", "frame list is empty")
        return []
    if not all(isinstance(frame, dict) for frame in frames):
        report.add("ERROR", "frame_type", "not every frame is a dict")
        return []
    return frames


def validate_metadata_counts(metadata: Dict[str, Any], frames: List[Dict[str, Any]], report: EpisodeReport) -> None:
    metadata_count = metadata.get("frame_count")
    if metadata_count is None:
        return
    try:
        count = int(metadata_count)
    except (TypeError, ValueError):
        report.add("ERROR", "metadata_frame_count_type", f"metadata frame_count is not an integer: {metadata_count!r}")
        return
    if count != len(frames):
        report.add(
            "ERROR",
            "metadata_frame_count_mismatch",
            f"metadata frame_count={count} but pkl frames={len(frames)}",
        )


def is_v3_schema(schema_version: Any) -> bool:
    return isinstance(schema_version, str) and schema_version.endswith(V3_SCHEMA_SUFFIX)


def validate_v3_contract(
    episode_dir: Path,
    frames: List[Dict[str, Any]],
    metadata: Dict[str, Any],
    schema_version: str,
    report: EpisodeReport,
) -> None:
    """Validate the media-free v3 contract without legacy fallbacks."""

    metadata_schema = metadata.get("schema_version")
    if metadata_schema != schema_version:
        report.add(
            "ERROR",
            "v3_metadata_schema_version",
            f"metadata schema_version must be {schema_version!r}, got {metadata_schema!r}",
        )

    frame_schemas = [frame.get("schema_version") for frame in frames]
    bad_schema_frames = [
        index for index, value in enumerate(frame_schemas) if value != schema_version
    ]
    if bad_schema_frames:
        report.add(
            "ERROR",
            "v3_frame_schema_version",
            f"frame schema_version differs at frames {bad_schema_frames[:8]}",
        )

    frame_indices: List[int] = []
    invalid_frame_indices: List[int] = []
    for index, frame in enumerate(frames):
        value = frame.get("frame_index")
        if isinstance(value, bool) or not isinstance(value, int):
            invalid_frame_indices.append(index)
        else:
            frame_indices.append(value)
    if invalid_frame_indices or frame_indices != list(range(len(frames))):
        actual = [frame.get("frame_index") for frame in frames[:12]]
        report.add(
            "ERROR",
            "v3_frame_index_sequence",
            (
                f"frame_index must be exactly 0..{len(frames) - 1}; "
                f"first values={actual}"
            ),
        )

    embedded_fields = sorted(
        {
            key
            for frame in frames
            for key in frame
            if key.endswith("_image")
        }
    )
    if embedded_fields:
        report.add(
            "ERROR",
            "v3_embedded_images",
            f"v3 trajectory must be media-free; embedded fields={embedded_fields}",
        )

    metadata_count = _strict_positive_int(metadata.get("frame_count"))
    if metadata_count is None:
        report.add(
            "ERROR",
            "v3_metadata_frame_count_type",
            f"metadata frame_count must be a positive integer, got {metadata.get('frame_count')!r}",
        )
    elif metadata_count != len(frames):
        report.add(
            "ERROR",
            "v3_metadata_frame_count_mismatch",
            f"metadata frame_count={metadata_count}, trajectory frames={len(frames)}",
        )

    raw_camera_names = metadata.get("camera_names")
    camera_names: List[str] = []
    if (
        not isinstance(raw_camera_names, list)
        or not raw_camera_names
        or not all(isinstance(name, str) and bool(name) for name in raw_camera_names)
        or len(set(raw_camera_names)) != len(raw_camera_names)
        or any(Path(name).name != name for name in raw_camera_names if isinstance(name, str))
    ):
        report.add(
            "ERROR",
            "v3_camera_names",
            "metadata camera_names must be a non-empty list of unique safe strings",
        )
    else:
        camera_names = list(raw_camera_names)

    storage = metadata.get("image_storage")
    if not isinstance(storage, dict):
        report.add("ERROR", "v3_image_storage_type", "image_storage must be an object")
        return
    if storage.get("type") != "video":
        report.add(
            "ERROR",
            "v3_image_storage_type",
            f"image_storage.type must be 'video', got {storage.get('type')!r}",
        )
    if storage.get("frame_alignment") != "frame_index":
        report.add(
            "ERROR",
            "v3_frame_alignment",
            (
                "image_storage.frame_alignment must be 'frame_index', got "
                f"{storage.get('frame_alignment')!r}"
            ),
        )
    if storage.get("decoded_color_order") != "BGR":
        report.add(
            "ERROR",
            "v3_decoded_color_order",
            (
                "image_storage.decoded_color_order must be 'BGR', got "
                f"{storage.get('decoded_color_order')!r}"
            ),
        )
    if storage.get("source_color_order") != "RGB":
        report.add(
            "ERROR",
            "v3_source_color_order",
            (
                "image_storage.source_color_order must be 'RGB', got "
                f"{storage.get('source_color_order')!r}"
            ),
        )

    cameras = storage.get("cameras")
    if not isinstance(cameras, dict) or not cameras:
        report.add(
            "ERROR",
            "v3_image_storage_cameras",
            "image_storage.cameras must be a non-empty object",
        )
        return
    if camera_names and set(cameras) != set(camera_names):
        report.add(
            "ERROR",
            "v3_camera_set_mismatch",
            (
                f"camera_names={sorted(camera_names)}, "
                f"image_storage.cameras={sorted(cameras, key=str)}"
            ),
        )

    mp4_camera_names = {
        path.stem for path in episode_dir.glob("*.mp4") if path.stem != PREVIEW_STEM
    }
    if camera_names and mp4_camera_names != set(camera_names):
        report.add(
            "ERROR",
            "v3_mp4_camera_set_mismatch",
            (
                f"camera_names={sorted(camera_names)}, "
                f"camera mp4 files={sorted(mp4_camera_names)}"
            ),
        )

    for camera_name in camera_names:
        camera = cameras.get(camera_name)
        if not isinstance(camera, dict):
            report.add(
                "ERROR",
                "v3_camera_entry_type",
                f"image_storage.cameras.{camera_name} must be an object",
            )
            continue
        expected_filename = f"{camera_name}.mp4"
        if camera.get("filename") != expected_filename:
            report.add(
                "ERROR",
                "v3_camera_filename",
                (
                    f"camera {camera_name} filename must be {expected_filename!r}, "
                    f"got {camera.get('filename')!r}"
                ),
            )
        for field_name in ("width", "height", "channels", "frame_count"):
            value = _strict_positive_int(camera.get(field_name))
            if value is None:
                report.add(
                    "ERROR",
                    f"v3_camera_{field_name}",
                    (
                        f"camera {camera_name} {field_name} must be a positive integer, "
                        f"got {camera.get(field_name)!r}"
                    ),
                )
                continue
            if field_name == "channels" and value != 3:
                report.add(
                    "ERROR",
                    "v3_camera_channels",
                    f"camera {camera_name} channels must be 3, got {value}",
                )
            if field_name == "frame_count" and value != len(frames):
                report.add(
                    "ERROR",
                    "v3_camera_frame_count",
                    (
                        f"camera {camera_name} frame_count={value}, "
                        f"trajectory frames={len(frames)}"
                    ),
                )


def _strict_positive_int(value: Any) -> Optional[int]:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        return None
    return value


def validate_keyframes(path: Path, frames: List[Dict[str, Any]], report: EpisodeReport) -> None:
    if not path.exists():
        report.add("WARN", "keyframes_missing", "keyframes.json is missing")
        return
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        report.add("ERROR", "keyframes_corrupt", f"failed to read keyframes.json: {exc}")
        return
    keyframes = data.get("keyframes") if isinstance(data, dict) else None
    if keyframes is None:
        report.add("WARN", "keyframes_field_missing", "keyframes.json has no keyframes field")
        return
    bad = [idx for idx in keyframes if not isinstance(idx, int) or idx < 0 or idx >= len(frames)]
    if bad:
        report.add("ERROR", "keyframes_out_of_range", f"keyframes outside frame range: {bad[:8]}")


def resolve_camera_names(
    metadata: Dict[str, Any],
    first_frame: Dict[str, Any],
    episode_dir: Path,
    report: EpisodeReport,
) -> List[str]:
    metadata_cameras = normalize_camera_list(metadata.get("camera_names"))
    image_cameras = sorted(name[:-6] for name in first_frame if name.endswith("_image"))
    mp4_cameras = sorted(path.stem for path in episode_dir.glob("*.mp4") if path.stem != PREVIEW_STEM)

    if metadata_cameras:
        cameras = metadata_cameras
    elif image_cameras:
        cameras = image_cameras
        report.add("WARN", "camera_names_from_pkl", "metadata camera_names missing; using pkl image fields")
    else:
        cameras = mp4_cameras
        report.add("WARN", "camera_names_from_mp4", "metadata/pkl camera names missing; using mp4 files")

    missing_in_pkl = sorted(set(cameras) - set(image_cameras))
    if missing_in_pkl and not is_video_backed_episode(metadata):
        report.add("ERROR", "camera_missing_in_pkl", f"camera image fields missing from pkl: {missing_in_pkl}")
    extra_in_pkl = sorted(set(image_cameras) - set(cameras))
    if extra_in_pkl:
        report.add("WARN", "camera_extra_in_pkl", f"extra camera image fields in pkl: {extra_in_pkl}")

    missing_mp4 = sorted(set(cameras) - set(mp4_cameras))
    if missing_mp4:
        report.add("ERROR", "camera_mp4_missing", f"mp4 files missing for cameras: {missing_mp4}")
    extra_mp4 = sorted(set(mp4_cameras) - set(cameras))
    if extra_mp4:
        report.add("WARN", "camera_mp4_extra", f"extra camera mp4 files: {extra_mp4}")
    return cameras


def normalize_camera_list(value: Any) -> List[str]:
    if not isinstance(value, list):
        return []
    cameras = []
    for item in value:
        text = str(item).strip()
        if text:
            cameras.append(text)
    return cameras


def is_video_backed_episode(metadata: Dict[str, Any]) -> bool:
    storage = metadata.get("image_storage")
    return isinstance(storage, dict) and storage.get("type") == "video"


def video_storage_camera(metadata: Dict[str, Any], camera_name: str) -> Dict[str, Any]:
    storage = metadata.get("image_storage")
    if not isinstance(storage, dict):
        return {}
    cameras = storage.get("cameras")
    if not isinstance(cameras, dict):
        return {}
    camera = cameras.get(camera_name)
    return camera if isinstance(camera, dict) else {}


def validate_camera_sets(
    camera_names: List[str],
    metadata: Dict[str, Any],
    schema_version: str,
    args: argparse.Namespace,
    report: EpisodeReport,
) -> None:
    if len(camera_names) < args.min_cameras:
        report.add(
            "ERROR",
            "too_few_cameras",
            f"expected at least {args.min_cameras} cameras, found {len(camera_names)}: {camera_names}",
        )

    strict_mode = args.strict_camera_set
    if strict_mode == "auto":
        if schema_version.startswith("franka_dual"):
            strict_mode = "dual"
        elif metadata.get("arm_side") in ("left", "right"):
            strict_mode = str(metadata["arm_side"])
        else:
            strict_mode = "none"
    if strict_mode == "none":
        return
    expected = KNOWN_CAMERA_SETS[strict_mode]
    if set(camera_names) != set(expected):
        report.add(
            "ERROR",
            "strict_camera_set_mismatch",
            f"{strict_mode} camera set expected {list(expected)}, found {camera_names}",
        )


def validate_frame_camera_fields(
    frames: List[Dict[str, Any]],
    camera_names: List[str],
    metadata: Dict[str, Any],
    schema_version: str,
    report: EpisodeReport,
) -> None:
    if is_video_backed_episode(metadata) or schema_version.endswith("_v3"):
        return
    missing_counts = {camera: 0 for camera in camera_names}
    bad_shape_counts = {camera: 0 for camera in camera_names}
    for frame in frames:
        for camera in camera_names:
            field = f"{camera}_image"
            if field not in frame:
                missing_counts[camera] += 1
                continue
            image = np.asarray(frame[field])
            if image.ndim != 3 or image.shape[2] != 3:
                bad_shape_counts[camera] += 1
    missing = {camera: count for camera, count in missing_counts.items() if count}
    if missing:
        report.add("ERROR", "camera_image_field_missing", f"missing image fields per camera: {missing}")
    bad_shapes = {camera: count for camera, count in bad_shape_counts.items() if count}
    if bad_shapes:
        report.add("ERROR", "camera_image_bad_shape", f"bad image shapes per camera: {bad_shapes}")


def validate_videos(
    episode_dir: Path,
    camera_names: List[str],
    frames: List[Dict[str, Any]],
    metadata: Dict[str, Any],
    report: EpisodeReport,
    args: argparse.Namespace,
    *,
    cancel_event: Optional[threading.Event] = None,
) -> None:
    pkl_count = len(frames)
    pkl_duration = timestamp_duration(frames)
    strict_v3 = is_v3_schema(report.schema_version)
    for camera in camera_names:
        _raise_if_cancelled(cancel_event)
        path = episode_dir / f"{camera}.mp4"
        if not path.exists():
            continue
        expected = video_storage_camera(metadata, camera)
        expected_dimensions: Optional[Tuple[int, int, int]] = None
        if expected:
            expected_width = _strict_positive_int(expected.get("width"))
            expected_height = _strict_positive_int(expected.get("height"))
            expected_channels = _strict_positive_int(expected.get("channels"))
            if expected_width and expected_height and expected_channels:
                expected_dimensions = (
                    expected_width,
                    expected_height,
                    expected_channels,
                )
        info = read_video_info(
            path,
            report=report,
            expected_dimensions=expected_dimensions,
            expected_frame_count=pkl_count,
            cancel_event=cancel_event,
        )
        report.videos[camera] = info
        validate_one_video(
            path,
            info,
            pkl_count,
            pkl_duration,
            report,
            args,
            strict_frame_count=strict_v3,
        )
        metadata_fps = safe_float(metadata.get("video_fps"))
        if metadata_fps and info.fps > 0.0:
            rel = abs(info.fps - metadata_fps) / max(metadata_fps, 1e-6)
            if rel > 0.1:
                report.add("WARN", "video_fps_metadata_mismatch", f"{path.name} fps={info.fps:.3f}, metadata={metadata_fps:.3f}")
    preview = episode_dir / f"{PREVIEW_STEM}.mp4"
    if preview.exists():
        info = read_video_info(
            preview,
            report=report,
            expected_frame_count=pkl_count,
            cancel_event=cancel_event,
        )
        report.videos[PREVIEW_STEM] = info
        validate_one_video(
            preview,
            info,
            pkl_count,
            pkl_duration,
            report,
            args,
            strict_frame_count=strict_v3,
        )
    elif metadata_declares_preview(metadata):
        report.add("WARN", "preview_missing", "declared preview_all.mp4 is missing")


def validate_one_video(
    path: Path,
    info: VideoInfo,
    pkl_count: int,
    pkl_duration: float,
    report: EpisodeReport,
    args: argparse.Namespace,
    *,
    strict_frame_count: bool = False,
) -> None:
    if info.frame_count <= 0:
        report.add("ERROR", "video_empty", f"{path.name} has no readable frames")
        return
    frame_delta = abs(info.frame_count - pkl_count)
    allowed_frame_delta = 0 if strict_frame_count else args.max_video_frame_delta
    if frame_delta > allowed_frame_delta:
        report.add(
            "ERROR",
            "video_frame_count_mismatch",
            (
                f"{path.name} decoded frames={info.frame_count}, pkl frames={pkl_count}, "
                f"delta={frame_delta}, allowed={allowed_frame_delta}"
            ),
        )
    if info.fps > 0.0 and pkl_duration > 0.0:
        video_interval_duration = max(info.frame_count - 1, 0) / info.fps
        duration_delta = abs(video_interval_duration - pkl_duration)
        if duration_delta > args.max_duration_delta_sec:
            report.add(
                "ERROR",
                "video_duration_mismatch",
                (
                    f"{path.name} duration by frame_count/fps={video_interval_duration:.3f}s, "
                    f"pkl timestamp duration={pkl_duration:.3f}s, delta={duration_delta:.3f}s"
                ),
            )


def read_video_info(
    path: Path,
    *,
    report: Optional[EpisodeReport] = None,
    expected_dimensions: Optional[Tuple[int, int, int]] = None,
    expected_frame_count: Optional[int] = None,
    cancel_event: Optional[threading.Event] = None,
) -> VideoInfo:
    """Sequentially decode every frame and report decoded dimensions/count."""

    cap = cv2.VideoCapture(str(path))
    try:
        if not cap.isOpened():
            if report is not None:
                report.add("ERROR", "video_unreadable", f"failed to open {path.name}")
            return VideoInfo(path=str(path), frame_count=0, fps=0.0, width=0, height=0)
        fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
        decoded_count = 0
        width = 0
        height = 0
        bad_frames: List[int] = []
        changing_size_frames: List[int] = []
        expected_size_frames: List[int] = []
        first_shape: Optional[Tuple[int, int, int]] = None

        while True:
            _raise_if_cancelled(cancel_event)
            ok, frame = cap.read()
            if not ok:
                break
            frame_index = decoded_count
            decoded_count += 1
            if (
                not isinstance(frame, np.ndarray)
                or frame.dtype != np.uint8
                or frame.ndim != 3
                or frame.shape[2] != 3
            ):
                bad_frames.append(frame_index)
                continue

            shape = (int(frame.shape[1]), int(frame.shape[0]), int(frame.shape[2]))
            if first_shape is None:
                first_shape = shape
                width, height, _ = shape
            elif shape != first_shape:
                changing_size_frames.append(frame_index)
            if expected_dimensions is not None and shape != expected_dimensions:
                expected_size_frames.append(frame_index)

        if report is not None:
            if bad_frames:
                report.add(
                    "ERROR",
                    "video_decoded_frame_invalid",
                    f"{path.name} has invalid decoded frames at {bad_frames[:8]}",
                )
            if changing_size_frames:
                report.add(
                    "ERROR",
                    "video_decoded_frame_size_changed",
                    f"{path.name} decoded size changed at frames {changing_size_frames[:8]}",
                )
            if expected_size_frames:
                report.add(
                    "ERROR",
                    "video_decoded_frame_size_mismatch",
                    (
                        f"{path.name} decoded frames {expected_size_frames[:8]} do not match "
                        f"metadata width/height/channels={expected_dimensions}"
                    ),
                )
            if expected_frame_count is not None and decoded_count < expected_frame_count:
                report.add(
                    "ERROR",
                    "video_frame_unreadable",
                    (
                        f"{path.name} stopped decoding after {decoded_count} frames; "
                        f"expected {expected_frame_count}"
                    ),
                )

        return VideoInfo(
            path=str(path),
            frame_count=decoded_count,
            fps=fps,
            width=width,
            height=height,
        )
    finally:
        cap.release()


def metadata_declares_preview(metadata: Dict[str, Any]) -> bool:
    storage = metadata.get("image_storage")
    if not isinstance(storage, dict):
        return False
    preview = storage.get("preview")
    return isinstance(preview, dict) and bool(preview.get("filename"))


def validate_timestamps(
    frames: List[Dict[str, Any]],
    metadata: Dict[str, Any],
    report: EpisodeReport,
    args: argparse.Namespace,
) -> np.ndarray:
    timestamps = collect_scalar(frames, "timestamp", report, required=True)
    if timestamps.size != len(frames):
        return np.asarray([], dtype=float)
    if not np.all(np.isfinite(timestamps)):
        report.add("ERROR", "timestamp_nonfinite", "timestamp contains NaN or Inf")
        return timestamps
    if len(timestamps) <= 1:
        report.add("WARN", "timestamp_short", "not enough timestamps to estimate cadence")
        return timestamps

    diffs = np.diff(timestamps)
    non_positive = np.where(diffs <= 0.0)[0]
    if non_positive.size:
        report.add("ERROR", "timestamp_not_monotonic", f"non-positive timestamp diffs at frames {non_positive[:8].tolist()}")
        return timestamps

    duration = float(timestamps[-1] - timestamps[0])
    report.duration_sec = duration
    report.average_fps = float((len(timestamps) - 1) / duration) if duration > 0.0 else 0.0
    target_fps = safe_float(metadata.get("video_fps")) or float(args.expected_fps)
    if target_fps > 0.0 and report.average_fps > 0.0:
        rel_error = abs(report.average_fps - target_fps) / target_fps
        if rel_error > args.max_fps_rel_error:
            report.add(
                "ERROR",
                "timestamp_fps_mismatch",
                f"timestamp avg fps={report.average_fps:.3f}, target={target_fps:.3f}, rel_error={rel_error:.3f}",
            )
        elif rel_error > args.max_fps_rel_error * 0.5:
            report.add(
                "WARN",
                "timestamp_fps_drift",
                f"timestamp avg fps={report.average_fps:.3f}, target={target_fps:.3f}, rel_error={rel_error:.3f}",
            )
    effective_warn_gap = args.warn_timestamp_gap
    effective_max_gap = args.max_timestamp_gap
    effective_p95_gap = args.max_timestamp_p95_gap
    if target_fps > 0.0:
        effective_warn_gap = max(effective_warn_gap, 1.5 / target_fps)
        effective_max_gap = max(effective_max_gap, 3.0 / target_fps)
        effective_p95_gap = max(effective_p95_gap, 1.35 / target_fps)

    large_gaps = np.where(diffs > effective_max_gap)[0]
    if large_gaps.size:
        max_gap = float(np.max(diffs))
        report.add(
            "ERROR",
            "timestamp_large_gap",
            f"{large_gaps.size} gaps exceed {effective_max_gap:.3f}s; max_gap={max_gap:.3f}s",
        )
    warn_gaps = np.where(diffs > effective_warn_gap)[0]
    if warn_gaps.size and not large_gaps.size:
        report.add(
            "WARN",
            "timestamp_gap_warn",
            f"{warn_gaps.size} gaps exceed {effective_warn_gap:.3f}s; max_gap={float(np.max(diffs)):.3f}s",
        )
    if diffs.size:
        p95_gap = float(np.percentile(diffs, 95))
        if p95_gap > effective_p95_gap:
            report.add(
                "ERROR",
                "timestamp_p95_gap_high",
                f"p95 timestamp gap={p95_gap:.3f}s, allowed={effective_p95_gap:.3f}s",
            )
    return timestamps


def timestamp_duration(frames: List[Dict[str, Any]]) -> float:
    if len(frames) <= 1:
        return 0.0
    try:
        first = float(frames[0]["timestamp"])
        last = float(frames[-1]["timestamp"])
    except Exception:
        return 0.0
    if math.isfinite(first) and math.isfinite(last) and last >= first:
        return last - first
    return 0.0


def validate_robot_timing(frames: List[Dict[str, Any]], report: EpisodeReport, args: argparse.Namespace) -> None:
    if "robot_state_valid" in frames[0]:
        values = np.asarray([bool(frame.get("robot_state_valid", False)) for frame in frames], dtype=bool)
        invalid_ratio = 1.0 - float(np.mean(values))
        if invalid_ratio > args.max_invalid_robot_ratio:
            report.add(
                "ERROR",
                "robot_state_invalid",
                f"invalid robot_state_valid ratio={invalid_ratio:.4f}, allowed={args.max_invalid_robot_ratio:.4f}",
            )
    check_optional_scalar_limit(
        frames,
        "robot_state_age_ms",
        args.warn_robot_state_age_p95_ms,
        args.max_robot_state_age_ms,
        "robot_state_age_high",
        report,
    )
    check_optional_scalar_limit(
        frames,
        "robot_read_duration_ms",
        args.warn_robot_read_duration_p95_ms,
        args.max_robot_read_duration_ms,
        "robot_read_duration_high",
        report,
    )


def check_optional_scalar_limit(
    frames: List[Dict[str, Any]],
    key: str,
    warn_p95_limit: float,
    limit: float,
    code: str,
    report: EpisodeReport,
) -> None:
    if key not in frames[0]:
        return
    values = collect_scalar(frames, key, report, required=False)
    if values.size == 0:
        return
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        report.add("ERROR", f"{code}_nonfinite", f"{key} has no finite values")
        return
    max_value = float(np.max(finite))
    if max_value > limit:
        report.add("ERROR", code, f"{key} max={max_value:.3f}, allowed={limit:.3f}")
    p95_value = float(np.percentile(finite, 95))
    if p95_value > warn_p95_limit:
        report.add("WARN", f"{code}_p95", f"{key} p95={p95_value:.3f}, warning={warn_p95_limit:.3f}")


def validate_actions(
    frames: List[Dict[str, Any]],
    timestamps: np.ndarray,
    report: EpisodeReport,
    args: argparse.Namespace,
) -> None:
    first = frames[0]
    schema = str(first.get("schema_version") or report.schema_version or "")
    is_dual = schema.startswith("franka_dual") or ("left_joint" in first and "right_joint" in first)
    if is_dual:
        for prefix in ("left", "right"):
            validate_arm_fields(frames, timestamps, prefix, report, args)
    else:
        validate_arm_fields(frames, timestamps, "", report, args)


def validate_arm_fields(
    frames: List[Dict[str, Any]],
    timestamps: np.ndarray,
    prefix: str,
    report: EpisodeReport,
    args: argparse.Namespace,
) -> None:
    label = prefix or "single"
    joint_key = f"{prefix}_joint" if prefix else "joint"
    pose_key = f"{prefix}_pose" if prefix else "pose"
    joint = collect_vector(frames, joint_key, 7, report, required=True)
    if joint.size:
        validate_joint_series(joint, timestamps, label, report, args)
    pose = collect_vector(frames, pose_key, 6, report, required=False)
    if pose.size and not np.all(np.isfinite(pose)):
        report.add("ERROR", f"{label}_pose_nonfinite", f"{pose_key} contains NaN or Inf")

    for suffix in ("gripper_width", "gripper_target_width"):
        key = f"{prefix}_{suffix}" if prefix else suffix
        if key in frames[0]:
            values = collect_scalar(frames, key, report, required=False)
            validate_gripper_width(values, key, label, report, args)
        else:
            report.add("WARN", f"{label}_{suffix}_missing", f"{key} is missing")


def collect_vector(
    frames: List[Dict[str, Any]],
    key: str,
    width: int,
    report: EpisodeReport,
    required: bool,
) -> np.ndarray:
    rows = []
    missing = 0
    bad_shape = 0
    for frame in frames:
        if key not in frame:
            missing += 1
            continue
        arr = np.asarray(frame[key], dtype=float)
        if arr.shape != (width,):
            bad_shape += 1
            continue
        rows.append(arr)
    if missing:
        level = "ERROR" if required else "WARN"
        report.add(level, f"{key}_missing", f"{key} missing in {missing}/{len(frames)} frames")
    if bad_shape:
        report.add("ERROR", f"{key}_bad_shape", f"{key} has wrong shape in {bad_shape}/{len(frames)} frames")
    if len(rows) != len(frames):
        return np.asarray([], dtype=float)
    return np.vstack(rows)


def collect_scalar(
    frames: List[Dict[str, Any]],
    key: str,
    report: EpisodeReport,
    required: bool,
) -> np.ndarray:
    values = []
    missing = 0
    bad = 0
    for frame in frames:
        if key not in frame:
            missing += 1
            continue
        try:
            values.append(float(frame[key]))
        except (TypeError, ValueError):
            bad += 1
    if missing:
        level = "ERROR" if required else "WARN"
        report.add(level, f"{key}_missing", f"{key} missing in {missing}/{len(frames)} frames")
    if bad:
        report.add("ERROR", f"{key}_bad_value", f"{key} is not numeric in {bad}/{len(frames)} frames")
    if missing or bad:
        return np.asarray([], dtype=float)
    return np.asarray(values, dtype=float)


def validate_joint_series(
    joint: np.ndarray,
    timestamps: np.ndarray,
    label: str,
    report: EpisodeReport,
    args: argparse.Namespace,
) -> None:
    if not np.all(np.isfinite(joint)):
        report.add("ERROR", f"{label}_joint_nonfinite", "joint contains NaN or Inf")
        return
    max_abs = float(np.max(np.abs(joint)))
    if max_abs > args.max_abs_joint:
        report.add("ERROR", f"{label}_joint_abs_high", f"max abs joint={max_abs:.3f} rad, allowed={args.max_abs_joint:.3f}")
    if len(joint) <= 1:
        return
    deltas = np.abs(np.diff(joint, axis=0))
    max_delta = float(np.max(deltas))
    if max_delta > args.max_joint_delta:
        frame_idx, joint_idx = np.unravel_index(int(np.argmax(deltas)), deltas.shape)
        report.add(
            "ERROR",
            f"{label}_joint_jump",
            (
                f"max joint delta={max_delta:.3f} rad at frame {frame_idx}->{frame_idx + 1}, "
                f"joint {joint_idx}, allowed={args.max_joint_delta:.3f}"
            ),
        )
    elif max_delta > args.warn_joint_delta:
        frame_idx, joint_idx = np.unravel_index(int(np.argmax(deltas)), deltas.shape)
        report.add(
            "WARN",
            f"{label}_joint_jump_warn",
            (
                f"max joint delta={max_delta:.3f} rad at frame {frame_idx}->{frame_idx + 1}, "
                f"joint {joint_idx}, warning={args.warn_joint_delta:.3f}"
            ),
        )
    if timestamps.size == len(joint) and len(joint) > 1:
        dt = np.diff(timestamps)
        valid_dt = dt > 0.0
        if np.any(valid_dt):
            velocity = deltas[valid_dt] / dt[valid_dt, None]
            max_velocity = float(np.max(velocity))
            if max_velocity > args.max_joint_velocity:
                frame_idx, joint_idx = np.unravel_index(int(np.argmax(velocity)), velocity.shape)
                valid_frames = np.where(valid_dt)[0]
                source_frame = int(valid_frames[frame_idx])
                report.add(
                    "ERROR",
                    f"{label}_joint_velocity_high",
                    (
                        f"max joint velocity={max_velocity:.3f} rad/s at frame "
                        f"{source_frame}->{source_frame + 1}, joint {joint_idx}, "
                        f"allowed={args.max_joint_velocity:.3f}"
                    ),
                )


def validate_gripper_width(
    values: np.ndarray,
    key: str,
    label: str,
    report: EpisodeReport,
    args: argparse.Namespace,
) -> None:
    if values.size == 0:
        return
    if not np.all(np.isfinite(values)):
        report.add("ERROR", f"{label}_{key}_nonfinite", f"{key} contains NaN or Inf")
        return
    below = np.where(values < args.min_gripper_width)[0]
    above = np.where(values > args.max_gripper_width)[0]
    if below.size or above.size:
        report.add(
            "ERROR",
            f"{label}_{key}_out_of_range",
            (
                f"{key} range=[{float(np.min(values)):.6f}, {float(np.max(values)):.6f}], "
                f"allowed=[{args.min_gripper_width:.6f}, {args.max_gripper_width:.6f}]"
            ),
        )


def safe_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def print_episode_report(
    report: EpisodeReport,
    task_root: Path,
    index: int,
    total: int,
    verbose: bool,
) -> None:
    try:
        rel = Path(report.path).relative_to(task_root)
    except ValueError:
        rel = Path(report.path)
    details = [
        f"frames={report.frame_count}",
        f"cams={len(report.camera_names)}",
    ]
    if report.average_fps:
        details.append(f"avg_fps={report.average_fps:.2f}")
    if report.schema_version:
        details.append(f"schema={report.schema_version}")
    print(f"[{report.status}] {index}/{total} {rel} ({', '.join(details)})")
    should_print = verbose or report.status != "PASS"
    if should_print:
        for issue in report.issues[:20]:
            print(f"  {issue.level}: {issue.code}: {issue.message}")
        if len(report.issues) > 20:
            print(f"  ... {len(report.issues) - 20} more issues")


def summarize_reports(reports: List[EpisodeReport]) -> Dict[str, int]:
    return {
        "episodes": len(reports),
        "pass": sum(1 for report in reports if report.status == "PASS"),
        "warn": sum(1 for report in reports if report.status == "WARN"),
        "fail": sum(1 for report in reports if report.status == "FAIL"),
        "errors": sum(report.error_count for report in reports),
        "warnings": sum(report.warning_count for report in reports),
    }


def print_summary(summary: Dict[str, int], strict: bool) -> None:
    print(
        "Summary: "
        f"episodes={summary['episodes']}, pass={summary['pass']}, warn={summary['warn']}, "
        f"fail={summary['fail']}, errors={summary['errors']}, warnings={summary['warnings']}"
    )
    if summary["fail"] > 0:
        print("Result: FAIL")
    elif strict and summary["warn"] > 0:
        print("Result: FAIL (strict warnings)")
    else:
        print("Result: PASS")


def write_json_report(
    path: Path,
    task_root: Path,
    args: argparse.Namespace,
    reports: List[EpisodeReport],
    summary: Dict[str, int],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "task_root": str(task_root),
        "args": vars(args),
        "summary": summary,
        "reports": [episode_report_to_json(report) for report in reports],
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def episode_report_to_json(report: EpisodeReport) -> Dict[str, Any]:
    data = asdict(report)
    data["issues"] = [asdict(issue) for issue in report.issues]
    data["videos"] = {name: asdict(info) for name, info in report.videos.items()}
    return data


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
    except Exception:
        traceback.print_exc()
        raise SystemExit(2)
