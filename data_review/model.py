from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import threading
from typing import Any

import numpy as np

from franka_replay.replay_fr3 import load_episode, load_episode_metadata, resolve_episode_file
from validate.validate_task import EpisodeReport, parse_args as parse_validator_args, validate_episode


JOINT_NAMES = tuple(f"J{index}" for index in range(1, 8))
JOINT_LIMITS_LOW = np.asarray(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    dtype=np.float64,
)
JOINT_LIMITS_HIGH = np.asarray(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    dtype=np.float64,
)
WARN_JOINT_DELTA = 0.10
FAIL_JOINT_DELTA = 0.20
MAX_JOINT_VELOCITY = 6.0
MAX_SAFE_LEGACY_COMPRESSED_BYTES = 256 * 1024 * 1024
MAX_SAFE_LEGACY_UNCOMPRESSED_BYTES = 1024 * 1024 * 1024


@dataclass(frozen=True)
class ReviewEvent:
    severity: str
    kind: str
    arm: str
    frame_index: int
    joint_index: int
    value: float
    threshold: float
    message: str


@dataclass(frozen=True)
class ArmSeries:
    name: str
    joints: np.ndarray
    poses: np.ndarray
    gripper_width: np.ndarray
    gripper_target_width: np.ndarray


@dataclass(frozen=True)
class EpisodeReview:
    pkl_path: Path
    metadata: dict[str, Any]
    frames: tuple[dict[str, Any], ...]
    timestamps: np.ndarray
    timeline: np.ndarray
    arms: dict[str, ArmSeries]
    camera_names: tuple[str, ...]
    embedded_camera_fields: dict[str, str]
    video_paths: dict[str, Path]
    keyframes: tuple[int, ...]
    events: tuple[ReviewEvent, ...]
    validator_report: EpisodeReport

    @property
    def frame_count(self) -> int:
        return len(self.frames)

    @property
    def duration(self) -> float:
        return float(self.timeline[-1]) if self.timeline.size else 0.0

    @property
    def fps(self) -> float:
        configured = self.metadata.get("video_fps")
        if isinstance(configured, (int, float)) and float(configured) > 0:
            return float(configured)
        if self.timeline.size > 1:
            median = float(np.median(np.diff(self.timeline)))
            if median > 0:
                return 1.0 / median
        return 30.0


def load_episode_review(
    path_like: str | Path,
    cancel_event: threading.Event | None = None,
) -> EpisodeReview:
    pkl_path = resolve_episode_file(str(path_like))
    metadata = load_episode_metadata(pkl_path)
    compressed_size = pkl_path.stat().st_size
    allow_large = os.environ.get("FRANKA_REVIEW_ALLOW_LARGE_V2", "") == "1"
    if compressed_size > MAX_SAFE_LEGACY_COMPRESSED_BYTES and not allow_large:
        raise MemoryError(
            "PKL 压缩文件过大，完整加载可能耗尽内存。"
            "确认机器内存充足后可设置 FRANKA_REVIEW_ALLOW_LARGE_V2=1。"
        )
    uncompressed_size = _gzip_uncompressed_size(
        pkl_path,
        cancel_event=cancel_event,
        stop_after=MAX_SAFE_LEGACY_UNCOMPRESSED_BYTES,
    )
    if (
        uncompressed_size > MAX_SAFE_LEGACY_UNCOMPRESSED_BYTES
        and not allow_large
    ):
        raise MemoryError(
            "PKL 解压数据超过 1 GiB，完整加载可能耗尽内存。"
            "请先转换为 v3，或确认机器内存充足后设置 FRANKA_REVIEW_ALLOW_LARGE_V2=1。"
        )
    validator_args = parse_validator_args([pkl_path.parent.name, "--min-cameras", "3"])
    validator_report = validate_episode(
        pkl_path.parent,
        validator_args,
        cancel_event=cancel_event,
    )
    if cancel_event is not None and cancel_event.is_set():
        raise RuntimeError("episode review load cancelled")
    payload, source_frames = load_episode(pkl_path)
    if cancel_event is not None and cancel_event.is_set():
        raise RuntimeError("episode review load cancelled")
    video_camera_names = {
        name
        for name in metadata.get("camera_names", [])
        if isinstance(name, str) and (pkl_path.parent / f"{name}.mp4").is_file()
    }
    frames = tuple(
        {
            key: value
            for key, value in frame.items()
            if not (
                key.endswith("_image")
                and key[: -len("_image")] in video_camera_names
            )
        }
        for frame in source_frames
    )
    timestamps = np.asarray([float(frame["timestamp"]) for frame in frames], dtype=np.float64)
    if not np.all(np.isfinite(timestamps)) or np.any(np.diff(timestamps) < 0):
        raise ValueError("Episode timestamps must be finite and monotonically nondecreasing")
    timeline = timestamps - timestamps[0]
    arms = _extract_arms(frames, metadata)
    camera_names, embedded_fields, video_paths = _camera_sources(pkl_path.parent, frames, metadata)
    keyframes = _load_keyframes(pkl_path.parent, payload, len(frames))
    events = tuple(event for arm in arms.values() for event in _review_arm(arm, timeline))
    return EpisodeReview(
        pkl_path=pkl_path,
        metadata=metadata,
        frames=frames,
        timestamps=timestamps,
        timeline=timeline,
        arms=arms,
        camera_names=camera_names,
        embedded_camera_fields=embedded_fields,
        video_paths=video_paths,
        keyframes=keyframes,
        events=events,
        validator_report=validator_report,
    )


def _gzip_uncompressed_size(
    path: Path,
    *,
    cancel_event: threading.Event | None = None,
    stop_after: int | None = None,
) -> int:
    import gzip

    total = 0
    try:
        with gzip.open(path, "rb") as handle:
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    raise RuntimeError("episode review load cancelled")
                chunk = handle.read(8 * 1024 * 1024)
                if not chunk:
                    return total
                total += len(chunk)
                if stop_after is not None and total > stop_after:
                    return total
    except OSError:
        return 0


def _extract_arms(frames: tuple[dict[str, Any], ...], metadata: dict[str, Any]) -> dict[str, ArmSeries]:
    first = frames[0]
    prefixes = ("left", "right") if "left_joint" in first and "right_joint" in first else ("",)
    arms: dict[str, ArmSeries] = {}
    for prefix in prefixes:
        key_prefix = f"{prefix}_" if prefix else ""
        name = prefix or str(metadata.get("arm_side") or "left")
        joints = _required_matrix(frames, f"{key_prefix}joint", 7)
        poses = _optional_matrix(frames, f"{key_prefix}pose", 6)
        gripper = _optional_scalar(frames, f"{key_prefix}gripper_width")
        target = _optional_scalar(frames, f"{key_prefix}gripper_target_width")
        arms[name] = ArmSeries(name, joints, poses, gripper, target)
    return arms


def _required_matrix(frames: tuple[dict[str, Any], ...], key: str, width: int) -> np.ndarray:
    values = np.asarray([frame.get(key) for frame in frames], dtype=np.float64)
    if values.shape != (len(frames), width) or not np.all(np.isfinite(values)):
        raise ValueError(f"{key} must be a finite {len(frames)}x{width} trajectory")
    return values


def _optional_matrix(frames: tuple[dict[str, Any], ...], key: str, width: int) -> np.ndarray:
    if key not in frames[0]:
        return np.empty((0, width), dtype=np.float64)
    return _required_matrix(frames, key, width)


def _optional_scalar(frames: tuple[dict[str, Any], ...], key: str) -> np.ndarray:
    if key not in frames[0]:
        return np.empty(0, dtype=np.float64)
    values = np.asarray([frame.get(key) for frame in frames], dtype=np.float64)
    return values if values.shape == (len(frames),) else np.empty(0, dtype=np.float64)


def _camera_sources(
    episode_dir: Path,
    frames: tuple[dict[str, Any], ...],
    metadata: dict[str, Any],
) -> tuple[tuple[str, ...], dict[str, str], dict[str, Path]]:
    embedded = {
        key[: -len("_image")]: key
        for key in frames[0]
        if key.endswith("_image") and isinstance(frames[0].get(key), np.ndarray)
    }
    video_paths: dict[str, Path] = {}
    configured_names = metadata.get("camera_names")
    expected_names = tuple(
        name
        for name in configured_names if isinstance(name, str) and Path(name).name == name
    ) if isinstance(configured_names, list) else ()
    for name in expected_names:
        path = episode_dir / f"{name}.mp4"
        if path.is_file():
            video_paths[name] = path
    if expected_names:
        missing = [name for name in expected_names if name not in video_paths and name not in embedded]
        if missing:
            raise ValueError(f"Episode is missing declared camera streams: {', '.join(missing)}")
        camera_names = expected_names
    else:
        camera_names = tuple(dict.fromkeys([*video_paths, *embedded]))
    if not camera_names:
        raise ValueError(f"Episode has no readable camera streams: {episode_dir}")
    return camera_names, embedded, video_paths


def _load_keyframes(episode_dir: Path, payload: dict[str, Any], frame_count: int) -> tuple[int, ...]:
    values = payload.get("keyframes", [])
    path = episode_dir / "keyframes.json"
    if path.is_file():
        try:
            values = json.loads(path.read_text(encoding="utf-8")).get("keyframes", values)
        except (OSError, json.JSONDecodeError, AttributeError):
            pass
    valid: set[int] = set()
    if not isinstance(values, (list, tuple)):
        return ()
    for value in values:
        try:
            index = int(value)
        except (TypeError, ValueError):
            continue
        if 0 <= index < frame_count:
            valid.add(index)
    return tuple(sorted(valid))


def _review_arm(arm: ArmSeries, timeline: np.ndarray) -> list[ReviewEvent]:
    events: list[ReviewEvent] = []
    low_hits = np.argwhere(arm.joints < JOINT_LIMITS_LOW[None, :])
    high_hits = np.argwhere(arm.joints > JOINT_LIMITS_HIGH[None, :])
    for frame_index, joint_index in np.vstack([low_hits, high_hits]):
        value = float(arm.joints[frame_index, joint_index])
        threshold = (
            float(JOINT_LIMITS_LOW[joint_index])
            if value < JOINT_LIMITS_LOW[joint_index]
            else float(JOINT_LIMITS_HIGH[joint_index])
        )
        events.append(
            ReviewEvent("fail", "position_limit", arm.name, int(frame_index), int(joint_index), value, threshold, f"{arm.name} J{joint_index + 1} 超出物理限位")
        )
    if len(arm.joints) > 1:
        deltas = np.abs(np.diff(arm.joints, axis=0))
        for frame_index, joint_index in np.argwhere(deltas > WARN_JOINT_DELTA):
            value = float(deltas[frame_index, joint_index])
            severity = "fail" if value > FAIL_JOINT_DELTA else "warn"
            threshold = FAIL_JOINT_DELTA if severity == "fail" else WARN_JOINT_DELTA
            events.append(
                ReviewEvent(severity, "joint_delta", arm.name, int(frame_index + 1), int(joint_index), value, threshold, f"{arm.name} J{joint_index + 1} 单帧变化 {value:.3f} rad")
            )
        dt = np.diff(timeline)
        valid = dt > 0
        if np.any(valid):
            velocity = deltas[valid] / dt[valid, None]
            source_frames = np.flatnonzero(valid)
            for row, joint_index in np.argwhere(velocity > MAX_JOINT_VELOCITY):
                frame_index = int(source_frames[row] + 1)
                value = float(velocity[row, joint_index])
                events.append(
                    ReviewEvent("fail", "joint_velocity", arm.name, frame_index, int(joint_index), value, MAX_JOINT_VELOCITY, f"{arm.name} J{joint_index + 1} 速度 {value:.2f} rad/s")
                )
    return sorted(events, key=lambda event: (event.frame_index, event.joint_index, event.kind))
