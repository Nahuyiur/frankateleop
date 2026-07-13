from __future__ import annotations

import gzip
import json
import pickle
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from franka_capture.gripper_fields import (
    GRIPPER_01CLOSEDNESS_FIELD,
    frame_gripper_01closedness,
    frame_gripper_closedness,
    frame_gripper_target_width,
    frame_gripper_width,
)
from franka_lerobot.converter import ConversionError, load_episode as load_capture_episode


DEFAULT_SOURCE_FPS = 30
DEFAULT_TARGET_FPS = 10
DEFAULT_CAMERA = "right"
OUTPUT_SCHEMA_VERSION = "franka_single_v2"
QUALITY_DIRS = ("High_Quality", "Low_Quality", "Failure")


class DownsampleError(RuntimeError):
    """Raised when a capture episode cannot be downsampled safely."""


@dataclass
class EpisodeResult:
    source_index: int | None
    output_index: int
    quality: str | None
    source_schema_version: str
    source_frames: int
    output_frames: int
    camera_names: list[str]
    output_dir: Path


def downsample_task(
    input_task_dir: str | Path,
    output_task_dir: str | Path,
    camera: str = DEFAULT_CAMERA,
    source_fps: int = DEFAULT_SOURCE_FPS,
    target_fps: int = DEFAULT_TARGET_FPS,
    overwrite: bool = False,
) -> tuple[Path, list[EpisodeResult]]:
    input_task_dir = Path(input_task_dir).expanduser().resolve()
    output_task_dir = Path(output_task_dir).expanduser().resolve()

    if not input_task_dir.is_dir():
        raise DownsampleError(f"Input task directory does not exist: {input_task_dir}")

    stride = _compute_stride(source_fps, target_fps)
    episode_sources = _discover_task_episode_dirs(input_task_dir)
    if not episode_sources:
        raise DownsampleError(f"No episode directories found in: {input_task_dir}")

    valid_sources = [source for source in episode_sources if _has_episode_pkl(source[1])]
    skipped_without_pkl = [source[1] for source in episode_sources if not _has_episode_pkl(source[1])]
    if not valid_sources:
        raise DownsampleError(
            f"No valid episode directories with .pkl.gz found in: {input_task_dir}"
        )

    _prepare_output_root(output_task_dir, overwrite)
    for episode_dir in skipped_without_pkl:
        print(f"Warning: skipped episode directory without .pkl.gz: {episode_dir}")

    results: list[EpisodeResult] = []
    next_index_by_quality: dict[str | None, int] = {}
    for quality, episode_dir in valid_sources:
        output_index = next_index_by_quality.get(quality, 0)
        next_index_by_quality[quality] = output_index + 1
        output_parent = output_task_dir / quality if quality else output_task_dir
        result = downsample_episode(
            input_episode=episode_dir,
            output_episode_dir=output_parent / str(output_index),
            output_index=output_index,
            camera=camera,
            stride=stride,
            target_fps=target_fps,
            quality=quality,
            task_name=output_task_dir.name,
        )
        results.append(result)
        print(
            f"Downsampled {episode_dir} -> {result.output_dir} "
            f"({result.source_frames} -> {result.output_frames} frames)"
        )

    _write_task_metadata(
        output_task_dir=output_task_dir,
        input_task_dir=input_task_dir,
        camera=camera,
        source_fps=source_fps,
        target_fps=target_fps,
        stride=stride,
        results=results,
        skipped_without_pkl=skipped_without_pkl,
    )
    print(f"10Hz capture-format dataset written to: {output_task_dir}")
    print(f"Total episodes: {len(results)}, total frames: {sum(result.output_frames for result in results)}")
    return output_task_dir, results


def downsample_episode(
    input_episode: str | Path,
    output_episode_dir: str | Path,
    output_index: int,
    camera: str,
    stride: int,
    target_fps: int,
    quality: str | None = None,
    task_name: str | None = None,
) -> EpisodeResult:
    input_episode = Path(input_episode).expanduser().resolve()
    output_episode_dir = Path(output_episode_dir).expanduser().resolve()
    pkl_path = _find_episode_pkl(input_episode)

    with gzip.open(pkl_path, "rb") as f:
        obj = pickle.load(f)

    if not isinstance(obj, dict) or "data" not in obj:
        raise DownsampleError(f"{pkl_path} must contain a dict with key 'data'")
    raw_frames = obj["data"]
    if not isinstance(raw_frames, list) or not raw_frames:
        raise DownsampleError(f"{pkl_path} contains no frames")

    source_metadata = _load_episode_metadata(input_episode)
    source_schema_version = _detect_source_schema(obj, raw_frames, source_metadata)
    _validate_supported_source_format(
        raw_frames,
        source_schema_version=source_schema_version,
        pkl_path=pkl_path,
    )

    try:
        loaded_episode = load_capture_episode(pkl_path)
    except ConversionError as exc:
        raise DownsampleError(str(exc)) from exc
    frames = loaded_episode.frames

    effective_quality = quality or _infer_quality(input_episode, source_metadata)
    camera_fields = _resolve_camera_fields(frames[0], camera, pkl_path)
    camera_names = [_camera_field_to_name(field) for field in camera_fields]
    sampled_indices = list(range(0, len(frames), stride))
    sampled_frames = [
        _filter_frame(frames[index], frame_index=index, pkl_path=pkl_path, camera_fields=camera_fields)
        for index in sampled_indices
    ]
    keyframes = _remap_keyframes(
        obj.get("keyframes", _load_keyframes_json(input_episode)),
        stride=stride,
        output_length=len(sampled_frames),
    )

    output_episode_dir.mkdir(parents=True, exist_ok=True)
    output_pkl = output_episode_dir / f"{output_index}.pkl.gz"
    with gzip.open(output_pkl, "wb", compresslevel=1) as f:
        pickle.dump(
            {
                "data": sampled_frames,
                "keyframes": keyframes,
                "schema_version": OUTPUT_SCHEMA_VERSION,
            },
            f,
            protocol=pickle.HIGHEST_PROTOCOL,
        )

    keyframes_path = output_episode_dir / "keyframes.json"
    keyframes_path.write_text(
        json.dumps({"keyframes": keyframes}, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    for camera_field in camera_fields:
        camera_name = _camera_field_to_name(camera_field)
        _write_mp4(output_episode_dir / f"{camera_name}.mp4", sampled_frames, camera_field, target_fps)
    _write_episode_metadata(
        output_episode_dir=output_episode_dir,
        input_episode=input_episode,
        source_metadata=source_metadata,
        source_schema_version=source_schema_version,
        task_name=task_name or output_episode_dir.parent.name,
        output_index=output_index,
        quality=effective_quality,
        camera_names=camera_names,
        target_fps=target_fps,
        stride=stride,
        source_frame_count=len(frames),
        output_frame_count=len(sampled_frames),
        keyframes=keyframes,
    )

    return EpisodeResult(
        source_index=_parse_source_episode_index(pkl_path),
        output_index=output_index,
        quality=effective_quality,
        source_schema_version=source_schema_version,
        source_frames=len(frames),
        output_frames=len(sampled_frames),
        camera_names=camera_names,
        output_dir=output_episode_dir,
    )


def _filter_frame(frame: dict[str, Any], frame_index: int, pkl_path: Path, camera_fields: list[str]) -> dict[str, Any]:
    required = {"pose", "joint", "timestamp", *camera_fields}
    missing = required - set(frame)
    if missing:
        raise DownsampleError(f"Frame {frame_index} in {pkl_path} is missing keys: {sorted(missing)}")

    for camera_field in camera_fields:
        image = frame[camera_field]
        if not hasattr(image, "shape") or len(image.shape) != 3 or image.shape[2] != 3:
            raise DownsampleError(f"Frame {frame_index} in {pkl_path} has invalid {camera_field}")
        if image.dtype != np.uint8:
            raise DownsampleError(f"Frame {frame_index} in {pkl_path} {camera_field} must be uint8")

    pose = np.asarray(frame["pose"], dtype=float)
    joint = np.asarray(frame["joint"], dtype=float)
    if pose.shape != (6,):
        raise DownsampleError(f"Frame {frame_index} in {pkl_path} has invalid pose shape: {pose.shape}")
    if joint.shape != (7,):
        raise DownsampleError(f"Frame {frame_index} in {pkl_path} has invalid joint shape: {joint.shape}")
    if not np.all(np.isfinite(pose)) or not np.all(np.isfinite(joint)):
        raise DownsampleError(f"Frame {frame_index} in {pkl_path} has NaN or Inf pose/joint")

    gripper_closedness = frame_gripper_closedness(frame)
    gripper_01closedness = frame_gripper_01closedness(frame)
    gripper_width = frame_gripper_width(frame)
    gripper_target_width = frame_gripper_target_width(frame)
    timestamp = float(frame["timestamp"])
    scalar_values = np.asarray(
        [
            gripper_closedness,
            gripper_01closedness,
            gripper_width,
            gripper_target_width,
            timestamp,
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(scalar_values)):
        raise DownsampleError(f"Frame {frame_index} in {pkl_path} has NaN or Inf scalar fields")

    filtered = {
        "schema_version": OUTPUT_SCHEMA_VERSION,
        "pose": pose.tolist(),
        "joint": joint.tolist(),
        "gripper_closedness": gripper_closedness,
        GRIPPER_01CLOSEDNESS_FIELD: gripper_01closedness,
        "gripper_width": gripper_width,
        "gripper_target_width": gripper_target_width,
        "timestamp": timestamp,
    }
    for camera_field in camera_fields:
        filtered[camera_field] = frame[camera_field].copy()
    return filtered


def _write_mp4(path: Path, frames: list[dict[str, Any]], camera_field: str, fps: int) -> None:
    try:
        import imageio.v2 as imageio

        with imageio.get_writer(
            path,
            fps=fps,
            codec="libx264",
            pixelformat="yuv420p",
            macro_block_size=1,
        ) as writer:
            for frame in frames:
                writer.append_data(_bgr_to_rgb(frame[camera_field]))
        return
    except Exception as exc:
        print(f"Warning: imageio/libx264 failed for {path}: {exc}. Falling back to OpenCV mp4v.")

    import cv2

    first = frames[0][camera_field]
    height, width = first.shape[:2]
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (width, height))
    if not writer.isOpened():
        raise DownsampleError(f"Failed to open video writer: {path}")
    try:
        for frame in frames:
            writer.write(frame[camera_field])
    finally:
        writer.release()


def _bgr_to_rgb(image: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(image[..., ::-1])


def _discover_task_episode_dirs(input_task_dir: Path) -> list[tuple[str | None, Path]]:
    sources: list[tuple[str | None, Path]] = []
    for quality in QUALITY_DIRS:
        quality_dir = input_task_dir / quality
        if not quality_dir.is_dir():
            continue
        episode_dirs = [
            path
            for path in quality_dir.iterdir()
            if path.is_dir() and not path.name.startswith((".", "#"))
        ]
        episode_dirs.sort(key=_episode_sort_key)
        sources.extend((quality, path) for path in episode_dirs)

    direct_quality = input_task_dir.name if input_task_dir.name in QUALITY_DIRS else None
    direct_episode_dirs = [
        path
        for path in input_task_dir.iterdir()
        if path.is_dir()
        and path.name not in QUALITY_DIRS
        and not path.name.startswith((".", "#"))
    ]
    direct_episode_dirs.sort(key=_episode_sort_key)
    sources.extend((direct_quality, path) for path in direct_episode_dirs)
    return sources


def _resolve_camera_fields(frame: dict[str, Any], camera: str, pkl_path: Path) -> list[str]:
    camera = camera.strip()
    if camera in {"all", "*", "both", "two_views"}:
        camera_fields = sorted([key for key in frame if key.endswith("_image")])
    else:
        camera_names = [name.strip() for name in camera.split(",") if name.strip()]
        camera_fields = [f"{name}_image" for name in camera_names]

    if not camera_fields:
        raise DownsampleError(f"No camera fields selected for {pkl_path}")

    missing = [field for field in camera_fields if field not in frame]
    if missing:
        available = sorted(key[: -len("_image")] for key in frame if key.endswith("_image"))
        raise DownsampleError(f"Requested camera fields {missing} are missing in {pkl_path}. Available: {available}")
    return camera_fields


def _remap_keyframes(keyframes: Any, stride: int, output_length: int) -> list[int]:
    values = []
    if isinstance(keyframes, (list, tuple)):
        values = keyframes
    elif isinstance(keyframes, np.ndarray):
        values = keyframes.tolist()

    remapped = {0}
    for keyframe in values:
        try:
            mapped = int(round(float(keyframe) / float(stride)))
        except (TypeError, ValueError):
            continue
        mapped = min(max(mapped, 0), output_length - 1)
        remapped.add(mapped)
    return sorted(remapped)


def _load_keyframes_json(input_episode: Path) -> list[int]:
    keyframes_path = input_episode / "keyframes.json"
    if not keyframes_path.exists():
        return [0]
    try:
        obj = json.loads(keyframes_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return [0]
    keyframes = obj.get("keyframes", [0])
    return keyframes if isinstance(keyframes, list) else [0]


def _load_episode_metadata(input_episode: Path) -> dict[str, Any]:
    episode_dir = input_episode.parent if input_episode.is_file() else input_episode
    metadata_path = episode_dir / "metadata.json"
    if not metadata_path.exists():
        return {}
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise DownsampleError(f"Failed to read {metadata_path}: {exc}") from exc
    if not isinstance(metadata, dict):
        raise DownsampleError(f"{metadata_path} must contain a JSON object")
    return metadata


def _detect_source_schema(
    payload: dict[str, Any],
    frames: list[Any],
    metadata: dict[str, Any],
) -> str:
    schemas = {
        str(value)
        for value in (
            metadata.get("schema_version"),
            payload.get("schema_version"),
            *(frame.get("schema_version") for frame in frames if isinstance(frame, dict)),
        )
        if value
    }
    if len(schemas) > 1:
        raise DownsampleError(f"Source episode mixes schema versions: {sorted(schemas)}")
    return next(iter(schemas), "legacy_single")


def _validate_supported_source_format(
    frames: list[Any],
    *,
    source_schema_version: str,
    pkl_path: Path,
) -> None:
    if not all(isinstance(frame, dict) for frame in frames):
        raise DownsampleError(f"{pkl_path} contains non-dict frames")
    first_frame = frames[0]
    is_dual = source_schema_version.startswith("franka_dual") or (
        "left_joint" in first_frame or "right_joint" in first_frame
    )
    if is_dual:
        raise DownsampleError(
            "Dual-arm downsample is not supported; this command only emits "
            f"{OUTPUT_SCHEMA_VERSION}"
        )
    if source_schema_version.endswith("_v3") and source_schema_version != "franka_single_v3":
        raise DownsampleError(f"Unsupported v3 source schema: {source_schema_version}")
    if source_schema_version != "franka_single_v3":
        return

    bad_indices = [
        index
        for index, frame in enumerate(frames)
        if isinstance(frame.get("frame_index"), bool)
        or not isinstance(frame.get("frame_index"), int)
        or frame.get("frame_index") != index
    ]
    if bad_indices:
        raise DownsampleError(
            f"{pkl_path} v3 frame_index must be exactly 0..N-1; bad frames: {bad_indices[:8]}"
        )
    embedded_fields = sorted(
        {key for frame in frames for key in frame if key.endswith("_image")}
    )
    if embedded_fields:
        raise DownsampleError(
            f"{pkl_path} mixes v3 with embedded images: {embedded_fields}"
        )


def _infer_quality(input_episode: Path, metadata: dict[str, Any]) -> str | None:
    metadata_quality = metadata.get("quality")
    if metadata_quality in QUALITY_DIRS:
        return str(metadata_quality)
    for part in reversed(input_episode.parts):
        if part in QUALITY_DIRS:
            return part
    return None


def _find_episode_pkl(input_episode: Path) -> Path:
    if input_episode.is_file():
        if input_episode.name.endswith(".pkl.gz"):
            return input_episode
        raise DownsampleError(f"Expected a .pkl.gz file, got: {input_episode}")
    if not input_episode.is_dir():
        raise DownsampleError(f"Episode path does not exist: {input_episode}")

    preferred = input_episode / f"{input_episode.name}.pkl.gz"
    if preferred.exists():
        return preferred
    candidates = sorted(input_episode.glob("*.pkl.gz"))
    if not candidates:
        raise DownsampleError(f"No .pkl.gz file found in episode directory: {input_episode}")
    if len(candidates) > 1:
        raise DownsampleError(f"Multiple .pkl.gz files found in {input_episode}: {candidates}")
    return candidates[0]


def _has_episode_pkl(input_episode: Path) -> bool:
    if not input_episode.is_dir():
        return False
    preferred = input_episode / f"{input_episode.name}.pkl.gz"
    return preferred.exists() or any(input_episode.glob("*.pkl.gz"))


def _compute_stride(source_fps: int, target_fps: int) -> int:
    if source_fps <= 0 or target_fps <= 0:
        raise DownsampleError(f"source_fps and target_fps must be positive, got {source_fps}/{target_fps}")
    if source_fps % target_fps != 0:
        raise DownsampleError(
            f"source_fps must be divisible by target_fps for fixed-stride downsampling, got {source_fps}/{target_fps}"
        )
    stride = source_fps // target_fps
    if stride < 1:
        raise DownsampleError(f"target_fps must be <= source_fps, got {source_fps}/{target_fps}")
    return stride


def _prepare_output_root(output_root: Path, overwrite: bool) -> None:
    if output_root.exists():
        if not overwrite:
            raise DownsampleError(f"Output directory already exists: {output_root}. Use --overwrite to replace it.")
        if output_root == Path(output_root.anchor):
            raise DownsampleError(f"Refusing to remove unsafe output directory: {output_root}")
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=True)


def _write_episode_metadata(
    *,
    output_episode_dir: Path,
    input_episode: Path,
    source_metadata: dict[str, Any],
    source_schema_version: str,
    task_name: str,
    output_index: int,
    quality: str | None,
    camera_names: list[str],
    target_fps: int,
    stride: int,
    source_frame_count: int,
    output_frame_count: int,
    keyframes: list[int],
) -> None:
    metadata: dict[str, Any] = {}
    preserved_fields = (
        "task_description",
        "text_instruction",
        "arm_side",
        "robot",
        "gripper_semantics",
        "gripper_closedness_field",
        "gripper_01closedness_field",
        "gripper_values",
        "gripper_closed_threshold",
        "gripper_width_field",
        "gripper_target_width_field",
    )
    for field_name in preserved_fields:
        if field_name in source_metadata:
            metadata[field_name] = source_metadata[field_name]

    source_cameras = source_metadata.get("cameras")
    if isinstance(source_cameras, dict):
        selected_cameras = {
            name: source_cameras[name]
            for name in camera_names
            if name in source_cameras
        }
        if selected_cameras:
            metadata["cameras"] = selected_cameras

    metadata.update(
        {
            "schema_version": OUTPUT_SCHEMA_VERSION,
            "source_schema_version": source_schema_version,
            "task": task_name,
            "index": output_index,
            "frame_count": output_frame_count,
            "camera_names": camera_names,
            "video_fps": int(target_fps),
            "keyframes": keyframes,
            "downsample": {
                "source_episode": str(input_episode),
                "source_frame_count": source_frame_count,
                "output_frame_count": output_frame_count,
                "stride": stride,
                "selection": "source_indices=0,stride,2*stride,...",
            },
        }
    )
    if quality is not None:
        metadata["quality"] = quality
    (output_episode_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _write_task_metadata(
    output_task_dir: Path,
    input_task_dir: Path,
    camera: str,
    source_fps: int,
    target_fps: int,
    stride: int,
    results: list[EpisodeResult],
    skipped_without_pkl: list[Path],
) -> None:
    metadata = {
        "format": "franka_capture_downsampled",
        "output_schema_version": OUTPUT_SCHEMA_VERSION,
        "source_task_dir": str(input_task_dir),
        "camera": camera,
        "camera_names": results[0].camera_names if results else [],
        "source_fps": int(source_fps),
        "target_fps": int(target_fps),
        "stride": int(stride),
        "total_episodes": len(results),
        "skipped_without_pkl": [str(path) for path in skipped_without_pkl],
        "total_source_frames": sum(result.source_frames for result in results),
        "total_output_frames": sum(result.output_frames for result in results),
        "episodes": [
            {
                "source_episode_index": result.source_index,
                "output_episode_index": result.output_index,
                "quality": result.quality,
                "source_schema_version": result.source_schema_version,
                "source_frames": result.source_frames,
                "output_frames": result.output_frames,
                "camera_names": result.camera_names,
                "path": str(result.output_dir.relative_to(output_task_dir)),
            }
            for result in results
        ],
    }
    (output_task_dir / "downsample_metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _parse_source_episode_index(pkl_path: Path) -> int | None:
    stem = pkl_path.name[: -len(".pkl.gz")] if pkl_path.name.endswith(".pkl.gz") else pkl_path.stem
    try:
        return int(stem)
    except ValueError:
        return None


def _camera_field_to_name(camera_field: str) -> str:
    return camera_field[: -len("_image")]


def _episode_sort_key(path: Path) -> tuple[int, int | str]:
    try:
        return (0, int(path.name))
    except ValueError:
        return (1, path.name)
