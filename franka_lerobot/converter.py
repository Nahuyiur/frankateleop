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
    GRIPPER_CLOSED_THRESHOLD,
    GRIPPER_SEMANTICS,
    frame_gripper_01closedness as _frame_gripper_01closedness,
    frame_gripper_closedness as _frame_gripper_closedness,
)


CODEBASE_VERSION = "v2.1"
DEFAULT_FPS = 10
DEFAULT_ROBOT_TYPE = "franka_fr3"
DEFAULT_CHUNKS_SIZE = 1000
DEFAULT_DATA_FILE_SIZE_IN_MB = 100
DEFAULT_VIDEO_FILE_SIZE_IN_MB = 200
GRIPPER_BINARY_THRESHOLD = GRIPPER_CLOSED_THRESHOLD
STATE_NAMES = [f"joint_{idx}" for idx in range(1, 8)] + ["gripper_closedness"]
POSE_NAMES = ["x", "y", "z", "rx", "ry", "rz"]
ACTION_NAMES = [f"ee_{name}" for name in POSE_NAMES] + STATE_NAMES
META_FEATURES = {
    "timestamp": {"dtype": "float32", "shape": (1,), "names": None},
    "frame_index": {"dtype": "int64", "shape": (1,), "names": None},
    "episode_index": {"dtype": "int64", "shape": (1,), "names": None},
    "index": {"dtype": "int64", "shape": (1,), "names": None},
    "task_index": {"dtype": "int64", "shape": (1,), "names": None},
}


class ConversionError(RuntimeError):
    """Raised when a capture episode cannot be converted safely."""


@dataclass
class EpisodeData:
    pkl_path: Path
    source_episode_index: int | None
    frames: list[dict[str, Any]]
    camera_fields: list[str]
    camera_shapes: dict[str, tuple[int, int, int]]


@dataclass
class EpisodeWriteResult:
    episode_index: int
    source_episode_index: int | None
    length: int
    stats: dict[str, dict[str, np.ndarray]]
    data_path: Path
    video_paths: dict[str, Path]


def check_runtime_dependencies() -> list[str]:
    missing = []
    for module_name in ("pyarrow", "pandas", "cv2", "imageio"):
        try:
            __import__(module_name)
        except ImportError:
            missing.append(module_name)
    return missing


def find_episode_pkl(path: str | Path) -> Path:
    path = Path(path).expanduser().resolve()
    if path.is_file():
        if path.name.endswith(".pkl.gz"):
            return path
        raise ConversionError(f"Expected a .pkl.gz file, got: {path}")

    if not path.is_dir():
        raise ConversionError(f"Episode path does not exist: {path}")

    preferred = path / f"{path.name}.pkl.gz"
    if preferred.exists():
        return preferred

    candidates = sorted(path.glob("*.pkl.gz"))
    if not candidates:
        raise FileNotFoundError(f"No .pkl.gz file found in episode directory: {path}")
    if len(candidates) > 1:
        raise ConversionError(f"Multiple .pkl.gz files found in {path}: {candidates}")
    return candidates[0]


def discover_task_episode_paths(task_dir: str | Path) -> tuple[list[Path], list[Path]]:
    task_dir = Path(task_dir).expanduser().resolve()
    if not task_dir.is_dir():
        raise ConversionError(f"Task directory does not exist: {task_dir}")

    episode_dirs = [path for path in task_dir.iterdir() if path.is_dir()]
    episode_dirs.sort(key=_episode_sort_key)

    valid: list[Path] = []
    skipped: list[Path] = []
    for episode_dir in episode_dirs:
        try:
            find_episode_pkl(episode_dir)
        except FileNotFoundError:
            skipped.append(episode_dir)
        else:
            valid.append(episode_dir)

    if not valid:
        raise ConversionError(f"No valid episode directories with .pkl.gz found in: {task_dir}")

    return valid, skipped


def load_episode(path: str | Path) -> EpisodeData:
    pkl_path = find_episode_pkl(path)
    with gzip.open(pkl_path, "rb") as f:
        obj = pickle.load(f)

    if not isinstance(obj, dict) or "data" not in obj:
        raise ConversionError(f"{pkl_path} must contain a dict with key 'data'")

    frames = obj["data"]
    if not isinstance(frames, list) or len(frames) == 0:
        raise ConversionError(f"{pkl_path} contains no frames")

    camera_fields = sorted([key for key in frames[0] if key.endswith("_image")])
    if not camera_fields:
        raise ConversionError(f"{pkl_path} contains no '*_image' camera fields")

    camera_shapes: dict[str, tuple[int, int, int]] = {}
    for idx, frame in enumerate(frames):
        _validate_frame(frame, idx, pkl_path)
        frame_camera_fields = sorted([key for key in frame if key.endswith("_image")])
        if frame_camera_fields != camera_fields:
            raise ConversionError(
                f"Camera fields changed at frame {idx} in {pkl_path}: "
                f"expected {camera_fields}, got {frame_camera_fields}"
            )

        for field in camera_fields:
            image = frame[field]
            shape = tuple(int(x) for x in image.shape)
            if field not in camera_shapes:
                camera_shapes[field] = shape
            elif camera_shapes[field] != shape:
                raise ConversionError(
                    f"Camera image shape for {field} changed at frame {idx} in {pkl_path}: "
                    f"expected {camera_shapes[field]}, got {shape}"
                )

    return EpisodeData(
        pkl_path=pkl_path,
        source_episode_index=_parse_source_episode_index(pkl_path),
        frames=frames,
        camera_fields=camera_fields,
        camera_shapes=camera_shapes,
    )


def convert_episode_dataset(
    episode_path: str | Path,
    output_root: str | Path,
    task_description: str,
    fps: int = DEFAULT_FPS,
    robot_type: str = DEFAULT_ROBOT_TYPE,
    overwrite: bool = False,
) -> Path:
    return convert_dataset(
        episode_paths=[Path(episode_path).expanduser().resolve()],
        output_root=output_root,
        task_description=task_description,
        fps=fps,
        robot_type=robot_type,
        overwrite=overwrite,
    )


def convert_task_dataset(
    task_dir: str | Path,
    output_root: str | Path,
    task_description: str,
    fps: int = DEFAULT_FPS,
    robot_type: str = DEFAULT_ROBOT_TYPE,
    overwrite: bool = False,
) -> tuple[Path, list[Path]]:
    episode_paths, skipped = discover_task_episode_paths(task_dir)
    root = convert_dataset(
        episode_paths=episode_paths,
        output_root=output_root,
        task_description=task_description,
        fps=fps,
        robot_type=robot_type,
        overwrite=overwrite,
    )
    return root, skipped


def convert_dataset(
    episode_paths: list[Path],
    output_root: str | Path,
    task_description: str,
    fps: int = DEFAULT_FPS,
    robot_type: str = DEFAULT_ROBOT_TYPE,
    overwrite: bool = False,
) -> Path:
    missing = check_runtime_dependencies()
    if missing:
        raise ConversionError(_dependency_message(missing))

    output_root = Path(output_root).expanduser().resolve()
    _prepare_output_root(output_root, overwrite)

    if fps <= 0:
        raise ConversionError(f"fps must be positive, got: {fps}")

    task_description = str(task_description).strip()
    if not task_description:
        raise ConversionError("task_description cannot be empty")

    episode_results: list[EpisodeWriteResult] = []
    expected_camera_fields: list[str] | None = None
    expected_camera_shapes: dict[str, tuple[int, int, int]] | None = None
    global_frame_start = 0

    for episode_index, source_path in enumerate(episode_paths):
        episode = load_episode(source_path)

        if expected_camera_fields is None:
            expected_camera_fields = episode.camera_fields
            expected_camera_shapes = episode.camera_shapes
        elif episode.camera_fields != expected_camera_fields or episode.camera_shapes != expected_camera_shapes:
            raise ConversionError(
                f"Camera schema mismatch in {episode.pkl_path}. "
                f"Expected {expected_camera_fields}/{expected_camera_shapes}, "
                f"got {episode.camera_fields}/{episode.camera_shapes}"
            )

        result = _write_episode(
            output_root=output_root,
            episode=episode,
            episode_index=episode_index,
            task_index=0,
            global_frame_start=global_frame_start,
            fps=fps,
        )
        episode_results.append(result)
        global_frame_start += result.length

        print(
            f"Converted source episode {episode.pkl_path} -> episode_{episode_index:06d} "
            f"({result.length} frames)"
        )

    assert expected_camera_fields is not None
    assert expected_camera_shapes is not None

    features = _build_features(expected_camera_fields, expected_camera_shapes, fps)
    stats = _aggregate_stats([result.stats for result in episode_results])
    total_frames = sum(result.length for result in episode_results)

    _write_metadata(
        output_root=output_root,
        features=features,
        episode_results=episode_results,
        stats=stats,
        task_description=task_description,
        fps=fps,
        robot_type=robot_type,
        total_frames=total_frames,
    )

    _validate_written_dataset(output_root, episode_results)

    print(f"LeRobot v2.1 dataset written to: {output_root}")
    print(f"Total episodes: {len(episode_results)}, total frames: {total_frames}")
    return output_root


def default_episode_output_root(episode_path: str | Path) -> Path:
    pkl_path = find_episode_pkl(episode_path)
    task_name = _infer_task_name_from_episode_path(pkl_path)
    source_index = _parse_source_episode_index(pkl_path)
    suffix = source_index if source_index is not None else pkl_path.stem.replace(".pkl", "")
    return Path.home() / "Desktop" / "franka_lerobot_data" / f"{task_name}_episode_{suffix}"


def default_task_output_root(task_dir: str | Path) -> Path:
    task_dir = Path(task_dir).expanduser().resolve()
    return Path.home() / "Desktop" / "franka_lerobot_data" / task_dir.name


def infer_task_description_from_episode(episode_path: str | Path) -> str:
    pkl_path = find_episode_pkl(episode_path)
    return _infer_task_name_from_episode_path(pkl_path)


def infer_task_description_from_task_dir(task_dir: str | Path) -> str:
    return Path(task_dir).expanduser().resolve().name


def _write_episode(
    output_root: Path,
    episode: EpisodeData,
    episode_index: int,
    task_index: int,
    global_frame_start: int,
    fps: int,
) -> EpisodeWriteResult:
    states, poses, actions = _build_arrays(episode.frames)
    num_frames = states.shape[0]
    chunk_index = episode_index // DEFAULT_CHUNKS_SIZE

    data_path = output_root / "data" / f"chunk-{chunk_index:03d}" / f"episode_{episode_index:06d}.parquet"
    data_path.parent.mkdir(parents=True, exist_ok=True)
    _write_episode_parquet(
        path=data_path,
        states=states,
        poses=poses,
        actions=actions,
        fps=fps,
        episode_index=episode_index,
        task_index=task_index,
        global_frame_start=global_frame_start,
    )

    video_paths: dict[str, Path] = {}
    for camera_field in episode.camera_fields:
        video_key = _camera_field_to_video_key(camera_field)
        video_path = (
            output_root
            / "videos"
            / f"chunk-{chunk_index:03d}"
            / video_key
            / f"episode_{episode_index:06d}.mp4"
        )
        video_path.parent.mkdir(parents=True, exist_ok=True)
        _write_video_from_episode(episode.frames, camera_field, video_path, fps)
        video_paths[video_key] = video_path

    stats = _compute_episode_stats(episode.frames, states, poses, actions, episode.camera_fields)
    return EpisodeWriteResult(
        episode_index=episode_index,
        source_episode_index=episode.source_episode_index,
        length=num_frames,
        stats=stats,
        data_path=data_path,
        video_paths=video_paths,
    )


def _write_episode_parquet(
    path: Path,
    states: np.ndarray,
    poses: np.ndarray,
    actions: np.ndarray,
    fps: int,
    episode_index: int,
    task_index: int,
    global_frame_start: int,
) -> None:
    import pyarrow as pa
    import pyarrow.parquet as pq

    num_frames = states.shape[0]
    columns = {
        "observation.state": pa.array(states.tolist(), type=_fixed_list_type(pa.float32(), 8)),
        "observation.ee_pose": pa.array(poses.tolist(), type=_fixed_list_type(pa.float32(), 6)),
        "action": pa.array(actions.tolist(), type=_fixed_list_type(pa.float32(), 14)),
        "timestamp": pa.array((np.arange(num_frames, dtype=np.float32) / float(fps)).tolist(), type=pa.float32()),
        "frame_index": pa.array(np.arange(num_frames, dtype=np.int64).tolist(), type=pa.int64()),
        "episode_index": pa.array(np.full(num_frames, episode_index, dtype=np.int64).tolist(), type=pa.int64()),
        "index": pa.array((global_frame_start + np.arange(num_frames, dtype=np.int64)).tolist(), type=pa.int64()),
        "task_index": pa.array(np.full(num_frames, task_index, dtype=np.int64).tolist(), type=pa.int64()),
    }
    pq.write_table(pa.Table.from_pydict(columns), path)


def _write_video_from_episode(frames: list[dict[str, Any]], camera_field: str, video_path: Path, fps: int) -> None:
    try:
        import imageio.v2 as imageio

        with imageio.get_writer(
            video_path,
            fps=fps,
            codec="libx264",
            pixelformat="yuv420p",
            macro_block_size=1,
        ) as writer:
            for frame in frames:
                writer.append_data(_bgr_to_rgb(frame[camera_field]))
        return
    except Exception as exc:
        print(f"Warning: imageio/libx264 failed for {video_path}: {exc}. Falling back to OpenCV mp4v.")

    import cv2

    first = frames[0][camera_field]
    height, width = first.shape[:2]
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (width, height))
    if not writer.isOpened():
        raise ConversionError(f"Failed to open video writer: {video_path}")
    try:
        for frame in frames:
            writer.write(frame[camera_field])
    finally:
        writer.release()


def _write_metadata(
    output_root: Path,
    features: dict[str, dict[str, Any]],
    episode_results: list[EpisodeWriteResult],
    stats: dict[str, dict[str, np.ndarray]],
    task_description: str,
    fps: int,
    robot_type: str,
    total_frames: int,
) -> None:
    meta_dir = output_root / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    total_episodes = len(episode_results)
    total_chunks = max((result.episode_index // DEFAULT_CHUNKS_SIZE for result in episode_results), default=0) + 1
    video_keys = [key for key, feature in features.items() if feature["dtype"] == "video"]

    info = {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": robot_type,
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": 1,
        "total_videos": total_episodes * len(video_keys),
        "total_chunks": total_chunks,
        "chunks_size": DEFAULT_CHUNKS_SIZE,
        "data_files_size_in_mb": DEFAULT_DATA_FILE_SIZE_IN_MB,
        "video_files_size_in_mb": DEFAULT_VIDEO_FILE_SIZE_IN_MB,
        "fps": int(fps),
        "splits": {"train": f"0:{total_episodes}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": _jsonable_features(features),
        "gripper_semantics": GRIPPER_SEMANTICS,
        "gripper_values": {"0": "open", "1": "closed"},
        "gripper_closed_threshold": float(GRIPPER_BINARY_THRESHOLD),
    }
    _write_json(meta_dir / "info.json", info)
    _write_json(meta_dir / "stats.json", _stats_to_jsonable(stats))

    _write_jsonl(meta_dir / "tasks.jsonl", [{"task_index": 0, "task": task_description}])
    _write_jsonl(
        meta_dir / "episodes.jsonl",
        [
            {"episode_index": result.episode_index, "tasks": [task_description], "length": result.length}
            for result in episode_results
        ],
    )
    _write_jsonl(
        meta_dir / "episodes_stats.jsonl",
        [
            {"episode_index": result.episode_index, "stats": _stats_to_jsonable(result.stats)}
            for result in episode_results
        ],
    )


def _build_features(
    camera_fields: list[str],
    camera_shapes: dict[str, tuple[int, int, int]],
    fps: int,
) -> dict[str, dict[str, Any]]:
    features: dict[str, dict[str, Any]] = {
        "observation.state": {"dtype": "float32", "shape": (8,), "names": STATE_NAMES},
        "observation.ee_pose": {"dtype": "float32", "shape": (6,), "names": POSE_NAMES},
        "action": {"dtype": "float32", "shape": (14,), "names": ACTION_NAMES},
    }

    for camera_field in camera_fields:
        height, width, channels = camera_shapes[camera_field]
        features[_camera_field_to_video_key(camera_field)] = {
            "dtype": "video",
            "shape": (height, width, channels),
            "names": ["height", "width", "channels"],
            "info": None,
            "video.fps": int(fps),
            "video.codec": "h264",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "has_audio": False,
        }

    features.update(META_FEATURES)
    return features


def _build_arrays(frames: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    states = []
    poses = []
    for frame in frames:
        joint = np.asarray(frame["joint"], dtype=np.float32)
        gripper = np.asarray([frame_gripper_closedness(frame)], dtype=np.float32)
        states.append(np.concatenate([joint, gripper], axis=0))
        poses.append(np.asarray(frame["pose"], dtype=np.float32))

    states_array = np.stack(states, axis=0).astype(np.float32)
    poses_array = np.stack(poses, axis=0).astype(np.float32)
    action_targets = np.concatenate([poses_array, states_array], axis=1)
    if len(states_array) == 1:
        actions = action_targets.copy()
    else:
        actions = np.concatenate([action_targets[1:], action_targets[-1:]], axis=0)
    return states_array, poses_array, actions.astype(np.float32)


def frame_gripper_closedness(frame: dict[str, Any]) -> float:
    """Return continuous command closedness: 0=open, 1=closed.

    v2 episodes store this in frame["gripper_closedness"]. Older episodes may
    only have gripper_command_raw, gripper_target_width, gripper_01closedness,
    gripper_width, or legacy gripper.
    """
    try:
        return _frame_gripper_closedness(frame)
    except (KeyError, TypeError, ValueError) as exc:
        raise ConversionError("Invalid gripper value for continuous closedness") from exc


def frame_gripper_01closedness(frame: dict[str, Any]) -> float:
    """Return binary closedness for compatibility: 0=open, 1=closed."""
    try:
        return _frame_gripper_01closedness(frame)
    except (KeyError, TypeError, ValueError) as exc:
        raise ConversionError("Invalid gripper value for binary closedness") from exc


def frame_gripper_command(frame: dict[str, Any]) -> float:
    """Legacy alias for older converter users."""

    return frame_gripper_01closedness(frame)


def _compute_episode_stats(
    frames: list[dict[str, Any]],
    states: np.ndarray,
    poses: np.ndarray,
    actions: np.ndarray,
    camera_fields: list[str],
) -> dict[str, dict[str, np.ndarray]]:
    stats = {
        "observation.state": _numeric_stats(states),
        "observation.ee_pose": _numeric_stats(poses),
        "action": _numeric_stats(actions),
    }

    for camera_field in camera_fields:
        stats[_camera_field_to_video_key(camera_field)] = _image_stats(frames, camera_field)

    return stats


def _numeric_stats(array: np.ndarray) -> dict[str, np.ndarray]:
    array = np.asarray(array, dtype=np.float32)
    if array.ndim == 1:
        array = array.reshape(-1, 1)
    return {
        "min": np.min(array, axis=0),
        "max": np.max(array, axis=0),
        "mean": np.mean(array, axis=0),
        "std": np.std(array, axis=0),
        "count": np.asarray([array.shape[0]], dtype=np.int64),
    }


def _image_stats(frames: list[dict[str, Any]], camera_field: str) -> dict[str, np.ndarray]:
    channel_sum = np.zeros(3, dtype=np.float64)
    channel_sum_sq = np.zeros(3, dtype=np.float64)
    channel_min = np.full(3, np.inf, dtype=np.float64)
    channel_max = np.full(3, -np.inf, dtype=np.float64)
    total_pixels = 0

    for frame in frames:
        rgb = _bgr_to_rgb(frame[camera_field]).astype(np.float32) / 255.0
        flat = rgb.reshape(-1, 3)
        channel_sum += flat.sum(axis=0)
        channel_sum_sq += np.square(flat).sum(axis=0)
        channel_min = np.minimum(channel_min, flat.min(axis=0))
        channel_max = np.maximum(channel_max, flat.max(axis=0))
        total_pixels += flat.shape[0]

    mean = channel_sum / total_pixels
    variance = np.maximum(0.0, channel_sum_sq / total_pixels - np.square(mean))
    return {
        "min": channel_min.astype(np.float32).reshape(3, 1, 1),
        "max": channel_max.astype(np.float32).reshape(3, 1, 1),
        "mean": mean.astype(np.float32).reshape(3, 1, 1),
        "std": np.sqrt(variance).astype(np.float32).reshape(3, 1, 1),
        "count": np.asarray([len(frames)], dtype=np.int64),
    }


def _aggregate_stats(stats_list: list[dict[str, dict[str, np.ndarray]]]) -> dict[str, dict[str, np.ndarray]]:
    aggregate: dict[str, dict[str, np.ndarray]] = {}
    for feature_key in sorted({key for stats in stats_list for key in stats}):
        feature_stats = [stats[feature_key] for stats in stats_list if feature_key in stats]
        counts = np.asarray([stats["count"][0] for stats in feature_stats], dtype=np.float64)
        total_count = counts.sum()
        means = np.stack([stats["mean"] for stats in feature_stats], axis=0).astype(np.float64)
        stds = np.stack([stats["std"] for stats in feature_stats], axis=0).astype(np.float64)

        count_shape = (len(counts),) + (1,) * (means.ndim - 1)
        weights = counts.reshape(count_shape)
        total_mean = (means * weights).sum(axis=0) / total_count
        total_var = ((np.square(stds) + np.square(means - total_mean)) * weights).sum(axis=0) / total_count

        aggregate[feature_key] = {
            "min": np.min(np.stack([stats["min"] for stats in feature_stats], axis=0), axis=0),
            "max": np.max(np.stack([stats["max"] for stats in feature_stats], axis=0), axis=0),
            "mean": total_mean.astype(np.float32),
            "std": np.sqrt(np.maximum(0.0, total_var)).astype(np.float32),
            "count": np.asarray([int(total_count)], dtype=np.int64),
        }
    return aggregate


def _validate_written_dataset(output_root: Path, episode_results: list[EpisodeWriteResult]) -> None:
    import cv2
    import pyarrow.parquet as pq

    for result in episode_results:
        num_rows = pq.read_metadata(result.data_path).num_rows
        if num_rows != result.length:
            raise ConversionError(f"{result.data_path} has {num_rows} rows, expected {result.length}")

        for video_key, video_path in result.video_paths.items():
            cap = cv2.VideoCapture(str(video_path))
            try:
                frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            finally:
                cap.release()
            if frame_count > 0 and frame_count != result.length:
                raise ConversionError(f"{video_path} has {frame_count} frames, expected {result.length}")
            print(f"Validated {video_key}: parquet_rows={num_rows}, video_frames={frame_count}")


def _validate_frame(frame: dict[str, Any], frame_index: int, pkl_path: Path) -> None:
    required = {"pose", "joint", "timestamp"}
    missing = required - set(frame)
    if missing:
        raise ConversionError(f"Frame {frame_index} in {pkl_path} is missing keys: {sorted(missing)}")

    if np.asarray(frame["pose"]).shape != (6,):
        raise ConversionError(f"Frame {frame_index} in {pkl_path} has invalid pose shape")
    if np.asarray(frame["joint"]).shape != (7,):
        raise ConversionError(f"Frame {frame_index} in {pkl_path} has invalid joint shape")
    try:
        frame_gripper_closedness(frame)
        frame_gripper_01closedness(frame)
        float(frame["timestamp"])
        for key in (
            "gripper_closedness",
            "gripper_01closedness",
            "gripper_closed",
            "gripper_width",
            "gripper_command_raw",
            "gripper_target_width",
            "gripper_command_timestamp",
        ):
            if key in frame:
                float(frame[key])
    except (TypeError, ValueError) as exc:
        raise ConversionError(f"Frame {frame_index} in {pkl_path} has invalid scalar fields") from exc

    for key, value in frame.items():
        if not key.endswith("_image"):
            continue
        if not hasattr(value, "shape") or len(value.shape) != 3 or value.shape[2] != 3:
            raise ConversionError(f"Frame {frame_index} in {pkl_path} has invalid image field {key}")
        if value.dtype != np.uint8:
            raise ConversionError(f"Frame {frame_index} in {pkl_path} image {key} must be uint8")


def _prepare_output_root(output_root: Path, overwrite: bool) -> None:
    if output_root.exists():
        if not overwrite:
            raise ConversionError(f"Output directory already exists: {output_root}. Use --overwrite to replace it.")
        if output_root == Path(output_root.anchor):
            raise ConversionError(f"Refusing to remove unsafe output directory: {output_root}")
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=True)


def _write_json(path: Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")


def _stats_to_jsonable(stats: dict[str, dict[str, np.ndarray]]) -> dict[str, dict[str, Any]]:
    return {
        feature_key: {stat_key: np.asarray(value).tolist() for stat_key, value in feature_stats.items()}
        for feature_key, feature_stats in stats.items()
    }


def _jsonable_features(features: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for key, feature in features.items():
        item = dict(feature)
        if isinstance(item.get("shape"), tuple):
            item["shape"] = list(item["shape"])
        result[key] = item
    return result


def _fixed_list_type(value_type: Any, size: int) -> Any:
    import pyarrow as pa

    try:
        return pa.list_(value_type, list_size=size)
    except TypeError:
        return pa.list_(value_type, size)


def _bgr_to_rgb(image: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(image[..., ::-1])


def _camera_field_to_video_key(camera_field: str) -> str:
    return f"observation.images.{camera_field[: -len('_image')]}"


def _parse_source_episode_index(pkl_path: Path) -> int | None:
    name = pkl_path.name
    if name.endswith(".pkl.gz"):
        stem = name[: -len(".pkl.gz")]
    else:
        stem = pkl_path.stem
    try:
        return int(stem)
    except ValueError:
        return None


def _infer_task_name_from_episode_path(pkl_path: Path) -> str:
    if pkl_path.parent.parent.name:
        return pkl_path.parent.parent.name
    return "task"


def _episode_sort_key(path: Path) -> tuple[int, int | str]:
    try:
        return (0, int(path.name))
    except ValueError:
        return (1, path.name)


def _dependency_message(missing: list[str]) -> str:
    return (
        "Missing Python dependencies for LeRobot conversion: "
        + ", ".join(missing)
        + "\nInstall them in franka_capture with:\n"
        + "  conda activate franka_capture\n"
        + "  python -m pip install pandas==2.0.3 pyarrow==14.0.2\n"
    )
