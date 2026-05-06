from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from franka_lerobot.converter import (
    ACTION_NAMES,
    DEFAULT_FPS,
    DEFAULT_ROBOT_TYPE,
    POSE_NAMES,
    STATE_NAMES,
    ConversionError,
    discover_task_episode_paths,
    find_episode_pkl,
    load_episode,
)


FORMAT_VERSION = "franka_hdf5_v1"
DEFAULT_COMPRESSION = "gzip"


@dataclass
class EpisodeWriteResult:
    episode_index: int
    source_episode_index: int | None
    length: int
    path: Path


def check_runtime_dependencies() -> list[str]:
    missing = []
    for module_name in ("h5py",):
        try:
            __import__(module_name)
        except ImportError:
            missing.append(module_name)
    return missing


def convert_episode_file(
    episode_path: str | Path,
    output_file: str | Path,
    task_description: str,
    fps: int = DEFAULT_FPS,
    robot_type: str = DEFAULT_ROBOT_TYPE,
    overwrite: bool = False,
    compression: str | None = DEFAULT_COMPRESSION,
    camera: str | None = None,
) -> Path:
    missing = check_runtime_dependencies()
    if missing:
        raise ConversionError(_dependency_message(missing))

    if fps <= 0:
        raise ConversionError(f"fps must be positive, got: {fps}")

    task_description = str(task_description).strip()
    if not task_description:
        raise ConversionError("task_description cannot be empty")

    output_file = Path(output_file).expanduser().resolve()
    _prepare_output_file(output_file, overwrite)

    episode = load_episode(episode_path)
    camera_fields, camera_shapes = _select_camera_schema(episode.camera_fields, episode.camera_shapes, camera)
    _write_hdf5_episode(
        output_file=output_file,
        episode_path=episode.pkl_path,
        source_episode_index=episode.source_episode_index,
        episode_index=0,
        task_description=task_description,
        fps=fps,
        robot_type=robot_type,
        frames=episode.frames,
        camera_fields=camera_fields,
        camera_shapes=camera_shapes,
        compression=compression,
    )
    _validate_hdf5_episode(output_file)
    print(f"Converted source episode {episode.pkl_path} -> {output_file} ({len(episode.frames)} frames)")
    return output_file


def convert_task_dataset(
    task_dir: str | Path,
    output_root: str | Path,
    task_description: str,
    fps: int = DEFAULT_FPS,
    robot_type: str = DEFAULT_ROBOT_TYPE,
    overwrite: bool = False,
    compression: str | None = DEFAULT_COMPRESSION,
    camera: str | None = None,
) -> tuple[Path, list[Path]]:
    missing = check_runtime_dependencies()
    if missing:
        raise ConversionError(_dependency_message(missing))

    if fps <= 0:
        raise ConversionError(f"fps must be positive, got: {fps}")

    task_description = str(task_description).strip()
    if not task_description:
        raise ConversionError("task_description cannot be empty")

    episode_paths, skipped = discover_task_episode_paths(task_dir)
    output_root = Path(output_root).expanduser().resolve()
    _prepare_output_root(output_root, overwrite)

    episode_dir = output_root / "episodes"
    episode_dir.mkdir(parents=True, exist_ok=True)

    expected_camera_fields: list[str] | None = None
    expected_camera_shapes: dict[str, tuple[int, int, int]] | None = None
    results: list[EpisodeWriteResult] = []

    for episode_index, episode_path in enumerate(episode_paths):
        episode = load_episode(episode_path)
        camera_fields, camera_shapes = _select_camera_schema(episode.camera_fields, episode.camera_shapes, camera)
        if expected_camera_fields is None:
            expected_camera_fields = camera_fields
            expected_camera_shapes = camera_shapes
        elif camera_fields != expected_camera_fields or camera_shapes != expected_camera_shapes:
            raise ConversionError(
                f"Camera schema mismatch in {episode.pkl_path}. "
                f"Expected {expected_camera_fields}/{expected_camera_shapes}, "
                f"got {camera_fields}/{camera_shapes}"
            )

        output_file = episode_dir / f"episode_{episode_index:06d}.hdf5"
        _write_hdf5_episode(
            output_file=output_file,
            episode_path=episode.pkl_path,
            source_episode_index=episode.source_episode_index,
            episode_index=episode_index,
            task_description=task_description,
            fps=fps,
            robot_type=robot_type,
            frames=episode.frames,
            camera_fields=camera_fields,
            camera_shapes=camera_shapes,
            compression=compression,
        )
        _validate_hdf5_episode(output_file)
        results.append(
            EpisodeWriteResult(
                episode_index=episode_index,
                source_episode_index=episode.source_episode_index,
                length=len(episode.frames),
                path=output_file,
            )
        )
        print(f"Converted source episode {episode.pkl_path} -> {output_file} ({len(episode.frames)} frames)")

    assert expected_camera_fields is not None
    assert expected_camera_shapes is not None
    _write_task_metadata(
        output_root=output_root,
        results=results,
        task_description=task_description,
        fps=fps,
        robot_type=robot_type,
        camera_fields=expected_camera_fields,
        camera_shapes=expected_camera_shapes,
        skipped=skipped,
    )
    print(f"HDF5 dataset written to: {output_root}")
    print(f"Total episodes: {len(results)}, total frames: {sum(result.length for result in results)}")
    return output_root, skipped


def default_episode_output_file(episode_path: str | Path) -> Path:
    pkl_path = find_episode_pkl(episode_path)
    task_name = _infer_task_name_from_episode_path(pkl_path)
    source_index = _parse_source_episode_index(pkl_path)
    suffix = source_index if source_index is not None else pkl_path.name.replace(".pkl.gz", "")
    return Path.home() / "Desktop" / "franka_hdf5_data" / f"{task_name}_episode_{suffix}.hdf5"


def default_task_output_root(task_dir: str | Path) -> Path:
    task_dir = Path(task_dir).expanduser().resolve()
    return Path.home() / "Desktop" / "franka_hdf5_data" / task_dir.name


def infer_task_description_from_episode(episode_path: str | Path) -> str:
    pkl_path = find_episode_pkl(episode_path)
    return _infer_task_name_from_episode_path(pkl_path)


def infer_task_description_from_task_dir(task_dir: str | Path) -> str:
    return Path(task_dir).expanduser().resolve().name


def _write_hdf5_episode(
    output_file: Path,
    episode_path: Path,
    source_episode_index: int | None,
    episode_index: int,
    task_description: str,
    fps: int,
    robot_type: str,
    frames: list[dict[str, Any]],
    camera_fields: list[str],
    camera_shapes: dict[str, tuple[int, int, int]],
    compression: str | None,
) -> None:
    import h5py

    states, poses, actions, joints, grippers, source_timestamps = _build_arrays(frames)
    num_frames = states.shape[0]
    frame_indices = np.arange(num_frames, dtype=np.int64)
    relative_timestamps = frame_indices.astype(np.float32) / float(fps)

    output_file.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_file, "w") as h5:
        h5.attrs["format_version"] = FORMAT_VERSION
        h5.attrs["task"] = task_description
        h5.attrs["robot_type"] = robot_type
        h5.attrs["fps"] = int(fps)
        h5.attrs["episode_index"] = int(episode_index)
        h5.attrs["source_episode_index"] = -1 if source_episode_index is None else int(source_episode_index)
        h5.attrs["source_pkl"] = str(episode_path)
        h5.attrs["num_frames"] = int(num_frames)
        h5.attrs["state_names"] = json.dumps(STATE_NAMES)
        h5.attrs["pose_names"] = json.dumps(POSE_NAMES)
        h5.attrs["action_names"] = json.dumps(ACTION_NAMES)
        h5.attrs["camera_names"] = json.dumps([_camera_field_to_name(field) for field in camera_fields])
        h5.attrs["image_color_order"] = "RGB"
        h5.attrs["source_image_color_order"] = "BGR"
        h5.attrs["action_semantics"] = "next_frame_absolute_ee_pose_joint_gripper"

        h5.create_dataset("frame_index", data=frame_indices, dtype="i8")
        h5.create_dataset("timestamp", data=relative_timestamps, dtype="f4")
        h5.create_dataset("source_timestamp", data=source_timestamps, dtype="f8")

        observations = h5.create_group("observations")
        observations.create_dataset("state", data=states, dtype="f4")
        observations.create_dataset("ee_pose", data=poses, dtype="f4")
        observations.create_dataset("joint", data=joints, dtype="f4")
        observations.create_dataset("gripper", data=grippers, dtype="f4")

        images = observations.create_group("images")
        for camera_field in camera_fields:
            camera_name = _camera_field_to_name(camera_field)
            image_array = _stack_rgb_images(frames, camera_field)
            dataset = images.create_dataset(
                camera_name,
                data=image_array,
                dtype="u1",
                compression=compression,
                chunks=(1,) + tuple(image_array.shape[1:]),
            )
            dataset.attrs["source_field"] = camera_field
            dataset.attrs["color_order"] = "RGB"
            dataset.attrs["shape"] = json.dumps(list(camera_shapes[camera_field]))

        h5.create_dataset("action", data=actions, dtype="f4")
        h5.create_dataset("keyframes", data=_load_keyframes_from_episode_dir(episode_path), dtype="i8")


def _select_camera_schema(
    camera_fields: list[str],
    camera_shapes: dict[str, tuple[int, int, int]],
    camera: str | None,
) -> tuple[list[str], dict[str, tuple[int, int, int]]]:
    if camera is None:
        return camera_fields, camera_shapes

    camera_field = f"{camera}_image"
    if camera_field not in camera_fields:
        raise ConversionError(
            f"Requested camera '{camera}' is missing. "
            f"Available cameras: {[field[: -len('_image')] for field in camera_fields]}"
        )
    return [camera_field], {camera_field: camera_shapes[camera_field]}


def _write_task_metadata(
    output_root: Path,
    results: list[EpisodeWriteResult],
    task_description: str,
    fps: int,
    robot_type: str,
    camera_fields: list[str],
    camera_shapes: dict[str, tuple[int, int, int]],
    skipped: list[Path],
) -> None:
    metadata = {
        "format_version": FORMAT_VERSION,
        "task": task_description,
        "robot_type": robot_type,
        "fps": int(fps),
        "total_episodes": len(results),
        "total_frames": sum(result.length for result in results),
        "state_names": STATE_NAMES,
        "pose_names": POSE_NAMES,
        "action_names": ACTION_NAMES,
        "action_semantics": "next_frame_absolute_ee_pose_joint_gripper",
        "image_color_order": "RGB",
        "source_image_color_order": "BGR",
        "camera_names": [_camera_field_to_name(field) for field in camera_fields],
        "camera_shapes": {
            _camera_field_to_name(field): list(camera_shapes[field])
            for field in camera_fields
        },
        "episodes": [
            {
                "episode_index": result.episode_index,
                "source_episode_index": result.source_episode_index,
                "length": result.length,
                "path": str(result.path.relative_to(output_root)),
            }
            for result in results
        ],
        "skipped_without_pkl": [str(path) for path in skipped],
    }
    _write_json(output_root / "metadata.json", metadata)


def _build_arrays(
    frames: list[dict[str, Any]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    joints = []
    grippers = []
    poses = []
    source_timestamps = []

    for frame in frames:
        joints.append(np.asarray(frame["joint"], dtype=np.float32))
        grippers.append(float(frame["gripper"]))
        poses.append(np.asarray(frame["pose"], dtype=np.float32))
        source_timestamps.append(float(frame["timestamp"]))

    joints_array = np.stack(joints, axis=0).astype(np.float32)
    grippers_array = np.asarray(grippers, dtype=np.float32).reshape(-1, 1)
    states_array = np.concatenate([joints_array, grippers_array], axis=1).astype(np.float32)
    poses_array = np.stack(poses, axis=0).astype(np.float32)
    action_targets = np.concatenate([poses_array, states_array], axis=1).astype(np.float32)
    if len(action_targets) == 1:
        actions = action_targets.copy()
    else:
        actions = np.concatenate([action_targets[1:], action_targets[-1:]], axis=0).astype(np.float32)
    return (
        states_array,
        poses_array,
        actions,
        joints_array,
        grippers_array[:, 0],
        np.asarray(source_timestamps, dtype=np.float64),
    )


def _stack_rgb_images(frames: list[dict[str, Any]], camera_field: str) -> np.ndarray:
    return np.stack([_bgr_to_rgb(frame[camera_field]) for frame in frames], axis=0)


def _bgr_to_rgb(image: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(image[..., ::-1])


def _load_keyframes_from_episode_dir(episode_path: Path) -> np.ndarray:
    keyframe_path = episode_path.parent / "keyframes.json"
    if not keyframe_path.exists():
        return np.asarray([0], dtype=np.int64)
    try:
        obj = json.loads(keyframe_path.read_text(encoding="utf-8"))
        keyframes = obj.get("keyframes", [0])
    except (OSError, json.JSONDecodeError):
        keyframes = [0]
    return np.asarray(keyframes, dtype=np.int64)


def _validate_hdf5_episode(path: Path) -> None:
    import h5py

    with h5py.File(path, "r") as h5:
        num_frames = int(h5.attrs["num_frames"])
        expected = {
            "observations/state": (num_frames, 8),
            "observations/ee_pose": (num_frames, 6),
            "observations/joint": (num_frames, 7),
            "observations/gripper": (num_frames,),
            "action": (num_frames, 14),
            "timestamp": (num_frames,),
            "source_timestamp": (num_frames,),
        }
        for key, shape in expected.items():
            if key not in h5:
                raise ConversionError(f"{path} is missing dataset: {key}")
            if tuple(h5[key].shape) != shape:
                raise ConversionError(f"{path}:{key} shape {h5[key].shape} != {shape}")

        images = h5["observations/images"]
        for camera_name in json.loads(h5.attrs["camera_names"]):
            if camera_name not in images:
                raise ConversionError(f"{path} is missing camera dataset: observations/images/{camera_name}")
            if images[camera_name].shape[0] != num_frames:
                raise ConversionError(
                    f"{path}:observations/images/{camera_name} has "
                    f"{images[camera_name].shape[0]} frames, expected {num_frames}"
                )
    print(f"Validated HDF5 episode: {path}")


def _prepare_output_file(output_file: Path, overwrite: bool) -> None:
    if output_file.exists():
        if not overwrite:
            raise ConversionError(f"Output file already exists: {output_file}. Use --overwrite to replace it.")
        output_file.unlink()
    output_file.parent.mkdir(parents=True, exist_ok=True)


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


def _camera_field_to_name(camera_field: str) -> str:
    return camera_field[: -len("_image")]


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


def _dependency_message(missing: list[str]) -> str:
    return (
        "Missing Python dependencies for HDF5 conversion: "
        + ", ".join(missing)
        + "\nInstall them in data_convert with:\n"
        + "  conda activate data_convert\n"
        + "  python -m pip install h5py\n"
    )
