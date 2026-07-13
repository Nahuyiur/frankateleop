"""Hydrate media-free v3 trajectory frames from per-camera videos."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


IMAGE_FIELD_SUFFIX = "_image"
METADATA_FILENAME = "metadata.json"


class VideoEpisodeReadError(RuntimeError):
    """Raised when a video-backed episode cannot satisfy frame alignment."""


@dataclass(frozen=True)
class VideoEpisodeReadResult:
    frames: list[dict[str, Any]]
    camera_fields: list[str]
    camera_shapes: dict[str, tuple[int, int, int]]


@dataclass(frozen=True)
class _CameraVideoSpec:
    name: str
    path: Path
    width: int
    height: int
    channels: int
    frame_count: int

    @property
    def field(self) -> str:
        return f"{self.name}{IMAGE_FIELD_SUFFIX}"

    @property
    def shape(self) -> tuple[int, int, int]:
        return (self.height, self.width, self.channels)


def read_video_episode(
    pkl_path: str | Path,
    action_frames: Sequence[Mapping[str, Any]],
) -> VideoEpisodeReadResult:
    """Decode v3 camera videos and return frames compatible with v2 consumers.

    OpenCV decodes every video sequentially. Images are attached only after all
    cameras pass validation, so a failed route cannot leave partially hydrated
    frames behind.
    """

    source_path = Path(pkl_path).expanduser().resolve()
    if not action_frames:
        raise VideoEpisodeReadError(f"{source_path} contains no action frames")

    embedded_fields = sorted(
        {
            key
            for frame in action_frames
            for key in frame
            if key.endswith(IMAGE_FIELD_SUFFIX)
        }
    )
    if embedded_fields:
        raise VideoEpisodeReadError(
            f"{source_path} is not media-free; embedded image fields found: "
            f"{embedded_fields}"
        )

    metadata_path = source_path.parent / METADATA_FILENAME
    metadata = _read_metadata(metadata_path)
    specs = _camera_specs(metadata, source_path.parent, len(action_frames))

    decoded_by_field: dict[str, list[np.ndarray]] = {}
    for spec in specs:
        decoded_by_field[spec.field] = _decode_camera_video(spec, len(action_frames))

    hydrated_frames = [dict(frame) for frame in action_frames]
    for frame_index, frame in enumerate(hydrated_frames):
        for spec in specs:
            frame[spec.field] = decoded_by_field[spec.field][frame_index]

    return VideoEpisodeReadResult(
        frames=hydrated_frames,
        camera_fields=[spec.field for spec in specs],
        camera_shapes={spec.field: spec.shape for spec in specs},
    )


def _read_metadata(metadata_path: Path) -> dict[str, Any]:
    if not metadata_path.is_file():
        raise VideoEpisodeReadError(
            f"Media-free trajectory requires {metadata_path}"
        )
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise VideoEpisodeReadError(f"Failed to read {metadata_path}: {exc}") from exc
    if not isinstance(metadata, dict):
        raise VideoEpisodeReadError(f"{metadata_path} must contain a JSON object")
    return metadata


def _camera_specs(
    metadata: Mapping[str, Any],
    episode_dir: Path,
    action_frame_count: int,
) -> list[_CameraVideoSpec]:
    metadata_path = episode_dir / METADATA_FILENAME
    schema_version = metadata.get("schema_version")
    if not isinstance(schema_version, str) or not schema_version.endswith("_v3"):
        raise VideoEpisodeReadError(
            f"{metadata_path} is not a trajectory v3 schema: {schema_version!r}"
        )

    metadata_frame_count = _required_positive_int(
        metadata, "frame_count", metadata_path
    )
    if metadata_frame_count != action_frame_count:
        raise VideoEpisodeReadError(
            f"{metadata_path} frame_count={metadata_frame_count}, but trajectory has "
            f"{action_frame_count} action frames"
        )

    camera_names = metadata.get("camera_names")
    if not isinstance(camera_names, list) or not camera_names:
        raise VideoEpisodeReadError(
            f"{metadata_path} camera_names must be a non-empty list"
        )
    if not all(isinstance(name, str) and name for name in camera_names):
        raise VideoEpisodeReadError(
            f"{metadata_path} camera_names must contain non-empty strings"
        )
    if len(set(camera_names)) != len(camera_names):
        raise VideoEpisodeReadError(f"{metadata_path} camera_names contains duplicates")

    storage = metadata.get("image_storage")
    if not isinstance(storage, dict) or storage.get("type") != "video":
        raise VideoEpisodeReadError(
            f"{metadata_path} image_storage.type must be 'video'"
        )
    if storage.get("frame_alignment") != "frame_index":
        raise VideoEpisodeReadError(
            f"{metadata_path} image_storage.frame_alignment must be 'frame_index'"
        )
    if storage.get("decoded_color_order") != "BGR":
        raise VideoEpisodeReadError(
            f"{metadata_path} image_storage.decoded_color_order must be 'BGR'"
        )

    cameras = storage.get("cameras")
    if not isinstance(cameras, dict) or not cameras:
        raise VideoEpisodeReadError(
            f"{metadata_path} image_storage.cameras must be a non-empty object"
        )
    if set(camera_names) != set(cameras):
        raise VideoEpisodeReadError(
            f"{metadata_path} camera set mismatch: camera_names={sorted(camera_names)}, "
            f"image_storage.cameras={sorted(cameras)}"
        )

    specs: list[_CameraVideoSpec] = []
    for camera_name in sorted(camera_names):
        if Path(camera_name).name != camera_name:
            raise VideoEpisodeReadError(
                f"{metadata_path} has unsafe camera name: {camera_name!r}"
            )
        camera = cameras[camera_name]
        if not isinstance(camera, dict):
            raise VideoEpisodeReadError(
                f"{metadata_path} image_storage.cameras.{camera_name} must be an object"
            )

        expected_filename = f"{camera_name}.mp4"
        if camera.get("filename") != expected_filename:
            raise VideoEpisodeReadError(
                f"{metadata_path} camera {camera_name} filename must be "
                f"{expected_filename!r}, got {camera.get('filename')!r}"
            )
        width = _required_positive_int(camera, "width", metadata_path, camera_name)
        height = _required_positive_int(camera, "height", metadata_path, camera_name)
        channels = _required_positive_int(
            camera, "channels", metadata_path, camera_name
        )
        if channels != 3:
            raise VideoEpisodeReadError(
                f"{metadata_path} camera {camera_name} channels={channels}, expected 3"
            )
        frame_count = _required_positive_int(
            camera, "frame_count", metadata_path, camera_name
        )
        if frame_count != action_frame_count:
            raise VideoEpisodeReadError(
                f"{metadata_path} camera {camera_name} frame_count={frame_count}, "
                f"but trajectory has {action_frame_count} action frames"
            )

        video_path = episode_dir / expected_filename
        if not video_path.is_file():
            raise VideoEpisodeReadError(f"Camera video does not exist: {video_path}")
        specs.append(
            _CameraVideoSpec(
                name=camera_name,
                path=video_path,
                width=width,
                height=height,
                channels=channels,
                frame_count=frame_count,
            )
        )
    return specs


def _required_positive_int(
    values: Mapping[str, Any],
    key: str,
    metadata_path: Path,
    camera_name: str | None = None,
) -> int:
    value = values.get(key)
    location = (
        f"camera {camera_name} {key}" if camera_name is not None else key
    )
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise VideoEpisodeReadError(
            f"{metadata_path} {location} must be a positive integer, got {value!r}"
        )
    return value


def _decode_camera_video(
    spec: _CameraVideoSpec,
    action_frame_count: int,
) -> list[np.ndarray]:
    try:
        import cv2
    except ImportError as exc:
        raise VideoEpisodeReadError(
            "OpenCV is required to read media-free trajectory v3 episodes"
        ) from exc

    capture = cv2.VideoCapture(str(spec.path))
    if not capture.isOpened():
        capture.release()
        raise VideoEpisodeReadError(f"Failed to open camera video: {spec.path}")

    decoded: list[np.ndarray] = []
    try:
        while True:
            ok, image = capture.read()
            if not ok:
                break
            frame_index = len(decoded)
            if frame_index >= action_frame_count:
                raise VideoEpisodeReadError(
                    f"{spec.path} decoded more than {action_frame_count} frames; "
                    "video/action alignment is invalid"
                )
            if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
                dtype = getattr(image, "dtype", None)
                raise VideoEpisodeReadError(
                    f"{spec.path} frame {frame_index} must decode to uint8, got "
                    f"{dtype}"
                )
            actual_shape = tuple(int(value) for value in image.shape)
            if actual_shape != spec.shape:
                raise VideoEpisodeReadError(
                    f"{spec.path} frame {frame_index} shape={actual_shape}, expected "
                    f"{spec.shape} from metadata"
                )
            decoded.append(np.ascontiguousarray(image))
    finally:
        capture.release()

    if len(decoded) != action_frame_count:
        raise VideoEpisodeReadError(
            f"{spec.path} decoded {len(decoded)} frames, expected "
            f"{action_frame_count} action-aligned frames"
        )
    return decoded
