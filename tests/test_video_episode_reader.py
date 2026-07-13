from __future__ import annotations

import gzip
import json
import pickle
import unittest
from pathlib import Path

import numpy as np
try:
    import pytest
except ImportError as exc:
    raise unittest.SkipTest("pytest is not installed") from exc

cv2 = pytest.importorskip("cv2")

from franka_capture.recording.video_episode_reader import (  # noqa: E402
    VideoEpisodeReadError,
    read_video_episode,
)
from franka_lerobot.converter import ConversionError, load_episode  # noqa: E402


WIDTH = 16
HEIGHT = 12


def _action_frames(count: int) -> list[dict[str, object]]:
    return [
        {
            "schema_version": "franka_single_v3",
            "frame_index": index,
            "timestamp": float(index) / 30.0,
            "pose": np.full(6, index, dtype=np.float64),
            "joint": np.full(7, index, dtype=np.float64),
            "gripper_closedness": 0.25,
        }
        for index in range(count)
    ]


def _write_pickle(path: Path, frames: list[dict[str, object]]) -> None:
    with gzip.open(path, "wb") as handle:
        pickle.dump({"data": frames}, handle, protocol=pickle.HIGHEST_PROTOCOL)


def _write_video(path: Path, bgr_colors: list[tuple[int, int, int]]) -> None:
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        30.0,
        (WIDTH, HEIGHT),
    )
    if not writer.isOpened():
        pytest.skip("OpenCV mp4v encoder is unavailable")
    try:
        for color in bgr_colors:
            image = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
            image[:] = color
            writer.write(image)
    finally:
        writer.release()


def _write_v3_episode(
    tmp_path: Path,
    *,
    action_count: int = 2,
    camera_colors: dict[str, list[tuple[int, int, int]]] | None = None,
) -> tuple[Path, list[dict[str, object]]]:
    episode_dir = tmp_path / "task" / "0"
    episode_dir.mkdir(parents=True)
    frames = _action_frames(action_count)
    pkl_path = episode_dir / "0.pkl.gz"
    _write_pickle(pkl_path, frames)

    if camera_colors is None:
        camera_colors = {
            "middle": [(220, 30, 20), (20, 210, 40)][:action_count]
        }
    cameras = {}
    for camera_name, colors in camera_colors.items():
        _write_video(episode_dir / f"{camera_name}.mp4", colors)
        cameras[camera_name] = {
            "filename": f"{camera_name}.mp4",
            "width": WIDTH,
            "height": HEIGHT,
            "channels": 3,
            "frame_count": action_count,
        }

    metadata = {
        "schema_version": "franka_single_v3",
        "frame_count": action_count,
        "camera_names": list(camera_colors),
        "image_storage": {
            "type": "video",
            "frame_alignment": "frame_index",
            "decoded_color_order": "BGR",
            "source_color_order": "RGB",
            "cameras": cameras,
        },
    }
    (episode_dir / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8"
    )
    return pkl_path, frames


def _metadata_for(pkl_path: Path) -> dict[str, object]:
    return json.loads((pkl_path.parent / "metadata.json").read_text(encoding="utf-8"))


def _replace_metadata(pkl_path: Path, metadata: dict[str, object]) -> None:
    (pkl_path.parent / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8"
    )


def test_load_episode_hydrates_v3_videos_as_bgr_uint8(tmp_path: Path) -> None:
    colors = {
        "middle": [(220, 30, 20), (20, 210, 40)],
        "left_wrist": [(30, 40, 220), (180, 60, 30)],
    }
    pkl_path, _ = _write_v3_episode(tmp_path, camera_colors=colors)

    episode = load_episode(pkl_path)

    assert episode.camera_fields == ["left_wrist_image", "middle_image"]
    assert episode.camera_shapes == {
        "left_wrist_image": (HEIGHT, WIDTH, 3),
        "middle_image": (HEIGHT, WIDTH, 3),
    }
    assert len(episode.frames) == 2
    for frame in episode.frames:
        for field in episode.camera_fields:
            image = frame[field]
            assert isinstance(image, np.ndarray)
            assert image.dtype == np.uint8
            assert image.shape == (HEIGHT, WIDTH, 3)
            assert image.flags.c_contiguous

    for camera_name, expected_colors in colors.items():
        field = f"{camera_name}_image"
        for frame, expected in zip(episode.frames, expected_colors):
            actual = frame[field].mean(axis=(0, 1))
            np.testing.assert_allclose(actual, expected, atol=12.0)


def test_load_episode_keeps_v2_embedded_images_working(tmp_path: Path) -> None:
    episode_dir = tmp_path / "legacy" / "0"
    episode_dir.mkdir(parents=True)
    frames = _action_frames(2)
    expected_images = []
    for index, frame in enumerate(frames):
        image = np.full((HEIGHT, WIDTH, 3), 20 + index, dtype=np.uint8)
        frame["middle_image"] = image
        expected_images.append(image)
    pkl_path = episode_dir / "0.pkl.gz"
    _write_pickle(pkl_path, frames)

    episode = load_episode(pkl_path)

    assert episode.camera_fields == ["middle_image"]
    for frame, expected in zip(episode.frames, expected_images):
        np.testing.assert_array_equal(frame["middle_image"], expected)


def test_reader_rejects_metadata_frame_count_mismatch(tmp_path: Path) -> None:
    pkl_path, frames = _write_v3_episode(tmp_path)
    metadata = _metadata_for(pkl_path)
    metadata["frame_count"] = 1
    _replace_metadata(pkl_path, metadata)

    with pytest.raises(VideoEpisodeReadError, match=r"frame_count=1.*2 action frames"):
        read_video_episode(pkl_path, frames)


def test_reader_rejects_per_camera_frame_count_mismatch(tmp_path: Path) -> None:
    pkl_path, frames = _write_v3_episode(tmp_path)
    metadata = _metadata_for(pkl_path)
    metadata["image_storage"]["cameras"]["middle"]["frame_count"] = 1
    _replace_metadata(pkl_path, metadata)

    with pytest.raises(
        VideoEpisodeReadError,
        match=r"camera middle frame_count=1.*2 action frames",
    ):
        read_video_episode(pkl_path, frames)


def test_load_episode_rejects_decoded_resolution_mismatch(tmp_path: Path) -> None:
    pkl_path, _ = _write_v3_episode(tmp_path)
    metadata = _metadata_for(pkl_path)
    metadata["image_storage"]["cameras"]["middle"]["width"] = WIDTH + 2
    _replace_metadata(pkl_path, metadata)

    with pytest.raises(ConversionError, match=r"shape=.*expected .* from metadata"):
        load_episode(pkl_path)


def test_reader_rejects_actual_video_frame_count_mismatch(tmp_path: Path) -> None:
    pkl_path, frames = _write_v3_episode(
        tmp_path,
        action_count=2,
        camera_colors={"middle": [(220, 30, 20)]},
    )

    with pytest.raises(
        VideoEpisodeReadError,
        match=r"decoded 1 frames, expected 2 action-aligned frames",
    ):
        read_video_episode(pkl_path, frames)
    assert all("middle_image" not in frame for frame in frames)


def test_hdf5_converter_reuses_lerobot_episode_loader() -> None:
    from franka_hdf5 import converter as hdf5_converter
    from franka_lerobot import converter as lerobot_converter

    assert hdf5_converter.load_episode is lerobot_converter.load_episode
