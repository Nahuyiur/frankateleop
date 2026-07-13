from __future__ import annotations

import gzip
import json
import pickle
import unittest
from pathlib import Path
from typing import Any

import cv2
import numpy as np
try:
    import pytest
except ImportError as exc:
    raise unittest.SkipTest("pytest is not installed") from exc

from franka_downsample.downsample import (
    DownsampleError,
    OUTPUT_SCHEMA_VERSION,
    QUALITY_DIRS,
    downsample_episode,
    downsample_task,
)
from validate.validate_task import parse_args, validate_episode


WIDTH = 32
HEIGHT = 24


def _single_frame(index: int, schema: str, *, embedded_image: bool) -> dict[str, Any]:
    frame: dict[str, Any] = {
        "schema_version": schema,
        "timestamp": 100.0 + index / 30.0,
        "pose": np.zeros(6, dtype=float),
        "joint": np.zeros(7, dtype=float),
        "gripper_closedness": 0.25,
        "gripper_01closedness": 0.0,
        "gripper_width": 0.07,
        "gripper_target_width": 0.06,
    }
    if schema.endswith("_v3"):
        frame["frame_index"] = index
    if embedded_image:
        frame["middle_image"] = np.full(
            (HEIGHT, WIDTH, 3), 30 + index * 10, dtype=np.uint8
        )
    return frame


def _write_pickle(episode_dir: Path, index: int, frames: list[dict[str, Any]]) -> Path:
    episode_dir.mkdir(parents=True)
    path = episode_dir / f"{index}.pkl.gz"
    with gzip.open(path, "wb") as handle:
        pickle.dump({"data": frames, "keyframes": [0]}, handle)
    return path


def _write_video(path: Path, frame_count: int, fps: float = 30.0) -> None:
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (WIDTH, HEIGHT)
    )
    if not writer.isOpened():
        pytest.skip("OpenCV mp4v encoder is unavailable")
    try:
        for index in range(frame_count):
            writer.write(np.full((HEIGHT, WIDTH, 3), 40 + index * 10, np.uint8))
    finally:
        writer.release()


def _write_v3_episode(episode_dir: Path, index: int = 0, frame_count: int = 4) -> None:
    frames = [
        _single_frame(frame_index, "franka_single_v3", embedded_image=False)
        for frame_index in range(frame_count)
    ]
    _write_pickle(episode_dir, index, frames)
    _write_video(episode_dir / "middle.mp4", frame_count)
    (episode_dir / "keyframes.json").write_text(
        json.dumps({"keyframes": [0]}), encoding="utf-8"
    )
    metadata = {
        "schema_version": "franka_single_v3",
        "frame_count": frame_count,
        "camera_names": ["middle"],
        "video_fps": 30,
        "keyframes": [0],
        "quality": episode_dir.parent.name,
        "image_storage": {
            "type": "video",
            "frame_alignment": "frame_index",
            "decoded_color_order": "BGR",
            "source_color_order": "RGB",
            "cameras": {
                "middle": {
                    "filename": "middle.mp4",
                    "width": WIDTH,
                    "height": HEIGHT,
                    "channels": 3,
                    "frame_count": frame_count,
                }
            },
        },
    }
    (episode_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")


def test_downsample_traverses_all_quality_directories(tmp_path: Path) -> None:
    input_task = tmp_path / "input_task"
    for quality in QUALITY_DIRS:
        frames = [
            _single_frame(index, "franka_single_v2", embedded_image=True)
            for index in range(2)
        ]
        _write_pickle(input_task / quality / "7", 7, frames)

    output_task, results = downsample_task(
        input_task,
        tmp_path / "output_task",
        camera="middle",
        source_fps=30,
        target_fps=10,
    )

    assert [result.quality for result in results] == list(QUALITY_DIRS)
    for quality in QUALITY_DIRS:
        assert (output_task / quality / "0" / "0.pkl.gz").is_file()
        metadata = json.loads(
            (output_task / quality / "0" / "metadata.json").read_text(encoding="utf-8")
        )
        assert metadata["schema_version"] == OUTPUT_SCHEMA_VERSION
        assert metadata["quality"] == quality


def test_downsample_zero_episode_fails_before_overwriting_output(tmp_path: Path) -> None:
    input_task = tmp_path / "empty_task"
    (input_task / "High_Quality").mkdir(parents=True)
    output_task = tmp_path / "existing_output"
    output_task.mkdir()
    sentinel = output_task / "keep.txt"
    sentinel.write_text("keep", encoding="utf-8")

    with pytest.raises(DownsampleError, match="No episode directories"):
        downsample_task(input_task, output_task, overwrite=True)

    assert sentinel.read_text(encoding="utf-8") == "keep"


def test_v3_downsample_emits_only_canonical_v2_and_revalidates(tmp_path: Path) -> None:
    input_task = tmp_path / "input_task"
    _write_v3_episode(input_task / "High_Quality" / "0")

    output_task, _ = downsample_task(
        input_task,
        tmp_path / "output_task",
        camera="middle",
        source_fps=30,
        target_fps=10,
    )
    episode_dir = output_task / "High_Quality" / "0"
    with gzip.open(episode_dir / "0.pkl.gz", "rb") as handle:
        payload = pickle.load(handle)
    metadata = json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))

    assert payload["schema_version"] == OUTPUT_SCHEMA_VERSION
    assert payload["data"]
    assert all(frame["schema_version"] == OUTPUT_SCHEMA_VERSION for frame in payload["data"])
    assert all("frame_index" not in frame for frame in payload["data"])
    assert all("middle_image" in frame for frame in payload["data"])
    assert metadata["schema_version"] == OUTPUT_SCHEMA_VERSION
    assert metadata["source_schema_version"] == "franka_single_v3"
    assert metadata["frame_count"] == len(payload["data"])
    assert metadata["camera_names"] == ["middle"]
    assert metadata["video_fps"] == 10
    assert "image_storage" not in metadata

    args = parse_args(["output_task"])
    args.min_cameras = 1
    args.strict_camera_set = "none"
    args.expected_fps = 10.0
    report = validate_episode(episode_dir, args)
    assert report.status == "PASS", [(issue.code, issue.message) for issue in report.issues]


def test_downsample_explicitly_rejects_dual_arm_input(tmp_path: Path) -> None:
    frame = {"timestamp": 1.0}
    for arm in ("left", "right"):
        frame[f"{arm}_pose"] = np.zeros(6)
        frame[f"{arm}_joint"] = np.zeros(7)
        frame[f"{arm}_gripper_width"] = 0.07
    episode_dir = tmp_path / "dual" / "0"
    _write_pickle(episode_dir, 0, [frame])

    with pytest.raises(DownsampleError, match="Dual-arm downsample is not supported"):
        downsample_episode(episode_dir, tmp_path / "out", 0, "middle", 3, 10)
