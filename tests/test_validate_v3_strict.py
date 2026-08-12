from __future__ import annotations

import gzip
import json
import pickle
import threading
import unittest
from pathlib import Path
from typing import Any

import cv2
import numpy as np
try:
    import pytest
except ImportError as exc:
    raise unittest.SkipTest("pytest is not installed") from exc

from validate.validate_task import (
    ValidationCancelled,
    parse_args,
    read_video_info,
    validate_episode,
)


WIDTH = 32
HEIGHT = 24
FRAME_COUNT = 3


def _frame(index: int) -> dict[str, Any]:
    return {
        "schema_version": "franka_single_v3",
        "frame_index": index,
        "timestamp": 100.0 + index / 30.0,
        "pose": np.zeros(6, dtype=float),
        "joint": np.zeros(7, dtype=float),
        "gripper_closedness": 0.25,
        "gripper_01closedness": 0.0,
        "gripper_width": 0.07,
        "gripper_target_width": 0.06,
    }


def _write_video(path: Path, frame_count: int, width: int = WIDTH, height: int = HEIGHT) -> None:
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        30.0,
        (width, height),
    )
    if not writer.isOpened():
        pytest.skip("OpenCV mp4v encoder is unavailable")
    try:
        for index in range(frame_count):
            image = np.full((height, width, 3), 30 + index * 20, dtype=np.uint8)
            writer.write(image)
    finally:
        writer.release()


def _write_v3_episode(
    root: Path,
    *,
    video_frame_count: int = FRAME_COUNT,
) -> tuple[Path, list[dict[str, Any]], dict[str, Any]]:
    episode_dir = root / "task" / "0"
    episode_dir.mkdir(parents=True)
    frames = [_frame(index) for index in range(FRAME_COUNT)]
    with gzip.open(episode_dir / "0.pkl.gz", "wb") as handle:
        pickle.dump({"data": frames}, handle, protocol=pickle.HIGHEST_PROTOCOL)
    _write_video(episode_dir / "middle.mp4", video_frame_count)
    (episode_dir / "keyframes.json").write_text(
        json.dumps({"keyframes": [0]}), encoding="utf-8"
    )

    metadata: dict[str, Any] = {
        "schema_version": "franka_single_v3",
        "frame_count": FRAME_COUNT,
        "camera_names": ["middle"],
        "video_fps": 30,
        "keyframes": [0],
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
                    "frame_count": FRAME_COUNT,
                }
            },
        },
    }
    _write_metadata(episode_dir, metadata)
    return episode_dir, frames, metadata


def _write_metadata(episode_dir: Path, metadata: dict[str, Any]) -> None:
    (episode_dir / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8"
    )


def test_video_validation_can_be_cancelled_between_decoded_frames(
    monkeypatch,
) -> None:
    cancelled = threading.Event()

    class FakeCapture:
        def isOpened(self):
            return True

        def get(self, _key):
            return 30.0

        def read(self):
            cancelled.set()
            return True, np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        def release(self):
            pass

    monkeypatch.setattr(cv2, "VideoCapture", lambda _path: FakeCapture())
    with pytest.raises(ValidationCancelled):
        read_video_info(Path("cancel.mp4"), cancel_event=cancelled)


def _validator_args() -> Any:
    args = parse_args(["task"])
    args.min_cameras = 1
    args.strict_camera_set = "none"
    return args


def _issue_codes(report: Any) -> set[str]:
    return {issue.code for issue in report.issues}


def test_valid_v3_episode_passes_strict_contract_and_full_decode(tmp_path: Path) -> None:
    episode_dir, _, _ = _write_v3_episode(tmp_path)

    report = validate_episode(episode_dir, _validator_args())

    assert report.status == "PASS", [(issue.code, issue.message) for issue in report.issues]
    assert report.videos["middle"].frame_count == FRAME_COUNT
    assert (report.videos["middle"].width, report.videos["middle"].height) == (WIDTH, HEIGHT)


@pytest.mark.parametrize(
    "path,value,expected_code",
    [
        (("image_storage", "type"), "frames", "v3_image_storage_type"),
        (("image_storage", "frame_alignment"), "timestamp", "v3_frame_alignment"),
        (("image_storage", "decoded_color_order"), "RGB", "v3_decoded_color_order"),
        (("image_storage", "source_color_order"), "BGR", "v3_source_color_order"),
        (("image_storage", "cameras", "middle", "filename"), "other.mp4", "v3_camera_filename"),
        (("image_storage", "cameras", "middle", "width"), WIDTH + 2, "video_decoded_frame_size_mismatch"),
        (("image_storage", "cameras", "middle", "height"), HEIGHT + 2, "video_decoded_frame_size_mismatch"),
        (("image_storage", "cameras", "middle", "channels"), 1, "v3_camera_channels"),
        (("image_storage", "cameras", "middle", "frame_count"), 2, "v3_camera_frame_count"),
    ],
)
def test_v3_rejects_each_strict_image_storage_field(
    path: tuple[str, ...],
    value: Any,
    expected_code: str,
    tmp_path: Path,
) -> None:
    episode_dir, _, metadata = _write_v3_episode(tmp_path)
    target: dict[str, Any] = metadata
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    _write_metadata(episode_dir, metadata)

    report = validate_episode(episode_dir, _validator_args())

    assert report.status == "FAIL"
    assert expected_code in _issue_codes(report)


def test_v3_rejects_camera_collection_mismatch(tmp_path: Path) -> None:
    episode_dir, _, metadata = _write_v3_episode(tmp_path)
    metadata["image_storage"]["cameras"]["ghost"] = dict(
        metadata["image_storage"]["cameras"]["middle"]
    )
    _write_metadata(episode_dir, metadata)

    report = validate_episode(episode_dir, _validator_args())

    assert "v3_camera_set_mismatch" in _issue_codes(report)


def test_v3_frame_index_must_be_exact_zero_to_n_minus_one(tmp_path: Path) -> None:
    episode_dir, frames, _ = _write_v3_episode(tmp_path)
    frames[1]["frame_index"] = 2
    with gzip.open(episode_dir / "0.pkl.gz", "wb") as handle:
        pickle.dump({"data": frames}, handle, protocol=pickle.HIGHEST_PROTOCOL)

    report = validate_episode(episode_dir, _validator_args())

    assert "v3_frame_index_sequence" in _issue_codes(report)


def test_v3_decodes_every_mp4_frame_and_rejects_short_video(tmp_path: Path) -> None:
    episode_dir, _, _ = _write_v3_episode(tmp_path, video_frame_count=FRAME_COUNT - 1)

    report = validate_episode(episode_dir, _validator_args())

    codes = _issue_codes(report)
    assert "video_frame_unreadable" in codes
    assert "video_frame_count_mismatch" in codes
    assert report.videos["middle"].frame_count == FRAME_COUNT - 1


def test_v3_rejects_unreadable_mp4(tmp_path: Path) -> None:
    episode_dir, _, _ = _write_v3_episode(tmp_path)
    (episode_dir / "middle.mp4").write_bytes(b"not an mp4")

    report = validate_episode(episode_dir, _validator_args())

    assert "video_unreadable" in _issue_codes(report)
