from __future__ import annotations

import gzip
import json
import pickle
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from franka_lerobot.converter import ConversionError, load_episode


class VideoEpisodeReaderTest(unittest.TestCase):
    def test_v3_video_is_hydrated_as_bgr_without_resolution_change(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            pkl_path = _write_v3_episode(Path(temp_dir), frame_count=4)
            episode = load_episode(pkl_path)
            self.assertEqual(episode.camera_fields, ["middle_image"])
            self.assertEqual(episode.camera_shapes["middle_image"], (48, 64, 3))
            self.assertEqual(len(episode.frames), 4)
            for frame in episode.frames:
                image = frame["middle_image"]
                self.assertEqual(image.shape, (48, 64, 3))
                self.assertEqual(image.dtype, np.uint8)

    def test_v3_rejects_declared_resolution_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            pkl_path = _write_v3_episode(Path(temp_dir), frame_count=3)
            metadata_path = pkl_path.parent / "metadata.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata["image_storage"]["cameras"]["middle"]["width"] = 65
            metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
            with self.assertRaisesRegex(ConversionError, "shape=.*expected"):
                load_episode(pkl_path)

    def test_v2_embedded_images_still_load(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            episode_dir = Path(temp_dir) / "legacy" / "0"
            episode_dir.mkdir(parents=True)
            image = np.zeros((48, 64, 3), dtype=np.uint8)
            frame = _action_frame(0)
            frame["middle_image"] = image
            pkl_path = episode_dir / "0.pkl.gz"
            with gzip.open(pkl_path, "wb") as handle:
                pickle.dump({"data": [frame]}, handle)
            episode = load_episode(pkl_path)
            np.testing.assert_array_equal(episode.frames[0]["middle_image"], image)


def _write_v3_episode(root: Path, frame_count: int) -> Path:
    episode_dir = root / "task" / "0"
    episode_dir.mkdir(parents=True)
    frames = [_action_frame(index) for index in range(frame_count)]
    pkl_path = episode_dir / "0.pkl.gz"
    with gzip.open(pkl_path, "wb") as handle:
        pickle.dump({"data": frames}, handle)

    video_path = episode_dir / "middle.mp4"
    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        30.0,
        (64, 48),
    )
    if not writer.isOpened():
        raise unittest.SkipTest("OpenCV mp4v encoder is unavailable")
    try:
        for index in range(frame_count):
            image = np.zeros((48, 64, 3), dtype=np.uint8)
            image[:, :, 0] = index * 20
            image[:, :, 1] = 80
            image[:, :, 2] = 180
            writer.write(image)
    finally:
        writer.release()

    metadata = {
        "schema_version": "franka_single_v3",
        "frame_count": frame_count,
        "camera_names": ["middle"],
        "image_storage": {
            "type": "video",
            "frame_alignment": "frame_index",
            "decoded_color_order": "BGR",
            "source_color_order": "RGB",
            "cameras": {
                "middle": {
                    "filename": "middle.mp4",
                    "width": 64,
                    "height": 48,
                    "channels": 3,
                    "frame_count": frame_count,
                }
            },
        },
    }
    (episode_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    return pkl_path


def _action_frame(index: int) -> dict:
    return {
        "schema_version": "franka_single_v3",
        "frame_index": index,
        "timestamp": 1000.0 + index / 30.0,
        "pose": np.zeros(6),
        "joint": np.zeros(7),
        "gripper_closedness": 0.0,
        "gripper_01closedness": 0.0,
        "gripper_width": 0.08,
        "gripper_target_width": 0.08,
    }


if __name__ == "__main__":
    unittest.main()
