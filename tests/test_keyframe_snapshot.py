from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError as exc:
    raise unittest.SkipTest(f"OpenCV is unavailable: {exc}") from exc

from franka_gui.keyframe_snapshot import save_snapshot_frames


class KeyframeSnapshotTest(unittest.TestCase):
    def test_saves_complete_rgb_batch_under_timestamp_directory(self) -> None:
        frames = {
            "left": np.full((4, 6, 3), [12, 34, 56], dtype=np.uint8),
            "middle": np.full((4, 6, 3), [78, 90, 123], dtype=np.uint8),
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            destination = save_snapshot_frames(
                temp_dir,
                frames,
                timestamp="20260713_180000_123456",
            )

            self.assertEqual(
                destination,
                Path(temp_dir) / "keyframe" / "20260713_180000_123456",
            )
            self.assertEqual(
                sorted(path.name for path in destination.iterdir()),
                ["left.png", "middle.png"],
            )
            restored = cv2.imread(str(destination / "left.png"), cv2.IMREAD_COLOR)
            self.assertIsNotNone(restored)
            self.assertTrue(np.array_equal(restored[:, :, ::-1], frames["left"]))

    def test_rejects_invalid_batch_without_publishing_a_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            with self.assertRaisesRegex(ValueError, "HxWx3"):
                save_snapshot_frames(
                    temp_dir,
                    {"left": np.zeros((4, 6), dtype=np.uint8)},
                    timestamp="20260713_180000_123456",
                )
            self.assertFalse((Path(temp_dir) / "keyframe").exists())


if __name__ == "__main__":
    unittest.main()
