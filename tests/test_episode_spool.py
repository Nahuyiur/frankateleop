from __future__ import annotations

import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from franka_gui.episode_spool import (
    EpisodeSpoolError,
    StreamingEpisodeSpool,
    _make_preview_rgb,
)


class StreamingEpisodeSpoolTest(unittest.TestCase):
    def test_preserves_camera_resolution_and_frame_count(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            spool = StreamingEpisodeSpool(
                ["left", "middle", "right"],
                30,
                cache_root=Path(temp_dir),
                queue_size=4,
            )
            for frame_index in range(7):
                frames = {
                    name: _rgb_frame(64, 48, frame_index + offset)
                    for offset, name in enumerate(spool.camera_names)
                }
                spool.append(frames)

            storage = spool.finish()
            self.assertEqual(storage["type"], "video")
            self.assertTrue(storage["camera_resolution_preserved"])
            for name in spool.camera_names:
                info = storage["cameras"][name]
                self.assertEqual((info["width"], info["height"]), (64, 48))
                self.assertEqual(info["frame_count"], 7)
                self.assertEqual(_video_shape_and_count(spool.episode_dir / f"{name}.mp4"), (64, 48, 7))
            preview = storage["preview"]
            self.assertEqual((preview["width"], preview["height"]), (192, 192))
            self.assertEqual(preview["channels"], 3)
            self.assertEqual(preview["frame_count"], 7)
            self.assertEqual(
                _video_shape_and_count(spool.episode_dir / preview["filename"]),
                (preview["width"], preview["height"], preview["frame_count"]),
            )

    def test_historical_preview_dimensions_are_preserved(self) -> None:
        three_names = ["left", "middle", "right"]
        five_names = ["left_wrist", "left", "middle", "right", "right_wrist"]
        frames = {
            name: np.full((480, 640, 3), index + 1, dtype=np.uint8)
            for index, name in enumerate(five_names)
        }

        three_camera_preview = _make_preview_rgb(frames, three_names)
        five_camera_preview = _make_preview_rgb(frames, five_names)

        self.assertEqual(three_camera_preview.shape, (192, 720, 3))
        self.assertEqual(five_camera_preview.shape, (192, 1200, 3))
        self.assertTrue(np.all(three_camera_preview[:6] == 0))
        self.assertTrue(np.all(three_camera_preview[-6:] == 0))

    def test_discard_removes_recording_session(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            spool = StreamingEpisodeSpool(
                ["middle"],
                30,
                cache_root=Path(temp_dir),
            )
            spool.append({"middle": _rgb_frame(64, 48, 1)})
            session_dir = spool.session_dir
            spool.discard()
            self.assertFalse(session_dir.exists())

    def test_append_owns_pixels_before_camera_buffer_is_reused(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            camera_writer = _BlockingWriter()
            preview_writer = _CollectingWriter()

            def create_writers(_output_dir, names, _fps, **_kwargs):
                if names == ["middle"]:
                    return {"middle": camera_writer}
                return {"preview_all": preview_writer}

            with patch(
                "franka_gui.episode_spool._create_video_writers",
                side_effect=create_writers,
            ):
                spool = StreamingEpisodeSpool(
                    ["middle"],
                    30,
                    cache_root=Path(temp_dir),
                )
                source = np.full((8, 10, 3), 17, dtype=np.uint8)
                spool.append({"middle": source})
                self.assertTrue(camera_writer.entered.wait(timeout=1.0))
                source.fill(231)
                camera_writer.release.set()
                spool.finish()

            self.assertEqual(len(camera_writer.frames), 1)
            self.assertTrue(np.all(camera_writer.frames[0] == 17))

    def test_init_failure_closes_created_writers_and_removes_session(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            camera_writer = _CollectingWriter()
            calls = 0

            def create_writers(_output_dir, names, _fps, **_kwargs):
                nonlocal calls
                calls += 1
                if calls == 1:
                    return {name: camera_writer for name in names}
                raise RuntimeError("preview init failed")

            with patch(
                "franka_gui.episode_spool._create_video_writers",
                side_effect=create_writers,
            ):
                with self.assertRaisesRegex(RuntimeError, "preview init failed"):
                    StreamingEpisodeSpool(
                        ["middle"],
                        30,
                        cache_root=Path(temp_dir),
                    )

            self.assertTrue(camera_writer.closed)
            recording_root = Path(temp_dir) / ".recording"
            self.assertEqual(list(recording_root.iterdir()), [])

    def test_discard_timeout_preserves_directory_until_writer_stops(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            camera_writer = _BlockingWriter()
            preview_writer = _CollectingWriter()

            def create_writers(_output_dir, names, _fps, **_kwargs):
                if names == ["middle"]:
                    return {"middle": camera_writer}
                return {"preview_all": preview_writer}

            with patch(
                "franka_gui.episode_spool._create_video_writers",
                side_effect=create_writers,
            ):
                spool = StreamingEpisodeSpool(
                    ["middle"],
                    30,
                    cache_root=Path(temp_dir),
                    close_timeout_sec=0.05,
                )
                spool.append({"middle": _rgb_frame(10, 8, 1)})
                self.assertTrue(camera_writer.entered.wait(timeout=1.0))

                with self.assertRaisesRegex(EpisodeSpoolError, "Timed out"):
                    spool.discard()
                self.assertTrue(spool.session_dir.is_dir())
                self.assertTrue(spool.writer_alive)

                camera_writer.release.set()
                spool.discard()
                self.assertFalse(spool.session_dir.exists())


def _rgb_frame(width: int, height: int, value: int) -> np.ndarray:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :, 0] = value * 11 % 255
    image[:, :, 1] = value * 23 % 255
    image[:, :, 2] = value * 37 % 255
    return image


def _video_shape_and_count(path: Path) -> tuple[int, int, int]:
    capture = cv2.VideoCapture(str(path))
    try:
        if not capture.isOpened():
            raise AssertionError(f"Could not open video: {path}")
        width = int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        height = int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        count = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame.shape[:2] != (height, width):
                raise AssertionError(f"Video frame shape changed in {path}: {frame.shape}")
            count += 1
        return width, height, count
    finally:
        capture.release()


class _CollectingWriter:
    def __init__(self) -> None:
        self.frames: list[np.ndarray] = []
        self.closed = False

    def append_data(self, rgb: np.ndarray) -> None:
        self.frames.append(np.array(rgb, copy=True))

    def close(self) -> None:
        self.closed = True


class _BlockingWriter(_CollectingWriter):
    def __init__(self) -> None:
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def append_data(self, rgb: np.ndarray) -> None:
        self.entered.set()
        if not self.release.wait(timeout=2.0):
            raise TimeoutError("test writer was not released")
        super().append_data(rgb)


if __name__ == "__main__":
    unittest.main()
