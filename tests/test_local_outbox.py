from __future__ import annotations

import gzip
import json
import os
import pickle
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

try:
    import PyQt6  # noqa: F401
except ImportError:
    PyQt6 = None

from franka_gui.episode_spool import StreamingEpisodeSpool


@unittest.skipIf(PyQt6 is None, "PyQt6 is not installed")
class LocalOutboxTest(unittest.TestCase):
    def test_action_only_episode_is_committed_to_outbox(self) -> None:
        from franka_gui.async_episode_saver import EpisodeSaveRequest, _save_episode

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            cache_root = root / "cache"
            nas_root = root / "nas"
            old_cache = os.environ.get("FRANKA_GUI_RECORD_CACHE_ROOT")
            os.environ["FRANKA_GUI_RECORD_CACHE_ROOT"] = str(cache_root)
            try:
                spool = StreamingEpisodeSpool(
                    ["left", "middle", "left_wrist"],
                    30,
                    cache_root=cache_root,
                )
                frames = []
                for index in range(6):
                    images = {
                        name: np.full((48, 64, 3), index * 5, dtype=np.uint8)
                        for name in spool.camera_names
                    }
                    spool.append(images)
                    frames.append(
                        {
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
                    )
                image_storage = spool.finish()
                request = EpisodeSaveRequest(
                    output_root=str(nas_root),
                    task="test_task",
                    quality="High_Quality",
                    index=0,
                    frames=frames,
                    keyframes=[0],
                    camera_names=list(spool.camera_names),
                    video_fps=30,
                    text_instruction="test instruction",
                    local_cache_dir=str(spool.episode_dir),
                    metadata={
                        "schema_version": "franka_single_v3",
                        "arm_side": "left",
                        "image_storage": image_storage,
                    },
                )
                local_episode, frame_count = _save_episode(request)

                self.assertEqual(frame_count, 6)
                self.assertTrue((local_episode.parent / "READY").is_file())
                manifest = json.loads(
                    (local_episode.parent / "outbox.json").read_text(encoding="utf-8")
                )
                self.assertEqual(manifest["task"], "test_task")
                self.assertEqual(manifest["quality"], "High_Quality")
                with gzip.open(local_episode / "0.pkl.gz", "rb") as handle:
                    payload = pickle.load(handle)
                self.assertEqual(len(payload["data"]), 6)
                self.assertFalse(
                    any(key.endswith("_image") for key in payload["data"][0])
                )
                metadata = json.loads(
                    (local_episode / "metadata.json").read_text(encoding="utf-8")
                )
                self.assertEqual(metadata["image_storage"]["type"], "video")
                self.assertEqual(metadata["relative_episode_dir"], "test_task/High_Quality/0")
            finally:
                if old_cache is None:
                    os.environ.pop("FRANKA_GUI_RECORD_CACHE_ROOT", None)
                else:
                    os.environ["FRANKA_GUI_RECORD_CACHE_ROOT"] = old_cache

    def test_direct_nas_episode_is_atomically_finalized(self) -> None:
        from franka_gui.async_episode_saver import EpisodeSaveRequest, _save_episode

        with tempfile.TemporaryDirectory() as temp_dir:
            nas_root = Path(temp_dir) / "nas"
            spool = StreamingEpisodeSpool(
                ["left", "middle", "left_wrist"],
                30,
                cache_root=nas_root,
            )
            frames = []
            for index in range(4):
                images = {
                    name: np.full((48, 64, 3), index * 10, dtype=np.uint8)
                    for name in spool.camera_names
                }
                spool.append(images)
                frames.append(
                    {
                        "schema_version": "franka_single_v3",
                        "frame_index": index,
                        "timestamp": 2000.0 + index / 30.0,
                        "joint": np.zeros(7),
                        "gripper_width": 0.08,
                    }
                )
            image_storage = spool.finish()
            request = EpisodeSaveRequest(
                output_root=str(nas_root),
                task="test_task",
                quality="High_Quality",
                index=0,
                frames=frames,
                keyframes=[0],
                camera_names=list(spool.camera_names),
                video_fps=30,
                local_cache_dir=str(spool.episode_dir),
                direct_to_output_root=True,
                metadata={
                    "schema_version": "franka_single_v3",
                    "image_storage": image_storage,
                },
            )
            final_episode, frame_count = _save_episode(request)

            self.assertEqual(frame_count, 4)
            self.assertEqual(final_episode, nas_root / "test_task" / "High_Quality" / "0")
            self.assertTrue((final_episode / "0.pkl.gz").is_file())
            self.assertFalse(any(nas_root.rglob(".partial-*")))
            self.assertFalse(any((nas_root / ".recording").iterdir()))
            metadata = json.loads((final_episode / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["storage_state"], "direct_nas")
            with gzip.open(final_episode / "0.pkl.gz", "rb") as handle:
                payload = pickle.load(handle)
            self.assertFalse(any(key.endswith("_image") for key in payload["data"][0]))

    def test_direct_nas_retry_publishes_preserved_partial(self) -> None:
        from franka_gui import async_episode_saver

        with tempfile.TemporaryDirectory() as temp_dir:
            nas_root = Path(temp_dir) / "nas"
            partial = nas_root / "test_task" / "High_Quality" / ".partial-0-retry"
            partial.mkdir(parents=True)
            (partial / "left.mp4").write_bytes(b"already-written-video")
            request = async_episode_saver.EpisodeSaveRequest(
                output_root=str(nas_root),
                task="test_task",
                quality="High_Quality",
                index=0,
                frames=[{"frame_index": 0, "timestamp": 1.0}],
                keyframes=[0],
                camera_names=["left"],
                video_fps=30,
                local_cache_dir=str(partial),
                direct_to_output_root=True,
                metadata={"schema_version": "franka_single_v3"},
            )
            with mock.patch.object(
                async_episode_saver,
                "_validate_staged_episode",
                return_value=async_episode_saver.EpisodeValidationResult("PASS", []),
            ):
                final_episode, _ = async_episode_saver._save_episode(request)

            self.assertEqual(final_episode, nas_root / "test_task" / "High_Quality" / "0")
            self.assertTrue((final_episode / "left.mp4").is_file())
            self.assertFalse(partial.exists())

    def test_local_fsync_failure_preserves_recording_and_never_creates_ready(self) -> None:
        from franka_gui import async_episode_saver

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            cache_root = root / "cache"
            episode_dir = cache_root / ".recording" / ("a" * 32) / "episode"
            episode_dir.mkdir(parents=True)
            (episode_dir / "left.mp4").write_bytes(b"local-video")
            old_cache = os.environ.get("FRANKA_GUI_RECORD_CACHE_ROOT")
            os.environ["FRANKA_GUI_RECORD_CACHE_ROOT"] = str(cache_root)
            request = async_episode_saver.EpisodeSaveRequest(
                output_root=str(root / "nas"),
                task="test_task",
                quality="High_Quality",
                index=0,
                frames=[{"frame_index": 0, "timestamp": 1.0}],
                keyframes=[0],
                camera_names=["left"],
                video_fps=30,
                local_cache_dir=str(episode_dir),
                metadata={"schema_version": "franka_single_v3"},
            )
            try:
                with mock.patch.object(
                    async_episode_saver,
                    "_validate_staged_episode",
                    return_value=async_episode_saver.EpisodeValidationResult("PASS", []),
                ), mock.patch.object(
                    async_episode_saver,
                    "_fsync_regular_tree",
                    side_effect=OSError("simulated local disk failure"),
                ):
                    with self.assertRaises(async_episode_saver.EpisodeSaveError):
                        async_episode_saver._save_episode(request)
                self.assertTrue(episode_dir.is_dir())
                self.assertFalse((cache_root / "outbox").exists())
                self.assertFalse(any(cache_root.rglob("READY")))
            finally:
                if old_cache is None:
                    os.environ.pop("FRANKA_GUI_RECORD_CACHE_ROOT", None)
                else:
                    os.environ["FRANKA_GUI_RECORD_CACHE_ROOT"] = old_cache

    def test_validation_failure_discards_staging_before_publish(self) -> None:
        from franka_gui import async_episode_saver

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            cache_root = root / "cache"
            session_dir = cache_root / ".recording" / ("b" * 32)
            episode_dir = session_dir / "episode"
            episode_dir.mkdir(parents=True)
            (episode_dir / "left.mp4").write_bytes(b"staged-video")
            request = async_episode_saver.EpisodeSaveRequest(
                output_root=str(root / "nas"),
                task="test_task",
                quality="High_Quality",
                index=0,
                frames=[{"frame_index": 0, "timestamp": 1.0}],
                keyframes=[0],
                camera_names=["left"],
                video_fps=30,
                local_cache_dir=str(episode_dir),
                metadata={"schema_version": "franka_single_v3"},
            )
            old_cache = os.environ.get("FRANKA_GUI_RECORD_CACHE_ROOT")
            os.environ["FRANKA_GUI_RECORD_CACHE_ROOT"] = str(cache_root)
            events = []
            try:
                validation = async_episode_saver.EpisodeValidationResult(
                    "FAIL", ["video_frame_count: expected 1, got 0"]
                )
                with mock.patch.object(
                    async_episode_saver,
                    "_validate_staged_episode",
                    return_value=validation,
                ), self.assertRaises(async_episode_saver.EpisodeSaveError) as raised:
                    async_episode_saver._save_episode(
                        request,
                        validation_started=lambda *args: events.append(("start", args)),
                        validation_finished=lambda *args: events.append(("finish", args)),
                    )
                self.assertEqual(
                    raised.exception.kind,
                    async_episode_saver.SAVE_ERROR_VALIDATION_FAILED,
                )
                self.assertFalse(session_dir.exists())
                self.assertFalse((root / "nas" / "test_task").exists())
                self.assertEqual(events[0][0], "start")
                self.assertEqual(events[1][1][2], "FAIL")
            finally:
                if old_cache is None:
                    os.environ.pop("FRANKA_GUI_RECORD_CACHE_ROOT", None)
                else:
                    os.environ["FRANKA_GUI_RECORD_CACHE_ROOT"] = old_cache

    def test_validation_warning_is_published_and_reported(self) -> None:
        from franka_gui import async_episode_saver

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            cache_root = root / "cache"
            session_dir = cache_root / ".recording" / ("c" * 32)
            episode_dir = session_dir / "episode"
            episode_dir.mkdir(parents=True)
            (episode_dir / "left.mp4").write_bytes(b"staged-video")
            request = async_episode_saver.EpisodeSaveRequest(
                output_root=str(root / "nas"),
                task="test_task",
                quality="Low_Quality",
                index=0,
                frames=[{"frame_index": 0, "timestamp": 1.0}],
                keyframes=[0],
                camera_names=["left"],
                video_fps=30,
                local_cache_dir=str(episode_dir),
                metadata={"schema_version": "franka_single_v3"},
            )
            old_cache = os.environ.get("FRANKA_GUI_RECORD_CACHE_ROOT")
            os.environ["FRANKA_GUI_RECORD_CACHE_ROOT"] = str(cache_root)
            events = []
            try:
                validation = async_episode_saver.EpisodeValidationResult(
                    "WARN", ["joint_delta_p95: p95 exceeds warning threshold"]
                )
                with mock.patch.object(
                    async_episode_saver,
                    "_validate_staged_episode",
                    return_value=validation,
                ):
                    saved, _ = async_episode_saver._save_episode(
                        request,
                        validation_started=lambda *args: events.append(("start", args)),
                        validation_finished=lambda *args: events.append(("finish", args)),
                    )
                self.assertTrue(saved.is_dir())
                self.assertEqual(events[1][1][2], "WARN")
                self.assertIn("joint_delta_p95", events[1][1][3][0])
            finally:
                if old_cache is None:
                    os.environ.pop("FRANKA_GUI_RECORD_CACHE_ROOT", None)
                else:
                    os.environ["FRANKA_GUI_RECORD_CACHE_ROOT"] = old_cache


if __name__ == "__main__":
    unittest.main()
