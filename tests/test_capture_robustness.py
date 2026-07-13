from __future__ import annotations

import json
import os
import shutil
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

try:
    import PyQt6  # noqa: F401
    import zmq  # noqa: F401

    from franka_gui.capture_controller import (
        CACHE_ROOT_ENV,
        HIGH_QUALITY_DIR,
        CaptureController,
        CaptureOptions,
        CaptureThread,
        _LatestFrameMailbox,
        _RobotStateSampler,
    )
except ImportError as exc:
    raise unittest.SkipTest(f"capture GUI dependencies are unavailable: {exc}") from exc


class CaptureRobustnessTest(unittest.TestCase):
    def test_robot_snapshot_rejects_latest_error_and_stale_state(self) -> None:
        sampler = _RobotStateSampler(
            "127.0.0.1",
            6001,
            100,
            "test",
            max_age_ms=50.0,
        )
        sampler._latest_state = {
            "robot_state_sample_monotonic": time.monotonic(),
            "robot_state_valid": False,
        }
        sampler._latest_error = "latest read failed"
        with self.assertRaisesRegex(RuntimeError, "latest read failed"):
            sampler.snapshot()

        sampler._latest_error = None
        sampler._latest_state["robot_state_sample_monotonic"] = time.monotonic() - 0.2
        with self.assertRaisesRegex(TimeoutError, "state is stale"):
            sampler.snapshot()

        sampler._latest_state["robot_state_sample_monotonic"] = time.monotonic()
        state = sampler.snapshot()
        self.assertTrue(state["robot_state_valid"])
        self.assertEqual(state["robot_sampler_error"], "")
        self.assertLessEqual(state["robot_state_age_ms"], 50.0)

    def test_preview_mailbox_keeps_only_latest_batch(self) -> None:
        mailbox = _LatestFrameMailbox()
        first = np.zeros((2, 3, 3), dtype=np.uint8)
        latest = np.ones((2, 3, 3), dtype=np.uint8)
        mailbox.publish({"middle": first})
        mailbox.publish({"middle": latest})

        frames = mailbox.take()
        self.assertIsNotNone(frames)
        self.assertIs(frames["middle"], latest)
        self.assertIsNone(mailbox.take())

    def test_boundary_commands_mark_current_camera_batch_for_skip(self) -> None:
        thread = CaptureThread(
            CaptureOptions(
                output_root="/tmp/unused",
                mock=True,
                camera_names=["middle"],
            )
        )
        try:
            with patch.object(thread, "_start_or_resume") as start, \
                    patch.object(thread, "_pause") as pause, \
                    patch.object(thread, "_finish") as finish:
                thread.enqueue("start")
                self.assertTrue(thread._drain_commands(["middle"], {"middle": {}}))
                start.assert_called_once()

                thread.enqueue("pause")
                self.assertTrue(thread._drain_commands(["middle"], {"middle": {}}))
                pause.assert_called_once()

                thread.enqueue("finish")
                self.assertTrue(thread._drain_commands(["middle"], {"middle": {}}))
                finish.assert_called_once()

            with patch.object(thread, "_add_keyframe") as keyframe:
                thread.enqueue("keyframe")
                self.assertFalse(thread._drain_commands(["middle"], {"middle": {}}))
                keyframe.assert_called_once()
        finally:
            thread._dual_read_executor.shutdown(wait=False)

    def test_camera_set_and_metadata_stay_frozen_after_dropout(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            with patch.dict(os.environ, {CACHE_ROOT_ENV: temp_dir}):
                thread = CaptureThread(
                    CaptureOptions(
                        output_root=temp_dir,
                        mock=True,
                        camera_names=["left", "middle"],
                    )
                )
                payload = {
                    "output_root": temp_dir,
                    "task": "test_task",
                    "task_description": "test instruction",
                    "user_metadata": {},
                    "index": 0,
                    "video_fps": 30,
                }
                try:
                    thread._start_or_resume(
                        payload,
                        ["left"],
                        {"left": {"serial": "L"}},
                    )
                    self.assertIsNone(thread._active)

                    live_metadata = {
                        "left": {"serial": "L", "nested": {"value": 1}},
                        "middle": {"serial": "M"},
                    }
                    thread._start_or_resume(
                        payload,
                        ["left", "middle"],
                        live_metadata,
                    )
                    active = thread._active
                    self.assertIsNotNone(active)
                    self.assertEqual(active.camera_names, ("left", "middle"))

                    images = {
                        "left": np.full((8, 12, 3), 11, dtype=np.uint8),
                        "middle": np.full((8, 12, 3), 22, dtype=np.uint8),
                    }
                    active.spool.append(images)
                    active.frames.append({"frame_index": 0, "timestamp": 1.0})

                    live_metadata["left"]["nested"]["value"] = 99
                    live_metadata.pop("middle")
                    self.assertTrue(
                        thread._pause_for_missing_episode_cameras({"left": images["left"]})
                    )
                    self.assertFalse(thread._recording)
                    self.assertEqual(active.camera_names, ("left", "middle"))
                    self.assertEqual(active.camera_metadata["left"]["nested"]["value"], 1)

                    requests = []
                    thread.episode_ready.connect(requests.append)
                    thread._finish()
                    self.assertEqual(len(requests), 1)
                    request = requests[0]
                    self.assertEqual(request.camera_names, ["left", "middle"])
                    self.assertEqual(
                        set(request.metadata["image_storage"]["cameras"]),
                        {"left", "middle"},
                    )
                    self.assertEqual(
                        set(request.metadata["cameras"]),
                        {"left", "middle"},
                    )
                    self.assertEqual(
                        request.metadata["cameras"]["left"]["nested"]["value"],
                        1,
                    )
                    self.assertEqual(len(request.frames), 1)
                    shutil.rmtree(Path(request.local_cache_dir).parent)
                finally:
                    if thread._active is not None:
                        thread._active.spool.discard()
                    thread._dual_read_executor.shutdown(wait=False)

    def test_outbox_indices_are_reserved_when_saving_directory_is_absent(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            cache_root = Path(temp_dir)
            session = cache_root / "outbox" / "abc123"
            session.mkdir(parents=True)
            (session / "READY").write_text("abc123\n", encoding="utf-8")
            (session / "outbox.json").write_text(
                json.dumps(
                    {
                        "task": "test_task",
                        "quality": HIGH_QUALITY_DIR,
                        "requested_index": 5,
                    }
                ),
                encoding="utf-8",
            )
            self.assertFalse((cache_root / ".saving").exists())

            with patch.dict(os.environ, {CACHE_ROOT_ENV: temp_dir}):
                controller = CaptureController(
                    CaptureOptions(
                        output_root=temp_dir,
                        mock=True,
                        camera_names=["middle"],
                    )
                )
                try:
                    self.assertEqual(controller.stale_cache_count(), 1)
                    self.assertEqual(
                        controller.peek_next_episode_index("test_task", HIGH_QUALITY_DIR),
                        6,
                    )
                finally:
                    controller.saver.shutdown()
                    controller._remove_activity_marker()

    def test_new_episode_is_blocked_at_local_save_backlog_limit(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            with patch.dict(
                os.environ,
                {
                    CACHE_ROOT_ENV: temp_dir,
                    "FRANKA_GUI_MAX_LOCAL_SAVE_BACKLOG": "2",
                },
            ):
                controller = CaptureController(
                    CaptureOptions(
                        output_root=temp_dir,
                        mock=True,
                        camera_names=["middle"],
                    )
                )
                errors = []
                controller.error.connect(errors.append)
                try:
                    with patch.object(
                        controller,
                        "inflight_save_count",
                        return_value=2,
                    ):
                        self.assertFalse(controller._has_local_capture_capacity())
                    self.assertTrue(any("已达到上限 2" in error for error in errors))
                finally:
                    controller.saver.shutdown()
                    controller._remove_activity_marker()


if __name__ == "__main__":
    unittest.main()
