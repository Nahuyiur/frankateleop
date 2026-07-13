from __future__ import annotations

import hashlib
import gzip
import json
import os
import pickle
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import franka_sync.nas_sync as nas_sync_module
from franka_sync import (
    NasSync,
    SyncConfig,
    SyncError,
    VerificationError,
    ensure_sync_daemon,
    is_real_mount,
)
from franka_sync.nas_sync import (
    InstanceLockError,
    NasDirectoryLock,
    RateLimiter,
    main,
    verify_published,
)


def _mp4_box(box_type: bytes, payload: bytes) -> bytes:
    return (8 + len(payload)).to_bytes(4, "big") + box_type + payload


def _minimal_mp4(payload: bytes = b"video-bytes") -> bytes:
    return b"".join(
        (
            _mp4_box(b"ftyp", b"isom\x00\x00\x02\x00isom"),
            _mp4_box(b"moov", b"metadata"),
            _mp4_box(b"mdat", payload),
        )
    )


class NasSyncTest(unittest.TestCase):
    def setUp(self) -> None:
        process_patch = mock.patch.object(
            nas_sync_module,
            "active_heavy_process",
            return_value=None,
        )
        process_patch.start()
        self.addCleanup(process_patch.stop)
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.cache = self.root / "cache"
        self.nas = self.root / "nas"
        self.cache.mkdir()
        self.nas.mkdir()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def config(self, **changes) -> SyncConfig:
        values = {
            "cache_root": self.cache,
            "nas_root": self.nas,
            "activity_fresh_seconds": 120.0,
            "max_load_per_cpu": 0.0,
            "min_available_memory_bytes": 0,
            "rate_limit_bytes_per_second": 0.0,
            "chunk_size": 7,
            "poll_seconds": 0.01,
            "retry_seconds": 0.01,
            "nas_lock_timeout_seconds": 0.1,
            "nas_lock_stale_seconds": 60.0,
            "skip_mount_check": True,
        }
        values.update(changes)
        return SyncConfig(**values)

    def make_outbox(
        self,
        outbox_uuid: str = "episode-a",
        *,
        task: str = "pick_block",
        quality: str = "High_Quality",
        requested_index: int = 7,
    ) -> Path:
        entry = self.cache / "outbox" / outbox_uuid
        episode = entry / "episode"
        episode.mkdir(parents=True)
        with gzip.open(episode / f"{requested_index}.pkl.gz", "wb") as handle:
            pickle.dump(
                {"data": [{"frame_index": index} for index in range(12)], "keyframes": [0]},
                handle,
                protocol=pickle.HIGHEST_PROTOCOL,
            )
        (episode / "wrist.mp4").write_bytes(_minimal_mp4())
        (episode / "keyframes.json").write_text('{"keyframes": [0]}\n', encoding="utf-8")
        (episode / "instruction.txt").write_text("pick the block\n", encoding="utf-8")
        metadata = {
            "task": task,
            "quality": quality,
            "index": requested_index,
            "frame_count": 12,
            "camera_names": ["wrist"],
            "relative_episode_dir": f"{task}/{quality}/{requested_index}",
            "episode_id": f"{task}/{quality}/{requested_index}",
            "storage_state": "local_outbox",
        }
        (episode / "metadata.json").write_text(
            json.dumps(metadata), encoding="utf-8"
        )
        manifest = {
            "output_root": str(self.nas),
            "task": task,
            "quality": quality,
            "requested_index": requested_index,
            "frame_count": 12,
            "camera_names": ["wrist"],
            "episode_subdir": "episode",
        }
        (entry / "outbox.json").write_text(
            json.dumps(manifest), encoding="utf-8"
        )
        (entry / "READY").write_text(outbox_uuid, encoding="utf-8")
        return entry

    def test_publish_allocates_nas_index_and_rewrites_indexed_files(self) -> None:
        entry = self.make_outbox()
        original_video = (entry / "episode" / "wrist.mp4").read_bytes()
        existing = self.nas / "pick_block" / "High_Quality" / "0"
        existing.mkdir(parents=True)
        (existing / "existing.txt").write_text("keep", encoding="utf-8")

        with NasSync(self.config()) as sync:
            result = sync.sync_once()

        self.assertEqual(result.status, "synced")
        final = self.nas / "pick_block" / "High_Quality" / "1"
        self.assertEqual(result.final_dir, final)
        self.assertFalse(entry.exists())
        with gzip.open(final / "1.pkl.gz", "rb") as handle:
            trajectory = pickle.load(handle)
        self.assertEqual(len(trajectory["data"]), 12)
        self.assertFalse((final / "7.pkl.gz").exists())
        self.assertEqual((final / "wrist.mp4").read_bytes(), original_video)
        metadata = json.loads((final / "metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["index"], 1)
        self.assertEqual(metadata["relative_episode_dir"], "pick_block/High_Quality/1")
        self.assertEqual(metadata["episode_id"], "pick_block/High_Quality/1")
        self.assertEqual(metadata["storage_state"], "nas_committed")

        marker = json.loads((final / ".franka-sync.json").read_text(encoding="utf-8"))
        self.assertEqual(marker["outbox_uuid"], "episode-a")
        self.assertEqual(marker["final_index"], 1)
        for record in marker["files"]:
            payload = (final / record["path"]).read_bytes()
            self.assertEqual(len(payload), record["size"])
            self.assertEqual(hashlib.sha256(payload).hexdigest(), record["sha256"])
        self.assertEqual((existing / "existing.txt").read_text(encoding="utf-8"), "keep")

    def test_fresh_capture_marker_defers_without_touching_nas(self) -> None:
        entry = self.make_outbox()
        activity = self.cache / ".capture-active"
        activity.mkdir()
        (activity / "gui.json").write_text("{}", encoding="utf-8")

        with mock.patch("franka_sync.nas_sync.validate_outbox_payload") as validate:
            with NasSync(
                self.config(),
                wall_time=lambda: os.path.getmtime(activity / "gui.json"),
            ) as sync:
                result = sync.sync_once()

        self.assertEqual(result.status, "deferred")
        self.assertIn("capture activity marker", result.reason)
        validate.assert_not_called()
        self.assertTrue(entry.exists())
        self.assertFalse((self.nas / "pick_block").exists())

    def test_idle_grace_remains_after_observed_marker_is_deleted(self) -> None:
        entry = self.make_outbox(requested_index=0)
        activity = self.cache / ".capture-active"
        activity.mkdir()
        marker = activity / "gui.json"
        marker.write_text("{}", encoding="utf-8")
        os.utime(marker, (1000.0, 1000.0))
        now = [1000.0]

        with NasSync(self.config(), wall_time=lambda: now[0]) as sync:
            self.assertEqual(sync.sync_once().status, "deferred")
            marker.unlink()
            now[0] = 1050.0
            deferred = sync.sync_once()
            self.assertEqual(deferred.status, "deferred")
            self.assertIn("idle grace", deferred.reason)
            now[0] = 1121.0
            result = sync.sync_once()

        self.assertEqual(result.status, "synced")
        self.assertFalse(entry.exists())

    def test_mount_check_is_injectable_and_preserves_local_outbox(self) -> None:
        entry = self.make_outbox()
        config = self.config(skip_mount_check=False)
        with mock.patch("franka_sync.nas_sync.validate_outbox_payload") as validate:
            with NasSync(config, mount_checker=lambda _: False) as sync:
                result = sync.sync_once()
        self.assertEqual(result.status, "deferred")
        self.assertIn("NAS mount identity is not exactly", result.reason)
        self.assertIn("fs='cifs'", result.reason)
        validate.assert_not_called()
        self.assertTrue(entry.exists())

    def test_mount_identity_change_before_atomic_publish_preserves_partial(self) -> None:
        entry = self.make_outbox(outbox_uuid="mount-before-rename", requested_index=0)
        checks = [0]

        def mount_checker(_path: Path) -> bool:
            checks[0] += 1
            return checks[0] < 5

        with NasSync(
            self.config(skip_mount_check=False),
            mount_checker=mount_checker,
        ) as sync:
            result = sync.sync_once()

        self.assertEqual(result.status, "deferred")
        self.assertIn("immediately before atomic NAS publication", result.reason)
        self.assertTrue(entry.exists())
        quality = self.nas / "pick_block" / "High_Quality"
        self.assertEqual(len(list(quality.glob(".partial-*"))), 1)
        self.assertFalse(any(path.name.isdigit() for path in quality.iterdir()))

    def test_mount_identity_change_before_local_delete_retries_without_duplicate(self) -> None:
        entry = self.make_outbox(outbox_uuid="mount-before-delete", requested_index=0)
        checks = [0]
        fail_on_check = [7]

        def mount_checker(_path: Path) -> bool:
            checks[0] += 1
            return checks[0] != fail_on_check[0]

        with NasSync(
            self.config(skip_mount_check=False),
            mount_checker=mount_checker,
        ) as sync:
            first = sync.sync_once()
            self.assertEqual(first.status, "deferred")
            self.assertIn("local deletion verification", first.reason)
            self.assertTrue(entry.exists())
            quality = self.nas / "pick_block" / "High_Quality"
            self.assertTrue((quality / "0").is_dir())

            fail_on_check[0] = -1
            second = sync.sync_once()

        self.assertEqual(second.status, "synced")
        self.assertFalse(entry.exists())
        self.assertEqual(
            sorted(path.name for path in quality.iterdir() if path.name.isdigit()),
            ["0"],
        )

    def test_resource_heavy_process_defers_sync(self) -> None:
        entry = self.make_outbox()
        with NasSync(
            self.config(),
            process_activity_getter=lambda: "python right_wrist_custom_demo_play.py",
        ) as sync:
            result = sync.sync_once()
        self.assertEqual(result.status, "deferred")
        self.assertIn("right_wrist_custom_demo_play.py", result.reason)
        self.assertTrue(entry.exists())

    def test_capture_resuming_mid_copy_preserves_local_outbox(self) -> None:
        entry = self.make_outbox(outbox_uuid="mid-copy")
        calls = [0]

        def process_activity() -> str | None:
            calls[0] += 1
            return "python record_fr3.py" if calls[0] >= 3 else None

        with NasSync(
            self.config(chunk_size=7),
            process_activity_getter=process_activity,
        ) as sync:
            result = sync.sync_once()
            self.assertEqual(result.status, "deferred")
            self.assertIn("record_fr3.py", result.reason)
            self.assertTrue(entry.exists())
            quality = self.nas / "pick_block" / "High_Quality"
            partials = list(quality.glob(".partial-*"))
            self.assertEqual(len(partials), 1)
            self.assertFalse(any(path.name.isdigit() for path in quality.iterdir()))

            matches: list[bool] = []
            original_matches = nas_sync_module._file_matches

            def track_match(path, prepared_file, chunk_size):
                matched = original_matches(path, prepared_file, chunk_size)
                matches.append(matched)
                return matched

            sync.process_activity_getter = lambda: None
            with mock.patch(
                "franka_sync.nas_sync._file_matches",
                side_effect=track_match,
            ):
                resumed = sync.sync_once()

        self.assertEqual(resumed.status, "synced")
        self.assertTrue(matches)
        self.assertTrue(all(matches))
        self.assertFalse(entry.exists())

    def test_dry_run_does_not_create_nas_destination_or_delete_local(self) -> None:
        entry = self.make_outbox()
        with NasSync(self.config()) as sync:
            result = sync.sync_once(dry_run=True)
        self.assertEqual(result.status, "dry_run")
        self.assertEqual(
            result.final_dir,
            self.nas / "pick_block" / "High_Quality" / "0",
        )
        self.assertTrue(entry.exists())
        self.assertEqual(list(self.nas.iterdir()), [])

    def test_incomplete_episode_is_never_published_or_deleted(self) -> None:
        entry = self.make_outbox(outbox_uuid="empty-payload")
        episode = entry / "episode"
        for child in list(episode.iterdir()):
            child.unlink()
        with NasSync(self.config()) as sync:
            result = sync.sync_once()
        self.assertEqual(result.status, "deferred")
        self.assertTrue(entry.exists())
        self.assertEqual(list(self.nas.iterdir()), [])

    def test_invalid_oldest_outbox_does_not_block_next_valid_episode(self) -> None:
        invalid = self.make_outbox(outbox_uuid="a-invalid")
        (invalid / "episode" / "metadata.json").unlink()
        valid = self.make_outbox(outbox_uuid="b-valid")
        with NasSync(self.config()) as sync:
            result = sync.sync_once()
        self.assertEqual(result.status, "synced")
        self.assertTrue(invalid.exists())
        self.assertFalse(valid.exists())

    def test_corrupt_and_missing_queue_heads_do_not_block_valid_episode(self) -> None:
        corrupt = self.make_outbox(outbox_uuid="a-corrupt")
        (corrupt / "episode" / "7.pkl.gz").write_bytes(
            gzip.compress(b"not-a-pickle")
        )
        missing = self.make_outbox(outbox_uuid="b-missing")
        (missing / "episode" / "wrist.mp4").unlink()
        valid = self.make_outbox(outbox_uuid="c-valid")

        with NasSync(self.config()) as sync:
            result = sync.sync_once()

        self.assertEqual(result.status, "synced")
        self.assertTrue(corrupt.exists())
        self.assertTrue(missing.exists())
        self.assertFalse(valid.exists())
        numeric = list((self.nas / "pick_block" / "High_Quality").glob("[0-9]*"))
        self.assertEqual([path.name for path in numeric], ["0"])

    def test_duplicate_sync_is_idempotent(self) -> None:
        self.make_outbox(outbox_uuid="idempotent", requested_index=0)
        with NasSync(self.config()) as sync:
            first = sync.sync_once()
            second = sync.sync_once()

        self.assertEqual(first.status, "synced")
        self.assertEqual(second.status, "no_work")
        quality = self.nas / "pick_block" / "High_Quality"
        self.assertEqual(
            sorted(path.name for path in quality.iterdir() if path.name.isdigit()),
            ["0"],
        )

    def test_local_payload_change_before_delete_is_preserved(self) -> None:
        entry = self.make_outbox(outbox_uuid="changed-before-delete", requested_index=0)
        changed = [False]

        def mutate_after_final(root, prepared, manifest, final_index, chunk_size):
            verify_published(root, prepared, manifest, final_index, chunk_size)
            if Path(root).name.isdigit() and not changed[0]:
                changed[0] = True
                video = entry / "episode" / "wrist.mp4"
                video.write_bytes(video.read_bytes() + b"corrupt-after-copy")

        with NasSync(self.config()) as sync:
            with mock.patch(
                "franka_sync.nas_sync.verify_published",
                side_effect=mutate_after_final,
            ):
                result = sync.sync_once()

        self.assertEqual(result.status, "deferred")
        self.assertIn("all READY outboxes are invalid", result.reason)
        self.assertTrue(entry.exists())
        self.assertTrue((self.nas / "pick_block" / "High_Quality" / "0").is_dir())

    def test_failed_local_delete_recovers_from_verified_tombstone(self) -> None:
        entry = self.make_outbox(outbox_uuid="delete-recovery", requested_index=0)
        tombstone_root = self.cache / ".synced-delete"

        with NasSync(self.config()) as sync:
            with mock.patch(
                "franka_sync.nas_sync.shutil.rmtree",
                side_effect=OSError("injected local delete failure"),
            ):
                result = sync.sync_once()

            tombstone = tombstone_root / "delete-recovery"
            receipt = tombstone_root / ".delete-recovery.receipt.json"
            self.assertEqual(result.status, "synced")
            self.assertFalse(entry.exists())
            self.assertTrue(tombstone.is_dir())
            self.assertTrue(receipt.is_file())

            removed = sync.cleanup_synced_tombstones()

        self.assertEqual(removed, 1)
        self.assertFalse(tombstone.exists())
        self.assertFalse(receipt.exists())

    def test_failed_post_rename_verification_keeps_local_and_retry_is_idempotent(self) -> None:
        entry = self.make_outbox(requested_index=0)

        def fail_only_final(root, prepared, manifest, final_index, chunk_size):
            verify_published(root, prepared, manifest, final_index, chunk_size)
            if Path(root).name.isdigit():
                raise VerificationError("injected final reread failure")

        with NasSync(self.config()) as sync:
            with mock.patch(
                "franka_sync.nas_sync.verify_published",
                side_effect=fail_only_final,
            ):
                with self.assertRaises(VerificationError):
                    sync.sync_once()
            self.assertTrue(entry.exists())
            final = self.nas / "pick_block" / "High_Quality" / "0"
            self.assertTrue(final.is_dir())

            result = sync.sync_once()

        self.assertEqual(result.status, "synced")
        self.assertEqual(result.final_dir, final)
        self.assertFalse(entry.exists())
        numeric = [path.name for path in final.parent.iterdir() if path.name.isdigit()]
        self.assertEqual(numeric, ["0"])

    def test_corrupt_published_marker_fails_closed_without_duplicate_index(self) -> None:
        entry = self.make_outbox(outbox_uuid="marker-corrupt", requested_index=0)

        def fail_final(root, prepared, manifest, final_index, chunk_size):
            verify_published(root, prepared, manifest, final_index, chunk_size)
            if Path(root).name.isdigit():
                raise VerificationError("keep local for retry")

        with NasSync(self.config()) as sync:
            with mock.patch("franka_sync.nas_sync.verify_published", side_effect=fail_final):
                with self.assertRaises(VerificationError):
                    sync.sync_once()
            final = self.nas / "pick_block" / "High_Quality" / "0"
            (final / ".franka-sync.json").write_text("not-json", encoding="utf-8")
            with self.assertRaises(VerificationError):
                sync.sync_once()

        self.assertTrue(entry.exists())
        numeric = sorted(path.name for path in final.parent.iterdir() if path.name.isdigit())
        self.assertEqual(numeric, ["0"])

    def test_local_flock_rejects_second_instance(self) -> None:
        first = NasSync(self.config())
        second = NasSync(self.config())
        try:
            first.instance_lock.acquire()
            with self.assertRaises(InstanceLockError):
                second.sync_once()
        finally:
            second.instance_lock.release()
            first.instance_lock.release()

    def test_stale_nas_lock_never_reaps_a_live_owner(self) -> None:
        lock_path = self.nas / ".lock"
        lock_path.mkdir()
        owner = {
            "token": "live-token",
            "pid": os.getpid(),
            "host": "test-host",
            "created_at_unix": 1.0,
        }
        (lock_path / "owner.json").write_text(json.dumps(owner), encoding="utf-8")
        os.utime(lock_path, (1.0, 1.0))
        lock = NasDirectoryLock(
            lock_path,
            timeout_seconds=0.0,
            stale_seconds=10.0,
            monotonic=lambda: 0.0,
            wall_time=lambda: 100.0,
            owner_probe=lambda _owner: True,
        )

        with self.assertRaises(SyncError):
            lock.acquire()

        self.assertTrue(lock_path.is_dir())
        self.assertEqual(
            json.loads((lock_path / "owner.json").read_text(encoding="utf-8"))["token"],
            "live-token",
        )

    def test_stale_nas_lock_reaps_only_a_confirmed_dead_local_owner(self) -> None:
        lock_path = self.nas / ".lock"
        lock_path.mkdir()
        (lock_path / "owner.json").write_text(
            json.dumps(
                {
                    "token": "dead-token",
                    "pid": 999999,
                    "host": "test-host",
                    "created_at_unix": 1.0,
                }
            ),
            encoding="utf-8",
        )
        os.utime(lock_path, (1.0, 1.0))
        lock = NasDirectoryLock(
            lock_path,
            timeout_seconds=0.0,
            stale_seconds=10.0,
            monotonic=lambda: 0.0,
            wall_time=lambda: 100.0,
            owner_probe=lambda _owner: False,
        )

        lock.acquire()
        try:
            current = json.loads(
                (lock_path / "owner.json").read_text(encoding="utf-8")
            )
            self.assertEqual(current["token"], lock.token)
        finally:
            lock.release()
        self.assertFalse(lock_path.exists())

    def test_complete_legacy_episode_migrates_without_reencoding(self) -> None:
        complete = (
            self.cache
            / ".saving"
            / "save-uuid"
            / "pick_block"
            / "Low_Quality"
            / "5"
        )
        complete.mkdir(parents=True)
        original_pickle = gzip.compress(b"already-encoded-pickle")
        original_video = b"\x00\x00\x00\x18ftypisomalready-encoded-video"
        (complete / "5.pkl.gz").write_bytes(original_pickle)
        (complete / "wrist.mp4").write_bytes(original_video)
        (complete / "metadata.json").write_text(
            json.dumps(
                {
                    "task": "pick_block",
                    "quality": "Low_Quality",
                    "index": 5,
                    "frame_count": 9,
                    "camera_names": ["wrist"],
                }
            ),
            encoding="utf-8",
        )
        incomplete = (
            self.cache
            / ".saving"
            / "other-save"
            / "pick_block"
            / "High_Quality"
            / "6"
        )
        incomplete.mkdir(parents=True)
        (incomplete / "6.pkl.gz").write_bytes(b"pickle")
        (incomplete / "metadata.json").write_text(
            json.dumps(
                {
                    "task": "pick_block",
                    "quality": "High_Quality",
                    "index": 6,
                    "frame_count": 3,
                    "camera_names": ["wrist"],
                }
            ),
            encoding="utf-8",
        )

        with NasSync(self.config()) as sync:
            migrated = sync.migrate_legacy_saving()

        self.assertEqual(migrated, 1)
        self.assertFalse(complete.exists())
        self.assertTrue(incomplete.exists())
        outboxes = [
            path
            for path in (self.cache / "outbox").iterdir()
            if path.is_dir() and not path.name.startswith(".")
        ]
        self.assertEqual(len(outboxes), 1)
        migrated_episode = outboxes[0] / "episode"
        self.assertEqual((migrated_episode / "5.pkl.gz").read_bytes(), original_pickle)
        self.assertEqual((migrated_episode / "wrist.mp4").read_bytes(), original_video)
        manifest = json.loads((outboxes[0] / "outbox.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["requested_index"], 5)
        self.assertEqual(manifest["episode_subdir"], "episode")
        self.assertTrue((outboxes[0] / "READY").is_file())

    def test_ready_recording_crash_window_is_recovered_to_outbox(self) -> None:
        entry = self.make_outbox(outbox_uuid="recording-crash")
        recording_root = self.cache / ".recording"
        recording_root.mkdir()
        crashed = recording_root / entry.name
        entry.rename(crashed)

        with NasSync(self.config()) as sync:
            migrated = sync.migrate_ready_recordings()

        self.assertEqual(migrated, 1)
        self.assertFalse(crashed.exists())
        self.assertTrue((self.cache / "outbox" / "recording-crash" / "READY").is_file())

    def test_mountinfo_requires_exact_mount_point(self) -> None:
        mountinfo = self.root / "mountinfo"
        mountinfo.write_text(
            f"42 31 0:40 / {self.nas} rw,relatime - cifs //server/share rw\n",
            encoding="utf-8",
        )
        self.assertTrue(is_real_mount(self.nas, mountinfo))
        self.assertTrue(
            is_real_mount(self.nas, mountinfo, expected_source="//server/share")
        )
        self.assertFalse(
            is_real_mount(self.nas, mountinfo, expected_source="//other/share")
        )
        self.assertFalse(is_real_mount(self.nas / "child", mountinfo))
        self.assertFalse(
            is_real_mount(
                self.nas,
                mountinfo,
                expected_source="//server/share",
                expected_filesystem="smb3",
            )
        )
        mountinfo.write_text(
            f"42 31 0:40 / {self.nas} rw,relatime - smb3 //server/share rw\n",
            encoding="utf-8",
        )
        self.assertFalse(is_real_mount(self.nas, mountinfo))
        self.assertTrue(
            is_real_mount(
                self.nas,
                mountinfo,
                expected_source="//server/share",
                expected_filesystem="smb3",
            )
        )

    def test_rate_limiter_sleeps_to_enforce_average_rate(self) -> None:
        clock = [0.0]
        sleeps = []

        def sleep(seconds: float) -> None:
            sleeps.append(seconds)
            clock[0] += seconds

        limiter = RateLimiter(10.0, sleep=sleep, monotonic=lambda: clock[0])
        limiter.consume(10)
        limiter.consume(10)
        self.assertEqual(sleeps, [1.0, 1.0])

    def test_environment_defaults_to_120_second_idle_grace(self) -> None:
        config = SyncConfig.from_env(
            {
                "FRANKA_SYNC_CACHE_ROOT": str(self.cache),
                "FRANKA_SYNC_NAS_ROOT": str(self.nas),
            }
        )
        self.assertEqual(config.activity_fresh_seconds, 120.0)
        overridden = SyncConfig.from_env(
            {
                "FRANKA_SYNC_IDLE_GRACE_SECONDS": "180",
                "FRANKA_SYNC_EXPECTED_NAS_FS_TYPE": "smb3",
                "FRANKA_SYNC_CACHE_ROOT": str(self.cache),
                "FRANKA_SYNC_NAS_ROOT": str(self.nas),
            }
        )
        self.assertEqual(overridden.activity_fresh_seconds, 180.0)
        self.assertEqual(overridden.expected_nas_fs_type, "smb3")

    def test_cli_dry_run_does_not_migrate_legacy_saving(self) -> None:
        with mock.patch.object(NasSync, "migrate_legacy_saving") as migrate:
            result = main(
                [
                    "--once",
                    "--dry-run",
                    "--cache-root",
                    str(self.cache),
                    "--nas-root",
                    str(self.nas),
                    "--skip-mount-check",
                    "--max-load-per-cpu",
                    "0",
                    "--min-available-memory-mib",
                    "0",
                ]
            )
        self.assertEqual(result, 0)
        migrate.assert_not_called()


class LauncherTest(unittest.TestCase):
    def test_launcher_uses_current_python_detached_session_and_cache_log(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            repo = root / "repo"
            cache = root / "cache"
            nas = root / "nas"
            (repo / "franka_sync").mkdir(parents=True)
            (repo / "franka_sync" / "__main__.py").write_text("", encoding="utf-8")
            process = mock.Mock(pid=4321)
            with mock.patch(
                "franka_sync.launcher.subprocess.Popen", return_value=process
            ) as popen:
                pid = ensure_sync_daemon(repo, cache, nas)

            self.assertEqual(pid, 4321)
            command = popen.call_args.args[0]
            self.assertIn(os.sys.executable, command)
            self.assertIn("--watch", command)
            self.assertTrue(popen.call_args.kwargs["start_new_session"])
            self.assertEqual(popen.call_args.kwargs["cwd"], repo.absolute())
            self.assertTrue((cache / "sync.log").is_file())


if __name__ == "__main__":
    unittest.main()
