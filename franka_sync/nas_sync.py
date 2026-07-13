"""Publish durable local episode outboxes to NAS without using rsync."""

from __future__ import annotations

import argparse
import errno
import fcntl
import gzip
import hashlib
import json
import logging
import os
import pickletools
import re
import shutil
import socket
import stat
import sys
import time
import uuid
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Iterable, Mapping, Sequence, TextIO

LOGGER = logging.getLogger("franka_sync")

DEFAULT_CACHE_ROOT = Path.home() / "Desktop" / "franka_record_cache"
DEFAULT_NAS_ROOT = Path("/home/pnp/Desktop/Muka_NAS")
OUTBOX_DIR_NAME = "outbox"
READY_NAME = "READY"
MANIFEST_NAME = "outbox.json"
ACTIVITY_DIR_NAME = ".capture-active"
LOCAL_LOCK_NAME = ".nas-sync.lock"
NAS_LOCK_NAME = ".franka-sync-index.lock"
SYNC_MARKER_NAME = ".franka-sync.json"
TOMBSTONE_DIR_NAME = ".synced-delete"
TOMBSTONE_RECEIPT_SUFFIX = ".receipt.json"
PARTIAL_PATTERN = re.compile(r"^\.partial-(\d+)-(.+)$")
SAFE_TOKEN_PATTERN = re.compile(r"^[^/\\\x00]+$")
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
MIB = 1024 * 1024
RESOURCE_HEAVY_PROCESS_TOKENS = (
    "franka_capture.scripts.record",
    "record_fr3.py",
    "record_fr3_dual.py",
    "record_rgb_pointclouds.py",
    "custom_demo_play.py",
    "replay_fr3.py",
    "replay_fr3_dual.py",
    "validate.validate_task",
    "franka_lerobot",
    "franka_hdf5",
    "franka_downsample",
)


class SyncError(RuntimeError):
    """Base error for a retryable outbox synchronization failure."""


class SyncDeferred(SyncError):
    """Raised when capture or system pressure resumes during a copy."""


class ManifestError(SyncError):
    """Raised when an outbox manifest is missing, unsafe, or malformed."""


class VerificationError(SyncError):
    """Raised when copied bytes do not match their expected digest."""


class InstanceLockError(SyncError):
    """Raised when another local sync process already owns the flock."""


@dataclass(frozen=True)
class SyncConfig:
    """Runtime settings, normally populated from ``FRANKA_SYNC_*`` variables."""

    cache_root: Path = DEFAULT_CACHE_ROOT
    nas_root: Path = DEFAULT_NAS_ROOT
    expected_nas_source: str = "//192.168.1.119/Muka"
    expected_nas_fs_type: str = "cifs"
    activity_fresh_seconds: float = 120.0
    max_load_per_cpu: float = 0.5
    min_available_memory_bytes: int = 4096 * MIB
    rate_limit_bytes_per_second: float = 20.0 * MIB
    chunk_size: int = 4 * MIB
    poll_seconds: float = 5.0
    retry_seconds: float = 15.0
    nas_lock_timeout_seconds: float = 30.0
    nas_lock_stale_seconds: float = 0.0
    skip_mount_check: bool = False
    local_lock_path: Path | None = None

    def __post_init__(self) -> None:
        if not self.expected_nas_source.strip():
            raise ValueError("expected_nas_source must be non-empty")
        if not self.expected_nas_fs_type.strip():
            raise ValueError("expected_nas_fs_type must be non-empty")
        if self.activity_fresh_seconds < 0:
            raise ValueError("activity_fresh_seconds must be non-negative")
        if self.max_load_per_cpu < 0:
            raise ValueError("max_load_per_cpu must be non-negative")
        if self.min_available_memory_bytes < 0:
            raise ValueError("min_available_memory_bytes must be non-negative")
        if self.rate_limit_bytes_per_second < 0:
            raise ValueError("rate_limit_bytes_per_second must be non-negative")
        if self.chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        if self.poll_seconds <= 0 or self.retry_seconds <= 0:
            raise ValueError("poll and retry intervals must be positive")
        if self.nas_lock_timeout_seconds < 0 or self.nas_lock_stale_seconds < 0:
            raise ValueError("NAS lock intervals must be non-negative")

    @property
    def outbox_root(self) -> Path:
        return self.cache_root / OUTBOX_DIR_NAME

    @property
    def activity_root(self) -> Path:
        return self.cache_root / ACTIVITY_DIR_NAME

    @property
    def instance_lock_path(self) -> Path:
        return self.local_lock_path or (self.cache_root / LOCAL_LOCK_NAME)

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None) -> "SyncConfig":
        values = os.environ if env is None else env
        cache_text = values.get("FRANKA_SYNC_CACHE_ROOT", "").strip()
        if not cache_text:
            cache_text = values.get("FRANKA_GUI_RECORD_CACHE_ROOT", "").strip()
        cache_root = Path(cache_text).expanduser() if cache_text else DEFAULT_CACHE_ROOT
        nas_text = values.get("FRANKA_SYNC_NAS_ROOT", "").strip()
        nas_root = Path(nas_text).expanduser() if nas_text else DEFAULT_NAS_ROOT
        expected_nas_source = values.get(
            "FRANKA_SYNC_EXPECTED_NAS_SOURCE", "//192.168.1.119/Muka"
        ).strip()
        expected_nas_fs_type = values.get(
            "FRANKA_SYNC_EXPECTED_NAS_FS_TYPE", "cifs"
        ).strip()
        lock_text = values.get("FRANKA_SYNC_LOCAL_LOCK_PATH", "").strip()
        idle_grace = _env_float(
            values, "FRANKA_SYNC_IDLE_GRACE_SECONDS", 120.0
        )

        return cls(
            cache_root=cache_root,
            nas_root=nas_root,
            expected_nas_source=expected_nas_source,
            expected_nas_fs_type=expected_nas_fs_type,
            activity_fresh_seconds=_env_float(
                values, "FRANKA_SYNC_ACTIVITY_FRESH_SECONDS", idle_grace
            ),
            max_load_per_cpu=_env_float(
                values, "FRANKA_SYNC_MAX_LOAD_PER_CPU", 0.5
            ),
            min_available_memory_bytes=int(
                _env_float(values, "FRANKA_SYNC_MIN_AVAILABLE_MEMORY_MIB", 4096.0)
                * MIB
            ),
            rate_limit_bytes_per_second=_env_float(
                values, "FRANKA_SYNC_RATE_LIMIT_MIB_S", 20.0
            )
            * MIB,
            chunk_size=max(
                1,
                int(_env_float(values, "FRANKA_SYNC_CHUNK_SIZE_MIB", 4.0) * MIB),
            ),
            poll_seconds=_env_float(values, "FRANKA_SYNC_POLL_SECONDS", 5.0),
            retry_seconds=_env_float(values, "FRANKA_SYNC_RETRY_SECONDS", 15.0),
            nas_lock_timeout_seconds=_env_float(
                values, "FRANKA_SYNC_NAS_LOCK_TIMEOUT_SECONDS", 30.0
            ),
            nas_lock_stale_seconds=_env_float(
                values, "FRANKA_SYNC_NAS_LOCK_STALE_SECONDS", 0.0
            ),
            skip_mount_check=_env_bool(
                values, "FRANKA_SYNC_SKIP_MOUNT_CHECK", False
            ),
            local_lock_path=Path(lock_text).expanduser() if lock_text else None,
        )


@dataclass(frozen=True)
class OutboxManifest:
    entry_dir: Path
    outbox_uuid: str
    output_root: Path
    task: str
    quality: str
    requested_index: int
    frame_count: int
    camera_names: tuple[str, ...]
    episode_subdir: Path
    manifest_sha256: str

    @property
    def episode_dir(self) -> Path:
        return self.entry_dir / self.episode_subdir

    @classmethod
    def load(cls, entry_dir: Path, expected_nas_root: Path) -> "OutboxManifest":
        entry_dir = _absolute_path(entry_dir)
        if entry_dir.is_symlink() or not entry_dir.is_dir():
            raise ManifestError(f"Outbox entry is not a real directory: {entry_dir}")

        ready_path = entry_dir / READY_NAME
        manifest_path = entry_dir / MANIFEST_NAME
        if ready_path.is_symlink() or not ready_path.is_file():
            raise ManifestError(f"Outbox READY marker is missing: {ready_path}")
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise ManifestError(f"Outbox manifest is missing: {manifest_path}")

        try:
            ready_value = ready_path.read_text(encoding="utf-8").strip()
            manifest_bytes = manifest_path.read_bytes()
            payload = json.loads(manifest_bytes.decode("utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ManifestError(f"Cannot read outbox manifest {manifest_path}: {exc}") from exc
        if not isinstance(payload, dict):
            raise ManifestError(f"Outbox manifest must be a JSON object: {manifest_path}")

        outbox_uuid = _safe_token(entry_dir.name, "outbox UUID")
        if ready_value != outbox_uuid:
            raise ManifestError(
                f"Outbox READY marker does not match directory UUID: {ready_path}"
            )
        schema_version = payload.get("schema_version", 1)
        if schema_version != 1:
            raise ManifestError(
                f"Unsupported outbox manifest schema_version: {schema_version!r}"
            )
        episode_uuid = payload.get("episode_uuid")
        if episode_uuid is not None and episode_uuid != outbox_uuid:
            raise ManifestError("Manifest episode_uuid does not match outbox UUID")
        task = _safe_token(_required_string(payload, "task"), "task")
        quality = _safe_token(_required_string(payload, "quality"), "quality")
        requested_index = _required_nonnegative_int(payload, "requested_index")
        frame_count = _required_nonnegative_int(payload, "frame_count")

        camera_names_value = payload.get("camera_names")
        if not isinstance(camera_names_value, list) or not camera_names_value:
            raise ManifestError("Manifest camera_names must be a list of non-empty strings")
        camera_names_list: list[str] = []
        for item in camera_names_value:
            if not isinstance(item, str):
                raise ManifestError("Manifest camera_names must contain only strings")
            camera_names_list.append(_safe_token(item, "camera name"))
        camera_names = tuple(camera_names_list)
        if len(set(camera_names)) != len(camera_names):
            raise ManifestError("Manifest camera_names must not contain duplicates")

        episode_subdir_text = _required_string(payload, "episode_subdir")
        episode_subdir = Path(episode_subdir_text)
        if (
            episode_subdir.is_absolute()
            or not episode_subdir.parts
            or any(part in {"", ".", ".."} for part in episode_subdir.parts)
        ):
            raise ManifestError(
                f"Manifest episode_subdir must stay inside its outbox: {episode_subdir_text!r}"
            )

        output_root_text = _required_string(payload, "output_root")
        output_root = Path(output_root_text).expanduser()
        if not output_root.is_absolute():
            raise ManifestError("Manifest output_root must be an absolute path")
        output_root = _absolute_path(output_root)
        expected_root = _absolute_path(expected_nas_root)
        if output_root != expected_root:
            raise ManifestError(
                f"Manifest output_root {output_root} does not match configured NAS root "
                f"{expected_root}"
            )

        episode_dir = entry_dir / episode_subdir
        try:
            resolved_entry = entry_dir.resolve(strict=True)
            resolved_episode = episode_dir.resolve(strict=True)
            resolved_episode.relative_to(resolved_entry)
        except (OSError, ValueError) as exc:
            raise ManifestError(
                f"Manifest episode_subdir is missing or escapes its outbox: {episode_dir}"
            ) from exc
        if episode_dir.is_symlink() or not episode_dir.is_dir():
            raise ManifestError(f"Episode payload is not a real directory: {episode_dir}")

        return cls(
            entry_dir=entry_dir,
            outbox_uuid=outbox_uuid,
            output_root=output_root,
            task=task,
            quality=quality,
            requested_index=requested_index,
            frame_count=frame_count,
            camera_names=camera_names,
            episode_subdir=episode_subdir,
            manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
        )


@dataclass(frozen=True)
class PreparedFile:
    relative_path: Path
    size: int
    sha256: str
    source_path: Path | None = None
    generated_bytes: bytes | None = None


@dataclass(frozen=True)
class PreparedEpisode:
    directories: tuple[Path, ...]
    files: tuple[PreparedFile, ...]
    source_directories: tuple[Path, ...] = ()
    source_files: tuple[PreparedFile, ...] = ()


@dataclass(frozen=True)
class LegacyEpisode:
    source_dir: Path
    outbox_uuid: str
    manifest: dict


@dataclass(frozen=True)
class SyncResult:
    status: str
    entry_dir: Path | None = None
    final_dir: Path | None = None
    reason: str = ""


class LocalInstanceLock:
    """Non-blocking process-local single-instance lock backed by ``flock``."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._handle: TextIO | None = None

    def acquire(self) -> None:
        if self._handle is not None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            handle.close()
            if exc.errno in {errno.EACCES, errno.EAGAIN}:
                raise InstanceLockError(
                    f"Another local NAS sync process owns {self.path}"
                ) from exc
            raise
        handle.seek(0)
        handle.truncate()
        handle.write(f"pid={os.getpid()} host={socket.gethostname()}\n")
        handle.flush()
        self._handle = handle

    def release(self) -> None:
        if self._handle is None:
            return
        try:
            fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)
        finally:
            self._handle.close()
            self._handle = None

    def __enter__(self) -> "LocalInstanceLock":
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.release()


class NasDirectoryLock:
    """Cross-host lock using atomic ``mkdir`` on the shared NAS."""

    def __init__(
        self,
        path: Path,
        *,
        timeout_seconds: float,
        stale_seconds: float,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        wall_time: Callable[[], float] = time.time,
        owner_probe: Callable[[Mapping], bool | None] | None = None,
    ) -> None:
        self.path = path
        self.timeout_seconds = timeout_seconds
        self.stale_seconds = stale_seconds
        self.sleep = sleep
        self.monotonic = monotonic
        self.wall_time = wall_time
        self.owner_probe = owner_probe or _nas_lock_owner_is_alive
        self.token = uuid.uuid4().hex
        self._owned = False

    def acquire(self) -> None:
        deadline = self.monotonic() + self.timeout_seconds
        while True:
            try:
                self.path.mkdir(mode=0o755)
            except FileExistsError:
                if self._reap_if_stale():
                    continue
                if self.monotonic() >= deadline:
                    raise SyncError(f"Timed out waiting for NAS lock {self.path}")
                self.sleep(min(0.25, max(0.01, deadline - self.monotonic())))
                continue
            except OSError as exc:
                raise SyncError(f"Cannot create NAS lock {self.path}: {exc}") from exc

            try:
                owner = {
                    "token": self.token,
                    "pid": os.getpid(),
                    "host": socket.gethostname(),
                    "created_at_unix": self.wall_time(),
                    "process_start_id": _process_start_identity(os.getpid()),
                    "boot_id": _host_boot_id(),
                }
                _write_json_atomic(self.path / "owner.json", owner)
                os.utime(self.path, None)
            except Exception:
                shutil.rmtree(self.path, ignore_errors=True)
                raise
            self._owned = True
            return

    def _reap_if_stale(self) -> bool:
        if self.stale_seconds <= 0:
            return False
        try:
            age = self.wall_time() - self.path.stat().st_mtime
        except FileNotFoundError:
            return True
        except OSError:
            return False
        if age <= self.stale_seconds:
            return False

        owner_path = self.path / "owner.json"
        try:
            owner = json.loads(owner_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            return False
        if not isinstance(owner, dict):
            return False
        token = owner.get("token")
        if not isinstance(token, str) or not token:
            return False
        try:
            owner_alive = self.owner_probe(owner)
        except OSError:
            return False
        # Remote, malformed, inaccessible, and live owners are all fail-closed.
        if owner_alive is not False:
            return False

        try:
            current_owner = json.loads(owner_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            return False
        if not isinstance(current_owner, dict) or current_owner.get("token") != token:
            return False

        stale_path = self.path.with_name(f"{self.path.name}.stale-{uuid.uuid4().hex}")
        try:
            self.path.rename(stale_path)
        except FileNotFoundError:
            return True
        except OSError:
            return False
        shutil.rmtree(stale_path, ignore_errors=True)
        return True

    def release(self) -> None:
        if not self._owned:
            return
        owner_path = self.path / "owner.json"
        try:
            owner = json.loads(owner_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            owner = {}
        if owner.get("token") == self.token:
            try:
                owner_path.unlink()
                self.path.rmdir()
            except FileNotFoundError:
                pass
            except OSError as exc:
                LOGGER.warning("Could not remove NAS lock %s: %s", self.path, exc)
        self._owned = False

    def __enter__(self) -> "NasDirectoryLock":
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.release()


class RateLimiter:
    """Simple average-rate limiter shared by all files in one episode copy."""

    def __init__(
        self,
        bytes_per_second: float,
        *,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.bytes_per_second = bytes_per_second
        self.sleep = sleep
        self.monotonic = monotonic
        self.started_at: float | None = None
        self.transferred = 0

    def consume(self, byte_count: int) -> None:
        if self.bytes_per_second <= 0 or byte_count <= 0:
            return
        now = self.monotonic()
        if self.started_at is None:
            self.started_at = now
        self.transferred += byte_count
        target_elapsed = self.transferred / self.bytes_per_second
        actual_elapsed = now - self.started_at
        delay = target_elapsed - actual_elapsed
        if delay > 0:
            self.sleep(delay)


class ChunkedCopier:
    """Copy regular files in chunks and verify size plus SHA-256."""

    def __init__(
        self,
        chunk_size: int,
        limiter: RateLimiter,
        progress_check: Callable[[], None] | None = None,
    ) -> None:
        self.chunk_size = chunk_size
        self.limiter = limiter
        self.progress_check = progress_check or (lambda: None)

    def copy(self, prepared_file: PreparedFile, destination: Path) -> None:
        if _file_matches(destination, prepared_file, self.chunk_size):
            return
        if destination.exists() and not destination.is_file():
            raise SyncError(f"Copy destination is not a regular file: {destination}")
        if destination.is_symlink():
            raise SyncError(f"Refusing to overwrite symlink: {destination}")

        destination.parent.mkdir(parents=True, exist_ok=True)
        temp_path = destination.with_name(
            f".{destination.name}.copying-{os.getpid()}-{uuid.uuid4().hex}"
        )
        digest = hashlib.sha256()
        byte_count = 0
        try:
            with temp_path.open("xb") as target:
                if prepared_file.generated_bytes is not None:
                    content = prepared_file.generated_bytes
                    for offset in range(0, len(content), self.chunk_size):
                        self.progress_check()
                        chunk = content[offset : offset + self.chunk_size]
                        target.write(chunk)
                        digest.update(chunk)
                        byte_count += len(chunk)
                        self.limiter.consume(len(chunk))
                else:
                    assert prepared_file.source_path is not None
                    before = prepared_file.source_path.stat()
                    with prepared_file.source_path.open("rb") as source:
                        while True:
                            self.progress_check()
                            chunk = source.read(self.chunk_size)
                            if not chunk:
                                break
                            target.write(chunk)
                            digest.update(chunk)
                            byte_count += len(chunk)
                            self.limiter.consume(len(chunk))
                    after = prepared_file.source_path.stat()
                    if (
                        before.st_size != after.st_size
                        or before.st_mtime_ns != after.st_mtime_ns
                    ):
                        raise SyncError(
                            f"Source changed while copying: {prepared_file.source_path}"
                        )
                target.flush()
                os.fsync(target.fileno())

            if byte_count != prepared_file.size or digest.hexdigest() != prepared_file.sha256:
                raise VerificationError(
                    f"Source digest changed while copying {prepared_file.relative_path}"
                )
            os.replace(temp_path, destination)
            if prepared_file.source_path is not None:
                try:
                    shutil.copystat(prepared_file.source_path, destination)
                except OSError:
                    pass
            _verify_file(destination, prepared_file, self.chunk_size)
        finally:
            try:
                temp_path.unlink()
            except FileNotFoundError:
                pass


class NasSync:
    """Scan and publish at most one READY outbox per ``sync_once`` call."""

    def __init__(
        self,
        config: SyncConfig,
        *,
        mount_checker: Callable[[Path], bool] | None = None,
        load_getter: Callable[[], float] | None = None,
        memory_getter: Callable[[], int | None] | None = None,
        process_activity_getter: Callable[[], str | None] | None = None,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        wall_time: Callable[[], float] = time.time,
    ) -> None:
        self.config = config
        self.mount_checker = mount_checker or (
            lambda path: is_real_mount(
                path,
                expected_source=config.expected_nas_source,
                expected_filesystem=config.expected_nas_fs_type,
            )
        )
        self.load_getter = load_getter or (lambda: os.getloadavg()[0])
        self.memory_getter = memory_getter or available_memory_bytes
        self.process_activity_getter = process_activity_getter or active_heavy_process
        self.sleep = sleep
        self.monotonic = monotonic
        self.wall_time = wall_time
        self.instance_lock = LocalInstanceLock(config.instance_lock_path)
        self._last_activity_seen_at: float | None = None

    def __enter__(self) -> "NasSync":
        self.instance_lock.acquire()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.instance_lock.release()

    def scan_ready(self) -> list[Path]:
        try:
            entries = list(self.config.outbox_root.iterdir())
        except FileNotFoundError:
            return []
        except OSError as exc:
            raise SyncError(f"Cannot scan outbox root {self.config.outbox_root}: {exc}") from exc

        ready_entries: list[tuple[float, str, Path]] = []
        for entry in entries:
            if entry.name.startswith("."):
                continue
            if entry.is_symlink() or not entry.is_dir():
                continue
            ready = entry / READY_NAME
            manifest = entry / MANIFEST_NAME
            if ready.is_symlink() or manifest.is_symlink():
                continue
            try:
                if ready.is_file() and manifest.is_file():
                    ready_entries.append((ready.stat().st_mtime, entry.name, entry))
            except OSError:
                continue
        ready_entries.sort(key=lambda item: (item[0], item[1]))
        return [item[2] for item in ready_entries]

    def sync_once(self, *, dry_run: bool = False) -> SyncResult:
        self.instance_lock.acquire()
        entries = self.scan_ready()
        if not entries:
            return SyncResult(status="no_work")
        invalid_entries: list[str] = []
        for entry in entries:
            try:
                manifest = OutboxManifest.load(entry, self.config.nas_root)
            except ManifestError as exc:
                invalid_entries.append(f"{entry.name}: {exc}")
                LOGGER.error("Skipping invalid local outbox %s: %s", entry, exc)
                continue

            gate_reason = self._gate_reason()
            if gate_reason:
                return SyncResult(
                    status="deferred",
                    entry_dir=manifest.entry_dir,
                    reason=gate_reason,
                )
            mount_reason = self._mount_gate_reason("before NAS publication")
            if mount_reason:
                return SyncResult(
                    status="deferred",
                    entry_dir=manifest.entry_dir,
                    reason=mount_reason,
                )

            try:
                validate_outbox_payload(manifest)
            except ManifestError as exc:
                invalid_entries.append(f"{entry.name}: {exc}")
                LOGGER.error("Skipping invalid local outbox %s: %s", entry, exc)
                continue

            if dry_run:
                final_index = self._preview_final_index(manifest)
                return SyncResult(
                    status="dry_run",
                    entry_dir=manifest.entry_dir,
                    final_dir=self.config.nas_root
                    / manifest.task
                    / manifest.quality
                    / str(final_index),
                    reason=f"would publish requested index {manifest.requested_index}",
                )

            try:
                final_dir = self._publish(manifest)
            except SyncDeferred as exc:
                return SyncResult(
                    status="deferred",
                    entry_dir=manifest.entry_dir,
                    reason=str(exc),
                )
            except ManifestError as exc:
                invalid_entries.append(f"{entry.name}: {exc}")
                LOGGER.error("Skipping invalid local outbox %s: %s", entry, exc)
                continue
            except SyncError:
                raise
            except Exception as exc:
                raise SyncError(
                    f"Unexpected publish failure for {manifest.entry_dir}: {exc}"
                ) from exc
            return SyncResult(
                status="synced",
                entry_dir=manifest.entry_dir,
                final_dir=final_dir,
            )

        return SyncResult(
            status="deferred",
            entry_dir=entries[0],
            reason="all READY outboxes are invalid: " + "; ".join(invalid_entries[:3]),
        )

    def migrate_legacy_saving(self) -> int:
        """Move only complete legacy ``.saving`` episodes into READY outboxes."""

        self.instance_lock.acquire()
        if self._gate_reason():
            return 0
        saving_root = self.config.cache_root / ".saving"
        try:
            save_sessions = _real_child_directories(saving_root)
        except FileNotFoundError:
            return 0
        except OSError as exc:
            LOGGER.error("Cannot inspect legacy saving root %s: %s", saving_root, exc)
            return 0

        migrated = 0
        for save_session in save_sessions:
            try:
                task_directories = _real_child_directories(save_session)
            except OSError as exc:
                LOGGER.error("Cannot inspect legacy session %s: %s", save_session, exc)
                continue
            for task_dir in task_directories:
                try:
                    quality_directories = _real_child_directories(task_dir)
                except OSError as exc:
                    LOGGER.error("Cannot inspect legacy task %s: %s", task_dir, exc)
                    continue
                for quality_dir in quality_directories:
                    try:
                        index_directories = _real_child_directories(quality_dir)
                    except OSError as exc:
                        LOGGER.error(
                            "Cannot inspect legacy quality directory %s: %s",
                            quality_dir,
                            exc,
                        )
                        continue
                    for episode_dir in index_directories:
                        try:
                            legacy = _inspect_legacy_episode(
                                saving_root,
                                episode_dir,
                                self.config.nas_root,
                            )
                        except ManifestError as exc:
                            LOGGER.warning(
                                "Keeping incomplete legacy episode at %s: %s",
                                episode_dir,
                                exc,
                            )
                            continue
                        try:
                            destination = _migrate_legacy_episode(
                                self.config.outbox_root,
                                legacy,
                            )
                        except Exception as exc:
                            LOGGER.error(
                                "Legacy migration failed; source was preserved at %s: %s",
                                episode_dir,
                                exc,
                            )
                            continue
                        migrated += 1
                        LOGGER.info(
                            "Migrated legacy episode %s -> %s",
                            episode_dir,
                            destination,
                        )
        return migrated

    def migrate_ready_recordings(self) -> int:
        """Recover the crash window before a READY recording is renamed to outbox."""

        self.instance_lock.acquire()
        if self._gate_reason():
            return 0
        recording_root = self.config.cache_root / ".recording"
        try:
            sessions = _real_child_directories(recording_root)
        except (FileNotFoundError, OSError):
            return 0
        self.config.outbox_root.mkdir(parents=True, exist_ok=True)
        migrated = 0
        for session in sessions:
            if not (session / READY_NAME).is_file() or not (session / MANIFEST_NAME).is_file():
                continue
            try:
                OutboxManifest.load(session, self.config.nas_root)
            except ManifestError as exc:
                LOGGER.warning("Keeping invalid READY recording at %s: %s", session, exc)
                continue
            destination = self.config.outbox_root / session.name
            if destination.exists():
                LOGGER.error(
                    "Cannot recover READY recording because outbox exists: %s",
                    destination,
                )
                continue
            try:
                session.rename(destination)
                _fsync_directory(self.config.outbox_root)
            except OSError as exc:
                LOGGER.error("Could not recover READY recording %s: %s", session, exc)
                continue
            migrated += 1
            LOGGER.info("Recovered READY recording %s -> %s", session, destination)
        return migrated

    def cleanup_synced_tombstones(self) -> int:
        """Retry deletion of outboxes already atomically removed from the live queue."""

        self.instance_lock.acquire()
        if self._gate_reason():
            return 0
        if self._mount_gate_reason("before tombstone recovery"):
            return 0
        tombstone_root = self.config.cache_root / TOMBSTONE_DIR_NAME
        try:
            tombstones = _real_child_directories(tombstone_root)
        except (FileNotFoundError, OSError):
            return 0
        removed = 0
        for tombstone in tombstones:
            try:
                receipt_path = self._ensure_tombstone_receipt(tombstone)
                if self._remove_verified_tombstone(tombstone, receipt_path):
                    removed += 1
            except SyncError as exc:
                LOGGER.warning(
                    "Keeping local tombstone %s because recovery verification failed: %s",
                    tombstone,
                    exc,
                )
                continue

        try:
            receipts = list(tombstone_root.glob(f".*{TOMBSTONE_RECEIPT_SUFFIX}"))
        except OSError:
            receipts = []
        for receipt_path in receipts:
            outbox_uuid = _receipt_outbox_uuid(receipt_path)
            if outbox_uuid is None:
                continue
            if (tombstone_root / outbox_uuid).exists():
                continue
            if (self.config.outbox_root / outbox_uuid).exists():
                continue
            try:
                self._verify_tombstone_receipt(receipt_path)
                self._require_expected_mount(
                    "immediately before removing an orphan tombstone receipt"
                )
                receipt_path.unlink()
                _fsync_directory(tombstone_root)
            except (OSError, SyncError) as exc:
                LOGGER.warning("Keeping tombstone receipt %s: %s", receipt_path, exc)
        return removed

    def _gate_reason(self) -> str:
        activity_reason = self._activity_gate_reason()
        if activity_reason:
            return activity_reason

        try:
            active_process = self.process_activity_getter()
        except OSError as exc:
            return f"cannot inspect active capture processes: {exc}"
        if active_process:
            return f"resource-heavy process is active: {active_process}"

        if self.config.max_load_per_cpu > 0:
            try:
                load = float(self.load_getter())
            except (OSError, ValueError) as exc:
                return f"cannot read system load: {exc}"
            cpu_count = max(1, os.cpu_count() or 1)
            load_per_cpu = load / cpu_count
            if load_per_cpu > self.config.max_load_per_cpu:
                return (
                    f"load per CPU {load_per_cpu:.2f} exceeds "
                    f"{self.config.max_load_per_cpu:.2f}"
                )

        if self.config.min_available_memory_bytes > 0:
            try:
                available = self.memory_getter()
            except OSError as exc:
                return f"cannot read available memory: {exc}"
            if available is None:
                return "available memory is unknown"
            if available < self.config.min_available_memory_bytes:
                return (
                    f"available memory {available / MIB:.0f} MiB is below "
                    f"{self.config.min_available_memory_bytes / MIB:.0f} MiB"
                )
        return ""

    def _activity_gate_reason(self) -> str:
        try:
            markers = list(self.config.activity_root.glob("*.json"))
        except OSError as exc:
            return f"cannot inspect capture activity markers: {exc}"
        now = self.wall_time()
        freshest_marker: tuple[float, str, float] | None = None
        for marker in markers:
            if marker.is_symlink():
                continue
            try:
                marker_mtime = marker.stat().st_mtime
                age = max(0.0, now - marker_mtime)
            except FileNotFoundError:
                continue
            except OSError as exc:
                return f"cannot stat capture activity marker {marker}: {exc}"
            if age <= self.config.activity_fresh_seconds and (
                freshest_marker is None or marker_mtime > freshest_marker[0]
            ):
                freshest_marker = (marker_mtime, marker.name, age)
        if freshest_marker is not None:
            marker_mtime, marker_name, age = freshest_marker
            self._last_activity_seen_at = max(
                self._last_activity_seen_at or marker_mtime,
                marker_mtime,
            )
            return f"capture activity marker {marker_name} is fresh ({age:.1f}s old)"
        if self._last_activity_seen_at is not None:
            idle_age = max(0.0, now - self._last_activity_seen_at)
            if idle_age <= self.config.activity_fresh_seconds:
                return f"capture idle grace is active ({idle_age:.1f}s elapsed)"
        return ""

    def _require_gate_open(self) -> None:
        reason = self._gate_reason()
        if reason:
            raise SyncDeferred(reason)

    def _mount_gate_reason(self, stage: str) -> str:
        if self.config.skip_mount_check:
            return ""
        try:
            mounted = bool(self.mount_checker(self.config.nas_root))
        except Exception as exc:
            return f"cannot verify NAS mount identity {stage}: {exc}"
        if mounted:
            return ""
        return (
            f"NAS mount identity is not exactly {self.config.nas_root} "
            f"source={self.config.expected_nas_source!r} "
            f"fs={self.config.expected_nas_fs_type!r} {stage}"
        )

    def _require_expected_mount(self, stage: str) -> None:
        reason = self._mount_gate_reason(stage)
        if reason:
            raise SyncDeferred(reason)

    def _copy_progress_gate(self) -> Callable[[], None]:
        last_check = [float("-inf")]

        def check() -> None:
            now = self.monotonic()
            if now - last_check[0] < 1.0:
                return
            last_check[0] = now
            self._require_gate_open()
            self._require_expected_mount("during NAS copy")

        return check

    def _publish(self, manifest: OutboxManifest) -> Path:
        self._require_expected_mount("before creating or resuming a NAS partial")
        quality_dir = _ensure_quality_dir(
            self.config.nas_root,
            manifest.task,
            manifest.quality,
        )
        final_index, partial_dir, already_published = self._reserve_destination(
            quality_dir, manifest
        )
        prepared = prepare_episode(manifest, final_index, self.config.chunk_size)

        if already_published:
            assert partial_dir is None
            final_dir = quality_dir / str(final_index)
            verify_source_snapshot(manifest, prepared, self.config.chunk_size)
            self._require_expected_mount("before verifying an existing NAS publication")
            verify_published(final_dir, prepared, manifest, final_index, self.config.chunk_size)
            self._require_gate_open()
            self._delete_verified_outbox(
                manifest,
                prepared,
                final_dir,
                final_index,
            )
            return final_dir

        assert partial_dir is not None
        _clean_partial(partial_dir, prepared)
        for directory in prepared.directories:
            (partial_dir / directory).mkdir(parents=True, exist_ok=True)

        limiter = RateLimiter(
            self.config.rate_limit_bytes_per_second,
            sleep=self.sleep,
            monotonic=self.monotonic,
        )
        copier = ChunkedCopier(
            self.config.chunk_size,
            limiter,
            progress_check=self._copy_progress_gate(),
        )
        for prepared_file in prepared.files:
            copier.copy(prepared_file, partial_dir / prepared_file.relative_path)

        verify_source_snapshot(manifest, prepared, self.config.chunk_size)
        verify_prepared_tree(partial_dir, prepared, self.config.chunk_size, allow_marker=False)
        marker = _sync_marker_payload(manifest, final_index, prepared)
        _write_json_atomic(partial_dir / SYNC_MARKER_NAME, marker)
        _fsync_directory(partial_dir)
        verify_published(
            partial_dir,
            prepared,
            manifest,
            final_index,
            self.config.chunk_size,
        )

        self._require_gate_open()
        verify_source_snapshot(manifest, prepared, self.config.chunk_size)
        final_dir = self._finalize_partial(
            quality_dir, partial_dir, manifest, final_index
        )
        # This read happens after the atomic rename from the final NAS path.
        self._require_expected_mount("before final NAS verification")
        verify_published(
            final_dir,
            prepared,
            manifest,
            final_index,
            self.config.chunk_size,
        )
        self._require_gate_open()
        self._delete_verified_outbox(
            manifest,
            prepared,
            final_dir,
            final_index,
        )
        return final_dir

    def _nas_lock(self, quality_dir: Path) -> NasDirectoryLock:
        return NasDirectoryLock(
            quality_dir / NAS_LOCK_NAME,
            timeout_seconds=self.config.nas_lock_timeout_seconds,
            stale_seconds=self.config.nas_lock_stale_seconds,
            sleep=self.sleep,
            monotonic=self.monotonic,
            wall_time=self.wall_time,
        )

    def _reserve_destination(
        self, quality_dir: Path, manifest: OutboxManifest
    ) -> tuple[int, Path | None, bool]:
        with self._nas_lock(quality_dir):
            self._require_expected_mount("while reserving a NAS destination")
            published_index = _find_published_index(quality_dir, manifest.outbox_uuid)
            if published_index is not None:
                return published_index, None, True

            own_partials: list[tuple[int, Path]] = []
            occupied: list[int] = []
            try:
                children = list(quality_dir.iterdir())
            except OSError as exc:
                raise SyncError(f"Cannot scan NAS quality directory {quality_dir}: {exc}") from exc
            for child in children:
                if child.is_dir() and child.name.isdigit():
                    occupied.append(int(child.name))
                    continue
                match = PARTIAL_PATTERN.match(child.name)
                if child.is_dir() and match:
                    index = int(match.group(1))
                    occupied.append(index)
                    if match.group(2) == manifest.outbox_uuid:
                        own_partials.append((index, child))

            if len(own_partials) > 1:
                raise SyncError(
                    f"Multiple NAS partials exist for outbox {manifest.outbox_uuid}: "
                    f"{[str(item[1]) for item in own_partials]}"
                )
            if own_partials:
                own_index, own_partial = own_partials[0]
                if (quality_dir / str(own_index)).exists():
                    try:
                        shutil.rmtree(own_partial)
                    except OSError as exc:
                        raise SyncError(
                            f"Cannot discard conflicted owned NAS partial {own_partial}: {exc}"
                        ) from exc
                else:
                    return own_index, own_partial, False

            final_index = max(occupied, default=-1) + 1
            partial_dir = quality_dir / (
                f".partial-{final_index}-{manifest.outbox_uuid}"
            )
            try:
                partial_dir.mkdir()
            except OSError as exc:
                raise SyncError(f"Cannot reserve NAS partial {partial_dir}: {exc}") from exc
            _fsync_directory(quality_dir)
            return final_index, partial_dir, False

    def _preview_final_index(self, manifest: OutboxManifest) -> int:
        quality_dir = self.config.nas_root / manifest.task / manifest.quality
        if not quality_dir.is_dir():
            return 0
        published_index = _find_published_index(quality_dir, manifest.outbox_uuid)
        if published_index is not None:
            return published_index
        occupied: list[int] = []
        own_index: int | None = None
        try:
            children = list(quality_dir.iterdir())
        except OSError as exc:
            raise SyncError(f"Cannot inspect NAS quality directory {quality_dir}: {exc}") from exc
        for child in children:
            if child.is_dir() and child.name.isdigit():
                occupied.append(int(child.name))
                continue
            match = PARTIAL_PATTERN.match(child.name)
            if child.is_dir() and match:
                index = int(match.group(1))
                occupied.append(index)
                if match.group(2) == manifest.outbox_uuid:
                    own_index = index
        return own_index if own_index is not None else max(occupied, default=-1) + 1

    def _finalize_partial(
        self,
        quality_dir: Path,
        partial_dir: Path,
        manifest: OutboxManifest,
        final_index: int,
    ) -> Path:
        final_dir = quality_dir / str(final_index)
        with self._nas_lock(quality_dir):
            self._require_expected_mount("immediately before atomic NAS publication")
            if final_dir.exists():
                published = _read_sync_marker(final_dir)
                if published is not None and published.get("outbox_uuid") == manifest.outbox_uuid:
                    return final_dir
                raise SyncError(f"Final NAS index appeared during copy: {final_dir}")
            if not partial_dir.is_dir():
                raise SyncError(f"Reserved NAS partial disappeared: {partial_dir}")
            try:
                partial_dir.rename(final_dir)
            except OSError as exc:
                raise SyncError(
                    f"Atomic NAS rename failed: {partial_dir} -> {final_dir}: {exc}"
                ) from exc
            _fsync_directory(quality_dir)
        return final_dir

    def _delete_verified_outbox(
        self,
        manifest: OutboxManifest,
        prepared: PreparedEpisode,
        final_dir: Path,
        final_index: int,
    ) -> None:
        self._require_gate_open()
        self._require_expected_mount("immediately before local deletion verification")
        current = OutboxManifest.load(manifest.entry_dir, self.config.nas_root)
        if current.manifest_sha256 != manifest.manifest_sha256:
            raise SyncError(
                f"Outbox manifest changed during sync; preserving {manifest.entry_dir}"
            )
        validate_outbox_payload(current)
        verify_source_snapshot(current, prepared, self.config.chunk_size)
        verify_published(
            final_dir,
            prepared,
            current,
            final_index,
            self.config.chunk_size,
        )
        expected_parent = _absolute_path(self.config.outbox_root)
        try:
            _absolute_path(manifest.entry_dir).relative_to(expected_parent)
        except ValueError as exc:
            raise SyncError(f"Refusing to delete outbox outside {expected_parent}") from exc
        tombstone_root = self.config.cache_root / TOMBSTONE_DIR_NAME
        tombstone_root.mkdir(parents=True, exist_ok=True)
        if tombstone_root.is_symlink() or not tombstone_root.is_dir():
            raise SyncError(f"Local tombstone root is unsafe: {tombstone_root}")
        try:
            if expected_parent.stat().st_dev != tombstone_root.stat().st_dev:
                raise SyncError("Local outbox and tombstone roots are not on one filesystem")
        except OSError as exc:
            raise SyncError(f"Cannot verify local tombstone filesystem: {exc}") from exc
        tombstone = tombstone_root / manifest.entry_dir.name
        if tombstone.exists():
            raise SyncError(f"Local synced-delete tombstone already exists: {tombstone}")
        receipt_path = _tombstone_receipt_path(tombstone_root, manifest.outbox_uuid)
        receipt = _tombstone_receipt_payload(manifest, final_index, prepared)
        _write_json_atomic(receipt_path, receipt)
        _fsync_directory(tombstone_root)
        try:
            manifest.entry_dir.rename(tombstone)
            _fsync_directory(expected_parent)
            _fsync_directory(tombstone_root)
        except OSError as exc:
            raise SyncError(
                f"NAS publish verified, but local outbox could not leave the live queue: {exc}"
            ) from exc
        try:
            self._remove_verified_tombstone(tombstone, receipt_path)
        except SyncError as exc:
            LOGGER.warning(
                "NAS publish verified; local cleanup will retry later at %s: %s",
                tombstone,
                exc,
            )

    def _ensure_tombstone_receipt(self, tombstone: Path) -> Path:
        receipt_path = _tombstone_receipt_path(tombstone.parent, tombstone.name)
        if receipt_path.is_file() and not receipt_path.is_symlink():
            self._verify_tombstone_receipt(receipt_path)
            return receipt_path

        manifest = OutboxManifest.load(tombstone, self.config.nas_root)
        validate_outbox_payload(manifest)
        self._require_expected_mount("while recovering a tombstone receipt")
        quality_dir = self.config.nas_root / manifest.task / manifest.quality
        final_index = _find_published_index(quality_dir, manifest.outbox_uuid)
        if final_index is None:
            raise VerificationError(
                f"No verified NAS publication exists for tombstone {tombstone}"
            )
        prepared = prepare_episode(manifest, final_index, self.config.chunk_size)
        verify_source_snapshot(manifest, prepared, self.config.chunk_size)
        final_dir = quality_dir / str(final_index)
        verify_published(
            final_dir,
            prepared,
            manifest,
            final_index,
            self.config.chunk_size,
        )
        _write_json_atomic(
            receipt_path,
            _tombstone_receipt_payload(manifest, final_index, prepared),
        )
        _fsync_directory(tombstone.parent)
        return receipt_path

    def _verify_tombstone_receipt(self, receipt_path: Path) -> dict:
        receipt = _read_tombstone_receipt(receipt_path)
        self._require_expected_mount("while verifying a tombstone receipt")
        final_dir = (
            self.config.nas_root
            / receipt["task"]
            / receipt["quality"]
            / str(receipt["final_index"])
        )
        marker = _read_sync_marker(final_dir)
        if marker != receipt["marker"]:
            raise VerificationError(
                f"NAS publication no longer matches tombstone receipt {receipt_path}"
            )
        verify_prepared_tree(
            final_dir,
            _prepared_from_sync_marker(marker),
            self.config.chunk_size,
            allow_marker=True,
        )
        return receipt

    def _remove_verified_tombstone(
        self,
        tombstone: Path,
        receipt_path: Path,
    ) -> bool:
        self._verify_tombstone_receipt(receipt_path)
        self._require_gate_open()
        self._require_expected_mount("immediately before local tombstone deletion")
        try:
            shutil.rmtree(tombstone)
        except OSError as exc:
            raise SyncError(f"Could not delete local tombstone {tombstone}: {exc}") from exc
        _fsync_directory(tombstone.parent)
        try:
            receipt_path.unlink()
            _fsync_directory(receipt_path.parent)
        except OSError as exc:
            LOGGER.warning(
                "Local tombstone was deleted but its recovery receipt remains at %s: %s",
                receipt_path,
                exc,
            )
        return True


def validate_outbox_payload(manifest: OutboxManifest) -> None:
    files, _ = _walk_regular_tree(manifest.episode_dir)
    _validate_episode_payload(manifest, files)


def verify_source_snapshot(
    manifest: OutboxManifest,
    prepared: PreparedEpisode,
    chunk_size: int,
) -> None:
    """Fail if any local payload byte or path changed after preparation."""

    actual_files, actual_directories = _walk_regular_tree(manifest.episode_dir)
    expected_files = {item.relative_path for item in prepared.source_files}
    if set(actual_files) != expected_files:
        missing = sorted(expected_files - set(actual_files), key=lambda path: path.as_posix())
        extra = sorted(set(actual_files) - expected_files, key=lambda path: path.as_posix())
        raise ManifestError(
            f"Local episode file set changed; missing={missing}, extra={extra}"
        )
    if set(actual_directories) != set(prepared.source_directories):
        raise ManifestError("Local episode directory set changed during sync")
    for expected in prepared.source_files:
        size, digest = _hash_stable_file(
            manifest.episode_dir / expected.relative_path,
            chunk_size,
        )
        if size != expected.size or digest != expected.sha256:
            raise ManifestError(
                f"Local episode file changed during sync: {expected.relative_path}"
            )


def prepare_episode(
    manifest: OutboxManifest,
    final_index: int,
    chunk_size: int,
) -> PreparedEpisode:
    source_root = manifest.episode_dir
    source_files, source_directories = _walk_regular_tree(source_root)

    _validate_episode_payload(manifest, source_files)

    root_pickles = [
        relative
        for relative in source_files
        if relative.parent == Path(".") and relative.name.endswith(".pkl.gz")
    ]
    if len(root_pickles) != 1:
        raise ManifestError("Episode payload must contain exactly one root-level pkl.gz file")
    pickle_to_rename: Path | None = None
    if root_pickles and root_pickles[0].name != f"{final_index}.pkl.gz":
        pickle_to_rename = root_pickles[0]

    prepared_files: list[PreparedFile] = []
    source_snapshots: list[PreparedFile] = []
    destination_paths: set[Path] = set()
    for relative in source_files:
        source_path = source_root / relative
        source_size, source_digest = _hash_stable_file(source_path, chunk_size)
        source_snapshots.append(
            PreparedFile(
                relative_path=relative,
                size=source_size,
                sha256=source_digest,
                source_path=source_path,
            )
        )
        destination_relative = relative
        generated_bytes: bytes | None = None

        if pickle_to_rename is not None and relative == pickle_to_rename:
            destination_relative = Path(f"{final_index}.pkl.gz")
        if relative == Path("metadata.json"):
            generated_bytes = _updated_metadata_bytes(
                source_path,
                manifest,
                final_index,
            )

        if destination_relative in destination_paths:
            raise ManifestError(
                f"Two episode files map to the same NAS path: {destination_relative}"
            )
        destination_paths.add(destination_relative)

        if generated_bytes is not None:
            size = len(generated_bytes)
            digest = hashlib.sha256(generated_bytes).hexdigest()
            prepared_files.append(
                PreparedFile(
                    relative_path=destination_relative,
                    size=size,
                    sha256=digest,
                    generated_bytes=generated_bytes,
                )
            )
        else:
            prepared_files.append(
                PreparedFile(
                    relative_path=destination_relative,
                    size=source_size,
                    sha256=source_digest,
                    source_path=source_path,
                )
            )

    destination_directories = set(source_directories)
    for prepared_file in prepared_files:
        parent = prepared_file.relative_path.parent
        while parent != Path("."):
            destination_directories.add(parent)
            parent = parent.parent

    return PreparedEpisode(
        directories=tuple(sorted(destination_directories, key=lambda path: path.as_posix())),
        files=tuple(sorted(prepared_files, key=lambda item: item.relative_path.as_posix())),
        source_directories=tuple(
            sorted(source_directories, key=lambda path: path.as_posix())
        ),
        source_files=tuple(
            sorted(source_snapshots, key=lambda item: item.relative_path.as_posix())
        ),
    )


def _validate_episode_payload(
    manifest: OutboxManifest,
    source_files: Sequence[Path],
) -> None:
    source_root = manifest.episode_dir
    files = set(source_files)
    if Path("metadata.json") not in files:
        raise ManifestError("Episode payload is missing metadata.json")
    if Path("keyframes.json") not in files:
        raise ManifestError("Episode payload is missing keyframes.json")
    if not manifest.camera_names:
        raise ManifestError("Episode manifest contains no camera names")

    pickle_paths = [
        path
        for path in files
        if path.parent == Path(".") and path.name.endswith(".pkl.gz")
    ]
    if len(pickle_paths) != 1:
        raise ManifestError("Episode payload must contain exactly one root-level pkl.gz file")
    pickle_path = source_root / pickle_paths[0]
    _validate_gzip_pickle(pickle_path)

    metadata_path = source_root / "metadata.json"
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ManifestError(f"Cannot read episode metadata {metadata_path}: {exc}") from exc
    if not isinstance(metadata, dict):
        raise ManifestError("Episode metadata must be a JSON object")
    if metadata.get("task") != manifest.task or metadata.get("quality") != manifest.quality:
        raise ManifestError("Episode metadata task/quality does not match outbox manifest")
    if metadata.get("index") != manifest.requested_index:
        raise ManifestError("Episode metadata index does not match outbox manifest")
    if metadata.get("frame_count") != manifest.frame_count or manifest.frame_count <= 0:
        raise ManifestError("Episode metadata frame_count does not match a positive manifest count")
    metadata_cameras = metadata.get("camera_names")
    if not isinstance(metadata_cameras, list) or set(metadata_cameras) != set(manifest.camera_names):
        raise ManifestError("Episode metadata camera_names does not match outbox manifest")
    expected_relative = f"{manifest.task}/{manifest.quality}/{manifest.requested_index}"
    metadata_relative = metadata.get("relative_episode_dir")
    if metadata_relative is not None and metadata_relative != expected_relative:
        raise ManifestError("Episode metadata relative_episode_dir is inconsistent")

    keyframes = _load_and_validate_keyframes(
        source_root / "keyframes.json",
        manifest.frame_count,
    )
    metadata_keyframes = metadata.get("keyframes")
    if metadata_keyframes is not None and metadata_keyframes != keyframes:
        raise ManifestError("Episode metadata keyframes does not match keyframes.json")

    for camera_name in manifest.camera_names:
        video_relative = Path(f"{camera_name}.mp4")
        if video_relative not in files:
            raise ManifestError(f"Episode payload is missing camera video {video_relative}")
        _validate_mp4_container(source_root / video_relative)

    storage = metadata.get("image_storage")
    if isinstance(storage, dict) and storage.get("type") == "video":
        cameras = storage.get("cameras")
        if not isinstance(cameras, dict) or set(cameras) != set(manifest.camera_names):
            raise ManifestError("image_storage camera set does not match outbox manifest")
        for camera_name in manifest.camera_names:
            camera = cameras.get(camera_name)
            if not isinstance(camera, dict):
                raise ManifestError(f"image_storage entry is invalid for {camera_name}")
            if camera.get("filename") != f"{camera_name}.mp4":
                raise ManifestError(f"image_storage filename is invalid for {camera_name}")
            if camera.get("frame_count") != manifest.frame_count:
                raise ManifestError(f"image_storage frame_count is invalid for {camera_name}")
            for dimension in ("width", "height", "channels"):
                value = camera.get(dimension)
                if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                    raise ManifestError(
                        f"image_storage {dimension} is invalid for {camera_name}"
                    )
        preview = storage.get("preview")
        if isinstance(preview, dict) and preview:
            preview_name = preview.get("filename")
            if preview_name != "preview_all.mp4":
                raise ManifestError("image_storage preview filename is invalid")
            if Path(preview_name) not in files:
                raise ManifestError("Episode payload is missing preview_all.mp4")
            if preview.get("frame_count") != manifest.frame_count:
                raise ManifestError("image_storage preview frame_count is invalid")
            _validate_mp4_container(source_root / preview_name)


def _validate_gzip_pickle(path: Path) -> None:
    """Validate gzip CRC/trailer and pickle syntax without executing the pickle."""

    try:
        last_opcode = ""
        with gzip.open(path, "rb") as handle:
            for opcode, _argument, _position in pickletools.genops(handle):
                last_opcode = opcode.name
            trailing = handle.read(1)
        if last_opcode != "STOP":
            raise ManifestError(f"Trajectory pickle has no STOP opcode: {path}")
        if trailing:
            raise ManifestError(f"Trajectory pickle contains trailing data: {path}")
    except ManifestError:
        raise
    except (OSError, EOFError, ValueError, gzip.BadGzipFile) as exc:
        raise ManifestError(f"Trajectory gzip/pickle is corrupt: {path}: {exc}") from exc


def _load_and_validate_keyframes(path: Path, frame_count: int) -> list[int]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ManifestError(f"Cannot read keyframes file {path}: {exc}") from exc
    if not isinstance(payload, dict) or not isinstance(payload.get("keyframes"), list):
        raise ManifestError("keyframes.json must contain a keyframes list")
    keyframes = payload["keyframes"]
    if not keyframes:
        raise ManifestError("keyframes.json must contain at least one keyframe")
    if any(
        not isinstance(value, int)
        or isinstance(value, bool)
        or value < 0
        or value >= frame_count
        for value in keyframes
    ):
        raise ManifestError("keyframes.json contains an out-of-range index")
    if keyframes != sorted(set(keyframes)):
        raise ManifestError("keyframes.json must be sorted and unique")
    return keyframes


def _validate_mp4_container(path: Path) -> None:
    """Check the finalized ISO-BMFF box structure without decoding or re-encoding."""

    try:
        file_size = path.stat().st_size
        box_types: set[bytes] = set()
        media_payload_bytes = 0
        with path.open("rb") as handle:
            offset = 0
            while offset < file_size:
                handle.seek(offset)
                header = handle.read(8)
                if len(header) != 8:
                    raise ManifestError(f"MP4 has a truncated box header: {path}")
                box_size = int.from_bytes(header[:4], "big")
                box_type = header[4:8]
                header_size = 8
                if box_size == 1:
                    extended = handle.read(8)
                    if len(extended) != 8:
                        raise ManifestError(f"MP4 has a truncated extended box: {path}")
                    box_size = int.from_bytes(extended, "big")
                    header_size = 16
                elif box_size == 0:
                    box_size = file_size - offset
                if box_size < header_size or offset + box_size > file_size:
                    raise ManifestError(f"MP4 box exceeds file bounds: {path}")
                box_types.add(box_type)
                if box_type == b"mdat":
                    media_payload_bytes += box_size - header_size
                offset += box_size
        if offset != file_size:
            raise ManifestError(f"MP4 box layout does not consume the full file: {path}")
    except ManifestError:
        raise
    except OSError as exc:
        raise ManifestError(f"Cannot inspect camera video {path}: {exc}") from exc
    if b"ftyp" not in box_types or b"moov" not in box_types or media_payload_bytes <= 0:
        raise ManifestError(f"Camera video is not a finalized MP4 container: {path}")


def _inspect_legacy_episode(
    saving_root: Path,
    episode_dir: Path,
    nas_root: Path,
) -> LegacyEpisode:
    try:
        relative = episode_dir.relative_to(saving_root)
    except ValueError as exc:
        raise ManifestError(f"Legacy episode escapes {saving_root}: {episode_dir}") from exc
    if len(relative.parts) != 4:
        raise ManifestError(
            "Expected .saving/<save_uuid>/<task>/<quality>/<index> layout"
        )

    save_uuid, task_value, quality_value, index_value = relative.parts
    _safe_token(save_uuid, "legacy save UUID")
    task = _safe_token(task_value, "legacy task")
    quality = _safe_token(quality_value, "legacy quality")
    if not index_value.isdigit() or str(int(index_value)) != index_value:
        raise ManifestError(f"Legacy index is not canonical: {index_value!r}")
    requested_index = int(index_value)

    files, _ = _walk_regular_tree(episode_dir)
    metadata_relative = Path("metadata.json")
    if metadata_relative not in files:
        raise ManifestError("metadata.json is missing")
    metadata_path = episode_dir / metadata_relative
    if metadata_path.stat().st_size <= 0:
        raise ManifestError("metadata.json is empty")

    pickle_paths = [path for path in files if path.parent == Path(".") and path.name.endswith(".pkl.gz")]
    video_paths = [path for path in files if path.parent == Path(".") and path.suffix.lower() == ".mp4"]
    if not pickle_paths:
        raise ManifestError("no pkl.gz trajectory is present")
    if not video_paths:
        raise ManifestError("no episode video is present")
    for relative_path in [*pickle_paths, *video_paths]:
        if (episode_dir / relative_path).stat().st_size <= 0:
            raise ManifestError(f"required file is empty: {relative_path}")
    for relative_path in pickle_paths:
        try:
            with (episode_dir / relative_path).open("rb") as handle:
                if handle.read(2) != b"\x1f\x8b":
                    raise ManifestError(f"trajectory is not gzip: {relative_path}")
        except OSError as exc:
            raise ManifestError(f"cannot inspect trajectory {relative_path}: {exc}") from exc
    for relative_path in video_paths:
        try:
            with (episode_dir / relative_path).open("rb") as handle:
                header = handle.read(64)
        except OSError as exc:
            raise ManifestError(f"cannot inspect video {relative_path}: {exc}") from exc
        if len(header) < 12 or b"ftyp" not in header:
            raise ManifestError(f"video does not have an MP4 header: {relative_path}")

    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ManifestError(f"cannot read metadata.json: {exc}") from exc
    if not isinstance(metadata, dict):
        raise ManifestError("metadata.json must contain an object")
    if "task" in metadata and metadata["task"] != task:
        raise ManifestError("metadata task does not match the legacy path")
    if "quality" in metadata and metadata["quality"] != quality:
        raise ManifestError("metadata quality does not match the legacy path")
    if "index" in metadata and metadata["index"] != requested_index:
        raise ManifestError("metadata index does not match the legacy path")
    frame_count = _required_nonnegative_int(metadata, "frame_count")

    camera_names_value = metadata.get("camera_names")
    if isinstance(camera_names_value, list) and all(
        isinstance(item, str) and item for item in camera_names_value
    ):
        camera_names = list(camera_names_value)
    else:
        camera_names = sorted(
            path.stem for path in video_paths if path.stem != "preview_all"
        )

    source_key = relative.as_posix().encode("utf-8")
    outbox_uuid = f"legacy-{hashlib.sha256(source_key).hexdigest()[:24]}"
    manifest = {
        "schema_version": 1,
        "episode_uuid": outbox_uuid,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "output_root": str(_absolute_path(nas_root)),
        "task": task,
        "quality": quality,
        "requested_index": requested_index,
        "frame_count": frame_count,
        "camera_names": camera_names,
        "episode_subdir": "episode",
        "legacy_source": relative.as_posix(),
    }
    return LegacyEpisode(
        source_dir=episode_dir,
        outbox_uuid=outbox_uuid,
        manifest=manifest,
    )


def _migrate_legacy_episode(outbox_root: Path, legacy: LegacyEpisode) -> Path:
    outbox_root.mkdir(parents=True, exist_ok=True)
    destination = outbox_root / legacy.outbox_uuid
    if destination.exists():
        raise SyncError(f"Legacy outbox destination already exists: {destination}")

    staging = outbox_root / (
        f".migrating-{legacy.outbox_uuid}-{uuid.uuid4().hex}"
    )
    staging.mkdir()
    moved_episode = staging / "episode"
    try:
        _write_json_atomic(staging / MANIFEST_NAME, legacy.manifest)
        _write_text_atomic(staging / READY_NAME, legacy.outbox_uuid)
        legacy.source_dir.rename(moved_episode)
        _fsync_directory(staging)
        staging.rename(destination)
        _fsync_directory(outbox_root)
        return destination
    except Exception:
        source_missing = not legacy.source_dir.exists()
        rollback_container = destination if destination.is_dir() else staging
        rollback_episode = rollback_container / "episode"
        if source_missing and rollback_episode.is_dir():
            try:
                rollback_episode.rename(legacy.source_dir)
            except OSError as rollback_error:
                LOGGER.critical(
                    "Could not roll back legacy episode %s from %s: %s",
                    legacy.source_dir,
                    rollback_episode,
                    rollback_error,
                )
        if legacy.source_dir.is_dir():
            shutil.rmtree(staging, ignore_errors=True)
            if destination.is_dir():
                shutil.rmtree(destination, ignore_errors=True)
        raise


def _real_child_directories(root: Path) -> list[Path]:
    children = list(root.iterdir())
    return sorted(
        [child for child in children if not child.is_symlink() and child.is_dir()],
        key=lambda path: path.name,
    )


def verify_prepared_tree(
    root: Path,
    prepared: PreparedEpisode,
    chunk_size: int,
    *,
    allow_marker: bool,
) -> None:
    actual_files, actual_directories = _walk_regular_tree(root)
    expected_files = {item.relative_path for item in prepared.files}
    if allow_marker:
        expected_files.add(Path(SYNC_MARKER_NAME))
    if set(actual_files) != expected_files:
        missing = sorted(expected_files - set(actual_files), key=lambda path: path.as_posix())
        extra = sorted(set(actual_files) - expected_files, key=lambda path: path.as_posix())
        raise VerificationError(
            f"NAS file set mismatch under {root}; missing={missing}, extra={extra}"
        )

    expected_directories = set(prepared.directories)
    for relative in expected_files:
        parent = relative.parent
        while parent != Path("."):
            expected_directories.add(parent)
            parent = parent.parent
    if set(actual_directories) != expected_directories:
        raise VerificationError(f"NAS directory set mismatch under {root}")

    for prepared_file in prepared.files:
        _verify_file(root / prepared_file.relative_path, prepared_file, chunk_size)


def verify_published(
    root: Path,
    prepared: PreparedEpisode,
    manifest: OutboxManifest,
    final_index: int,
    chunk_size: int,
) -> None:
    verify_prepared_tree(root, prepared, chunk_size, allow_marker=True)
    marker = _read_sync_marker(root)
    expected_marker = _sync_marker_payload(manifest, final_index, prepared)
    if marker != expected_marker:
        raise VerificationError(f"NAS sync marker mismatch under {root}")


def is_real_mount(
    path: Path,
    mountinfo_path: Path = Path("/proc/self/mountinfo"),
    *,
    expected_source: str | None = None,
    expected_filesystem: str = "cifs",
) -> bool:
    """Require an exact mount point, SMB source, and filesystem type."""

    target = _absolute_path(path).as_posix()
    try:
        lines = mountinfo_path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return False

    for line in lines:
        fields = line.split()
        if len(fields) < 10 or "-" not in fields:
            continue
        mount_point = _unescape_mountinfo(fields[4])
        if mount_point != target:
            continue
        separator = fields.index("-")
        if len(fields) <= separator + 2:
            return False
        filesystem = fields[separator + 1].casefold()
        source = _unescape_mountinfo(fields[separator + 2])
        if filesystem != expected_filesystem.strip().casefold():
            return False
        if expected_source and source.casefold() != expected_source.casefold():
            return False
        return True
    return False


def available_memory_bytes(meminfo_path: Path = Path("/proc/meminfo")) -> int | None:
    try:
        with meminfo_path.open("r", encoding="utf-8") as handle:
            for line in handle:
                if line.startswith("MemAvailable:"):
                    parts = line.split()
                    return int(parts[1]) * 1024
    except (OSError, ValueError, IndexError):
        pass

    try:
        pages = os.sysconf("SC_AVPHYS_PAGES")
        page_size = os.sysconf("SC_PAGE_SIZE")
    except (OSError, ValueError):
        return None
    if not isinstance(pages, int) or not isinstance(page_size, int):
        return None
    return pages * page_size


def active_heavy_process(proc_root: Path = Path("/proc")) -> str | None:
    """Return one active capture/replay/conversion command, if visible."""

    try:
        processes = list(proc_root.iterdir())
    except OSError:
        return None
    current_pid = os.getpid()
    for process_dir in processes:
        if not process_dir.name.isdigit() or int(process_dir.name) == current_pid:
            continue
        try:
            raw = (process_dir / "cmdline").read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        except OSError:
            continue
        if not raw:
            continue
        command = raw.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()
        lowered = command.casefold()
        if any(token in lowered for token in RESOURCE_HEAVY_PROCESS_TOKENS):
            return command[:240]
    return None


def _process_start_identity(pid: int, proc_root: Path = Path("/proc")) -> str | None:
    try:
        stat_text = (proc_root / str(pid) / "stat").read_text(encoding="utf-8")
    except OSError:
        return None
    closing_paren = stat_text.rfind(")")
    if closing_paren < 0:
        return None
    fields_after_name = stat_text[closing_paren + 1 :].split()
    # Field 22 (starttime) is index 19 after the process name and closing paren.
    if len(fields_after_name) <= 19:
        return None
    return fields_after_name[19]


def _host_boot_id(path: Path = Path("/proc/sys/kernel/random/boot_id")) -> str | None:
    try:
        value = path.read_text(encoding="utf-8").strip()
    except OSError:
        return None
    return value or None


def _nas_lock_owner_is_alive(owner: Mapping) -> bool | None:
    host = owner.get("host")
    pid = owner.get("pid")
    if not isinstance(host, str) or host != socket.gethostname():
        return None
    if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 0:
        return None

    owner_boot_id = owner.get("boot_id")
    current_boot_id = _host_boot_id()
    if (
        isinstance(owner_boot_id, str)
        and current_boot_id is not None
        and owner_boot_id != current_boot_id
    ):
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        return None

    expected_start = owner.get("process_start_id")
    if isinstance(expected_start, str) and expected_start:
        actual_start = _process_start_identity(pid)
        if actual_start is None:
            return None
        return actual_start == expected_start
    return True


def _walk_regular_tree(root: Path) -> tuple[list[Path], list[Path]]:
    if root.is_symlink() or not root.is_dir():
        raise ManifestError(f"Episode tree is not a real directory: {root}")
    files: list[Path] = []
    directories: list[Path] = []

    def visit(directory: Path, relative_directory: Path) -> None:
        try:
            children = sorted(directory.iterdir(), key=lambda path: path.name)
        except OSError as exc:
            raise ManifestError(f"Cannot scan episode directory {directory}: {exc}") from exc
        for child in children:
            relative = relative_directory / child.name
            try:
                mode = child.lstat().st_mode
            except OSError as exc:
                raise ManifestError(f"Cannot stat episode path {child}: {exc}") from exc
            if stat.S_ISLNK(mode):
                raise ManifestError(f"Episode payload may not contain symlinks: {child}")
            if stat.S_ISDIR(mode):
                directories.append(relative)
                visit(child, relative)
            elif stat.S_ISREG(mode):
                files.append(relative)
            else:
                raise ManifestError(f"Episode payload contains a special file: {child}")

    visit(root, Path("."))
    return files, directories


def _updated_metadata_bytes(
    path: Path,
    manifest: OutboxManifest,
    final_index: int,
) -> bytes:
    try:
        metadata = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ManifestError(f"Cannot read episode metadata {path}: {exc}") from exc
    if not isinstance(metadata, dict):
        raise ManifestError(f"Episode metadata must be a JSON object: {path}")
    relative_episode_dir = f"{manifest.task}/{manifest.quality}/{final_index}"
    metadata["index"] = final_index
    metadata["relative_episode_dir"] = relative_episode_dir
    metadata["storage_state"] = "nas_committed"
    if "episode_id" in metadata:
        metadata["episode_id"] = relative_episode_dir
    return (
        json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


def _hash_stable_file(path: Path, chunk_size: int) -> tuple[int, str]:
    try:
        before = path.stat()
        if not stat.S_ISREG(before.st_mode) or path.is_symlink():
            raise ManifestError(f"Episode file is not a regular file: {path}")
        digest = hashlib.sha256()
        size = 0
        with path.open("rb") as handle:
            while True:
                chunk = handle.read(chunk_size)
                if not chunk:
                    break
                digest.update(chunk)
                size += len(chunk)
        after = path.stat()
    except OSError as exc:
        raise ManifestError(f"Cannot hash episode file {path}: {exc}") from exc
    if before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
        raise ManifestError(f"Episode file changed while hashing: {path}")
    if size != before.st_size:
        raise ManifestError(f"Episode file size changed while hashing: {path}")
    return size, digest.hexdigest()


def _verify_file(path: Path, prepared_file: PreparedFile, chunk_size: int) -> None:
    if path.is_symlink() or not path.is_file():
        raise VerificationError(f"NAS path is not a regular file: {path}")
    try:
        size = path.stat().st_size
    except OSError as exc:
        raise VerificationError(f"Cannot stat NAS file {path}: {exc}") from exc
    if size != prepared_file.size:
        raise VerificationError(
            f"NAS size mismatch for {prepared_file.relative_path}: "
            f"expected={prepared_file.size}, actual={size}"
        )
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            while True:
                chunk = handle.read(chunk_size)
                if not chunk:
                    break
                digest.update(chunk)
    except OSError as exc:
        raise VerificationError(f"Cannot reread NAS file {path}: {exc}") from exc
    if digest.hexdigest() != prepared_file.sha256:
        raise VerificationError(f"NAS SHA-256 mismatch for {prepared_file.relative_path}")


def _file_matches(path: Path, prepared_file: PreparedFile, chunk_size: int) -> bool:
    if path.is_symlink() or not path.is_file():
        return False
    try:
        _verify_file(path, prepared_file, chunk_size)
    except VerificationError:
        return False
    return True


def _clean_partial(partial_dir: Path, prepared: PreparedEpisode) -> None:
    allowed_files = {item.relative_path for item in prepared.files}
    allowed_directories = set(prepared.directories)
    for relative in allowed_files:
        parent = relative.parent
        while parent != Path("."):
            allowed_directories.add(parent)
            parent = parent.parent

    files, directories = _walk_regular_tree(partial_dir)
    for relative in files:
        if relative not in allowed_files:
            try:
                (partial_dir / relative).unlink()
            except OSError as exc:
                raise SyncError(f"Cannot clean stale NAS partial file {relative}: {exc}") from exc
    for relative in sorted(directories, key=lambda path: len(path.parts), reverse=True):
        if relative not in allowed_directories:
            try:
                (partial_dir / relative).rmdir()
            except OSError as exc:
                raise SyncError(
                    f"Cannot clean stale NAS partial directory {relative}: {exc}"
                ) from exc


def _sync_marker_payload(
    manifest: OutboxManifest,
    final_index: int,
    prepared: PreparedEpisode,
) -> dict:
    payload = {
        "schema_version": 2,
        "outbox_uuid": manifest.outbox_uuid,
        "task": manifest.task,
        "quality": manifest.quality,
        "requested_index": manifest.requested_index,
        "final_index": final_index,
        "frame_count": manifest.frame_count,
        "manifest_sha256": manifest.manifest_sha256,
        "directories": [path.as_posix() for path in prepared.directories],
        "files": [
            {
                "path": item.relative_path.as_posix(),
                "size": item.size,
                "sha256": item.sha256,
            }
            for item in prepared.files
        ],
    }
    payload["marker_sha256"] = _payload_sha256(payload)
    return payload


def _tombstone_receipt_payload(
    manifest: OutboxManifest,
    final_index: int,
    prepared: PreparedEpisode,
) -> dict:
    payload = {
        "schema_version": 1,
        "outbox_uuid": manifest.outbox_uuid,
        "task": manifest.task,
        "quality": manifest.quality,
        "final_index": final_index,
        "marker": _sync_marker_payload(manifest, final_index, prepared),
    }
    payload["receipt_sha256"] = _payload_sha256(payload)
    return payload


def _tombstone_receipt_path(root: Path, outbox_uuid: str) -> Path:
    safe_uuid = _safe_token(outbox_uuid, "tombstone outbox UUID")
    return root / f".{safe_uuid}{TOMBSTONE_RECEIPT_SUFFIX}"


def _receipt_outbox_uuid(path: Path) -> str | None:
    name = path.name
    if not name.startswith(".") or not name.endswith(TOMBSTONE_RECEIPT_SUFFIX):
        return None
    value = name[1 : -len(TOMBSTONE_RECEIPT_SUFFIX)]
    try:
        return _safe_token(value, "tombstone receipt UUID")
    except ManifestError:
        return None


def _read_tombstone_receipt(path: Path) -> dict:
    if path.is_symlink() or not path.is_file():
        raise VerificationError(f"Tombstone receipt is not a regular file: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise VerificationError(f"Cannot read tombstone receipt {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise VerificationError(f"Tombstone receipt must contain an object: {path}")
    checksum = payload.get("receipt_sha256")
    unsigned = dict(payload)
    unsigned.pop("receipt_sha256", None)
    if (
        not isinstance(checksum, str)
        or not SHA256_PATTERN.fullmatch(checksum)
        or _payload_sha256(unsigned) != checksum
    ):
        raise VerificationError(f"Tombstone receipt checksum is invalid: {path}")
    if payload.get("schema_version") != 1:
        raise VerificationError(f"Unsupported tombstone receipt schema: {path}")
    marker = payload.get("marker")
    if not isinstance(marker, dict):
        raise VerificationError(f"Tombstone receipt marker is invalid: {path}")
    _validate_sync_marker_payload(marker, path)
    for key in ("outbox_uuid", "task", "quality", "final_index"):
        if payload.get(key) != marker.get(key):
            raise VerificationError(f"Tombstone receipt {key} does not match marker: {path}")
    filename_uuid = _receipt_outbox_uuid(path)
    if filename_uuid != payload.get("outbox_uuid"):
        raise VerificationError(f"Tombstone receipt filename does not match UUID: {path}")
    return payload


def _read_sync_marker(root: Path) -> dict | None:
    marker_path = root / SYNC_MARKER_NAME
    if marker_path.is_symlink():
        raise VerificationError(f"NAS sync marker may not be a symlink: {marker_path}")
    if not marker_path.exists():
        return None
    if not marker_path.is_file():
        raise VerificationError(f"NAS sync marker is not a regular file: {marker_path}")
    try:
        payload = json.loads(marker_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise VerificationError(f"Cannot read NAS sync marker {marker_path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise VerificationError(f"NAS sync marker must contain an object: {marker_path}")
    _validate_sync_marker_payload(payload, marker_path)
    return payload


def _validate_sync_marker_payload(payload: Mapping, marker_path: Path) -> None:
    checksum = payload.get("marker_sha256")
    unsigned = dict(payload)
    unsigned.pop("marker_sha256", None)
    if not isinstance(checksum, str) or not SHA256_PATTERN.fullmatch(checksum):
        raise VerificationError(f"NAS sync marker checksum is invalid: {marker_path}")
    if _payload_sha256(unsigned) != checksum:
        raise VerificationError(f"NAS sync marker checksum mismatch: {marker_path}")
    if payload.get("schema_version") != 2:
        raise VerificationError(f"Unsupported NAS sync marker schema: {marker_path}")
    try:
        _safe_token(payload.get("outbox_uuid"), "marker outbox UUID")
        _safe_token(payload.get("task"), "marker task")
        _safe_token(payload.get("quality"), "marker quality")
    except (ManifestError, TypeError) as exc:
        raise VerificationError(f"NAS sync marker identity is invalid: {marker_path}") from exc
    for key in ("requested_index", "final_index", "frame_count"):
        value = payload.get(key)
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise VerificationError(
                f"NAS sync marker {key} is invalid: {marker_path}"
            )
    if payload.get("frame_count", 0) <= 0:
        raise VerificationError(f"NAS sync marker frame_count is invalid: {marker_path}")
    manifest_digest = payload.get("manifest_sha256")
    if not isinstance(manifest_digest, str) or not SHA256_PATTERN.fullmatch(
        manifest_digest
    ):
        raise VerificationError(f"NAS sync marker manifest digest is invalid: {marker_path}")

    directories_value = payload.get("directories")
    if not isinstance(directories_value, list):
        raise VerificationError(f"NAS sync marker directories are invalid: {marker_path}")
    directories = [
        _marker_relative_path(value, marker_path, "directory")
        for value in directories_value
    ]
    if len(set(directories)) != len(directories):
        raise VerificationError(f"NAS sync marker directories are duplicated: {marker_path}")

    files_value = payload.get("files")
    if not isinstance(files_value, list) or not files_value:
        raise VerificationError(f"NAS sync marker files are invalid: {marker_path}")
    file_paths: list[Path] = []
    for record in files_value:
        if not isinstance(record, dict):
            raise VerificationError(f"NAS sync marker file record is invalid: {marker_path}")
        relative = _marker_relative_path(record.get("path"), marker_path, "file")
        if relative == Path(SYNC_MARKER_NAME):
            raise VerificationError(f"NAS sync marker lists itself as payload: {marker_path}")
        size = record.get("size")
        digest = record.get("sha256")
        if not isinstance(size, int) or isinstance(size, bool) or size < 0:
            raise VerificationError(f"NAS sync marker file size is invalid: {marker_path}")
        if not isinstance(digest, str) or not SHA256_PATTERN.fullmatch(digest):
            raise VerificationError(f"NAS sync marker file digest is invalid: {marker_path}")
        file_paths.append(relative)
    if len(set(file_paths)) != len(file_paths):
        raise VerificationError(f"NAS sync marker file paths are duplicated: {marker_path}")


def _marker_relative_path(value: object, marker_path: Path, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise VerificationError(f"NAS sync marker {label} path is invalid: {marker_path}")
    relative = Path(value)
    if (
        relative.is_absolute()
        or not relative.parts
        or any(part in {"", ".", ".."} for part in relative.parts)
        or relative.as_posix() != value
    ):
        raise VerificationError(f"NAS sync marker {label} path is unsafe: {marker_path}")
    return relative


def _prepared_from_sync_marker(marker: Mapping) -> PreparedEpisode:
    files = tuple(
        PreparedFile(
            relative_path=Path(record["path"]),
            size=record["size"],
            sha256=record["sha256"],
        )
        for record in marker["files"]
    )
    return PreparedEpisode(
        directories=tuple(Path(value) for value in marker["directories"]),
        files=files,
    )


def _payload_sha256(payload: Mapping) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _find_published_index(quality_dir: Path, outbox_uuid: str) -> int | None:
    try:
        children = list(quality_dir.iterdir())
    except OSError as exc:
        raise SyncError(f"Cannot scan published NAS indices under {quality_dir}: {exc}") from exc
    matches: list[int] = []
    for child in children:
        if not child.is_dir() or not child.name.isdigit():
            continue
        marker = _read_sync_marker(child)
        if marker is not None and marker.get("outbox_uuid") == outbox_uuid:
            matches.append(int(child.name))
    if len(matches) > 1:
        raise SyncError(
            f"Outbox {outbox_uuid} appears in multiple NAS indices: {sorted(matches)}"
        )
    return matches[0] if matches else None


def _ensure_quality_dir(nas_root: Path, task: str, quality: str) -> Path:
    nas_root = _absolute_path(nas_root)
    if nas_root.is_symlink() or not nas_root.is_dir():
        raise SyncError(f"NAS mount root is not a real directory: {nas_root}")
    current = nas_root
    for token in (task, quality):
        current = current / token
        try:
            current.mkdir()
        except FileExistsError:
            pass
        except OSError as exc:
            raise SyncError(f"Cannot create NAS directory {current}: {exc}") from exc
        if current.is_symlink() or not current.is_dir():
            raise SyncError(f"NAS destination component is not a real directory: {current}")
    return current


def _write_json_atomic(path: Path, payload: Mapping) -> None:
    temp_path = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    try:
        with temp_path.open("x", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, path)
    finally:
        try:
            temp_path.unlink()
        except FileNotFoundError:
            pass


def _write_text_atomic(path: Path, text: str) -> None:
    temp_path = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    try:
        with temp_path.open("x", encoding="utf-8") as handle:
            handle.write(text)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, path)
    finally:
        try:
            temp_path.unlink()
        except FileNotFoundError:
            pass


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    except OSError:
        return
    try:
        os.fsync(descriptor)
    except OSError:
        pass
    finally:
        os.close(descriptor)


def _absolute_path(path: Path) -> Path:
    return Path(os.path.abspath(os.path.expanduser(os.fspath(path))))


def _safe_token(value: str, label: str) -> str:
    if not isinstance(value, str):
        raise ManifestError(f"Invalid {label}: {value!r}")
    token = value.strip()
    if token in {"", ".", ".."} or not SAFE_TOKEN_PATTERN.fullmatch(token):
        raise ManifestError(f"Invalid {label}: {value!r}")
    return token


def _required_string(payload: Mapping, key: str) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ManifestError(f"Manifest {key} must be a non-empty string")
    return value.strip()


def _required_nonnegative_int(payload: Mapping, key: str) -> int:
    value = payload.get(key)
    if value is None or isinstance(value, bool):
        raise ManifestError(f"Manifest {key} must be a non-negative integer")
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise ManifestError(f"Manifest {key} must be a non-negative integer") from exc
    if number < 0 or str(number) != str(value).strip():
        raise ManifestError(f"Manifest {key} must be a non-negative integer")
    return number


def _unescape_mountinfo(value: str) -> str:
    replacements = {
        "\\040": " ",
        "\\011": "\t",
        "\\012": "\n",
        "\\134": "\\",
    }
    for escaped, literal in replacements.items():
        value = value.replace(escaped, literal)
    return value


def _env_float(values: Mapping[str, str], name: str, default: float) -> float:
    text = values.get(name, "").strip()
    if not text:
        return default
    try:
        return float(text)
    except ValueError as exc:
        raise ValueError(f"{name} must be numeric, got {text!r}") from exc


def _env_bool(values: Mapping[str, str], name: str, default: bool) -> bool:
    text = values.get(name, "").strip().lower()
    if not text:
        return default
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be a boolean, got {text!r}")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish one READY Franka local outbox at a time to NAS."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--once", action="store_const", const="once", dest="mode")
    mode.add_argument("--watch", action="store_const", const="watch", dest="mode")
    parser.set_defaults(mode="once")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="inspect the next outbox and planned index without writing NAS or deleting local data",
    )
    parser.add_argument("--cache-root", type=Path, help="override FRANKA_SYNC_CACHE_ROOT")
    parser.add_argument("--nas-root", type=Path, help="override FRANKA_SYNC_NAS_ROOT")
    parser.add_argument("--activity-fresh-seconds", type=float)
    parser.add_argument("--max-load-per-cpu", type=float)
    parser.add_argument("--min-available-memory-mib", type=float)
    parser.add_argument("--rate-limit-mib-s", type=float)
    parser.add_argument("--poll-seconds", type=float)
    parser.add_argument(
        "--skip-mount-check",
        action="store_true",
        help="test-only override; never use this for normal NAS publishing",
    )
    parser.add_argument("--verbose", action="store_true")
    return parser


def _config_from_args(args: argparse.Namespace) -> SyncConfig:
    config = SyncConfig.from_env()
    updates = {}
    if args.cache_root is not None:
        updates["cache_root"] = args.cache_root.expanduser()
    if args.nas_root is not None:
        updates["nas_root"] = args.nas_root.expanduser()
    if args.activity_fresh_seconds is not None:
        updates["activity_fresh_seconds"] = args.activity_fresh_seconds
    if args.max_load_per_cpu is not None:
        updates["max_load_per_cpu"] = args.max_load_per_cpu
    if args.min_available_memory_mib is not None:
        updates["min_available_memory_bytes"] = int(args.min_available_memory_mib * MIB)
    if args.rate_limit_mib_s is not None:
        updates["rate_limit_bytes_per_second"] = args.rate_limit_mib_s * MIB
    if args.poll_seconds is not None:
        updates["poll_seconds"] = args.poll_seconds
    if args.skip_mount_check:
        updates["skip_mount_check"] = True
    return replace(config, **updates)


def _log_result(result: SyncResult) -> None:
    if result.status == "synced":
        LOGGER.info("Published %s -> %s", result.entry_dir, result.final_dir)
    elif result.status == "dry_run":
        LOGGER.info("Dry run: %s -> %s (%s)", result.entry_dir, result.final_dir, result.reason)
    elif result.status == "deferred":
        LOGGER.debug("Sync deferred for %s: %s", result.entry_dir, result.reason)
    else:
        LOGGER.debug("No READY outbox found")


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        config = _config_from_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    sync = NasSync(config)
    try:
        with sync:
            if not args.dry_run:
                sync.cleanup_synced_tombstones()
                sync.migrate_ready_recordings()
                sync.migrate_legacy_saving()
            if args.mode == "once":
                try:
                    result = sync.sync_once(dry_run=args.dry_run)
                except SyncError as exc:
                    LOGGER.error("Sync failed; local outbox was preserved: %s", exc)
                    return 1
                _log_result(result)
                return 0

            while True:
                try:
                    result = sync.sync_once(dry_run=args.dry_run)
                except SyncError as exc:
                    LOGGER.error("Sync failed; local outbox was preserved: %s", exc)
                    sync.sleep(config.retry_seconds)
                    continue
                _log_result(result)
                sync.sleep(config.poll_seconds)
    except InstanceLockError as exc:
        LOGGER.error("%s", exc)
        return 2
    except KeyboardInterrupt:
        LOGGER.info("NAS sync stopped")
        return 130


if __name__ == "__main__":
    sys.exit(main())
