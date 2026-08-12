"""Finalize streamed GUI episodes into a local outbox or NAS staging area."""

from __future__ import annotations

import gzip
import json
import os
import pickle
import shutil
import traceback
import uuid
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from PyQt6 import QtCore

from .storage_paths import CACHE_ROOT_ENV, DEFAULT_CACHE_ROOT, record_cache_root

OUTBOX_DIR_NAME = "outbox"
OUTBOX_MANIFEST_NAME = "outbox.json"
OUTBOX_READY_NAME = "READY"
INVALID_PATH_PARTS = {"", ".", ".."}
SAVE_ERROR_CACHE_MISSING = "cache_missing"
SAVE_ERROR_FINAL_CONFLICT = "final_conflict"
SAVE_ERROR_LOCAL_WRITE_FAILED = "local_write_failed"
SAVE_ERROR_PUBLISH_FAILED = "publish_failed"
SAVE_ERROR_VALIDATION_FAILED = "validation_failed"
SAVE_ERROR_VALIDATION_RUNTIME_FAILED = "validation_runtime_failed"
SAVE_ERROR_UNKNOWN = "unknown"


class EpisodeSaveError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        kind: str,
        cache_dir: str = "",
        validation_issues: Optional[List[str]] = None,
    ) -> None:
        super().__init__(message)
        self.kind = kind
        self.cache_dir = cache_dir
        self.validation_issues = list(validation_issues or [])


@dataclass(frozen=True)
class EpisodeValidationResult:
    status: str
    issues: List[str]


@dataclass
class EpisodeSaveRequest:
    output_root: str
    task: str
    index: int
    frames: List[Dict[str, Any]]
    keyframes: List[int]
    camera_names: List[str]
    video_fps: int
    quality: str = ""
    text_instruction: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)
    local_cache_dir: str = ""
    publish_from_cache: bool = False
    direct_to_output_root: bool = False
    work_attempt_id: str = ""

    @property
    def relative_episode_dir(self) -> Path:
        task = _validate_path_token(self.task, "task")
        episode_dir = Path(task)
        if self.quality:
            episode_dir = episode_dir / _validate_path_token(self.quality, "quality")
        return episode_dir / str(self.index)

    @property
    def output_dir(self) -> Path:
        return Path(self.output_root).expanduser() / self.relative_episode_dir


class AsyncEpisodeSaver(QtCore.QObject):
    """Finalize episodes in one bounded background worker."""

    save_started = QtCore.pyqtSignal(str, int, str)
    save_finished = QtCore.pyqtSignal(str, int, str, int)
    save_failed = QtCore.pyqtSignal(str, int, str, str, str)
    validation_started = QtCore.pyqtSignal(str, int, str)
    validation_finished = QtCore.pyqtSignal(str, int, str, list)
    queue_changed = QtCore.pyqtSignal(int)

    def __init__(self, max_workers: int = 1, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self._executor = ThreadPoolExecutor(
            max_workers=max_workers,
            thread_name_prefix="episode-local-save",
        )
        self._pending = 0
        self._lock = QtCore.QMutex()

    def pending_count(self) -> int:
        locker = QtCore.QMutexLocker(self._lock)
        try:
            return self._pending
        finally:
            del locker

    def enqueue(self, request: EpisodeSaveRequest) -> None:
        output_dir = str(request.output_dir)
        self._increment_pending()
        try:
            self.save_started.emit(request.task, request.index, output_dir)
            future = self._executor.submit(
                _save_episode,
                request,
                validation_started=self.validation_started.emit,
                validation_finished=self.validation_finished.emit,
            )
        except Exception:
            self.queue_changed.emit(self._decrement_pending())
            raise
        future.add_done_callback(lambda fut: self._handle_done(request, fut))

    def shutdown(self) -> None:
        self._executor.shutdown(wait=True)

    def _increment_pending(self) -> None:
        locker = QtCore.QMutexLocker(self._lock)
        try:
            self._pending += 1
            pending = self._pending
        finally:
            del locker
        self.queue_changed.emit(pending)

    def _decrement_pending(self) -> int:
        locker = QtCore.QMutexLocker(self._lock)
        try:
            self._pending = max(0, self._pending - 1)
            return self._pending
        finally:
            del locker

    def _handle_done(self, request: EpisodeSaveRequest, future) -> None:
        output_dir = str(request.output_dir)
        try:
            _, frame_count = future.result()
        except Exception as exc:
            kind = _failure_kind(exc)
            error = traceback.format_exc()
            self.save_failed.emit(request.task, request.index, output_dir, kind, error)
        else:
            self.save_finished.emit(request.task, request.index, output_dir, frame_count)
        finally:
            self.queue_changed.emit(self._decrement_pending())


def _save_episode(
    request: EpisodeSaveRequest,
    *,
    validation_started: Optional[Callable[[str, int, str], None]] = None,
    validation_finished: Optional[Callable[[str, int, str, List[str]], None]] = None,
) -> tuple[Path, int]:
    if not request.local_cache_dir:
        raise EpisodeSaveError(
            "Streamed episode has no local cache directory",
            kind=SAVE_ERROR_CACHE_MISSING,
        )
    source_episode_dir = Path(request.local_cache_dir).expanduser()
    if not source_episode_dir.is_dir():
        raise EpisodeSaveError(
            f"Local streamed episode does not exist: {source_episode_dir}",
            kind=SAVE_ERROR_CACHE_MISSING,
            cache_dir=str(source_episode_dir),
        )
    if not request.quality:
        raise EpisodeSaveError(
            "Episode quality must be selected before local finalization",
            kind=SAVE_ERROR_LOCAL_WRITE_FAILED,
            cache_dir=str(source_episode_dir),
        )

    if request.direct_to_output_root and request.output_dir.exists():
        raise EpisodeSaveError(
            f"Final NAS episode directory already exists: {request.output_dir}",
            kind=SAVE_ERROR_FINAL_CONFLICT,
            cache_dir=str(source_episode_dir),
        )

    try:
        _write_episode_sidecars(source_episode_dir, request)
        if validation_started is not None:
            validation_started(request.task, request.index, str(request.output_dir))
        validation = _validate_staged_episode(source_episode_dir)
        if validation_finished is not None:
            validation_finished(
                request.task,
                request.index,
                validation.status,
                validation.issues,
            )
        if validation.status == "FAIL":
            _discard_validation_failed_episode(source_episode_dir, request)
            raise EpisodeSaveError(
                "Episode failed automatic validation and was discarded before publication: "
                + "; ".join(validation.issues),
                kind=SAVE_ERROR_VALIDATION_FAILED,
                validation_issues=validation.issues,
            )
        if request.direct_to_output_root:
            saved_episode_dir = _commit_direct_to_output_root(source_episode_dir, request)
        else:
            saved_episode_dir = _commit_to_outbox(source_episode_dir, request)
    except EpisodeSaveError as exc:
        if exc.cache_dir:
            request.local_cache_dir = exc.cache_dir
        raise
    except Exception as exc:
        location = "NAS staging" if request.direct_to_output_root else "the local outbox"
        raise EpisodeSaveError(
            f"Episode videos are preserved, but finalizing {location} failed. "
            f"The cache is preserved at: {source_episode_dir}",
            kind=SAVE_ERROR_LOCAL_WRITE_FAILED,
            cache_dir=str(source_episode_dir),
        ) from exc

    request.local_cache_dir = str(saved_episode_dir)
    return saved_episode_dir, len(request.frames)


def _validate_staged_episode(episode_dir: Path) -> EpisodeValidationResult:
    """Run the same default episode checks exposed through V_validate_task_data.sh."""
    try:
        from validate.validate_task import default_episode_validation_args, validate_episode

        report = validate_episode(episode_dir, default_episode_validation_args())
    except Exception as exc:
        raise EpisodeSaveError(
            "Automatic episode validation could not run; the episode was not published. "
            f"Staging cache is preserved at: {episode_dir}. Error: {exc}",
            kind=SAVE_ERROR_VALIDATION_RUNTIME_FAILED,
            cache_dir=str(episode_dir),
        ) from exc
    return EpisodeValidationResult(
        status=report.status,
        issues=[f"{issue.code}: {issue.message}" for issue in report.issues],
    )


def _discard_validation_failed_episode(
    source_episode_dir: Path,
    request: EpisodeSaveRequest,
) -> None:
    """Remove only the known staging session after a data-quality failure."""
    source = source_episode_dir.resolve()
    if request.direct_to_output_root:
        output_root = Path(request.output_root).expanduser().resolve()
        recording_root = (output_root / ".recording").resolve()
        if _is_relative_to(source, recording_root):
            discard_root = source.parent
        elif (
            source.name.startswith(f".partial-{request.index}-")
            and source.parent == request.output_dir.parent.resolve()
        ):
            discard_root = source
        else:
            raise EpisodeSaveError(
                f"Refusing to discard validation-failed episode outside managed staging: {source}",
                kind=SAVE_ERROR_VALIDATION_FAILED,
            )
    else:
        recording_root = (_cache_root() / ".recording").resolve()
        if not _is_relative_to(source, recording_root):
            raise EpisodeSaveError(
                f"Refusing to discard validation-failed episode outside local staging: {source}",
                kind=SAVE_ERROR_VALIDATION_FAILED,
            )
        discard_root = source.parent
    shutil.rmtree(discard_root)


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _write_episode_sidecars(output_dir: Path, request: EpisodeSaveRequest) -> None:
    relative_episode_dir = request.relative_episode_dir
    metadata = dict(request.metadata)
    metadata.update(
        {
            "task": request.task,
            "index": request.index,
            "frame_count": len(request.frames),
            "camera_names": list(request.camera_names),
            "video_fps": int(request.video_fps),
            "keyframes": list(request.keyframes),
            "relative_episode_dir": relative_episode_dir.as_posix(),
            "episode_id": relative_episode_dir.as_posix(),
            "quality": request.quality,
            "storage_state": (
                "direct_nas" if request.direct_to_output_root else "local_outbox"
            ),
        }
    )
    text_instruction = request.text_instruction.strip()
    if text_instruction:
        metadata["text_instruction"] = text_instruction

    payload = {
        "data": request.frames,
        "keyframes": request.keyframes,
        "schema_version": metadata.get("schema_version", ""),
        "image_storage": metadata.get("image_storage", {}),
    }
    for stale_pickle in output_dir.glob("*.pkl.gz"):
        if stale_pickle.name != f"{request.index}.pkl.gz":
            stale_pickle.unlink()
    _write_pickle_atomic(output_dir / f"{request.index}.pkl.gz", payload)
    _write_json_atomic(output_dir / "keyframes.json", {"keyframes": request.keyframes})
    _write_json_atomic(output_dir / "metadata.json", metadata)
    if text_instruction:
        _write_text_atomic(output_dir / "instruction.txt", text_instruction)


def _commit_to_outbox(source_episode_dir: Path, request: EpisodeSaveRequest) -> Path:
    session_dir = source_episode_dir.parent
    if session_dir.name in {"", ".", ".."}:
        raise ValueError(f"Invalid recording session directory: {session_dir}")

    cache_root = _cache_root()
    recording_root = (cache_root / ".recording").resolve()
    try:
        session_dir.resolve().relative_to(recording_root)
    except ValueError as exc:
        raise ValueError(
            f"Refusing to move a streamed episode outside {recording_root}: {session_dir}"
        ) from exc

    episode_uuid = session_dir.name
    if not episode_uuid or any(char not in "0123456789abcdef" for char in episode_uuid.lower()):
        episode_uuid = uuid.uuid4().hex

    # READY means every episode byte has reached the local filesystem, not just
    # Python's page cache. The NAS worker only scans READY outboxes, so a power
    # loss cannot expose a partially durable episode as publishable work.
    _fsync_regular_tree(source_episode_dir)
    _fsync_directory(source_episode_dir)
    manifest = {
        "schema_version": 1,
        "episode_uuid": episode_uuid,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "output_root": str(Path(request.output_root).expanduser()),
        "task": request.task,
        "quality": request.quality,
        "requested_index": int(request.index),
        "frame_count": len(request.frames),
        "camera_names": list(request.camera_names),
        "episode_subdir": source_episode_dir.name,
    }
    _write_json_atomic(session_dir / OUTBOX_MANIFEST_NAME, manifest)
    _write_text_atomic(session_dir / OUTBOX_READY_NAME, episode_uuid)
    _fsync_directory(session_dir)

    outbox_root = cache_root / OUTBOX_DIR_NAME
    outbox_root.mkdir(parents=True, exist_ok=True)
    destination = outbox_root / episode_uuid
    if destination.exists():
        raise FileExistsError(f"Local outbox UUID already exists: {destination}")
    session_dir.replace(destination)
    _fsync_directory(outbox_root)
    return destination / source_episode_dir.name


def _commit_direct_to_output_root(
    source_episode_dir: Path,
    request: EpisodeSaveRequest,
) -> Path:
    """Atomically publish a fully-written NAS staging episode.

    Streamed videos and sidecars live under the NAS .recording/<uuid> directory
    until every writer is closed. Both staging and the final task directory are
    on the same NAS mount, so the final rename cannot expose a partial episode.
    """

    session_dir = source_episode_dir.parent
    output_root = Path(request.output_root).expanduser().resolve()
    recording_root = (output_root / ".recording").resolve()
    final_dir = request.output_dir
    final_parent = final_dir.parent
    final_parent.mkdir(parents=True, exist_ok=True)
    if final_dir.exists():
        raise EpisodeSaveError(
            f"Final NAS episode directory already exists: {final_dir}",
            kind=SAVE_ERROR_FINAL_CONFLICT,
            cache_dir=str(source_episode_dir),
        )

    # A retry after an interrupted final rename uses the preserved hidden
    # directory directly. Sidecars may be safely rewritten before this call.
    resume_partial = (
        source_episode_dir.parent == final_parent
        and source_episode_dir.name.startswith(f".partial-{request.index}-")
    )
    if resume_partial:
        partial_dir = source_episode_dir
    else:
        try:
            session_dir.resolve().relative_to(recording_root)
        except ValueError as exc:
            raise ValueError(
                f"Refusing to publish an episode outside NAS staging {recording_root}: "
                f"{session_dir}"
            ) from exc
        partial_dir = final_parent / f".partial-{request.index}-{session_dir.name}"
        if partial_dir.exists():
            raise EpisodeSaveError(
                f"NAS staging directory already exists: {partial_dir}",
                kind=SAVE_ERROR_PUBLISH_FAILED,
                cache_dir=str(source_episode_dir),
            )

    # Flush before the first rename. A failure after this point leaves an
    # inspectable hidden .partial directory rather than a visible episode.
    _fsync_regular_tree(source_episode_dir)
    _fsync_directory(source_episode_dir)
    try:
        if not resume_partial:
            source_episode_dir.replace(partial_dir)
            _fsync_directory(final_parent)
        partial_dir.replace(final_dir)
        _fsync_directory(final_parent)
        if not resume_partial:
            try:
                session_dir.rmdir()
            except OSError:
                pass
            _fsync_directory(recording_root)
    except Exception as exc:
        preserved = partial_dir if partial_dir.exists() else source_episode_dir
        raise EpisodeSaveError(
            "NAS staging publish failed; the incomplete episode remains hidden at: "
            f"{preserved}",
            kind=SAVE_ERROR_PUBLISH_FAILED,
            cache_dir=str(preserved),
        ) from exc
    return final_dir


def _write_pickle_atomic(path: Path, payload: Dict[str, Any]) -> None:
    temp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        with gzip.open(temp, "wb", compresslevel=1) as handle:
            pickle.dump(payload, handle, protocol=pickle.HIGHEST_PROTOCOL)
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    _write_text_atomic(
        path,
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True),
    )


def _write_text_atomic(path: Path, text: str) -> None:
    temp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        with temp.open("w", encoding="utf-8") as handle:
            handle.write(text)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


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


def _fsync_regular_tree(root: Path) -> None:
    if root.is_symlink() or not root.is_dir():
        raise ValueError(f"Episode cache is not a real directory: {root}")
    directories = [root]
    for current_root, directory_names, file_names in os.walk(root, followlinks=False):
        current = Path(current_root)
        for directory_name in directory_names:
            directory = current / directory_name
            if directory.is_symlink():
                raise ValueError(f"Episode cache contains a directory symlink: {directory}")
            directories.append(directory)
        for file_name in file_names:
            path = current / file_name
            if path.is_symlink() or not path.is_file():
                raise ValueError(f"Episode cache contains a non-regular file: {path}")
            descriptor = os.open(
                path,
                os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0),
            )
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
    for directory in reversed(directories):
        _fsync_directory(directory)


def _failure_kind(exc: Exception) -> str:
    if isinstance(exc, EpisodeSaveError):
        return exc.kind
    if isinstance(exc, FileNotFoundError):
        return SAVE_ERROR_CACHE_MISSING
    return SAVE_ERROR_UNKNOWN


def _cache_root() -> Path:
    return record_cache_root()


def _validate_path_token(value: str, label: str) -> str:
    token = str(value).strip()
    if token in INVALID_PATH_PARTS or "/" in token or "\\" in token or "\x00" in token:
        raise ValueError(f"Invalid {label} path token: {value!r}")
    return token
