"""Background episode persistence for the capture GUI."""

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
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

from PyQt6 import QtCore

from franka_capture.recording.episode_writer import _create_video_writers

PREVIEW_VIDEO_STEM = "preview_all"
PREVIEW_VIDEO_MAX_HEIGHT = 180
DEFAULT_CACHE_ROOT = Path.home() / "Desktop" / "franka_record_cache"
CACHE_ROOT_ENV = "FRANKA_GUI_RECORD_CACHE_ROOT"
PARTIAL_PREFIX = ".partial"
INVALID_PATH_PARTS = {"", ".", ".."}
SAVE_ERROR_CACHE_MISSING = "cache_missing"
SAVE_ERROR_FINAL_CONFLICT = "final_conflict"
SAVE_ERROR_LOCAL_WRITE_FAILED = "local_write_failed"
SAVE_ERROR_PUBLISH_FAILED = "publish_failed"
SAVE_ERROR_UNKNOWN = "unknown"


class EpisodeOutputConflictError(FileExistsError):
    """Raised when the final episode directory has already been claimed."""


class EpisodeSaveError(RuntimeError):
    def __init__(self, message: str, *, kind: str, cache_dir: str = "") -> None:
        super().__init__(message)
        self.kind = kind
        self.cache_dir = cache_dir


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
    """Save completed episodes without blocking camera preview."""

    save_started = QtCore.pyqtSignal(str, int, str)
    save_finished = QtCore.pyqtSignal(str, int, str, int)
    save_failed = QtCore.pyqtSignal(str, int, str, str, str)
    queue_changed = QtCore.pyqtSignal(int)

    def __init__(self, max_workers: int = 1, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self._executor = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="episode-save")
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
            future = self._executor.submit(_save_episode, request)
        except Exception:
            self.queue_changed.emit(self._decrement_pending())
            raise
        future.add_done_callback(lambda fut: self._handle_done(request, fut))

    def shutdown(self) -> None:
        self._executor.shutdown(wait=False)

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
            saved_output_dir, frame_count = future.result()
        except Exception as exc:
            kind = _failure_kind(exc)
            error = traceback.format_exc()
            self.save_failed.emit(request.task, request.index, output_dir, kind, error)
        else:
            self.save_finished.emit(request.task, request.index, str(saved_output_dir), frame_count)
        finally:
            self.queue_changed.emit(self._decrement_pending())


def _save_episode(request: EpisodeSaveRequest) -> tuple[Path, int]:
    final_output_dir = request.output_dir

    if request.publish_from_cache:
        if not request.local_cache_dir:
            raise EpisodeSaveError(
                "publish_from_cache requires local_cache_dir",
                kind=SAVE_ERROR_CACHE_MISSING,
            )
        staging_output_dir = Path(request.local_cache_dir).expanduser()
        if not staging_output_dir.is_dir():
            raise EpisodeSaveError(
                f"Local episode cache does not exist: {staging_output_dir}",
                kind=SAVE_ERROR_CACHE_MISSING,
                cache_dir=str(staging_output_dir),
            )
        save_id = uuid.uuid4().hex
        try:
            _publish_episode_to_output_root(staging_output_dir, final_output_dir, save_id)
        except Exception as exc:
            raise EpisodeSaveError(
                "Publishing preserved local cache to final output failed. "
                f"Local cache is still at: {staging_output_dir}",
                kind=_publish_failure_kind(exc),
                cache_dir=str(staging_output_dir),
            ) from exc
        _remove_cache_session(staging_output_dir, request.relative_episode_dir)
        return final_output_dir, len(request.frames)

    save_id = uuid.uuid4().hex
    staging_root = _cache_root() / ".saving" / save_id
    staging_output_dir = staging_root / request.relative_episode_dir

    try:
        staging_output_dir.mkdir(parents=True, exist_ok=False)
        _write_episode_files(staging_output_dir, request, request.relative_episode_dir)
        request.local_cache_dir = str(staging_output_dir)
    except Exception as exc:
        if staging_root.exists():
            shutil.rmtree(staging_root, ignore_errors=True)
        raise EpisodeSaveError(
            "Episode local cache write failed.",
            kind=SAVE_ERROR_LOCAL_WRITE_FAILED,
        ) from exc

    try:
        _publish_episode_to_output_root(staging_output_dir, final_output_dir, save_id)
    except Exception as exc:
        raise EpisodeSaveError(
            "Episode was written to local cache, but publishing to final output "
            f"failed. Local cache preserved at: {staging_output_dir}",
            kind=_publish_failure_kind(exc),
            cache_dir=str(staging_output_dir),
        ) from exc

    shutil.rmtree(staging_root, ignore_errors=True)
    return final_output_dir, len(request.frames)


def _write_episode_files(
    output_dir: Path,
    request: EpisodeSaveRequest,
    relative_episode_dir: Path,
) -> None:
    try:
        _write_videos(output_dir, request.camera_names, request.video_fps, request.frames)
        preview_metadata = _write_preview_video(
            output_dir,
            request.camera_names,
            request.video_fps,
            request.frames,
        )
        _write_pickle(output_dir, request.index, request.frames, request.keyframes)
        _write_json(output_dir / "keyframes.json", {"keyframes": request.keyframes})
        metadata = dict(request.metadata)
        metadata.update(
            {
                "task": request.task,
                "index": request.index,
                "frame_count": len(request.frames),
                "camera_names": request.camera_names,
                "video_fps": request.video_fps,
                "preview_video": preview_metadata,
                "keyframes": request.keyframes,
                "relative_episode_dir": relative_episode_dir.as_posix(),
                "episode_id": relative_episode_dir.as_posix(),
            }
        )
        text_instruction = request.text_instruction.strip()
        if text_instruction:
            metadata["text_instruction"] = text_instruction
        if request.quality:
            metadata["quality"] = request.quality
        _write_json(output_dir / "metadata.json", metadata)
        if text_instruction:
            _write_text(output_dir / "instruction.txt", text_instruction)
    except Exception:
        if output_dir.exists():
            shutil.rmtree(output_dir, ignore_errors=True)
        raise


def _publish_episode_to_output_root(
    staging_output_dir: Path,
    final_output_dir: Path,
    save_id: str,
) -> None:
    _ensure_publish_mount(final_output_dir)
    final_parent = final_output_dir.parent
    final_parent.mkdir(parents=True, exist_ok=True)
    if final_output_dir.exists():
        raise EpisodeOutputConflictError(
            f"Episode output directory already exists: {final_output_dir}"
        )

    partial_dir = final_parent / f"{PARTIAL_PREFIX}-{final_output_dir.name}-{save_id}"
    if partial_dir.exists():
        shutil.rmtree(partial_dir, ignore_errors=True)
    try:
        shutil.copytree(staging_output_dir, partial_dir, copy_function=shutil.copy2)
        if final_output_dir.exists():
            raise EpisodeOutputConflictError(
                f"Episode output directory already exists: {final_output_dir}"
            )
        partial_dir.rename(final_output_dir)
    except Exception:
        if partial_dir.exists():
            shutil.rmtree(partial_dir, ignore_errors=True)
        raise


def _ensure_publish_mount(final_output_dir: Path) -> None:
    nas_root = Path.home() / "Desktop" / "Muka_NAS"
    try:
        final_output_dir.expanduser().absolute().relative_to(nas_root.absolute())
    except ValueError:
        return
    if not _is_mount_path(nas_root):
        raise RuntimeError(
            f"NAS output root is not mounted at {nas_root}; local cache will be kept."
        )


def _is_mount_path(path: Path) -> bool:
    target = path.expanduser().absolute().as_posix()
    try:
        lines = Path("/proc/self/mountinfo").read_text(encoding="utf-8").splitlines()
    except OSError:
        return path.is_mount()
    for line in lines:
        parts = line.split()
        if len(parts) >= 5 and parts[4].replace("\\040", " ") == target:
            return True
    return False


def _publish_failure_kind(exc: Exception) -> str:
    if isinstance(exc, EpisodeOutputConflictError):
        return SAVE_ERROR_FINAL_CONFLICT
    return SAVE_ERROR_PUBLISH_FAILED


def _failure_kind(exc: Exception) -> str:
    if isinstance(exc, EpisodeSaveError):
        return exc.kind
    if isinstance(exc, EpisodeOutputConflictError):
        return SAVE_ERROR_FINAL_CONFLICT
    if isinstance(exc, FileNotFoundError):
        return SAVE_ERROR_CACHE_MISSING
    return SAVE_ERROR_UNKNOWN


def _cache_root() -> Path:
    configured = os.environ.get(CACHE_ROOT_ENV, "").strip()
    root = Path(configured).expanduser() if configured else DEFAULT_CACHE_ROOT
    root.mkdir(parents=True, exist_ok=True)
    return root


def _validate_path_token(value: str, label: str) -> str:
    token = str(value).strip()
    if token in INVALID_PATH_PARTS or "/" in token or "\\" in token or "\x00" in token:
        raise ValueError(f"Invalid {label} path token: {value!r}")
    return token


def _remove_cache_session(staging_output_dir: Path, relative_episode_dir: Path) -> None:
    session_dir = staging_output_dir
    for _ in relative_episode_dir.parts:
        session_dir = session_dir.parent
    shutil.rmtree(session_dir, ignore_errors=True)


def _write_videos(
    output_dir: Path,
    camera_names: List[str],
    video_fps: int,
    frames: List[Dict[str, Any]],
) -> None:
    writers = _create_video_writers(output_dir, camera_names, video_fps)
    try:
        for frame in frames:
            for name in camera_names:
                bgr = frame.get(f"{name}_image")
                if bgr is None:
                    continue
                rgb = np.ascontiguousarray(bgr[:, :, ::-1])
                writers[name].append_data(rgb)
    finally:
        for writer in writers.values():
            writer.close()


def _write_preview_video(
    output_dir: Path,
    camera_names: List[str],
    video_fps: int,
    frames: List[Dict[str, Any]],
) -> Dict[str, Any]:
    if not camera_names:
        return {}

    writers = _create_video_writers(output_dir, [PREVIEW_VIDEO_STEM], video_fps)
    writer = writers[PREVIEW_VIDEO_STEM]
    written = 0
    try:
        for frame in frames:
            preview_rgb = _make_preview_rgb(frame, camera_names)
            if preview_rgb is None:
                continue
            writer.append_data(preview_rgb)
            written += 1
    finally:
        for active_writer in writers.values():
            active_writer.close()

    return {
        "filename": f"{PREVIEW_VIDEO_STEM}.mp4",
        "camera_names": camera_names,
        "layout": "horizontal",
        "max_height": PREVIEW_VIDEO_MAX_HEIGHT,
        "frame_count": written,
    }


def _make_preview_rgb(frame: Dict[str, Any], camera_names: List[str]) -> np.ndarray | None:
    images: List[np.ndarray | None] = []
    for name in camera_names:
        bgr = frame.get(f"{name}_image")
        if bgr is None:
            images.append(None)
            continue
        if not hasattr(bgr, "shape") or len(bgr.shape) != 3 or bgr.shape[2] != 3:
            images.append(None)
            continue
        images.append(np.asarray(bgr))

    valid_images = [image for image in images if image is not None]
    if not valid_images:
        return None

    target_height = min(PREVIEW_VIDEO_MAX_HEIGHT, *(image.shape[0] for image in valid_images))
    target_height = max(1, int(target_height))
    default_width = max(1, int(round(valid_images[0].shape[1] * target_height / max(1, valid_images[0].shape[0]))))
    resized_rgb = []
    for bgr in images:
        if bgr is None:
            resized_rgb.append(np.zeros((target_height, default_width, 3), dtype=np.uint8))
            continue
        height, width = bgr.shape[:2]
        target_width = max(1, int(round(width * target_height / max(1, height))))
        resized_bgr = _resize_image(bgr, target_width, target_height)
        resized_rgb.append(resized_bgr[:, :, ::-1])

    return np.ascontiguousarray(np.concatenate(resized_rgb, axis=1))


def _resize_image(image: np.ndarray, width: int, height: int) -> np.ndarray:
    try:
        import cv2

        return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    except Exception:
        y_idx = np.linspace(0, image.shape[0] - 1, height).astype(int)
        x_idx = np.linspace(0, image.shape[1] - 1, width).astype(int)
        return image[y_idx][:, x_idx]


def _write_pickle(
    output_dir: Path,
    index: int,
    frames: List[Dict[str, Any]],
    keyframes: List[int],
) -> None:
    trajectory_path = output_dir / f"{index}.pkl.gz"
    with gzip.open(trajectory_path, "wb", compresslevel=1) as f:
        pickle.dump(
            {"data": frames, "keyframes": keyframes},
            f,
            protocol=pickle.HIGHEST_PROTOCOL,
        )


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")


def _write_text(path: Path, text: str) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write(text)
        f.write("\n")
