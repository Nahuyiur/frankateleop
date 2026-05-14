"""Background episode persistence for the capture GUI."""

from __future__ import annotations

import gzip
import json
import pickle
import shutil
import traceback
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

from PyQt6 import QtCore

from franka_capture.recording.episode_writer import _create_video_writers


@dataclass
class EpisodeSaveRequest:
    output_root: str
    task: str
    index: int
    frames: List[Dict[str, Any]]
    keyframes: List[int]
    camera_names: List[str]
    video_fps: int
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def output_dir(self) -> Path:
        return Path(self.output_root).expanduser() / self.task / str(self.index)


class AsyncEpisodeSaver(QtCore.QObject):
    """Save completed episodes without blocking camera preview."""

    save_started = QtCore.pyqtSignal(str, int, str)
    save_finished = QtCore.pyqtSignal(str, int, str, int)
    save_failed = QtCore.pyqtSignal(str, int, str, str)
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
        self._increment_pending()
        self.save_started.emit(request.task, request.index, str(request.output_dir))
        future = self._executor.submit(_save_episode, request)
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
        try:
            output_dir, frame_count = future.result()
        except Exception:
            error = traceback.format_exc()
            self.save_failed.emit(request.task, request.index, str(request.output_dir), error)
        else:
            self.save_finished.emit(request.task, request.index, str(output_dir), frame_count)
        finally:
            self.queue_changed.emit(self._decrement_pending())


def _save_episode(request: EpisodeSaveRequest) -> tuple[Path, int]:
    output_dir = request.output_dir
    if output_dir.exists():
        raise FileExistsError(f"Episode output directory already exists: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=False)

    try:
        _write_videos(output_dir, request.camera_names, request.video_fps, request.frames)
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
                "keyframes": request.keyframes,
            }
        )
        _write_json(output_dir / "metadata.json", metadata)
    except Exception:
        if output_dir.exists():
            shutil.rmtree(output_dir, ignore_errors=True)
        raise

    return output_dir, len(request.frames)


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
