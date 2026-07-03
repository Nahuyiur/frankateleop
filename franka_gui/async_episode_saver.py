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

PREVIEW_VIDEO_STEM = "preview_all"
PREVIEW_VIDEO_MAX_HEIGHT = 180


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

    @property
    def output_dir(self) -> Path:
        task_dir = Path(self.output_root).expanduser() / self.task
        if self.quality:
            task_dir = task_dir / self.quality
        return task_dir / str(self.index)


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
        try:
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
    relative_episode_dir = Path(request.task)
    if request.quality:
        relative_episode_dir = relative_episode_dir / request.quality
    relative_episode_dir = relative_episode_dir / str(request.index)

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
