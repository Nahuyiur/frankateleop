"""Bounded-memory local video spool used by the capture GUI."""

from __future__ import annotations

import os
import queue
import shutil
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Iterable

import numpy as np

from franka_capture.recording.episode_writer import _create_video_writers

from .storage_paths import record_cache_root

PREVIEW_VIDEO_STEM = "preview_all"
PREVIEW_VIDEO_MAX_HEIGHT = 180
PREVIEW_VIDEO_ENCODED_HEIGHT = 192
DEFAULT_VIDEO_QUEUE_SIZE = 8
DEFAULT_VIDEO_CLOSE_TIMEOUT_SEC = 10.0


class EpisodeSpoolError(RuntimeError):
    """Raised when the local streaming spool cannot keep a complete episode."""


class EpisodeSpoolBackpressure(EpisodeSpoolError):
    """Raised instead of growing memory when video writers fall behind."""


class StreamingEpisodeSpool:
    """Stream RGB frames to local videos while retaining only a bounded queue."""

    def __init__(
        self,
        camera_names: Iterable[str],
        video_fps: int,
        *,
        cache_root: Path | None = None,
        queue_size: int | None = None,
        close_timeout_sec: float | None = None,
    ) -> None:
        self.camera_names = tuple(camera_names)
        if len(set(self.camera_names)) != len(self.camera_names):
            raise ValueError(f"Duplicate camera names are not allowed: {self.camera_names}")
        self.video_fps = max(1, int(video_fps))
        configured_size = os.environ.get("FRANKA_GUI_VIDEO_QUEUE_SIZE", "").strip()
        if queue_size is None:
            queue_size = int(configured_size) if configured_size else DEFAULT_VIDEO_QUEUE_SIZE
        configured_close_timeout = os.environ.get(
            "FRANKA_GUI_VIDEO_CLOSE_TIMEOUT_SEC", ""
        ).strip()
        if close_timeout_sec is None:
            close_timeout_sec = (
                float(configured_close_timeout)
                if configured_close_timeout
                else DEFAULT_VIDEO_CLOSE_TIMEOUT_SEC
            )
        self._close_timeout_sec = float(close_timeout_sec)
        if not np.isfinite(self._close_timeout_sec) or self._close_timeout_sec <= 0:
            raise ValueError(f"Invalid video close timeout: {self._close_timeout_sec}")
        self._queue: queue.Queue[Dict[str, np.ndarray] | object] = queue.Queue(
            maxsize=max(1, int(queue_size))
        )
        self._sentinel = object()
        self._sentinel_enqueued = False
        self.session_id = uuid.uuid4().hex
        self.session_dir = (cache_root or record_cache_root()) / ".recording" / self.session_id
        self.episode_dir = self.session_dir / "episode"
        self._writers: Dict[str, Any] = {}
        self._preview_writer = None
        self._camera_shapes: Dict[str, tuple[int, int, int]] = {}
        self._preview_shape: tuple[int, int, int] | None = None
        self._enqueued = 0
        self._written = 0
        self._error: BaseException | None = None
        self._closed = False
        self._closing = False
        self._worker: threading.Thread | None = None

        preview_writers: Dict[str, Any] = {}
        try:
            self.episode_dir.mkdir(parents=True, exist_ok=False)
            self._writers = _create_video_writers(
                self.episode_dir,
                list(self.camera_names),
                self.video_fps,
                low_cpu=True,
            )
            missing_writers = set(self.camera_names) - set(self._writers)
            if missing_writers:
                raise EpisodeSpoolError(
                    f"Video backend did not create writers for: {sorted(missing_writers)}"
                )
            preview_writers = _create_video_writers(
                self.episode_dir,
                [PREVIEW_VIDEO_STEM] if self.camera_names else [],
                self.video_fps,
                low_cpu=True,
            )
            self._preview_writer = preview_writers.get(PREVIEW_VIDEO_STEM)
            if self.camera_names and self._preview_writer is None:
                raise EpisodeSpoolError("Video backend did not create the preview writer")
            self._worker = threading.Thread(
                target=self._writer_loop,
                name=f"episode-video-{self.session_id[:8]}",
                daemon=True,
            )
            self._worker.start()
        except BaseException:
            _close_writers([*self._writers.values(), *preview_writers.values()])
            self._writers.clear()
            self._preview_writer = None
            shutil.rmtree(self.session_dir, ignore_errors=True)
            raise

    @property
    def frame_count(self) -> int:
        return self._enqueued

    @property
    def writer_alive(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def append(self, rgb_frames: Dict[str, np.ndarray]) -> None:
        if self._closed or self._closing:
            raise EpisodeSpoolError("Cannot append to a closed episode spool")
        self._raise_worker_error()

        missing = [name for name in self.camera_names if name not in rgb_frames]
        if missing:
            raise EpisodeSpoolError(
                f"Missing camera frame(s) while recording: {missing}. "
                "The episode was paused to avoid camera/action misalignment."
            )

        owned_frames: Dict[str, np.ndarray] = {}
        for name in self.camera_names:
            image = np.asarray(rgb_frames[name])
            if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
                raise EpisodeSpoolError(
                    f"Camera {name} returned invalid RGB frame: "
                    f"shape={image.shape}, dtype={image.dtype}"
                )
            shape = (
                int(image.shape[0]),
                int(image.shape[1]),
                int(image.shape[2]),
            )
            expected = self._camera_shapes.setdefault(name, shape)
            if shape != expected:
                raise EpisodeSpoolError(
                    f"Camera {name} resolution changed during one episode: "
                    f"expected={expected}, got={shape}"
                )
            # RealSense and other camera SDKs may reuse their backing buffers.
            # The queued frame must own its pixels until the writer consumes it.
            owned_frames[name] = np.array(image, copy=True, order="C")

        try:
            self._queue.put(owned_frames, timeout=0.05)
        except queue.Full as exc:
            raise EpisodeSpoolBackpressure(
                "Local video writer cannot keep up with capture. Recording was paused "
                "instead of allowing the image queue to consume unbounded memory."
            ) from exc
        self._enqueued += 1

    def finish(self) -> Dict[str, Any]:
        self._close()
        self._raise_worker_error()
        if self._written != self._enqueued:
            raise EpisodeSpoolError(
                f"Video spool frame mismatch: queued={self._enqueued}, written={self._written}"
            )

        cameras = {
            name: {
                "filename": f"{name}.mp4",
                "width": int(self._camera_shapes[name][1]),
                "height": int(self._camera_shapes[name][0]),
                "channels": int(self._camera_shapes[name][2]),
                "frame_count": self._written,
            }
            for name in self.camera_names
            if name in self._camera_shapes
        }
        return {
            "type": "video",
            "frame_alignment": "frame_index",
            "decoded_color_order": "BGR",
            "source_color_order": "RGB",
            "camera_resolution_preserved": True,
            "cameras": cameras,
            "preview": self._preview_metadata(),
        }

    def discard(self) -> None:
        self._close()
        shutil.rmtree(self.session_dir, ignore_errors=True)

    def _close(self) -> None:
        if self._closed:
            return
        self._closing = True
        deadline = time.monotonic() + self._close_timeout_sec
        worker = self._worker
        if worker is None or not worker.is_alive():
            self._closed = True
            return

        if not self._sentinel_enqueued:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                self._queue.put(self._sentinel, timeout=remaining)
            except queue.Full as exc:
                if not worker.is_alive():
                    self._closed = True
                    return
                raise EpisodeSpoolError(
                    "Timed out stopping the local video writer; the recording directory "
                    f"was preserved at {self.session_dir}"
                ) from exc
            self._sentinel_enqueued = True

        worker.join(timeout=max(0.0, deadline - time.monotonic()))
        if worker.is_alive():
            raise EpisodeSpoolError(
                "Timed out waiting for the local video writer; the recording directory "
                f"was preserved at {self.session_dir}"
            )
        self._closed = True

    def _writer_loop(self) -> None:
        try:
            while True:
                item = self._queue.get()
                try:
                    if item is self._sentinel:
                        return
                    assert isinstance(item, dict)
                    if self._error is not None:
                        continue
                    for name in self.camera_names:
                        self._writers[name].append_data(item[name])
                    if self._preview_writer is not None:
                        preview = _make_preview_rgb(item, self.camera_names)
                        preview_shape = tuple(int(value) for value in preview.shape)
                        if self._preview_shape is None:
                            self._preview_shape = preview_shape
                        elif preview_shape != self._preview_shape:
                            raise EpisodeSpoolError(
                                "Preview resolution changed during one episode: "
                                f"expected={self._preview_shape}, got={preview_shape}"
                            )
                        self._preview_writer.append_data(preview)
                    self._written += 1
                except BaseException as exc:
                    if self._error is None:
                        self._error = exc
                finally:
                    self._queue.task_done()
        finally:
            for writer in self._writers.values():
                try:
                    writer.close()
                except BaseException as exc:
                    if self._error is None:
                        self._error = exc
            if self._preview_writer is not None:
                try:
                    self._preview_writer.close()
                except BaseException as exc:
                    if self._error is None:
                        self._error = exc

    def _raise_worker_error(self) -> None:
        if self._error is not None:
            raise EpisodeSpoolError(f"Local video writer failed: {self._error}") from self._error

    def _preview_metadata(self) -> Dict[str, Any]:
        if self._preview_shape is None:
            return {}
        height, width, channels = self._preview_shape
        return {
            "filename": f"{PREVIEW_VIDEO_STEM}.mp4",
            "camera_names": list(self.camera_names),
            "layout": "horizontal",
            "max_height": PREVIEW_VIDEO_ENCODED_HEIGHT,
            "content_max_height": PREVIEW_VIDEO_MAX_HEIGHT,
            "width": width,
            "height": height,
            "channels": channels,
            "frame_count": self._written,
        }


def _close_writers(writers: Iterable[Any]) -> None:
    seen = set()
    for writer in writers:
        if writer is None or id(writer) in seen:
            continue
        seen.add(id(writer))
        try:
            writer.close()
        except BaseException:
            pass


def _make_preview_rgb(
    rgb_frames: Dict[str, np.ndarray],
    camera_names: Iterable[str],
) -> np.ndarray:
    names = list(camera_names)
    images = [rgb_frames[name] for name in names]
    target_height = max(
        1,
        min(PREVIEW_VIDEO_MAX_HEIGHT, *(int(image.shape[0]) for image in images)),
    )
    resized = []
    for image in images:
        height, width = image.shape[:2]
        target_width = max(1, int(round(width * target_height / max(1, height))))
        resized.append(_resize_rgb(image, target_width, target_height))
    preview = np.ascontiguousarray(np.concatenate(resized, axis=1))
    padding = PREVIEW_VIDEO_ENCODED_HEIGHT - int(preview.shape[0])
    if padding <= 0:
        return preview
    top = padding // 2
    padded = np.zeros(
        (PREVIEW_VIDEO_ENCODED_HEIGHT, int(preview.shape[1]), 3),
        dtype=np.uint8,
    )
    padded[top : top + preview.shape[0]] = preview
    return padded


def _resize_rgb(image: np.ndarray, width: int, height: int) -> np.ndarray:
    try:
        import cv2

        return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    except Exception:
        y_idx = np.linspace(0, image.shape[0] - 1, height).astype(int)
        x_idx = np.linspace(0, image.shape[1] - 1, width).astype(int)
        return image[y_idx][:, x_idx]
