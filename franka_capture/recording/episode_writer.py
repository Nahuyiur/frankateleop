"""Episode file writer for FR3 capture."""

import gzip
import json
import pickle
from pathlib import Path
from typing import Any, Dict, Iterable, List

import numpy as np


class _OpenCVVideoWriter:
    def __init__(self, path: Path, fps: int) -> None:
        import cv2

        self._cv2 = cv2
        self._path = path
        self._fps = fps
        self._writer = None

    def append_data(self, rgb: np.ndarray) -> None:
        if self._writer is None:
            height, width = rgb.shape[:2]
            fourcc = self._cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = self._cv2.VideoWriter(
                str(self._path),
                fourcc,
                self._fps,
                (width, height),
            )
            if not self._writer.isOpened():
                raise RuntimeError(f"Failed to open OpenCV video writer: {self._path}")
        self._writer.write(rgb[:, :, ::-1])

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None


def _create_video_writers(output_dir: Path, camera_names: List[str], fps: int):
    if not camera_names:
        return {}

    try:
        import imageio
        import imageio_ffmpeg

        imageio_ffmpeg.get_ffmpeg_exe()
        return {
            name: imageio.get_writer(str(output_dir / f"{name}.mp4"), fps=fps)
            for name in camera_names
        }
    except Exception as imageio_error:
        try:
            return {
                name: _OpenCVVideoWriter(output_dir / f"{name}.mp4", fps)
                for name in camera_names
            }
        except Exception:
            raise RuntimeError(
                "No mp4 video backend is available. Install ffmpeg for imageio "
                "or install opencv-python in the capture environment."
            ) from imageio_error


class EpisodeWriter:
    """Collect frames in memory, stream RGB mp4 files, then save an episode."""

    def __init__(
        self,
        output_root: str,
        task: str,
        index: int,
        camera_names: Iterable[str],
        video_fps: int,
    ) -> None:
        self.output_root = Path(output_root).expanduser()
        self.task = task
        self.index = index
        self.output_dir = self.output_root / task / str(index)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.video_fps = video_fps
        self.camera_names = list(camera_names)
        self.frames: List[Dict[str, Any]] = []
        self.keyframes: List[int] = [0]
        self._closed = False

        self._writers = _create_video_writers(
            self.output_dir, self.camera_names, video_fps
        )

    def append(self, frame: Dict[str, Any], rgb_frames: Dict[str, np.ndarray]) -> None:
        if self._closed:
            raise RuntimeError("Cannot append to a closed EpisodeWriter")

        for name in self.camera_names:
            rgb = rgb_frames.get(name)
            if rgb is None:
                continue
            self._writers[name].append_data(np.ascontiguousarray(rgb))
        self.frames.append(frame)

    def add_keyframe(self) -> int:
        keyframe = len(self.frames)
        if not self.keyframes or self.keyframes[-1] != keyframe:
            self.keyframes.append(keyframe)
        return keyframe

    def close_videos(self) -> None:
        for writer in self._writers.values():
            writer.close()
        self._writers.clear()

    def finish(self) -> Path:
        if self._closed:
            return self.output_dir

        self.close_videos()

        trajectory_path = self.output_dir / f"{self.index}.pkl.gz"
        with gzip.open(trajectory_path, "wb", compresslevel=1) as f:
            pickle.dump(
                {"data": self.frames, "keyframes": self.keyframes},
                f,
                protocol=pickle.HIGHEST_PROTOCOL,
            )

        keyframes_path = self.output_dir / "keyframes.json"
        with keyframes_path.open("w", encoding="utf-8") as f:
            json.dump({"keyframes": self.keyframes}, f, ensure_ascii=False, indent=2)

        self._closed = True
        return self.output_dir

    def close(self) -> None:
        if self._closed:
            return
        self.close_videos()
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is None:
            self.finish()
        else:
            self.close()
        return False
