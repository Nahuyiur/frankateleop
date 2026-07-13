"""Episode file writer for FR3 capture."""

import gzip
import json
import os
import pickle
import shutil
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

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


def _create_video_writers(
    output_dir: Path,
    camera_names: List[str],
    fps: int,
    *,
    low_cpu: bool = False,
):
    if not camera_names:
        return {}

    try:
        import imageio
        import imageio_ffmpeg

        imageio_ffmpeg.get_ffmpeg_exe()
        writer_options = {}
        if low_cpu:
            writer_options = {
                "codec": os.environ.get("FRANKA_GUI_VIDEO_CODEC", "libx264"),
                "pixelformat": "yuv420p",
                "macro_block_size": 1,
                "quality": None,
                "output_params": [
                    "-preset",
                    os.environ.get("FRANKA_GUI_VIDEO_PRESET", "ultrafast"),
                    "-crf",
                    os.environ.get("FRANKA_GUI_VIDEO_CRF", "18"),
                    "-threads",
                    os.environ.get("FRANKA_GUI_VIDEO_THREADS", "1"),
                ],
            }
        return {
            name: imageio.get_writer(
                str(output_dir / f"{name}.mp4"),
                fps=fps,
                **writer_options,
            )
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
        metadata: Optional[Dict[str, Any]] = None,
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
        self.metadata = dict(metadata) if metadata is not None else None
        self._closed = False
        self._camera_shapes: Dict[str, tuple[int, int, int]] = {}

        self._writers = _create_video_writers(
            self.output_dir, self.camera_names, video_fps
        )

    def append(self, frame: Dict[str, Any], rgb_frames: Dict[str, np.ndarray]) -> None:
        if self._closed:
            raise RuntimeError("Cannot append to a closed EpisodeWriter")

        embedded_images = sorted(key for key in frame if key.endswith("_image"))
        if embedded_images:
            raise ValueError(
                "EpisodeWriter stores RGB only in MP4 files; refusing embedded "
                f"image fields in pkl: {embedded_images}"
            )

        for name in self.camera_names:
            rgb = rgb_frames.get(name)
            if rgb is None:
                raise RuntimeError(f"Missing RGB frame for configured camera: {name}")
            rgb = np.ascontiguousarray(rgb)
            shape = tuple(int(value) for value in rgb.shape)
            expected = self._camera_shapes.setdefault(name, shape)
            if shape != expected:
                raise ValueError(f"Camera {name} resolution changed within episode")
            self._writers[name].append_data(rgb)
        self.frames.append(frame)

    def add_keyframe(self) -> int:
        keyframe = len(self.frames)
        if not self.keyframes or self.keyframes[-1] != keyframe:
            self.keyframes.append(keyframe)
        return keyframe

    def update_metadata(self, values: Dict[str, Any]) -> None:
        if self.metadata is None:
            self.metadata = {}
        self.metadata.update(values)

    def close_videos(self) -> None:
        for writer in self._writers.values():
            writer.close()
        self._writers.clear()

    def finish(self) -> Path:
        if self._closed:
            return self.output_dir

        self.close_videos()
        image_storage = self._image_storage()

        trajectory_path = self.output_dir / f"{self.index}.pkl.gz"
        with gzip.open(trajectory_path, "wb", compresslevel=1) as f:
            pickle.dump(
                {
                    "data": self.frames,
                    "keyframes": self.keyframes,
                    "schema_version": (self.metadata or {}).get("schema_version", ""),
                    "image_storage": image_storage,
                },
                f,
                protocol=pickle.HIGHEST_PROTOCOL,
            )

        keyframes_path = self.output_dir / "keyframes.json"
        with keyframes_path.open("w", encoding="utf-8") as f:
            json.dump({"keyframes": self.keyframes}, f, ensure_ascii=False, indent=2)

        if self.metadata is not None:
            metadata = dict(self.metadata)
            metadata.update(
                {
                    "image_storage": image_storage,
                    "task": self.task,
                    "index": self.index,
                    "frame_count": len(self.frames),
                    "camera_names": self.camera_names,
                    "video_fps": self.video_fps,
                    "keyframes": self.keyframes,
                }
            )
            metadata_path = self.output_dir / "metadata.json"
            with metadata_path.open("w", encoding="utf-8") as f:
                json.dump(metadata, f, ensure_ascii=False, indent=2)
                f.write("\n")

        self._closed = True
        return self.output_dir

    def _image_storage(self) -> Dict[str, Any]:
        cameras = {
            name: {
                "filename": f"{name}.mp4",
                "width": self._camera_shapes[name][1],
                "height": self._camera_shapes[name][0],
                "channels": self._camera_shapes[name][2],
                "frame_count": len(self.frames),
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
        }

    def discard(self) -> Path:
        self.close_videos()
        if self.output_dir.exists():
            shutil.rmtree(self.output_dir)
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
