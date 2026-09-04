from __future__ import annotations

import threading

import cv2
import numpy as np

from .model import EpisodeReview


class EpisodeFrameProvider:
    def __init__(self, review: EpisodeReview) -> None:
        self.review = review
        self._captures = {name: cv2.VideoCapture(str(path)) for name, path in review.video_paths.items()}
        failed = [name for name, capture in self._captures.items() if not capture.isOpened()]
        if failed:
            self.close()
            raise RuntimeError(f"无法打开相机视频: {', '.join(failed)}")
        truncated = []
        for name, capture in self._captures.items():
            frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
            if frame_count > 0 and frame_count < review.frame_count:
                truncated.append(f"{name}({frame_count}/{review.frame_count})")
        if truncated:
            self.close()
            raise RuntimeError(f"相机视频帧数不足: {', '.join(truncated)}")
        self._next_index = {name: 0 for name in self._captures}
        self._lock = threading.Lock()

    def read(self, frame_index: int) -> dict[str, np.ndarray]:
        index = max(0, min(int(frame_index), self.review.frame_count - 1))
        images: dict[str, np.ndarray] = {}
        with self._lock:
            for name in self.review.camera_names:
                if name in self._captures:
                    capture = self._captures[name]
                    if self._next_index[name] != index:
                        capture.set(cv2.CAP_PROP_POS_FRAMES, index)
                    ok, image = capture.read()
                    if not ok or image is None:
                        raise RuntimeError(f"{name} 无法读取 frame {index}")
                    self._next_index[name] = index + 1
                    images[name] = image
                    continue
                field = self.review.embedded_camera_fields.get(name)
                if field:
                    image = self.review.frames[index].get(field)
                    if (
                        not isinstance(image, np.ndarray)
                        or image.ndim != 3
                        or image.shape[2] != 3
                        or image.dtype != np.uint8
                    ):
                        raise RuntimeError(f"{name} 的内嵌 frame {index} 缺失或格式错误")
                    images[name] = image
                    continue
                raise RuntimeError(f"{name} 没有可读取的 frame source")
        return images

    def close(self) -> None:
        for capture in self._captures.values():
            capture.release()
        self._captures.clear()
