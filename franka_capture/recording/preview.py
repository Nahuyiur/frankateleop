"""OpenCV preview helpers for RGB camera streams."""

from typing import Iterable, Optional, Tuple

import numpy as np


def concatenate_rgb_images(
    images: Iterable[np.ndarray],
    line_color: Tuple[int, int, int] = (255, 0, 0),
    line_width: int = 4,
) -> Optional[np.ndarray]:
    import cv2

    images = [img for img in images if img is not None]
    if not images:
        return None

    min_height = min(img.shape[0] for img in images)
    resized = []
    for img in images:
        scale = min_height / img.shape[0]
        resized.append(cv2.resize(img, (int(img.shape[1] * scale), min_height)))

    total_width = sum(img.shape[1] for img in resized)
    total_width += line_width * max(0, len(resized) - 1)
    display = np.zeros((min_height, total_width, 3), dtype=resized[0].dtype)

    x = 0
    for idx, img in enumerate(resized):
        width = img.shape[1]
        display[:min_height, x : x + width] = img
        x += width
        if idx < len(resized) - 1:
            display[:min_height, x : x + line_width] = line_color
            x += line_width
    return display


def show_rgb_preview(window_name: str, image_rgb: np.ndarray) -> int:
    import cv2

    if image_rgb is not None:
        cv2.imshow(window_name, image_rgb[:, :, ::-1])
    else:
        placeholder = np.zeros((240, 640, 3), dtype=np.uint8)
        cv2.putText(
            placeholder,
            "No RealSense cameras connected",
            (36, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            placeholder,
            "s=start, e=save, q=quit",
            (36, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (160, 160, 160),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(window_name, placeholder)
    return cv2.waitKey(1) & 0xFF
