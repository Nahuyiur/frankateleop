"""Atomic PNG snapshot persistence for the capture GUIs."""

from __future__ import annotations

import os
import re
import shutil
import uuid
from datetime import datetime
from pathlib import Path
from typing import Mapping

import numpy as np


KEYFRAME_DIR_NAME = "keyframe"
_TIMESTAMP_RE = re.compile(r"^[0-9]{8}_[0-9]{6}_[0-9]{6}$")
_CAMERA_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def save_snapshot_frames(
    output_root: str | Path,
    frames: Mapping[str, np.ndarray],
    *,
    timestamp: str | None = None,
) -> Path:
    """Save a complete RGB camera batch as PNGs and atomically publish it.

    The caller owns copying frames out of the GUI thread. A hidden sibling
    directory is used while encoding so consumers only observe complete
    timestamp directories under ``<output_root>/keyframe``.
    """
    _validate_frames(frames)
    timestamp = timestamp or datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    if not _TIMESTAMP_RE.fullmatch(timestamp):
        raise ValueError(f"Invalid snapshot timestamp: {timestamp!r}")

    keyframe_root = Path(output_root).expanduser() / KEYFRAME_DIR_NAME
    keyframe_root.mkdir(parents=True, exist_ok=True)
    destination = keyframe_root / timestamp
    if destination.exists():
        raise FileExistsError(f"Snapshot directory already exists: {destination}")

    staging = keyframe_root / f".partial-{timestamp}-{uuid.uuid4().hex}"
    try:
        staging.mkdir()
        for name, rgb in frames.items():
            _write_rgb_png(staging / f"{name}.png", rgb)
        _fsync_directory(staging)
        os.replace(staging, destination)
        _fsync_directory(keyframe_root)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return destination


def _validate_frames(frames: Mapping[str, np.ndarray]) -> None:
    if not frames:
        raise ValueError("No camera frames are available for the snapshot")
    for name, rgb in frames.items():
        if not _CAMERA_NAME_RE.fullmatch(name):
            raise ValueError(f"Invalid camera name for snapshot: {name!r}")
        if not isinstance(rgb, np.ndarray) or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"Camera {name} must provide an HxWx3 RGB array")
        if rgb.dtype != np.uint8:
            raise ValueError(f"Camera {name} RGB frame must have uint8 dtype")


def _write_rgb_png(path: Path, rgb: np.ndarray) -> None:
    import cv2

    success, encoded = cv2.imencode(".png", np.ascontiguousarray(rgb[:, :, ::-1]))
    if not success:
        raise RuntimeError(f"Failed to encode PNG: {path.name}")
    with path.open("wb") as handle:
        handle.write(encoded.tobytes())
        handle.flush()
        os.fsync(handle.fileno())


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
