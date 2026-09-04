"""Backend-neutral camera construction."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, Iterable

from .realsense import RealSenseCapture, list_devices
from .v4l2 import V4L2Capture


def _config_dict(config: object) -> dict:
    if hasattr(config, "to_dict"):
        return dict(config.to_dict())
    if isinstance(config, dict):
        return dict(config)
    raise TypeError(f"Unsupported camera config: {type(config)}")


def create_cameras(camera_configs: Dict[str, object], allow_missing: bool = False):
    """Create a mixed set of RealSense and V4L2 cameras."""

    raw_configs = {name: _config_dict(config) for name, config in camera_configs.items()}
    needs_realsense_inventory = allow_missing and any(
        str(config.get("backend", "realsense")) == "realsense"
        for config in raw_configs.values()
    )
    available_realsense = set(list_devices()) if needs_realsense_inventory else None
    cameras = {}
    for name, raw in raw_configs.items():
        params = dict(raw)
        backend = str(params.pop("backend", "realsense")).strip().lower()
        params.setdefault("name", name)
        try:
            if backend == "realsense":
                params.pop("device_path", None)
                params.pop("pixel_format", None)
                serial = params.get("serial_number")
                if available_realsense is not None and serial not in available_realsense:
                    raise RuntimeError(f"serial {serial} is not connected")
                camera = RealSenseCapture(**params)
            elif backend == "v4l2":
                device_path = str(params.get("device_path", ""))
                if allow_missing and not Path(device_path).exists():
                    raise RuntimeError(f"device {device_path} is not connected")
                params.pop("serial_number", None)
                params.pop("depth", None)
                params.pop("align_depth", None)
                camera = V4L2Capture(**params)
            else:
                raise ValueError(f"Unknown camera backend {backend!r} for {name}")
        except Exception as exc:
            if allow_missing and not isinstance(exc, ValueError):
                print(
                    f"WARNING: skipping unavailable {backend} camera {name}: {exc}",
                    file=sys.stderr,
                )
                continue
            for opened in cameras.values():
                opened.close()
            raise
        cameras[name] = camera
    return cameras


def close_cameras(cameras: Iterable[object]) -> None:
    for camera in cameras:
        camera.close()
