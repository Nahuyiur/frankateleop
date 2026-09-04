from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

from franka_capture.cameras import V4L2Capture, create_cameras
from franka_capture.config.fr3_single import V4L2CameraConfig


class _FakeVideoCapture:
    def __init__(self, device_path: str, backend: int) -> None:
        self.device_path = device_path
        self.backend = backend
        self.opened = True
        self.settings: list[tuple[int, float]] = []

    def isOpened(self) -> bool:
        return self.opened

    def set(self, key: int, value: float) -> bool:
        self.settings.append((key, value))
        return True

    def read(self):
        time.sleep(0.005)
        if not self.opened:
            return False, None
        bgr = np.zeros((2, 3, 3), dtype=np.uint8)
        bgr[:, :, 0] = 11
        bgr[:, :, 1] = 22
        bgr[:, :, 2] = 33
        return True, bgr

    def release(self) -> None:
        self.opened = False


def test_v4l2_capture_returns_rgb_and_backend_metadata(
    tmp_path: Path, monkeypatch
) -> None:
    device = tmp_path / "video0"
    device.touch()
    fake = _FakeVideoCapture(str(device), cv2.CAP_V4L2)
    monkeypatch.setattr(cv2, "VideoCapture", lambda path, backend: fake)

    camera = V4L2Capture(
        name="left_wrist",
        device_path=str(device),
        dim=(640, 480),
        fps=30,
        pixel_format="MJPG",
    )
    try:
        rgb, depth = camera.read()
        assert depth is None
        assert rgb.shape == (2, 3, 3)
        assert rgb[0, 0].tolist() == [33, 22, 11]
        assert camera.metadata()["backend"] == "v4l2"
        assert camera.metadata()["device_path"] == str(device)
    finally:
        camera.close()
    assert not fake.opened


def test_camera_factory_constructs_v4l2_profile(tmp_path: Path, monkeypatch) -> None:
    device = tmp_path / "video0"
    device.touch()
    created: list[_FakeVideoCapture] = []

    def fake_capture(path: str, backend: int):
        capture = _FakeVideoCapture(path, backend)
        created.append(capture)
        return capture

    monkeypatch.setattr(cv2, "VideoCapture", fake_capture)
    cameras = create_cameras(
        {
            "left_wrist": V4L2CameraConfig(
                name="left_wrist",
                device_path=str(device),
            )
        }
    )
    try:
        assert set(cameras) == {"left_wrist"}
        assert cameras["left_wrist"].metadata()["backend"] == "v4l2"
    finally:
        cameras["left_wrist"].close()
    assert created and not created[0].opened
