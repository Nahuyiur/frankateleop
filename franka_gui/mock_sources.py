"""Mock robot and camera sources for developing the GUI without hardware."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np


@dataclass
class MockCameraConfig:
    name: str
    dim: Tuple[int, int] = (640, 480)
    fps: int = 30

    def to_dict(self):
        return {"name": self.name, "dim": self.dim, "fps": self.fps}


class MockCamera:
    def __init__(self, name: str, dim: Tuple[int, int] = (640, 480), fps: int = 30) -> None:
        self.name = name
        self.dim = dim
        self.fps = fps
        self._t0 = time.time()
        self._last = 0.0

    def read(self):
        import cv2

        width, height = self.dim
        now = time.time()
        min_dt = 1.0 / max(1, self.fps)
        if self._last and now - self._last < min_dt:
            time.sleep(min_dt - (now - self._last))
        self._last = time.time()

        t = self._last - self._t0
        x_grad = np.linspace(0, 255, width, dtype=np.uint8)[None, :]
        y_grad = np.linspace(0, 255, height, dtype=np.uint8)[:, None]
        rgb = np.zeros((height, width, 3), dtype=np.uint8)
        rgb[:, :, 0] = (x_grad + int(t * 25)) % 255
        rgb[:, :, 1] = (y_grad + int(t * 40)) % 255
        rgb[:, :, 2] = 120
        cv2.putText(
            rgb,
            f"MOCK {self.name}",
            (24, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            rgb,
            time.strftime("%H:%M:%S"),
            (24, 96),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return rgb, None

    def close(self) -> None:
        return None

    def metadata(self):
        return {
            "name": self.name,
            "serial_number": "mock",
            "dim": list(self.dim),
            "fps": self.fps,
            "depth": False,
            "align_depth": False,
            "flip": False,
            "mock": True,
        }


class MockRobot:
    def __init__(self) -> None:
        self._t0 = time.time()

    def num_dofs(self) -> int:
        return 8

    def get_joint_state(self) -> np.ndarray:
        t = time.time() - self._t0
        joints = np.asarray([0.35 * math.sin(t * 0.6 + i * 0.35) for i in range(7)], dtype=float)
        gripper_norm = 0.5 + 0.4 * math.sin(t * 0.8)
        return np.concatenate([joints, [gripper_norm]])

    def get_observations(self) -> Dict[str, np.ndarray]:
        t = time.time() - self._t0
        gripper_norm = float(self.get_joint_state()[-1])
        gripper_command = 1.0 if gripper_norm < 0.5 else 0.0
        return {
            "ee_pose_euler": np.asarray(
                [
                    0.45 + 0.03 * math.sin(t * 0.5),
                    0.02 * math.cos(t * 0.5),
                    0.35 + 0.02 * math.sin(t * 0.7),
                    0.0,
                    0.0,
                    0.25 * math.sin(t * 0.4),
                ],
                dtype=float,
            ),
            "gripper_command": np.asarray([gripper_command], dtype=float),
            "gripper_command_raw": np.asarray([gripper_command], dtype=float),
            "gripper_target_width": np.asarray([0.09 * (1.0 - gripper_command)], dtype=float),
            "gripper_command_timestamp": np.asarray([time.time()], dtype=float),
            "gripper_command_source": "mock",
        }

    def close(self) -> None:
        return None


def create_mock_cameras(camera_names, fps: int):
    return {name: MockCamera(name=name, fps=fps) for name in camera_names}
