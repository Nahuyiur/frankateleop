"""Polymetis Cartesian-controller health checks shared by replay entrypoints."""

from __future__ import annotations

import time

import numpy as np
import torch
from polymetis import RobotInterface
from scipy.spatial.transform import Rotation


class ControllerMonitor:
    def __init__(self, host: str, port: int) -> None:
        self.robot = RobotInterface(ip_address=host, port=port)

    def is_running(self) -> bool:
        return bool(self.robot.is_running_policy())

    def ensure_running(self, timeout: float = 3.0) -> bool:
        if self.is_running():
            return False
        print("Cartesian controller is stopped; starting it at the current pose.")
        self.robot.start_cartesian_impedance()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.is_running():
                return True
            time.sleep(0.05)
        raise RuntimeError("Cartesian controller did not become ready")

    def forward_kinematics_euler(self, joints: np.ndarray) -> np.ndarray:
        position, quaternion = self.robot.robot_model.forward_kinematics(
            torch.as_tensor(joints, dtype=torch.float32)
        )
        position = position.detach().cpu().numpy()
        quaternion = quaternion.detach().cpu().numpy()
        return np.r_[position, Rotation.from_quat(quaternion).as_euler("xyz")]
