from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "teleop"))

from teleop.agents.teleop_agent import PORT_CONFIG_MAP  # noqa: E402


CURRENT_LEFT_TELEOP_PORT = (
    "/dev/serial/by-id/"
    "usb-FTDI_USB__-__Serial_Converter_FTBJKECV-if00-port0"
)


def test_current_left_teleop_joint_zero_has_no_pi_offset() -> None:
    config = PORT_CONFIG_MAP[CURRENT_LEFT_TELEOP_PORT]
    assert np.isclose(config.joint_offsets[0], 0.0)
