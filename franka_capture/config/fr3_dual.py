"""Default dual-arm FR3 recording configuration."""

import os

from franka_capture.config.fr3_single import (
    DEFAULT_CAMERAS,
    DEFAULT_RECORDING,
    CameraConfig,
    RecordingConfig,
    RobotEndpointConfig,
    cameras_as_metadata,
)

DUAL_SCHEMA_VERSION = "franka_dual_v2"


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return int(value)


DEFAULT_LEFT_ROBOT = RobotEndpointConfig(host="127.0.0.1", port=6002, timeout_ms=2000)
DEFAULT_RIGHT_ROBOT = RobotEndpointConfig(
    host=os.environ.get("FRANKA_RIGHT_ZMQ_HOST", "192.168.1.131"),
    port=_env_int("FRANKA_RIGHT_ZMQ_PORT", 6001),
    timeout_ms=2000,
)

__all__ = [
    "CameraConfig",
    "RecordingConfig",
    "RobotEndpointConfig",
    "DEFAULT_CAMERAS",
    "DEFAULT_RECORDING",
    "DEFAULT_LEFT_ROBOT",
    "DEFAULT_RIGHT_ROBOT",
    "DUAL_SCHEMA_VERSION",
    "cameras_as_metadata",
]
