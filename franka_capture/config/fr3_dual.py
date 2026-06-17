"""Default dual-arm FR3 recording configuration."""

from franka_capture.config.fr3_single import (
    DEFAULT_CAMERAS,
    DEFAULT_RECORDING,
    CameraConfig,
    RecordingConfig,
    RobotEndpointConfig,
    cameras_as_metadata,
)

DUAL_SCHEMA_VERSION = "franka_dual_v2"

DEFAULT_LEFT_ROBOT = RobotEndpointConfig(host="127.0.0.1", port=6002, timeout_ms=2000)
DEFAULT_RIGHT_ROBOT = RobotEndpointConfig(host="127.0.0.1", port=16001, timeout_ms=2000)

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
