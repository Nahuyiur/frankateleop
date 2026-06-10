"""Default single-arm FR3 recording configuration.

Edit this file for local hardware names and RealSense serial numbers. The
capture package does not import teleop or polymetis; these values only describe
how to read the already-running robot node and cameras.
"""

from dataclasses import asdict, dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class RobotEndpointConfig:
    host: str = "127.0.0.1"
    port: int = 6001
    timeout_ms: int = 2000

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class CameraConfig:
    name: str
    serial_number: str
    dim: Tuple[int, int] = (640, 480)
    fps: int = 30
    depth: bool = False
    align_depth: bool = True
    flip: bool = False
    read_timeout_ms: int = 15000

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class RecordingConfig:
    output_root: str = "./record_data"
    video_fps: int = 30
    preview_line_width: int = 4

    def to_dict(self):
        return asdict(self)


DEFAULT_ROBOT = RobotEndpointConfig()
SINGLE_SCHEMA_VERSION = "franka_single_v1"
SINGLE_GUI_CAMERA_NAMES = ("left_wrist", "left", "middle")

# Update these serial numbers after running the camera self-check on the target
# machine. Depth is disabled by default to minimize USB bandwidth pressure.
DEFAULT_CAMERAS: Dict[str, CameraConfig] = {
    # 按照这个格式后续添加相机
    "left_wrist": CameraConfig(
        name="left_wrist",
        serial_number="348122072222",
        fps=30,
        depth=False,
    ),
    "left": CameraConfig(
        name="left",
        serial_number="347522070848",
        fps=30,
        depth=False,
    ),
    "middle": CameraConfig(
        name="middle",
        serial_number="332522072275",
        fps=30,
        depth=False,
    ),
    "right": CameraConfig(
        name="right",
        serial_number="332522072875",
        fps=30,
        depth=False,
    ),
    "right_wrist": CameraConfig(
        name="right_wrist",
        serial_number="347622075798",
        fps=30,
        depth=False,
    ),
    # "front": CameraConfig(
    #     name="front",
    #     serial_number="401622071099",
    #     fps=30,
    #     depth=False,
    # )
}

DEFAULT_RECORDING = RecordingConfig()


def cameras_as_metadata():
    return {name: config.to_dict() for name, config in DEFAULT_CAMERAS.items()}
