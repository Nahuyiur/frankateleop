"""RealSense camera capture for RGB and optional aligned depth."""

from typing import Dict, Iterable, Optional, Tuple

import numpy as np


def _load_rs():
    import pyrealsense2 as rs

    return rs


def list_devices():
    rs = _load_rs()
    devices = rs.context().query_devices()
    return [dev.get_info(rs.camera_info.serial_number) for dev in devices]


def _intrinsics_to_dict(intrinsics):
    return {
        "width": intrinsics.width,
        "height": intrinsics.height,
        "ppx": intrinsics.ppx,
        "ppy": intrinsics.ppy,
        "fx": intrinsics.fx,
        "fy": intrinsics.fy,
        "model": str(intrinsics.model),
        "coeffs": list(intrinsics.coeffs),
    }


class RealSenseCapture:
    """Single RealSense RGB/depth stream.

    The camera returns RGB images because the recording schema names the field
    `*_rgb`. OpenCV display/write helpers convert back to BGR where required.
    """

    def __init__(
        self,
        name: str,
        serial_number: str,
        dim: Tuple[int, int] = (640, 480),
        fps: int = 15,
        depth: bool = False,
        align_depth: bool = True,
        flip: bool = False,
        read_timeout_ms: int = 15000,
    ) -> None:
        rs = _load_rs()
        connected = list_devices()
        if serial_number not in connected:
            raise RuntimeError(
                f"RealSense camera {name} with serial {serial_number} is not connected. "
                f"Connected devices: {connected}"
            )

        self.name = name
        self.serial_number = serial_number
        self.dim = dim
        self.fps = fps
        self.depth_enabled = depth
        self.align_depth = align_depth
        self.flip = flip
        self.read_timeout_ms = read_timeout_ms
        self.depth_scale: Optional[float] = None
        self.intrinsics = None
        self.intrinsics_dict = None

        self._rs = rs
        self._pipeline = rs.pipeline()
        self._config = rs.config()
        self._config.enable_device(serial_number)
        self._config.enable_stream(rs.stream.color, dim[0], dim[1], rs.format.bgr8, fps)
        if depth:
            self._config.enable_stream(
                rs.stream.depth, dim[0], dim[1], rs.format.z16, fps
            )

        self._profile = self._pipeline.start(self._config)
        self._align = rs.align(rs.stream.color) if depth and align_depth else None

        color_stream = self._profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        self.intrinsics_dict = _intrinsics_to_dict(self.intrinsics)

        if depth:
            depth_sensor = self._profile.get_device().first_depth_sensor()
            self.depth_scale = float(depth_sensor.get_depth_scale())

    def read(self):
        import cv2

        try:
            frames = self._pipeline.wait_for_frames(self.read_timeout_ms)
        except RuntimeError as exc:
            raise RuntimeError(
                f"Camera {self.name} did not return frames within "
                f"{self.read_timeout_ms} ms"
            ) from exc
        if self._align is not None:
            frames = self._align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError(f"Camera {self.name} did not return a color frame")

        image_bgr = np.asarray(color_frame.get_data())
        image_rgb = image_bgr[:, :, ::-1].copy()
        depth_image = None

        if self.depth_enabled:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_raw = np.asarray(depth_frame.get_data())
                scale = 1.0 if self.depth_scale is None else self.depth_scale
                depth_image = (depth_raw.astype(np.float32) * scale)[:, :, None]

        if self.flip:
            image_rgb = cv2.rotate(image_rgb, cv2.ROTATE_180)
            if depth_image is not None:
                depth_image = cv2.rotate(depth_image[:, :, 0], cv2.ROTATE_180)[
                    :, :, None
                ]

        return image_rgb, depth_image

    def close(self) -> None:
        self._pipeline.stop()
        self._config.disable_all_streams()

    def metadata(self):
        return {
            "name": self.name,
            "serial_number": self.serial_number,
            "dim": list(self.dim),
            "fps": self.fps,
            "depth": self.depth_enabled,
            "align_depth": self.align_depth,
            "flip": self.flip,
            "read_timeout_ms": self.read_timeout_ms,
            "depth_scale": self.depth_scale,
            "intrinsics": self.intrinsics_dict,
        }

    def __repr__(self) -> str:
        return (
            f"RealSenseCapture(name={self.name!r}, serial_number={self.serial_number!r}, "
            f"depth={self.depth_enabled})"
        )


def create_realsense_cameras(camera_configs: Dict[str, object]):
    cameras = {}
    try:
        for name, config in camera_configs.items():
            if hasattr(config, "to_dict"):
                kwargs = config.to_dict()
            elif isinstance(config, dict):
                kwargs = dict(config)
            else:
                raise TypeError(f"Unsupported camera config for {name}: {type(config)}")
            kwargs.setdefault("name", name)
            cameras[name] = RealSenseCapture(**kwargs)
        return cameras
    except Exception:
        for camera in cameras.values():
            camera.close()
        raise


def close_cameras(cameras: Iterable[RealSenseCapture]) -> None:
    for camera in cameras:
        camera.close()
