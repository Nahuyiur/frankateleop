"""Apply a calibrated camera-to-base transform to a recorded RGB-D episode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict

import numpy as np

from pointcloud.depth_proof import color_from_frame, depth_from_frame, depth_to_point_cloud, load_episode

from .geometry import matrix_from_list, transform_points
from .io import read_json, write_json


def _camera_serial(metadata: Dict[str, Any]) -> Any:
    for key in ("serial_number", "serial", "camera_serial", "device_serial"):
        value = metadata.get(key)
        if value is not None:
            return value
    return None


def _intrinsics_delta(calibration_metadata: Dict[str, Any], episode_metadata: Dict[str, Any]) -> Dict[str, float]:
    calib = calibration_metadata.get("intrinsics") or {}
    episode = episode_metadata.get("intrinsics") or {}
    delta: Dict[str, float] = {}
    for key in ("width", "height", "fx", "fy", "ppx", "ppy"):
        if key in calib and key in episode:
            delta[key] = abs(float(calib[key]) - float(episode[key]))
    return delta


def _load_extrinsic(path: Path) -> tuple[np.ndarray, Dict[str, Any]]:
    payload = read_json(path)
    if "T_base_camera" in payload:
        return matrix_from_list(payload["T_base_camera"], name="T_base_camera"), payload
    if "matrix" in payload:
        return matrix_from_list(payload["matrix"], name="matrix"), payload
    return matrix_from_list(payload, name="extrinsic"), payload


def validate_extrinsic_camera(
    extrinsic_payload: Dict[str, Any],
    *,
    camera_name: str,
    camera_metadata: Dict[str, Any],
    allow_camera_mismatch: bool = False,
    allow_metadata_mismatch: bool = False,
) -> list[str]:
    warnings: list[str] = []
    extrinsic_camera = extrinsic_payload.get("camera_name")
    if extrinsic_camera and str(extrinsic_camera) != str(camera_name) and not allow_camera_mismatch:
        raise RuntimeError(
            f"Extrinsic was solved for camera {extrinsic_camera!r}, "
            f"but --camera is {camera_name!r}. Use --allow-camera-mismatch only for debugging."
        )
    if not extrinsic_camera:
        warnings.append("extrinsic has no camera_name; camera/extrinsic match is only partially checked")

    extrinsic_metadata = extrinsic_payload.get("camera_metadata") or {}
    if isinstance(extrinsic_metadata, dict):
        extrinsic_serial = _camera_serial(extrinsic_metadata)
        episode_serial = _camera_serial(camera_metadata)
        if extrinsic_serial and episode_serial and str(extrinsic_serial) != str(episode_serial):
            message = (
                f"extrinsic serial {extrinsic_serial!r} does not match episode camera "
                f"serial {episode_serial!r}"
            )
            if allow_metadata_mismatch:
                warnings.append(message)
            else:
                raise RuntimeError(message + "; use --allow-metadata-mismatch only for debugging")
        delta = _intrinsics_delta(extrinsic_metadata, camera_metadata)
        if delta:
            resolution_mismatch = any(delta.get(key, 0.0) > 0.0 for key in ("width", "height"))
            intrinsics_mismatch = any(delta.get(key, 0.0) > 1e-3 for key in ("fx", "fy", "ppx", "ppy"))
            if resolution_mismatch or intrinsics_mismatch:
                message = f"calibration camera intrinsics differ from episode metadata: {delta}"
                if allow_metadata_mismatch:
                    warnings.append(message)
                else:
                    raise RuntimeError(message + "; use --allow-metadata-mismatch only for debugging")
    return warnings


def _write_ascii_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for point, color in zip(points, colors):
            f.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def apply_to_episode(
    episode_dir: Path,
    extrinsic_path: Path,
    *,
    camera_name: str,
    frame_spec: str = "middle",
    stride: int = 4,
    max_points: int = 120000,
    output_dir: Path = None,
    allow_camera_mismatch: bool = False,
    allow_metadata_mismatch: bool = False,
) -> Dict[str, Any]:
    frames, metadata, index = load_episode(episode_dir)
    if not frames:
        raise RuntimeError(f"No frames in {episode_dir}")
    if frame_spec == "first":
        frame_index = 0
    elif frame_spec == "middle":
        frame_index = len(frames) // 2
    elif frame_spec == "last":
        frame_index = len(frames) - 1
    else:
        frame_index = int(frame_spec)
        if frame_index < 0:
            frame_index = len(frames) + frame_index
    if frame_index < 0 or frame_index >= len(frames):
        raise IndexError(f"frame index {frame_index} out of range")

    camera_metadata = dict(metadata.get("cameras", {})).get(camera_name, {})
    intrinsics = camera_metadata.get("intrinsics") or {}
    if not intrinsics:
        raise RuntimeError(f"No intrinsics for camera {camera_name}")
    frame = frames[frame_index]
    bgr = color_from_frame(episode_dir, frame, camera_name, frame_index)
    depth = depth_from_frame(episode_dir, frame, camera_name, camera_metadata)
    if depth is None:
        raise RuntimeError(f"No depth for camera {camera_name} frame {frame_index}")
    transform_base_camera, extrinsic_payload = _load_extrinsic(extrinsic_path)
    metadata_warnings = validate_extrinsic_camera(
        extrinsic_payload,
        camera_name=camera_name,
        camera_metadata=camera_metadata,
        allow_camera_mismatch=allow_camera_mismatch,
        allow_metadata_mismatch=allow_metadata_mismatch,
    )
    points_camera, colors = depth_to_point_cloud(
        depth=depth,
        intrinsics=intrinsics,
        bgr_image=bgr,
        flip=bool(camera_metadata.get("flip", False)),
        stride=stride,
        max_points=max_points,
    )
    extrinsic_camera = extrinsic_payload.get("camera_name")
    points_base = transform_points(transform_base_camera, points_camera).astype(np.float32)
    output_dir = output_dir or episode_dir / "world_pointcloud"
    output_path = output_dir / f"{camera_name}_frame{frame_index:06d}_base_cloud.ply"
    _write_ascii_ply(output_path, points_base, colors)
    summary = {
        "episode_dir": str(episode_dir),
        "episode_index": int(index),
        "camera_name": camera_name,
        "frame_index": int(frame_index),
        "point_count": int(points_base.shape[0]),
        "pointcloud_frame": "robot_base",
        "extrinsic_path": str(extrinsic_path),
        "extrinsic_camera_name": extrinsic_camera,
        "metadata_warnings": metadata_warnings,
        "ply": str(output_path),
    }
    write_json(output_dir / f"{camera_name}_frame{frame_index:06d}_base_summary.json", summary)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_dir")
    parser.add_argument("--camera", required=True)
    parser.add_argument("--extrinsic", required=True)
    parser.add_argument("--frame", default="middle")
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--max-points", type=int, default=120000)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--allow-camera-mismatch", action="store_true")
    parser.add_argument("--allow-metadata-mismatch", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = apply_to_episode(
        Path(args.episode_dir).expanduser(),
        Path(args.extrinsic).expanduser(),
        camera_name=args.camera,
        frame_spec=args.frame,
        stride=args.stride,
        max_points=args.max_points,
        output_dir=Path(args.output_dir).expanduser() if args.output_dir else None,
        allow_camera_mismatch=args.allow_camera_mismatch,
        allow_metadata_mismatch=args.allow_metadata_mismatch,
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
