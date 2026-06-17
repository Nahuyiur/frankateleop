"""Utilities for validating recorded aligned depth frames.

Depth is stored as a per-pixel metric image. A point cloud is a derived view:
given depth, RGB, and color-camera intrinsics, each valid pixel can be
back-projected into camera coordinates.
"""

from __future__ import annotations

import gzip
import json
import pickle
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

import numpy as np


def load_episode(output_dir: Union[str, Path], index: Optional[int] = None) -> Tuple[List[Dict[str, Any]], Dict[str, Any], int]:
    """Load frames and metadata from an episode directory."""

    episode_dir = Path(output_dir).expanduser()
    if index is None:
        candidates = sorted(
            int(path.name[:-len(".pkl.gz")])
            for path in episode_dir.glob("*.pkl.gz")
            if path.name[:-len(".pkl.gz")].isdigit()
        )
        if not candidates:
            raise FileNotFoundError(f"No numeric *.pkl.gz found in {episode_dir}")
        index = candidates[0]

    trajectory_path = episode_dir / f"{index}.pkl.gz"
    with gzip.open(trajectory_path, "rb") as f:
        payload = pickle.load(f)

    metadata_path = episode_dir / "metadata.json"
    metadata: Dict[str, Any] = {}
    if metadata_path.exists():
        with metadata_path.open("r", encoding="utf-8") as f:
            metadata = json.load(f)

    return list(payload["data"]), metadata, int(index)


def write_depth_proof(
    output_dir: Union[str, Path],
    index: int,
    frames: Sequence[Mapping[str, Any]],
    metadata: Mapping[str, Any],
    *,
    stride: int = 4,
    max_points: int = 80000,
    max_frames_per_camera: int = 1,
) -> Dict[str, Any]:
    """Write depth stats, depth preview PNGs, and sample point clouds."""

    episode_dir = Path(output_dir).expanduser()
    proof_dir = episode_dir / "depth_proof"
    proof_dir.mkdir(parents=True, exist_ok=True)

    camera_metadata = dict(metadata.get("cameras", {}))
    camera_names = _camera_names(frames, metadata)
    summary: Dict[str, Any] = {
        "episode_dir": str(episode_dir),
        "index": int(index),
        "frame_count": len(frames),
        "pointcloud_frame": "camera_color_optical_frame",
        "depth_units": "meters",
        "stride": max(1, int(stride)),
        "max_points": max(1, int(max_points)),
        "cameras": {},
    }

    for camera_name in camera_names:
        camera_summary = _summarize_camera_depth(
            proof_dir=proof_dir,
            episode_dir=episode_dir,
            camera_name=camera_name,
            frames=frames,
            camera_metadata=camera_metadata.get(camera_name, {}),
            stride=max(1, int(stride)),
            max_points=max(1, int(max_points)),
            max_frames_per_camera=max(1, int(max_frames_per_camera)),
        )
        if camera_summary["frames_with_depth"] > 0:
            summary["cameras"][camera_name] = camera_summary

    summary_path = proof_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
        f.write("\n")

    return summary


def _camera_names(frames: Sequence[Mapping[str, Any]], metadata: Mapping[str, Any]) -> List[str]:
    names: List[str] = []
    for name in metadata.get("camera_names", []):
        if isinstance(name, str):
            names.append(name)

    seen = set(names)
    for frame in frames:
        for key in frame.keys():
            if key.endswith("_depth"):
                name = key[: -len("_depth")]
            elif key.endswith("_depth_path"):
                name = key[: -len("_depth_path")]
            elif key.endswith("_image_path"):
                name = key[: -len("_image_path")]
            else:
                continue
            if name not in seen:
                names.append(name)
                seen.add(name)
    return names


def depth_png_relative_path(camera_name: str, frame_index: int) -> Path:
    return Path("depth") / camera_name / f"{int(frame_index):06d}.png"


def write_metric_depth_png(path: Union[str, Path], depth: np.ndarray, depth_scale: float) -> None:
    """Write metric depth as a uint16 PNG using the camera depth scale."""

    import cv2

    if depth_scale <= 0:
        raise ValueError(f"depth_scale must be positive, got {depth_scale}")

    depth = _as_depth_array(depth)
    if depth is None:
        raise ValueError("depth is empty")

    raw = np.zeros(depth.shape, dtype=np.uint16)
    mask = np.isfinite(depth) & (depth > 0)
    if np.any(mask):
        values = np.rint(depth[mask].astype(np.float64) / float(depth_scale))
        raw[mask] = np.clip(values, 0, np.iinfo(np.uint16).max).astype(np.uint16)

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), raw):
        raise RuntimeError(f"Failed to write depth PNG: {path}")


def read_metric_depth_png(path: Union[str, Path], depth_scale: float) -> Optional[np.ndarray]:
    """Read a uint16 depth PNG and convert it back to meters."""

    import cv2

    raw = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if raw is None:
        return None
    if raw.ndim == 3:
        raw = raw[:, :, 0]
    depth = raw.astype(np.float32) * float(depth_scale)
    depth[raw == 0] = 0.0
    return depth


def depth_from_frame(
    episode_dir: Union[str, Path],
    frame: Mapping[str, Any],
    camera_name: str,
    camera_metadata: Mapping[str, Any],
) -> Optional[np.ndarray]:
    depth = _as_depth_array(frame.get(f"{camera_name}_depth"))
    if depth is not None:
        return depth

    depth_path = frame.get(f"{camera_name}_depth_path")
    if not depth_path:
        return None

    depth_scale = float(
        frame.get(f"{camera_name}_depth_scale")
        or camera_metadata.get("depth_scale")
        or 0.001
    )
    return read_metric_depth_png(Path(episode_dir) / str(depth_path), depth_scale)


def color_from_frame(
    episode_dir: Union[str, Path],
    frame: Mapping[str, Any],
    camera_name: str,
    frame_index: int,
) -> Optional[np.ndarray]:
    image = _as_color_array(frame.get(f"{camera_name}_image"))
    if image is not None:
        return image
    return read_video_frame(episode_dir, camera_name, frame_index)


def read_video_frame(
    episode_dir: Union[str, Path],
    camera_name: str,
    frame_index: int,
) -> Optional[np.ndarray]:
    import cv2

    video_path = Path(episode_dir) / f"{camera_name}.mp4"
    if not video_path.exists():
        return None

    cap = cv2.VideoCapture(str(video_path))
    try:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(frame_index))
        ok, frame = cap.read()
        if not ok:
            return None
        return frame
    finally:
        cap.release()


def _summarize_camera_depth(
    *,
    proof_dir: Path,
    episode_dir: Path,
    camera_name: str,
    frames: Sequence[Mapping[str, Any]],
    camera_metadata: Mapping[str, Any],
    stride: int,
    max_points: int,
    max_frames_per_camera: int,
) -> Dict[str, Any]:
    frames_with_depth = 0
    valid_pixels = 0
    total_pixels = 0
    depth_sum = 0.0
    depth_min = float("inf")
    depth_max = 0.0
    first_shape: Optional[List[int]] = None
    proof_files: List[Dict[str, Any]] = []

    intrinsics = camera_metadata.get("intrinsics") or {}

    for frame_index, frame in enumerate(frames):
        depth = depth_from_frame(episode_dir, frame, camera_name, camera_metadata)
        if depth is None:
            continue

        frames_with_depth += 1
        first_shape = first_shape or [int(depth.shape[0]), int(depth.shape[1])]

        mask = np.isfinite(depth) & (depth > 0)
        count = int(mask.sum())
        valid_pixels += count
        total_pixels += int(depth.size)
        if count:
            values = depth[mask]
            depth_sum += float(values.sum(dtype=np.float64))
            depth_min = min(depth_min, float(values.min()))
            depth_max = max(depth_max, float(values.max()))

        if count and len(proof_files) < max_frames_per_camera and intrinsics:
            png_name = f"{camera_name}_frame{frame_index:06d}_depth.png"
            ply_name = f"{camera_name}_frame{frame_index:06d}_cloud.ply"
            _write_depth_png(proof_dir / png_name, depth)
            points, colors = depth_to_point_cloud(
                depth=depth,
                intrinsics=intrinsics,
                bgr_image=color_from_frame(episode_dir, frame, camera_name, frame_index),
                flip=bool(camera_metadata.get("flip", False)),
                stride=stride,
                max_points=max_points,
            )
            _write_ascii_ply(proof_dir / ply_name, points, colors)
            proof_files.append(
                {
                    "frame_index": int(frame_index),
                    "depth_png": png_name,
                    "pointcloud_ply": ply_name,
                    "point_count": int(points.shape[0]),
                }
            )

    return {
        "frames_with_depth": int(frames_with_depth),
        "shape": first_shape,
        "valid_pixels": int(valid_pixels),
        "total_pixels": int(total_pixels),
        "valid_ratio": float(valid_pixels / total_pixels) if total_pixels else 0.0,
        "min_m": float(depth_min) if valid_pixels else None,
        "max_m": float(depth_max) if valid_pixels else None,
        "mean_m": float(depth_sum / valid_pixels) if valid_pixels else None,
        "intrinsics": dict(intrinsics),
        "proof_files": proof_files,
    }


def depth_to_point_cloud(
    *,
    depth: np.ndarray,
    intrinsics: Mapping[str, Any],
    bgr_image: Optional[np.ndarray] = None,
    flip: bool = False,
    stride: int = 4,
    max_points: int = 80000,
) -> Tuple[np.ndarray, np.ndarray]:
    """Back-project a metric depth image to XYZ camera coordinates."""

    stride = max(1, int(stride))
    max_points = max(1, int(max_points))
    depth = _as_depth_array(depth)
    if depth is None:
        raise ValueError("depth is empty")

    height, width = depth.shape
    sampled_depth = depth[::stride, ::stride]
    mask = np.isfinite(sampled_depth) & (sampled_depth > 0)
    if not np.any(mask):
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

    yy, xx = np.mgrid[0:height:stride, 0:width:stride]
    z = sampled_depth[mask].astype(np.float32)
    u = xx[mask].astype(np.float32)
    v = yy[mask].astype(np.float32)

    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    ppx = float(intrinsics["ppx"])
    ppy = float(intrinsics["ppy"])
    if flip:
        ppx = float(width - 1) - ppx
        ppy = float(height - 1) - ppy

    x = (u - ppx) * z / fx
    y = (v - ppy) * z / fy
    points = np.column_stack([x, y, z]).astype(np.float32)

    if bgr_image is not None and bgr_image.shape[:2] == depth.shape:
        colors = bgr_image[::stride, ::stride][mask][:, ::-1].astype(np.uint8)
    else:
        colors = np.full((points.shape[0], 3), 255, dtype=np.uint8)

    if points.shape[0] > max_points:
        indices = np.linspace(0, points.shape[0] - 1, max_points).astype(np.int64)
        points = points[indices]
        colors = colors[indices]

    return points, colors


def _as_depth_array(value: Any) -> Optional[np.ndarray]:
    if value is None:
        return None
    depth = np.asarray(value)
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2:
        return None
    return depth.astype(np.float32, copy=False)


def _as_color_array(value: Any) -> Optional[np.ndarray]:
    if value is None:
        return None
    image = np.asarray(value)
    if image.ndim != 3 or image.shape[2] != 3:
        return None
    return image.astype(np.uint8, copy=False)


def _write_depth_png(path: Path, depth: np.ndarray) -> None:
    import cv2

    mask = np.isfinite(depth) & (depth > 0)
    if not np.any(mask):
        preview = np.zeros((*depth.shape, 3), dtype=np.uint8)
        cv2.imwrite(str(path), preview)
        return

    values = depth[mask]
    vmin = float(np.percentile(values, 1))
    vmax = float(np.percentile(values, 99))
    if vmax <= vmin:
        vmax = vmin + 1e-6

    scaled = np.zeros(depth.shape, dtype=np.uint8)
    scaled[mask] = np.clip((depth[mask] - vmin) * 255.0 / (vmax - vmin), 0, 255).astype(np.uint8)
    preview = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    preview[~mask] = 0
    cv2.imwrite(str(path), preview)


def _write_ascii_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
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
