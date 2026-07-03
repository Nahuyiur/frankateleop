"""Fuse multiple calibrated RGB-D camera clouds into one robot-base cloud."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import numpy as np

from pointcloud.depth_proof import color_from_frame, depth_from_frame, depth_to_point_cloud, load_episode

from .apply_extrinsics import _load_extrinsic, _write_ascii_ply, validate_extrinsic_camera
from .geometry import transform_points
from .io import read_json, write_json


def parse_extrinsic_specs(specs: Sequence[str]) -> Dict[str, Path]:
    mapping: Dict[str, Path] = {}
    for spec in specs:
        if "=" not in spec:
            raise ValueError(f"--extrinsic must be camera=path, got {spec!r}")
        camera_name, path_text = spec.split("=", 1)
        camera_name = camera_name.strip()
        path_text = path_text.strip()
        if not camera_name or not path_text:
            raise ValueError(f"--extrinsic must be camera=path, got {spec!r}")
        if camera_name in mapping:
            raise ValueError(f"Duplicate extrinsic for camera {camera_name!r}")
        mapping[camera_name] = Path(path_text).expanduser()
    return mapping


def parse_extrinsics_map(path: Path) -> Dict[str, Path]:
    path = Path(path).expanduser()
    payload = read_json(path)
    base_dir = path.parent
    mapping: Dict[str, Path] = {}
    cameras = payload.get("cameras", payload)
    if not isinstance(cameras, dict):
        raise ValueError(f"extrinsics map must contain a cameras object: {path}")
    for camera_name, entry in cameras.items():
        if isinstance(entry, str):
            result_path = entry
        elif isinstance(entry, dict):
            result_path = entry.get("calibration_result") or entry.get("path") or entry.get("extrinsic")
        else:
            result_path = None
        if not result_path:
            raise ValueError(f"extrinsics map entry for {camera_name!r} has no calibration_result/path")
        result = Path(str(result_path)).expanduser()
        if not result.is_absolute():
            result = base_dir / result
        mapping[str(camera_name)] = result
    return mapping


def merge_extrinsic_inputs(specs: Sequence[str] | None, map_path: Path | None) -> Dict[str, Path]:
    mapping: Dict[str, Path] = {}
    if map_path is not None:
        mapping.update(parse_extrinsics_map(map_path))
    if specs:
        mapping.update(parse_extrinsic_specs(specs))
    if not mapping:
        raise ValueError("Provide at least one --extrinsic camera=path or --extrinsics-map map.json")
    return mapping


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_frame_index(frame_spec: str, frame_count: int) -> int:
    text = str(frame_spec).strip().lower()
    if text == "first":
        return 0
    if text == "middle":
        return frame_count // 2
    if text == "last":
        return frame_count - 1
    index = int(text)
    if index < 0:
        index = frame_count + index
    if index < 0 or index >= frame_count:
        raise IndexError(f"frame index {index} out of range [0, {frame_count - 1}]")
    return index


def camera_names_from_episode(frames: Sequence[Mapping[str, Any]], metadata: Mapping[str, Any]) -> List[str]:
    names: List[str] = []
    seen = set()
    for name in metadata.get("camera_names", []):
        if isinstance(name, str) and name not in seen:
            names.append(name)
            seen.add(name)

    for frame in frames:
        for key in frame.keys():
            if key.endswith("_image"):
                name = key[: -len("_image")]
            elif key.endswith("_image_path"):
                name = key[: -len("_image_path")]
            elif key.endswith("_depth"):
                name = key[: -len("_depth")]
            elif key.endswith("_depth_path"):
                name = key[: -len("_depth_path")]
            else:
                continue
            if name not in seen:
                names.append(name)
                seen.add(name)
    return names


def resolve_camera_selection(
    camera_names_spec: str,
    episode_camera_names: Sequence[str],
    extrinsic_map: Mapping[str, Path],
) -> List[str]:
    spec = str(camera_names_spec or "extrinsics").strip()
    if spec in {"extrinsics", "calibrated"}:
        ordered = [name for name in episode_camera_names if name in extrinsic_map]
        for name in extrinsic_map:
            if name not in ordered:
                ordered.append(name)
        return ordered
    if spec in {"all", "*"}:
        return list(episode_camera_names)
    return [name.strip() for name in spec.split(",") if name.strip()]


def voxel_downsample(
    points: np.ndarray,
    colors: np.ndarray,
    *,
    voxel_size_m: float,
    max_points: int,
) -> Tuple[np.ndarray, np.ndarray]:
    if points.shape[0] == 0:
        return points, colors
    if float(voxel_size_m) > 0:
        keys = np.floor(points / float(voxel_size_m)).astype(np.int64)
        _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        summed_points = np.zeros((counts.shape[0], 3), dtype=np.float64)
        summed_colors = np.zeros((counts.shape[0], 3), dtype=np.float64)
        np.add.at(summed_points, inverse, points.astype(np.float64))
        np.add.at(summed_colors, inverse, colors.astype(np.float64))
        points = (summed_points / counts[:, None]).astype(np.float32)
        colors = np.clip(np.rint(summed_colors / counts[:, None]), 0, 255).astype(np.uint8)
    max_points = int(max_points)
    if max_points > 0 and points.shape[0] > max_points:
        indices = np.linspace(0, points.shape[0] - 1, max_points).astype(np.int64)
        points = points[indices]
        colors = colors[indices]
    return points, colors


def bounds_xyz(points: np.ndarray) -> Dict[str, Any]:
    if points.shape[0] == 0:
        return {"x": None, "y": None, "z": None}
    return {
        "x": [float(np.min(points[:, 0])), float(np.max(points[:, 0]))],
        "y": [float(np.min(points[:, 1])), float(np.max(points[:, 1]))],
        "z": [float(np.min(points[:, 2])), float(np.max(points[:, 2]))],
    }


def _camera_cloud(
    *,
    episode_dir: Path,
    frame: Mapping[str, Any],
    frame_index: int,
    camera_name: str,
    camera_metadata: Dict[str, Any],
    extrinsic_path: Path,
    stride: int,
    max_points_per_camera: int,
    allow_camera_mismatch: bool,
    allow_metadata_mismatch: bool,
    require_validated_extrinsic: bool,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    intrinsics = camera_metadata.get("intrinsics") or {}
    if not intrinsics:
        raise RuntimeError(f"No intrinsics for camera {camera_name}")

    depth = depth_from_frame(episode_dir, frame, camera_name, camera_metadata)
    if depth is None:
        raise RuntimeError(f"No depth for camera {camera_name} frame {frame_index}")
    bgr = color_from_frame(episode_dir, frame, camera_name, frame_index)

    transform_base_camera, extrinsic_payload = _load_extrinsic(extrinsic_path)
    validation_summary = validate_calibration_result(
        extrinsic_path,
        extrinsic_payload,
        require_validated=require_validated_extrinsic,
    )
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
        max_points=max_points_per_camera,
    )
    points_base = transform_points(transform_base_camera, points_camera).astype(np.float32)
    valid_ratio = float((np.isfinite(depth) & (depth > 0)).mean()) if depth.size else 0.0
    summary = {
        "camera_name": camera_name,
        "extrinsic_path": str(extrinsic_path),
        "extrinsic_sha256": file_sha256(extrinsic_path),
        "extrinsic_camera_name": extrinsic_payload.get("camera_name"),
        "world_frame": extrinsic_payload.get("world_frame"),
        "camera_frame": extrinsic_payload.get("camera_frame"),
        "provisional": bool(extrinsic_payload.get("provisional", False)),
        "validation": validation_summary,
        "serial_number": camera_metadata.get("serial_number"),
        "depth_scale": camera_metadata.get("depth_scale"),
        "point_count": int(points_base.shape[0]),
        "depth_valid_ratio": valid_ratio,
        "flip": bool(camera_metadata.get("flip", False)),
        "bounds_xyz_m": bounds_xyz(points_base),
        "metadata_warnings": metadata_warnings,
    }
    return points_base, colors, summary


def validate_calibration_result(
    extrinsic_path: Path,
    extrinsic_payload: Dict[str, Any],
    *,
    require_validated: bool,
) -> Dict[str, Any]:
    world_frame = extrinsic_payload.get("world_frame")
    camera_frame = extrinsic_payload.get("camera_frame")
    if world_frame and world_frame != "robot_base":
        raise RuntimeError(f"{extrinsic_path} has world_frame={world_frame!r}, expected 'robot_base'")
    if camera_frame and camera_frame != "camera_color_optical":
        raise RuntimeError(f"{extrinsic_path} has camera_frame={camera_frame!r}, expected 'camera_color_optical'")
    if extrinsic_payload.get("provisional") is True:
        raise RuntimeError(f"{extrinsic_path} is provisional; collect enough samples and solve again")

    validation_path = Path(extrinsic_path).expanduser().parent / "validation_report.json"
    doctor_path = Path(extrinsic_path).expanduser().parent / "doctor_report.json"
    summary: Dict[str, Any] = {
        "validation_report": str(validation_path),
        "validation_pass": None,
        "doctor_report": str(doctor_path),
        "doctor_status": None,
    }
    if validation_path.exists():
        validation = read_json(validation_path)
        summary["validation_pass"] = validation.get("pass")
        summary["validation_sample_mode"] = validation.get("validation_sample_mode")
        if validation.get("pass") is not True:
            raise RuntimeError(f"{validation_path} pass is not true")
    elif require_validated:
        raise RuntimeError(f"Missing validation_report.json next to {extrinsic_path}")

    if doctor_path.exists():
        doctor = read_json(doctor_path)
        session = doctor.get("session") or {}
        summary["doctor_status"] = session.get("status")
        if session.get("status") == "FAIL":
            raise RuntimeError(f"{doctor_path} status is FAIL")
    elif require_validated:
        summary["doctor_status"] = "missing"
    return summary


def fuse_episode(
    episode_dir: Path,
    extrinsic_map: Mapping[str, Path],
    *,
    camera_names_spec: str = "extrinsics",
    frame_spec: str = "middle",
    stride: int = 4,
    max_points_per_camera: int = 120000,
    voxel_size_m: float = 0.006,
    max_fused_points: int = 300000,
    output_dir: Path | None = None,
    write_per_camera: bool = True,
    allow_missing_extrinsics: bool = False,
    allow_camera_mismatch: bool = False,
    allow_metadata_mismatch: bool = False,
    require_validated_extrinsics: bool = True,
) -> Dict[str, Any]:
    episode_dir = Path(episode_dir).expanduser()
    frames, metadata, episode_index = load_episode(episode_dir)
    if not frames:
        raise RuntimeError(f"No frames in {episode_dir}")
    frame_index = resolve_frame_index(frame_spec, len(frames))
    frame = frames[frame_index]
    camera_metadata = dict(metadata.get("cameras", {}))
    depth_recording = metadata.get("depth_recording") or {}
    if depth_recording.get("aligned_to") not in (None, "color"):
        raise RuntimeError(f"Episode depth is aligned to {depth_recording.get('aligned_to')!r}, expected 'color'")
    episode_camera_names = camera_names_from_episode(frames, metadata)
    selected_camera_names = resolve_camera_selection(camera_names_spec, episode_camera_names, extrinsic_map)
    if not selected_camera_names:
        raise RuntimeError("No cameras selected for fusion")

    output_dir = output_dir or episode_dir / "world_pointcloud"
    output_dir.mkdir(parents=True, exist_ok=True)

    all_points: List[np.ndarray] = []
    all_colors: List[np.ndarray] = []
    camera_summaries: Dict[str, Any] = {}
    skipped: Dict[str, str] = {}
    errors: Dict[str, str] = {}

    for camera_name in selected_camera_names:
        extrinsic_path = extrinsic_map.get(camera_name)
        if extrinsic_path is None:
            message = "missing extrinsic"
            if allow_missing_extrinsics:
                skipped[camera_name] = message
                continue
            raise RuntimeError(f"Camera {camera_name!r} selected for fusion but has no --extrinsic")
        try:
            metadata_for_camera = dict(camera_metadata.get(camera_name, {}))
            if metadata_for_camera.get("align_depth") is False:
                raise RuntimeError(f"Camera {camera_name} depth is not aligned to color")
            points_base, colors, camera_summary = _camera_cloud(
                episode_dir=episode_dir,
                frame=frame,
                frame_index=frame_index,
                camera_name=camera_name,
                camera_metadata=metadata_for_camera,
                extrinsic_path=extrinsic_path,
                stride=stride,
                max_points_per_camera=max_points_per_camera,
                allow_camera_mismatch=allow_camera_mismatch,
                allow_metadata_mismatch=allow_metadata_mismatch,
                require_validated_extrinsic=require_validated_extrinsics,
            )
        except Exception as exc:
            errors[camera_name] = str(exc)
            if allow_missing_extrinsics:
                skipped[camera_name] = str(exc)
                continue
            raise

        per_camera_ply = None
        if write_per_camera:
            per_camera_ply = output_dir / f"{camera_name}_frame{frame_index:06d}_base_cloud.ply"
            _write_ascii_ply(per_camera_ply, points_base, colors)
            camera_summary["ply"] = str(per_camera_ply)
        camera_summaries[camera_name] = camera_summary
        if points_base.shape[0]:
            all_points.append(points_base)
            all_colors.append(colors)

    if not all_points:
        raise RuntimeError(f"No valid camera point clouds to fuse; skipped={skipped} errors={errors}")

    fused_points = np.concatenate(all_points, axis=0)
    fused_colors = np.concatenate(all_colors, axis=0)
    pre_downsample_count = int(fused_points.shape[0])
    fused_points, fused_colors = voxel_downsample(
        fused_points,
        fused_colors,
        voxel_size_m=voxel_size_m,
        max_points=max_fused_points,
    )

    fused_ply = output_dir / f"multi_camera_frame{frame_index:06d}_base_cloud.ply"
    _write_ascii_ply(fused_ply, fused_points, fused_colors)
    summary = {
        "episode_dir": str(episode_dir),
        "episode_index": int(episode_index),
        "frame_index": int(frame_index),
        "frame_count": int(len(frames)),
        "pointcloud_frame": "robot_base",
        "input_camera_frame": "camera_color_optical",
        "camera_names_requested": selected_camera_names,
        "camera_names_fused": list(camera_summaries.keys()),
        "camera_names_skipped": skipped,
        "camera_errors": errors,
        "stride": int(stride),
        "max_points_per_camera": int(max_points_per_camera),
        "voxel_size_m": float(voxel_size_m),
        "require_validated_extrinsics": bool(require_validated_extrinsics),
        "point_count_before_downsample": pre_downsample_count,
        "point_count": int(fused_points.shape[0]),
        "bounds_xyz_m": bounds_xyz(fused_points),
        "cameras": camera_summaries,
        "ply": str(fused_ply),
    }
    summary_path = output_dir / f"multi_camera_frame{frame_index:06d}_base_summary.json"
    summary["summary_json"] = str(summary_path)
    write_json(summary_path, summary)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_dir")
    parser.add_argument(
        "--extrinsic",
        action="append",
        default=[],
        help="Camera extrinsic mapping, camera=calibration_result.json. Repeat for each camera.",
    )
    parser.add_argument(
        "--extrinsics-map",
        default=None,
        help="JSON map with cameras.{name}.calibration_result entries. --extrinsic overrides duplicate cameras.",
    )
    parser.add_argument(
        "--camera-names",
        default="extrinsics",
        help="Comma-separated cameras to fuse, all, or extrinsics/calibrated. Defaults to extrinsics.",
    )
    parser.add_argument("--frame", default="middle")
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--max-points-per-camera", type=int, default=120000)
    parser.add_argument("--voxel-size-m", type=float, default=0.006)
    parser.add_argument("--max-fused-points", type=int, default=300000)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--no-per-camera", action="store_true")
    parser.add_argument("--allow-missing-extrinsics", action="store_true")
    parser.add_argument("--allow-camera-mismatch", action="store_true")
    parser.add_argument("--allow-metadata-mismatch", action="store_true")
    parser.add_argument("--allow-unvalidated-extrinsics", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = fuse_episode(
        Path(args.episode_dir).expanduser(),
        merge_extrinsic_inputs(
            args.extrinsic,
            Path(args.extrinsics_map).expanduser() if args.extrinsics_map else None,
        ),
        camera_names_spec=args.camera_names,
        frame_spec=args.frame,
        stride=args.stride,
        max_points_per_camera=args.max_points_per_camera,
        voxel_size_m=args.voxel_size_m,
        max_fused_points=args.max_fused_points,
        output_dir=Path(args.output_dir).expanduser() if args.output_dir else None,
        write_per_camera=not args.no_per_camera,
        allow_missing_extrinsics=args.allow_missing_extrinsics,
        allow_camera_mismatch=args.allow_camera_mismatch,
        allow_metadata_mismatch=args.allow_metadata_mismatch,
        require_validated_extrinsics=not args.allow_unvalidated_extrinsics,
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
