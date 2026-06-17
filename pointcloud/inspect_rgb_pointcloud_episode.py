"""Export RGB/depth/point-cloud views from a recorded episode."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np

from pointcloud.depth_proof import (
    _write_ascii_ply,
    _write_depth_png,
    color_from_frame,
    depth_from_frame,
    depth_to_point_cloud,
    load_episode,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "episode_dir",
        nargs="?",
        default=None,
        help="Episode directory, e.g. /home/pnp/Desktop/franka_record_data/pick_block/0",
    )
    parser.add_argument(
        "--output-root",
        default=str(Path.home() / "Desktop" / "franka_record_data"),
        help="Default recording root used when episode_dir is omitted.",
    )
    parser.add_argument(
        "--task",
        default="rgb_pointcloud",
        help="Default task used when episode_dir is omitted.",
    )
    parser.add_argument(
        "--index",
        type=int,
        default=None,
        help="Episode index. Defaults to the numeric pkl.gz in the directory.",
    )
    parser.add_argument(
        "--frame",
        default="0",
        help="Frame to export: integer, first, middle, or last. Default: 0.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory. Defaults to episode_dir/rgb_pointcloud_view/frame_XXXXXX.",
    )
    parser.add_argument(
        "--pointcloud-stride",
        type=int,
        default=3,
        help="Pixel stride for exported PLY files. Lower is denser; 1 is full resolution.",
    )
    parser.add_argument(
        "--pointcloud-max-points",
        type=int,
        default=120000,
        help="Maximum points per camera PLY file.",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Open the output directory after export using xdg-open.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    episode_dir = (
        Path(args.episode_dir).expanduser()
        if args.episode_dir
        else _latest_episode_dir(args.output_root, args.task)
    )
    frames, metadata, index = load_episode(episode_dir, args.index)
    if not frames:
        raise RuntimeError(f"Episode has no frames: {episode_dir}")

    frame_index = _resolve_frame_index(args.frame, len(frames))
    frame = frames[frame_index]
    output_dir = (
        Path(args.output_dir).expanduser()
        if args.output_dir
        else episode_dir / "rgb_pointcloud_view" / f"frame_{frame_index:06d}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    camera_names = _camera_names(frames, metadata)
    camera_metadata = dict(metadata.get("cameras", {}))

    summaries: Dict[str, Any] = {}
    rgb_tiles = []

    for camera_name in camera_names:
        metadata_for_camera = camera_metadata.get(camera_name, {})
        bgr = color_from_frame(episode_dir, frame, camera_name, frame_index)
        depth = depth_from_frame(episode_dir, frame, camera_name, metadata_for_camera)
        intrinsics = metadata_for_camera.get("intrinsics") or {}

        camera_summary: Dict[str, Any] = {
            "has_rgb": bgr is not None,
            "has_depth": depth is not None,
            "rgb_png": None,
            "depth_png": None,
            "pointcloud_ply": None,
            "point_count": 0,
            "depth": _depth_stats(depth),
            "intrinsics": intrinsics,
        }

        if bgr is not None:
            rgb_path = output_dir / f"{camera_name}_rgb.png"
            _write_rgb_png(rgb_path, bgr)
            camera_summary["rgb_png"] = rgb_path.name
            rgb_tiles.append((camera_name, bgr))

        if depth is not None:
            depth_path = output_dir / f"{camera_name}_depth.png"
            _write_depth_png(depth_path, depth)
            camera_summary["depth_png"] = depth_path.name

            if intrinsics:
                points, colors = depth_to_point_cloud(
                    depth=depth,
                    intrinsics=intrinsics,
                    bgr_image=bgr,
                    flip=bool(metadata_for_camera.get("flip", False)),
                    stride=args.pointcloud_stride,
                    max_points=args.pointcloud_max_points,
                )
                ply_path = output_dir / f"{camera_name}_cloud.ply"
                _write_ascii_ply(ply_path, points, colors)
                camera_summary["pointcloud_ply"] = ply_path.name
                camera_summary["point_count"] = int(points.shape[0])

        summaries[camera_name] = camera_summary

    if rgb_tiles:
        contact_sheet_path = output_dir / "all_cameras_rgb.png"
        _write_contact_sheet(contact_sheet_path, rgb_tiles)

    summary = {
        "episode_dir": str(episode_dir),
        "episode_index": int(index),
        "frame_index": int(frame_index),
        "frame_count": len(frames),
        "output_dir": str(output_dir),
        "pointcloud_frame": "camera_color_optical_frame_per_camera",
        "depth_units": "meters",
        "pointcloud_stride": int(args.pointcloud_stride),
        "pointcloud_max_points": int(args.pointcloud_max_points),
        "camera_names": camera_names,
        "cameras": summaries,
    }

    summary_path = output_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
        f.write("\n")

    _print_summary(summary)

    if args.open:
        subprocess.Popen(["xdg-open", str(output_dir)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _latest_episode_dir(output_root: str, task: str) -> Path:
    task_dir = Path(output_root).expanduser() / task
    if not task_dir.exists():
        raise FileNotFoundError(
            f"No default task directory found: {task_dir}. "
            "Run 18_record_rgb_pointclouds.sh first or pass episode_dir explicitly."
        )

    candidates = sorted(
        [path for path in task_dir.iterdir() if path.is_dir() and path.name.isdigit()],
        key=lambda path: int(path.name),
    )
    if not candidates:
        raise FileNotFoundError(
            f"No numeric episode directories found in {task_dir}. "
            "Run 18_record_rgb_pointclouds.sh first or pass episode_dir explicitly."
        )
    return candidates[-1]


def _resolve_frame_index(spec: str, frame_count: int) -> int:
    normalized = str(spec).strip().lower()
    if normalized == "first":
        return 0
    if normalized == "middle":
        return frame_count // 2
    if normalized == "last":
        return frame_count - 1
    try:
        index = int(normalized)
    except ValueError as exc:
        raise ValueError("--frame must be an integer, first, middle, or last") from exc
    if index < 0:
        index = frame_count + index
    if index < 0 or index >= frame_count:
        raise IndexError(f"Frame index {index} out of range [0, {frame_count - 1}]")
    return index


def _camera_names(frames: Sequence[Mapping[str, Any]], metadata: Mapping[str, Any]) -> List[str]:
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


def _depth_stats(depth: Optional[np.ndarray]) -> Dict[str, Any]:
    if depth is None:
        return {
            "shape": None,
            "valid_pixels": 0,
            "total_pixels": 0,
            "valid_ratio": 0.0,
            "min_m": None,
            "mean_m": None,
            "max_m": None,
        }

    mask = np.isfinite(depth) & (depth > 0)
    valid_pixels = int(mask.sum())
    total_pixels = int(depth.size)
    if valid_pixels == 0:
        return {
            "shape": [int(depth.shape[0]), int(depth.shape[1])],
            "valid_pixels": 0,
            "total_pixels": total_pixels,
            "valid_ratio": 0.0,
            "min_m": None,
            "mean_m": None,
            "max_m": None,
        }

    values = depth[mask]
    return {
        "shape": [int(depth.shape[0]), int(depth.shape[1])],
        "valid_pixels": valid_pixels,
        "total_pixels": total_pixels,
        "valid_ratio": float(valid_pixels / total_pixels),
        "min_m": float(values.min()),
        "mean_m": float(values.mean(dtype=np.float64)),
        "max_m": float(values.max()),
    }


def _write_rgb_png(path: Path, bgr: np.ndarray) -> None:
    import cv2

    cv2.imwrite(str(path), bgr)


def _write_contact_sheet(path: Path, tiles: Sequence[tuple[str, np.ndarray]]) -> None:
    import cv2

    if not tiles:
        return

    target_height = 240
    label_height = 28
    resized = []
    for name, bgr in tiles:
        height, width = bgr.shape[:2]
        tile_width = max(1, int(round(width * target_height / max(1, height))))
        tile = cv2.resize(bgr, (tile_width, target_height), interpolation=cv2.INTER_AREA)
        labeled = np.zeros((target_height + label_height, tile_width, 3), dtype=np.uint8)
        labeled[:target_height] = tile
        cv2.putText(
            labeled,
            name,
            (8, target_height + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        resized.append(labeled)

    sheet = np.concatenate(resized, axis=1)
    cv2.imwrite(str(path), sheet)


def _print_summary(summary: Mapping[str, Any]) -> None:
    print(f"Episode: {summary['episode_dir']}")
    print(f"Frame: {summary['frame_index']} / {summary['frame_count'] - 1}")
    print(f"Output: {summary['output_dir']}")
    print("Cameras:")
    for name, camera in summary["cameras"].items():
        depth = camera["depth"]
        print(
            f"  {name}: "
            f"rgb={camera['has_rgb']} depth={camera['has_depth']} "
            f"valid={depth['valid_ratio']:.3f} "
            f"mean_m={_fmt(depth['mean_m'])} "
            f"points={camera['point_count']}"
        )
    print("Files:")
    print("  all_cameras_rgb.png")
    print("  summary.json")
    print("  <camera>_rgb.png, <camera>_depth.png, <camera>_cloud.ply")


def _fmt(value: Any) -> str:
    if value is None:
        return "None"
    return f"{float(value):.4f}"


if __name__ == "__main__":
    main()
