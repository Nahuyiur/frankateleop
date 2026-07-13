"""Build and validate a pixel-to-robot-XY mapping model for Jenga retargeting."""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from DAgger.jenga_retarget import (  # noqa: E402
    assign_pick_place,
    camera_list,
    detect_blocks,
    discover_demo_files,
    frame_pose_xyz,
    infer_anchor_frames,
    load_episode,
    load_episode_images,
    resolve_episode_file,
    write_overlay,
)


DEFAULT_DEMO = Path.home() / "Desktop/Muka_NAS/stack_jenga/High_Quality/0/0.pkl.gz"
DEFAULT_DEMO_ROOT = Path.home() / "Desktop/Muka_NAS/stack_jenga/High_Quality"
DEFAULT_OUTPUT = Path.home() / "Desktop/Muka_NAS/stack_jenga/DAgger/mapping_model.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", default=str(DEFAULT_DEMO), help="Template demo to include first.")
    parser.add_argument("--demo-root", default=str(DEFAULT_DEMO_ROOT), help="High_Quality demo root.")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="Output mapping_model.json.")
    parser.add_argument("--cameras", default="left,middle")
    parser.add_argument("--max-demos", type=int, default=100)
    parser.add_argument(
        "--train-demos",
        type=int,
        default=None,
        help="Randomly choose this many episodes for fitting. Use with --validation-demos.",
    )
    parser.add_argument(
        "--validation-demos",
        type=int,
        default=None,
        help="Randomly choose this many disjoint held-out episodes for validation.",
    )
    parser.add_argument("--random-seed", type=int, default=7)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--model-type", choices=("affine", "homography"), default="affine")
    parser.add_argument("--min-samples", type=int, default=8)
    parser.add_argument("--ransac-threshold-m", type=float, default=0.025)
    parser.add_argument(
        "--no-video-first-frame",
        action="store_true",
        help="Use embedded pkl images for frame 0 instead of camera mp4 files.",
    )
    parser.add_argument("--debug-dir", default=None)
    return parser.parse_args()


def transform_homography(matrix: Sequence[Sequence[float]], pixels: np.ndarray) -> np.ndarray:
    pts = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    mapped = cv2.perspectiveTransform(pts, np.asarray(matrix, dtype=np.float64))
    return mapped.reshape(-1, 2)


def transform_affine(matrix: Sequence[Sequence[float]], pixels: np.ndarray) -> np.ndarray:
    affine = np.asarray(matrix, dtype=np.float64)
    if affine.shape != (2, 3):
        raise ValueError(f"affine matrix must have shape 2x3, got {affine.shape}")
    pts = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    pts_h = np.concatenate([pts, np.ones((pts.shape[0], 1), dtype=np.float64)], axis=1)
    return pts_h @ affine.T


def transform_mapping(model: Mapping[str, Any], pixels: np.ndarray) -> np.ndarray:
    model_type = model.get("type")
    if model_type == "homography":
        return transform_homography(model["matrix"], pixels)
    if model_type == "affine":
        return transform_affine(model["matrix"], pixels)
    raise ValueError(f"Unsupported mapping model type: {model_type!r}")


def robust_stats(values: Sequence[float]) -> Dict[str, Any]:
    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        return {"count": 0, "mean": None, "median": None, "p90": None, "max": None}
    return {
        "count": int(arr.size),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p90": float(np.percentile(arr, 90)),
        "max": float(np.max(arr)),
    }


def discover_all_demo_files(template_demo: Path, demo_root: str | Path) -> List[Path]:
    template_demo = resolve_episode_file(template_demo)
    root = Path(demo_root).expanduser()
    candidates: List[Path] = []
    if root.exists():
        if root.is_file() and root.name.endswith(".pkl.gz"):
            candidates = [root]
        elif root.is_dir():
            candidates = sorted(
                root.glob("*/*.pkl.gz"),
                key=lambda path: (
                    int(path.parent.name) if path.parent.name.isdigit() else 10**9,
                    path.name,
                ),
            )

    unique: List[Path] = []
    for candidate in [template_demo, *candidates]:
        try:
            resolved = resolve_episode_file(candidate)
        except (FileNotFoundError, RuntimeError, ValueError):
            continue
        if resolved not in unique:
            unique.append(resolved)
    return unique


def choose_demo_splits(args: argparse.Namespace) -> Tuple[List[Tuple[Path, str]], Dict[str, Any]]:
    template_demo = resolve_episode_file(args.demo)
    if args.train_demos is None and args.validation_demos is None:
        demo_files = discover_demo_files(template_demo, args.demo_root, args.max_demos)
        return [(path, "auto") for path in demo_files], {
            "selection": "even_sample_then_fraction_split",
            "max_demos": int(args.max_demos),
        }
    if args.train_demos is None or args.validation_demos is None:
        raise ValueError("--train-demos and --validation-demos must be used together")
    if args.train_demos <= 0 or args.validation_demos < 0:
        raise ValueError("--train-demos must be > 0 and --validation-demos must be >= 0")

    pool = discover_all_demo_files(template_demo, args.demo_root)
    required = int(args.train_demos + args.validation_demos)
    if len(pool) < required:
        raise ValueError(f"Need {required} demos for random split, found {len(pool)} under {args.demo_root}")
    rng = random.Random(int(args.random_seed))
    shuffled = list(pool)
    rng.shuffle(shuffled)
    train_files = shuffled[: args.train_demos]
    val_files = shuffled[args.train_demos : args.train_demos + args.validation_demos]
    plan = [(path, "train") for path in train_files] + [(path, "validation") for path in val_files]
    return plan, {
        "selection": "random_episode_split",
        "random_seed": int(args.random_seed),
        "train_demos_requested": int(args.train_demos),
        "validation_demos_requested": int(args.validation_demos),
        "candidate_demo_count": int(len(pool)),
        "train_demo_files": [str(path) for path in train_files],
        "validation_demo_files": [str(path) for path in val_files],
    }


def load_first_video_images(episode_file: Path, cameras: Sequence[str]) -> Dict[str, np.ndarray]:
    images: Dict[str, np.ndarray] = {}
    for camera in cameras:
        video_path = episode_file.parent / f"{camera}.mp4"
        if not video_path.is_file():
            raise FileNotFoundError(f"Missing camera video: {video_path}")
        capture = cv2.VideoCapture(str(video_path))
        try:
            if not capture.isOpened():
                raise RuntimeError(f"Failed to open camera video: {video_path}")
            ok, bgr = capture.read()
            if not ok or bgr is None:
                raise RuntimeError(f"Failed to read first frame from: {video_path}")
            images[camera] = cv2.cvtColor(np.asarray(bgr, dtype=np.uint8), cv2.COLOR_BGR2RGB)
        finally:
            capture.release()
    return images


def sample_demo(
    demo_path: Path,
    cameras: Sequence[str],
    debug_dir: Path | None,
    *,
    split: str,
    prefer_video_first_frame: bool,
) -> List[Dict[str, Any]]:
    episode_file = resolve_episode_file(demo_path)
    video_images: Dict[str, np.ndarray] | None = None
    if prefer_video_first_frame:
        try:
            video_images = load_first_video_images(episode_file, cameras)
        except Exception:
            video_images = None
    episode_file, _, frames, _ = load_episode(episode_file)
    anchors = infer_anchor_frames(frames, None, None)
    images = video_images if video_images is not None else load_episode_images(frames, 0, cameras)
    pick_xy = frame_pose_xyz(frames[anchors.pickup])[:2]
    place_xy = frame_pose_xyz(frames[anchors.place])[:2]
    samples: List[Dict[str, Any]] = []
    for camera in cameras:
        detections = detect_blocks(camera, images[camera])
        pair = assign_pick_place(camera, detections)
        if debug_dir is not None:
            write_overlay(
                debug_dir / f"{episode_file.parent.name}_{camera}.jpg",
                images[camera],
                detections,
                pair,
            )
        if pair is None:
            continue
        pick_det, place_det = pair
        samples.append(
            {
                "demo_episode": str(episode_file),
                "episode_id": episode_file.parent.name,
                "split": split,
                "camera": camera,
                "role": "pick",
                "pixel_xy": [float(pick_det.center[0]), float(pick_det.center[1])],
                "robot_xy": pick_xy.astype(float).tolist(),
                "anchor_frame": int(anchors.pickup),
                "detection": pick_det.to_json(),
            }
        )
        samples.append(
            {
                "demo_episode": str(episode_file),
                "episode_id": episode_file.parent.name,
                "split": split,
                "camera": camera,
                "role": "place",
                "pixel_xy": [float(place_det.center[0]), float(place_det.center[1])],
                "robot_xy": place_xy.astype(float).tolist(),
                "anchor_frame": int(anchors.place),
                "detection": place_det.to_json(),
            }
        )
    return samples


def fit_homography(samples: Sequence[Mapping[str, Any]], threshold_m: float) -> Dict[str, Any]:
    src = np.asarray([sample["pixel_xy"] for sample in samples], dtype=np.float64)
    dst = np.asarray([sample["robot_xy"] for sample in samples], dtype=np.float64)
    if len(samples) < 4:
        raise ValueError("homography needs at least 4 samples")
    matrix, inlier_mask = cv2.findHomography(src, dst, cv2.RANSAC, threshold_m)
    if matrix is None:
        raise RuntimeError("cv2.findHomography failed")
    pred = transform_homography(matrix, src)
    errors = np.linalg.norm(pred - dst, axis=1)
    inliers = inlier_mask.reshape(-1).astype(bool) if inlier_mask is not None else np.ones(len(samples), dtype=bool)
    return {
        "type": "homography",
        "matrix": [[float(v) for v in row] for row in matrix],
        "sample_count": int(len(samples)),
        "inlier_count": int(np.count_nonzero(inliers)),
        "train_error_m": robust_stats(errors.tolist()),
        "train_inlier_error_m": robust_stats(errors[inliers].tolist()),
    }


def fit_affine(samples: Sequence[Mapping[str, Any]], threshold_m: float) -> Dict[str, Any]:
    src = np.asarray([sample["pixel_xy"] for sample in samples], dtype=np.float64)
    dst = np.asarray([sample["robot_xy"] for sample in samples], dtype=np.float64)
    if len(samples) < 3:
        raise ValueError("affine mapping needs at least 3 samples")
    matrix, inlier_mask = cv2.estimateAffine2D(
        src,
        dst,
        method=cv2.RANSAC,
        ransacReprojThreshold=threshold_m,
        maxIters=5000,
        confidence=0.995,
        refineIters=20,
    )
    if matrix is None:
        raise RuntimeError("cv2.estimateAffine2D failed")
    pred = transform_affine(matrix, src)
    errors = np.linalg.norm(pred - dst, axis=1)
    inliers = inlier_mask.reshape(-1).astype(bool) if inlier_mask is not None else np.ones(len(samples), dtype=bool)
    return {
        "type": "affine",
        "matrix": [[float(v) for v in row] for row in matrix],
        "sample_count": int(len(samples)),
        "inlier_count": int(np.count_nonzero(inliers)),
        "train_error_m": robust_stats(errors.tolist()),
        "train_inlier_error_m": robust_stats(errors[inliers].tolist()),
    }


def fit_mapping(samples: Sequence[Mapping[str, Any]], threshold_m: float, model_type: str) -> Dict[str, Any]:
    if model_type == "homography":
        return fit_homography(samples, threshold_m)
    if model_type == "affine":
        return fit_affine(samples, threshold_m)
    raise ValueError(f"Unsupported --model-type: {model_type}")


def validate_model(
    model: Mapping[str, Any],
    samples: Sequence[Mapping[str, Any]],
) -> Dict[str, Any]:
    if not samples:
        return {"sample_count": 0, "episode_count": 0, "error_m": robust_stats([]), "by_role": {}}
    src = np.asarray([sample["pixel_xy"] for sample in samples], dtype=np.float64)
    dst = np.asarray([sample["robot_xy"] for sample in samples], dtype=np.float64)
    pred = transform_mapping(model, src)
    errors = np.linalg.norm(pred - dst, axis=1)
    by_role = {}
    for role in sorted({str(sample["role"]) for sample in samples}):
        mask = np.asarray([sample["role"] == role for sample in samples], dtype=bool)
        by_role[role] = robust_stats(errors[mask].tolist())
    return {
        "sample_count": int(len(samples)),
        "episode_count": int(len({sample["demo_episode"] for sample in samples})),
        "error_m": robust_stats(errors.tolist()),
        "by_role": by_role,
    }


def split_train_validation(samples: Sequence[Mapping[str, Any]], fraction: float) -> tuple[List[Mapping[str, Any]], List[Mapping[str, Any]]]:
    episodes = sorted({str(sample["demo_episode"]) for sample in samples})
    if len(episodes) < 5 or fraction <= 0:
        return list(samples), []
    val_count = max(1, int(round(len(episodes) * min(max(fraction, 0.0), 0.8))))
    val_indices = set(np.linspace(0, len(episodes) - 1, val_count).round().astype(int).tolist())
    val_episodes = {episodes[index] for index in val_indices}
    train = [sample for sample in samples if sample["demo_episode"] not in val_episodes]
    val = [sample for sample in samples if sample["demo_episode"] in val_episodes]
    return train, val


def build_model(args: argparse.Namespace) -> Dict[str, Any]:
    cameras = camera_list(args.cameras)
    demo_plan, selection_info = choose_demo_splits(args)
    demo_files = [path for path, _ in demo_plan]
    debug_dir = Path(args.debug_dir).expanduser() if args.debug_dir else None
    if debug_dir is not None:
        debug_dir.mkdir(parents=True, exist_ok=True)

    all_samples: List[Dict[str, Any]] = []
    failures: Dict[str, str] = {}
    for demo_index, (demo_path, split) in enumerate(demo_plan, start=1):
        print(f"[{demo_index}/{len(demo_plan)}] {split}: {demo_path}", flush=True)
        try:
            all_samples.extend(
                sample_demo(
                    demo_path,
                    cameras,
                    debug_dir,
                    split=split,
                    prefer_video_first_frame=not args.no_video_first_frame,
                )
            )
        except Exception as exc:
            failures[str(demo_path)] = f"{type(exc).__name__}: {exc}"

    camera_models: Dict[str, Any] = {}
    explicit_split = any(split in {"train", "validation"} for _, split in demo_plan)
    for camera in cameras:
        camera_samples = [sample for sample in all_samples if sample["camera"] == camera]
        if explicit_split:
            train_samples = [sample for sample in camera_samples if sample["split"] == "train"]
            val_samples = [sample for sample in camera_samples if sample["split"] == "validation"]
        else:
            train_samples, val_samples = split_train_validation(camera_samples, args.validation_fraction)

        if len(train_samples) < args.min_samples:
            camera_models[camera] = {
                "available": False,
                "reason": f"only {len(train_samples)} train samples, need {args.min_samples}",
                "sample_count": len(camera_samples),
                "train_sample_count": len(train_samples),
                "validation_sample_count": len(val_samples),
            }
            continue
        model = fit_mapping(train_samples, args.ransac_threshold_m, args.model_type)
        validation = validate_model(model, val_samples)
        camera_models[camera] = {
            "available": True,
            "model": model,
            "cross_validation": validation,
            "train_episode_count": int(len({sample["demo_episode"] for sample in train_samples})),
            "validation_episode_count": int(len({sample["demo_episode"] for sample in val_samples})),
            "sample_count": int(len(camera_samples)),
            "train_sample_count": int(len(train_samples)),
            "validation_sample_count": int(len(val_samples)),
        }

    return {
        "schema_version": "jenga_pixel_robot_mapping_v1",
        "created_at_unix": time.time(),
        "demo_root": str(Path(args.demo_root).expanduser()),
        "demo_files": [str(path) for path in demo_files],
        "selection": selection_info,
        "cameras": cameras,
        "max_demos": int(args.max_demos),
        "train_demos": None if args.train_demos is None else int(args.train_demos),
        "validation_demos": None if args.validation_demos is None else int(args.validation_demos),
        "random_seed": int(args.random_seed),
        "validation_fraction": float(args.validation_fraction),
        "model_type": args.model_type,
        "ransac_threshold_m": float(args.ransac_threshold_m),
        "failures": failures,
        "sample_count": int(len(all_samples)),
        "camera_models": camera_models,
    }


def main() -> int:
    args = parse_args()
    model = build_model(args)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(model, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Wrote mapping model: {output}")
    for camera, camera_model in model["camera_models"].items():
        if not camera_model.get("available"):
            print(f"  {camera}: unavailable ({camera_model.get('reason')})")
            continue
        model_info = camera_model["model"]
        cv_info = camera_model["cross_validation"]
        print(
            f"  {camera}: train_samples={camera_model['train_sample_count']} "
            f"val_samples={camera_model['validation_sample_count']} "
            f"inliers={model_info['inlier_count']} "
            f"train_median={model_info['train_inlier_error_m']['median']:.4f}m "
            f"cv_median={cv_info['error_m']['median']} "
            f"cv_p90={cv_info['error_m']['p90']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
