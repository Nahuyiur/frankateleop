"""Build a Cartesian Stack-Ring trajectory for offline or live-scene validation.

This module never connects to, or sends commands to, a robot.  It detects the
pick/place rings in a successful reference episode and in a newly observed
scene, maps their relative pixel displacement to robot XY, and writes a new
replayable ``.pkl.gz`` episode plus QA artifacts.
"""

from __future__ import annotations

import argparse
import copy
import gzip
import importlib.util
import json
import pickle
import sys
import time
from pathlib import Path
from typing import Any, Dict, Mapping, MutableMapping, Optional, Sequence, Tuple

import cv2
import numpy as np


DEFAULT_WORKSPACE = ((0.40, 0.68), (-0.20, 0.28), (0.24, 0.72))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", required=True, help="Successful Stack-Ring episode")
    current = parser.add_mutually_exclusive_group()
    current.add_argument("--current-episode", help="Episode providing the new initial image")
    current.add_argument("--current-image", help="PNG/JPEG providing the new initial left view")
    current.add_argument(
        "--capture-live",
        action="store_true",
        help="Capture the current left RealSense view; no robot command is sent",
    )
    parser.add_argument("--current-frame", type=int, default=0)
    parser.add_argument("--source-frame", type=int, default=0)
    parser.add_argument("--camera-warmup-sec", type=float, default=3.0)
    parser.add_argument("--mapping-model", required=True)
    parser.add_argument("--calibration-module", required=True)
    parser.add_argument("--projection-report", default=None)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-name", default="stack_ring_retargeted.pkl.gz")
    parser.add_argument(
        "--coordinate-mode",
        choices=("relative_to_source", "absolute"),
        default="relative_to_source",
    )
    parser.add_argument("--max-pick-shift-m", type=float, default=0.12)
    parser.add_argument("--max-place-shift-m", type=float, default=0.12)
    parser.add_argument("--max-role-pixel-shift", type=float, default=120.0)
    parser.add_argument("--min-detection-score", type=float, default=0.12)
    parser.add_argument("--pickup-frame", type=int, default=None)
    parser.add_argument("--place-frame", type=int, default=None)
    parser.add_argument("--keep-images", action="store_true")
    parser.add_argument("--workspace-x", default="0.40,0.68")
    parser.add_argument("--workspace-y", default="-0.20,0.28")
    parser.add_argument("--workspace-z", default="0.24,0.72")
    return parser.parse_args()


def _parse_range(value: str, label: str) -> Tuple[float, float]:
    parts = [part.strip() for part in str(value).split(",")]
    if len(parts) != 2:
        raise ValueError(f"{label} must be MIN,MAX, got {value!r}")
    lower, upper = (float(parts[0]), float(parts[1]))
    if not np.isfinite([lower, upper]).all() or lower >= upper:
        raise ValueError(f"Invalid {label}: {value!r}")
    return lower, upper


def _load_module(module_path: Path):
    path = module_path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    spec = importlib.util.spec_from_file_location("stack_ring_calibration_api", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load calibration module {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def resolve_episode_file(path_like: str) -> Path:
    path = Path(path_like).expanduser().resolve()
    if path.is_file() and path.name.endswith(".pkl.gz"):
        return path
    if not path.is_dir():
        raise FileNotFoundError(path)
    preferred = path / f"{path.name}.pkl.gz"
    if preferred.is_file():
        return preferred
    candidates = sorted(path.glob("*.pkl.gz"))
    if len(candidates) != 1:
        raise RuntimeError(f"Expected one .pkl.gz in {path}, found {len(candidates)}")
    return candidates[0]


def load_episode(path_like: str) -> Tuple[Path, Dict[str, Any], list]:
    path = resolve_episode_file(path_like)
    with gzip.open(path, "rb") as handle:
        payload = pickle.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"Episode payload must be a dict: {path}")
    frames = payload.get("data") or payload.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError(f"Episode has no frames: {path}")
    return path, payload, frames


def _read_video_rgb(path: Path, frame_index: int) -> np.ndarray:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open video {path}")
    try:
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        index = int(frame_index)
        if index < 0:
            index += frame_count
        if index < 0 or (frame_count > 0 and index >= frame_count):
            raise IndexError(f"Frame {frame_index} outside video length {frame_count}")
        capture.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, bgr = capture.read()
        if not ok or bgr is None:
            raise RuntimeError(f"Could not read frame {index} from {path}")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    finally:
        capture.release()


def _read_image_rgb(path: Path) -> np.ndarray:
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError(f"Could not read image {path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _to_model_image(image_rgb: np.ndarray) -> np.ndarray:
    image = np.asarray(image_rgb, dtype=np.uint8)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 RGB image, got {image.shape}")
    if image.shape[:2] == (180, 320):
        return image.copy()
    # MUKA v3/LJM uses direct_resize_320x180_each, without crop or padding.
    return cv2.resize(image, (320, 180), interpolation=cv2.INTER_AREA)


def load_episode_left_image(episode_file: Path, frame_index: int) -> np.ndarray:
    video = episode_file.parent / "left.mp4"
    if video.is_file():
        return _to_model_image(_read_video_rgb(video, frame_index))
    _, _, frames = load_episode(str(episode_file))
    index = frame_index if frame_index >= 0 else len(frames) + frame_index
    if index < 0 or index >= len(frames):
        raise IndexError(index)
    for key in ("left_image", "exterior_1_image"):
        if key in frames[index]:
            return _to_model_image(np.asarray(frames[index][key], dtype=np.uint8))
    raise FileNotFoundError(
        f"No companion left.mp4 or embedded left image for {episode_file}"
    )


def capture_live_left_image(warmup_sec: float) -> np.ndarray:
    from dataclasses import replace
    from franka_capture.cameras.realsense import create_realsense_cameras
    from franka_capture.config.fr3_single import DEFAULT_CAMERAS

    if "left" not in DEFAULT_CAMERAS:
        raise KeyError("franka_capture DEFAULT_CAMERAS has no 'left' camera")
    handles = create_realsense_cameras(
        {"left": replace(DEFAULT_CAMERAS["left"], depth=False, align_depth=True)},
        allow_missing=False,
    )
    try:
        deadline = time.monotonic() + max(0.0, float(warmup_sec))
        reads = 0
        while time.monotonic() < deadline or reads < 10:
            handles["left"].read()
            reads += 1
        rgb, _ = handles["left"].read()
        return _to_model_image(np.asarray(rgb, dtype=np.uint8))
    finally:
        handles["left"].close()


def _frame_closedness(frame: Mapping[str, Any]) -> float:
    for key in ("gripper_01closedness", "gripper_closedness"):
        if key in frame:
            return float(frame[key])
    for key in ("gripper_target_width", "gripper_width"):
        if key in frame:
            return float(1.0 - np.clip(float(frame[key]) / 0.08, 0.0, 1.0))
    raise KeyError("Frame has neither gripper closedness nor width")


def episode_states7(frames: Sequence[Mapping[str, Any]]) -> np.ndarray:
    states = []
    for index, frame in enumerate(frames):
        pose = np.asarray(frame.get("pose"), dtype=np.float64)
        if pose.shape != (6,):
            raise ValueError(f"Frame {index} pose must be [6], got {pose.shape}")
        states.append(np.r_[pose, _frame_closedness(frame)])
    result = np.asarray(states, dtype=np.float64)
    if not np.isfinite(result).all():
        raise ValueError("Episode states contain NaN or Inf")
    return result


def apply_mapping(model: Mapping[str, Any], pixels: np.ndarray) -> np.ndarray:
    matrix = np.asarray(model["matrix"], dtype=np.float64)
    pixels = np.asarray(pixels, dtype=np.float64)
    if matrix.shape != (2, 3) or pixels.ndim != 2 or pixels.shape[1] != 2:
        raise ValueError(f"Bad mapping shapes matrix={matrix.shape}, pixels={pixels.shape}")
    return np.c_[pixels, np.ones(len(pixels))] @ matrix.T


def retarget_payload(
    payload: Mapping[str, Any],
    frames: Sequence[Mapping[str, Any]],
    pickup: int,
    place: int,
    target_pick_xyz: np.ndarray,
    target_place_xyz: np.ndarray,
    report: Mapping[str, Any],
    keep_images: bool,
) -> Dict[str, Any]:
    output = {
        key: copy.deepcopy(value)
        for key, value in payload.items()
        if key not in {"data", "frames"}
    }
    source_states = episode_states7(frames)
    pick_delta = target_pick_xyz - source_states[pickup, :3]
    place_delta = target_place_xyz - source_states[place, :3]
    new_frames = []
    for index, frame in enumerate(frames):
        cloned: MutableMapping[str, Any] = {}
        for key, value in frame.items():
            if not keep_images and (key.endswith("_image") or key.endswith("_depth")):
                continue
            cloned[key] = copy.deepcopy(value)
        pose = np.asarray(cloned["pose"], dtype=np.float64)
        if index <= pickup:
            delta = pick_delta
        elif index >= place:
            delta = place_delta
        else:
            alpha = (index - pickup) / float(place - pickup)
            delta = (1.0 - alpha) * pick_delta + alpha * place_delta
        pose[:3] += delta
        cloned["pose"] = pose.astype(float).tolist()
        cloned["retarget_delta_xyz"] = delta.astype(float).tolist()
        new_frames.append(cloned)
    output["data"] = new_frames
    output["dagger_retarget"] = {
        "schema_version": "stack_ring_robot_retarget_v1",
        "task": "stack_ring",
        "created_at_unix": time.time(),
        "anchor_frames": {"pickup": int(pickup), "place": int(place)},
        "target_pickup_xyz": target_pick_xyz.astype(float).tolist(),
        "target_place_xyz": target_place_xyz.astype(float).tolist(),
        "pickup_delta_xyz": pick_delta.astype(float).tolist(),
        "place_delta_xyz": place_delta.astype(float).tolist(),
        "output_images_kept": bool(keep_images),
        "mapping_report": dict(report),
    }
    return output


def _write_rgb(path: Path, image_rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"Could not write {path}")


def _trajectory_overlay(
    image_rgb: np.ndarray,
    states7: np.ndarray,
    matrix_4x2: Optional[np.ndarray],
    pickup: int,
    place: int,
    title: str,
) -> np.ndarray:
    canvas = image_rgb.copy()
    shade = canvas.copy()
    cv2.rectangle(shade, (0, 19), (319, 38), (0, 0, 0), -1)
    canvas = cv2.addWeighted(shade, 0.60, canvas, 0.40, 0.0)
    cv2.putText(
        canvas,
        title,
        (8, 33),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    if matrix_4x2 is None:
        return canvas
    segment = states7[max(0, pickup - 10) : min(len(states7), place + 11), :3]
    pixels = np.c_[segment, np.ones(len(segment))] @ matrix_4x2
    points = np.round(pixels).astype(np.int32)
    valid = (
        (points[:, 0] >= 0) & (points[:, 0] < 320)
        & (points[:, 1] >= 0) & (points[:, 1] < 180)
    )
    for first, second, ok1, ok2 in zip(points[:-1], points[1:], valid[:-1], valid[1:]):
        if ok1 and ok2:
            cv2.line(canvas, tuple(first), tuple(second), (40, 220, 255), 1, cv2.LINE_AA)
    for index, color, label in ((pickup, (40, 230, 80), "PICK"), (place, (40, 150, 255), "PLACE")):
        p = np.r_[states7[index, :3], 1.0] @ matrix_4x2
        point = tuple(np.round(p).astype(int))
        cv2.circle(canvas, point, 4, color, -1, cv2.LINE_AA)
        cv2.putText(canvas, label, (point[0] + 5, point[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.34, color, 1, cv2.LINE_AA)
    return canvas


def _load_projection_matrix(path_like: Optional[str]) -> Optional[np.ndarray]:
    if not path_like:
        return None
    payload = json.loads(Path(path_like).expanduser().read_text())
    entry = payload.get("trajectory_projection_qa", payload)
    matrix = np.asarray(entry.get("matrix_4x2"), dtype=np.float64)
    if matrix.shape != (4, 2):
        raise ValueError(f"Projection matrix must be [4,2], got {matrix.shape}")
    return matrix


def _inside_workspace(point: np.ndarray, workspace: Sequence[Tuple[float, float]]) -> bool:
    return all(lower <= float(value) <= upper for value, (lower, upper) in zip(point, workspace))


def main() -> int:
    args = parse_args()
    if not (args.current_episode or args.current_image or args.capture_live):
        raise ValueError("Provide one of --current-episode, --current-image, --capture-live")
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    calibration = _load_module(Path(args.calibration_module))
    model_payload = json.loads(Path(args.mapping_model).expanduser().read_text())
    if model_payload.get("schema") != "ring_pixel_robot_mapping_v1":
        raise ValueError("Unsupported Ring mapping model schema")
    if model_payload.get("camera") != "exterior_1" or model_payload.get("image_size") != [320, 180]:
        raise ValueError("Ring mapping model must target exterior_1 at 320x180")

    demo_file, demo_payload, demo_frames = load_episode(args.demo)
    source_states = episode_states7(demo_frames)
    anchors = calibration.infer_ring_task_anchors(source_states)
    pickup = int(args.pickup_frame if args.pickup_frame is not None else anchors["pickup"])
    place = int(args.place_frame if args.place_frame is not None else anchors["place"])
    if not 0 <= pickup < place < len(source_states):
        raise ValueError(f"Invalid anchors pickup={pickup}, place={place}, T={len(source_states)}")

    source_image = load_episode_left_image(demo_file, args.source_frame)
    if args.current_episode:
        current_file = resolve_episode_file(args.current_episode)
        current_image = load_episode_left_image(current_file, args.current_frame)
        current_source = str(current_file)
    elif args.current_image:
        current_image = _to_model_image(_read_image_rgb(Path(args.current_image).expanduser()))
        current_source = str(Path(args.current_image).expanduser().resolve())
    else:
        current_image = capture_live_left_image(args.camera_warmup_sec)
        current_source = "live_realsense_left"

    source_detections = calibration.detect_rings(source_image)
    current_detections = calibration.detect_rings(current_image)
    source_pair = calibration.assign_pick_place(source_detections)
    current_pair = calibration.assign_pick_place(current_detections)
    source_overlay = calibration.draw_detection_overlay(source_image, source_detections, source_pair, "SOURCE rings")
    current_overlay = calibration.draw_detection_overlay(current_image, current_detections, current_pair, "CURRENT rings")
    _write_rgb(output_dir / "source_ring_detections.png", source_overlay)
    _write_rgb(output_dir / "current_ring_detections.png", current_overlay)
    if source_pair is None or current_pair is None:
        raise RuntimeError("Source/current image did not produce a pick/place Ring pair")
    scores = [source_pair[0].score, source_pair[1].score, current_pair[0].score, current_pair[1].score]
    if min(scores) < args.min_detection_score:
        raise ValueError(f"Ring detection score below {args.min_detection_score}: {scores}")

    source_pixels = np.asarray([source_pair[0].center, source_pair[1].center], dtype=np.float64)
    current_pixels = np.asarray([current_pair[0].center, current_pair[1].center], dtype=np.float64)
    role_shift = np.linalg.norm(current_pixels - source_pixels, axis=1)
    if np.any(role_shift > args.max_role_pixel_shift):
        raise ValueError(f"Ring role pixel shift too large: {role_shift.tolist()}")
    source_mapped = apply_mapping(model_payload["model"], source_pixels)
    current_mapped = apply_mapping(model_payload["model"], current_pixels)
    demo_pick_xyz = source_states[pickup, :3].copy()
    demo_place_xyz = source_states[place, :3].copy()
    if args.coordinate_mode == "relative_to_source":
        target_pick_xy = demo_pick_xyz[:2] + current_mapped[0] - source_mapped[0]
        target_place_xy = demo_place_xyz[:2] + current_mapped[1] - source_mapped[1]
    else:
        target_pick_xy, target_place_xy = current_mapped
    target_pick_xyz = np.r_[target_pick_xy, demo_pick_xyz[2]]
    target_place_xyz = np.r_[target_place_xy, demo_place_xyz[2]]
    pick_shift = float(np.linalg.norm(target_pick_xyz - demo_pick_xyz))
    place_shift = float(np.linalg.norm(target_place_xyz - demo_place_xyz))
    if pick_shift > args.max_pick_shift_m:
        raise ValueError(f"Pickup shift {pick_shift:.4f} m exceeds {args.max_pick_shift_m:.4f}")
    if place_shift > args.max_place_shift_m:
        raise ValueError(f"Place shift {place_shift:.4f} m exceeds {args.max_place_shift_m:.4f}")
    workspace = (
        _parse_range(args.workspace_x, "workspace-x"),
        _parse_range(args.workspace_y, "workspace-y"),
        _parse_range(args.workspace_z, "workspace-z"),
    )
    if not _inside_workspace(target_pick_xyz, workspace) or not _inside_workspace(target_place_xyz, workspace):
        raise ValueError(
            f"Mapped target outside workspace: pick={target_pick_xyz.tolist()} place={target_place_xyz.tolist()}"
        )

    report: Dict[str, Any] = {
        "schema": "stack_ring_robot_mapping_report_v1",
        "task": "stack_ring",
        "execution_boundary": "mapping_only_no_robot_command",
        "reference_episode": str(demo_file),
        "current_scene_source": current_source,
        "coordinate_mode": args.coordinate_mode,
        "mapping_model": str(Path(args.mapping_model).expanduser().resolve()),
        "mapping_model_validation": model_payload.get("validation"),
        "mapping_model_external_policy_eval": model_payload.get("external_policy_eval"),
        "anchors": {**anchors, "pickup": pickup, "place": place},
        "source_pixels_320x180": source_pixels.astype(float).tolist(),
        "current_pixels_320x180": current_pixels.astype(float).tolist(),
        "role_pixel_shift": role_shift.astype(float).tolist(),
        "source_mapped_xy_m": source_mapped.astype(float).tolist(),
        "current_mapped_xy_m": current_mapped.astype(float).tolist(),
        "demo_pick_xyz_m": demo_pick_xyz.astype(float).tolist(),
        "demo_place_xyz_m": demo_place_xyz.astype(float).tolist(),
        "target_pick_xyz_m": target_pick_xyz.astype(float).tolist(),
        "target_place_xyz_m": target_place_xyz.astype(float).tolist(),
        "target_pick_pose6": np.r_[target_pick_xyz, source_states[pickup, 3:6]].astype(float).tolist(),
        "target_place_pose6": np.r_[target_place_xyz, source_states[place, 3:6]].astype(float).tolist(),
        "pickup_shift_norm_m": pick_shift,
        "place_shift_norm_m": place_shift,
        "workspace_xyz": [[float(a), float(b)] for a, b in workspace],
        "z_policy": "preserve_reference_anchor_z",
        "orientation_policy": "preserve_reference_rpy",
        "gripper_policy": "preserve_reference_fields",
        "approved_execution_scope": "hover_only",
        "full_replay_eligible": False,
        "manual_review_required": True,
        "hard_checks": {"passed": True},
    }
    retargeted_payload = retarget_payload(
        demo_payload, demo_frames, pickup, place, target_pick_xyz, target_place_xyz, report, args.keep_images
    )
    retargeted_frames = retargeted_payload["data"]
    retargeted_states = episode_states7(retargeted_frames)
    if not np.isfinite(retargeted_states).all():
        raise ValueError("Retargeted trajectory contains NaN or Inf")
    max_rpy_diff = float(
        np.max(np.abs(retargeted_states[:, 3:6] - source_states[:, 3:6]))
    )
    max_gripper_diff = float(
        np.max(np.abs(retargeted_states[:, 6] - source_states[:, 6]))
    )
    if max_rpy_diff > 1e-9 or max_gripper_diff > 1e-9:
        raise ValueError(
            "Retargeting changed RPY/gripper: "
            f"rpy={max_rpy_diff}, gripper={max_gripper_diff}"
        )
    report["trajectory_checks"] = {
        "state_count": int(len(retargeted_states)),
        "finite": True,
        "max_rpy_change": max_rpy_diff,
        "max_gripper_change": max_gripper_diff,
        "reference_max_xyz_step_m": float(
            np.linalg.norm(np.diff(source_states[:, :3], axis=0), axis=1).max()
        ),
        "retargeted_max_xyz_step_m": float(
            np.linalg.norm(np.diff(retargeted_states[:, :3], axis=0), axis=1).max()
        ),
        "retargeted_xyz_min": retargeted_states[:, :3]
        .min(axis=0)
        .astype(float)
        .tolist(),
        "retargeted_xyz_max": retargeted_states[:, :3]
        .max(axis=0)
        .astype(float)
        .tolist(),
        "boundary": "recorded for QA; full Cartesian replay remains disabled",
    }
    retargeted_payload["dagger_retarget"]["mapping_report"] = dict(report)
    output_episode = output_dir / args.output_name
    with gzip.open(output_episode, "wb") as handle:
        pickle.dump(retargeted_payload, handle, protocol=pickle.HIGHEST_PROTOCOL)
    np.save(output_dir / "reference_absolute_eef_xyzrpy_closedness.npy", source_states.astype(np.float32))
    np.save(output_dir / "retargeted_absolute_eef_xyzrpy_closedness.npy", retargeted_states.astype(np.float32))

    projection = _load_projection_matrix(args.projection_report)
    source_path_overlay = _trajectory_overlay(source_overlay, source_states, projection, pickup, place, "SOURCE trajectory")
    current_path_overlay = _trajectory_overlay(current_overlay, retargeted_states, projection, pickup, place, "MAPPED trajectory")
    _write_rgb(output_dir / "source_trajectory_overlay.png", source_path_overlay)
    _write_rgb(output_dir / "mapped_trajectory_overlay.png", current_path_overlay)
    sheet = np.concatenate([source_path_overlay, current_path_overlay], axis=1)
    _write_rgb(output_dir / "source_vs_mapped_contact_sheet.png", sheet)
    report["artifacts"] = {
        "retargeted_episode": str(output_episode),
        "retargeted_states": str(output_dir / "retargeted_absolute_eef_xyzrpy_closedness.npy"),
        "contact_sheet": str(output_dir / "source_vs_mapped_contact_sheet.png"),
    }
    report_path = output_dir / "mapping_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    (output_dir / "_MAPPING_SUCCESS").write_text("ok\n")
    print(json.dumps({
        "status": "mapping_success",
        "report": str(report_path),
        "episode": str(output_episode),
        "pickup_shift_m": pick_shift,
        "place_shift_m": place_shift,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
