"""Retarget a successful Jenga stacking replay to a newly observed scene.

The tool builds a new replayable ``.pkl.gz`` episode from one successful demo:

1. Infer the pickup anchor from the first gripper close event.
2. Infer the place anchor from the first gripper open event after pickup.
3. Detect the left/pick and right/place Jenga blocks in side-view RGB images.
4. Estimate image-to-robot XY mappings from demo detections and anchors.
5. Shift the recorded Cartesian poses so pickup and place anchors move to the
   newly detected blocks, while preserving joints, timing, and gripper fields.

The generated file is intended to be replayed with ``7_replay_fr3.sh``. It does
not send commands to the robot by itself.
"""

from __future__ import annotations

import argparse
import copy
import gzip
import json
import math
import pickle
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


DEFAULT_DEMO = Path.home() / "Desktop/Muka_NAS/stack_jenga/High_Quality/0/0.pkl.gz"
DEFAULT_DEMO_ROOT = Path.home() / "Desktop/Muka_NAS/stack_jenga/High_Quality"
DEFAULT_OUTPUT = Path.home() / "Desktop/Muka_NAS/stack_jenga/DAgger/retargeted.pkl.gz"
DEFAULT_MAPPING_MODEL = Path.home() / "Desktop/Muka_NAS/stack_jenga/DAgger/mapping_model.json"
DEFAULT_CAMERAS = ("left", "middle")


@dataclass
class BlockDetection:
    camera: str
    center: Tuple[float, float]
    size: Tuple[float, float]
    angle_deg: float
    area: float
    aspect: float
    score: float
    bbox: Tuple[int, int, int, int]
    role: Optional[str] = None

    def to_json(self) -> Dict[str, Any]:
        return {
            "camera": self.camera,
            "center": [float(self.center[0]), float(self.center[1])],
            "size": [float(self.size[0]), float(self.size[1])],
            "angle_deg": float(self.angle_deg),
            "area": float(self.area),
            "aspect": float(self.aspect),
            "score": float(self.score),
            "bbox": [int(v) for v in self.bbox],
            "role": self.role,
        }


@dataclass
class AnchorFrames:
    pickup: int
    place: int
    closed_threshold_m: float
    closed_range: Tuple[int, int]

    def to_json(self) -> Dict[str, Any]:
        return {
            "pickup": int(self.pickup),
            "place": int(self.place),
            "closed_threshold_m": float(self.closed_threshold_m),
            "closed_range": [int(self.closed_range[0]), int(self.closed_range[1])],
        }


@dataclass
class CameraPrediction:
    camera: str
    pickup_xy: np.ndarray
    place_xy: np.ndarray
    demo_pick_px: np.ndarray
    demo_place_px: np.ndarray
    current_pick_px: np.ndarray
    current_place_px: np.ndarray
    quality: float

    def to_json(self) -> Dict[str, Any]:
        return {
            "camera": self.camera,
            "pickup_xy": self.pickup_xy.astype(float).tolist(),
            "place_xy": self.place_xy.astype(float).tolist(),
            "demo_pick_px": self.demo_pick_px.astype(float).tolist(),
            "demo_place_px": self.demo_place_px.astype(float).tolist(),
            "current_pick_px": self.current_pick_px.astype(float).tolist(),
            "current_place_px": self.current_place_px.astype(float).tolist(),
            "quality": float(self.quality),
        }


@dataclass
class ModelCameraPrediction:
    camera: str
    pickup_xy: np.ndarray
    place_xy: np.ndarray
    current_pick_px: np.ndarray
    current_place_px: np.ndarray
    quality: float
    model_error_m: Optional[float]
    model_sample_count: int

    def to_json(self) -> Dict[str, Any]:
        return {
            "camera": self.camera,
            "pickup_xy": self.pickup_xy.astype(float).tolist(),
            "place_xy": self.place_xy.astype(float).tolist(),
            "current_pick_px": self.current_pick_px.astype(float).tolist(),
            "current_place_px": self.current_place_px.astype(float).tolist(),
            "quality": float(self.quality),
            "model_error_m": None if self.model_error_m is None else float(self.model_error_m),
            "model_sample_count": int(self.model_sample_count),
        }


@dataclass
class StackHeightEstimate:
    camera: str
    stack_count: int
    stack_height_px: float
    block_step_px: float
    visual_layer_pitch_px: float
    single_block_bbox_height_px: float
    pick_area_px: float
    stack_area_px: float
    silhouette_count: int
    calibrated_silhouette_count: int
    cluster_count: int
    area_count: int
    method: str
    quality: float
    place_detection: Dict[str, Any]
    candidate_detections: List[Dict[str, Any]]
    reason: Optional[str] = None

    def to_json(self) -> Dict[str, Any]:
        return {
            "camera": self.camera,
            "stack_count": int(self.stack_count),
            "stack_height_px": float(self.stack_height_px),
            "block_step_px": float(self.block_step_px),
            "visual_layer_pitch_px": float(self.visual_layer_pitch_px),
            "single_block_bbox_height_px": float(self.single_block_bbox_height_px),
            "pick_area_px": float(self.pick_area_px),
            "stack_area_px": float(self.stack_area_px),
            "silhouette_count": int(self.silhouette_count),
            "calibrated_silhouette_count": int(self.calibrated_silhouette_count),
            "cluster_count": int(self.cluster_count),
            "area_count": int(self.area_count),
            "method": self.method,
            "quality": float(self.quality),
            "place_detection": self.place_detection,
            "candidate_detections": self.candidate_detections,
            "reason": self.reason,
        }


def parse_xyz_offset(value: str) -> Tuple[float, float, float]:
    parts = str(value).replace(",", " ").split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"Expected three offsets 'dx,dy,dz' in meters, got {value!r}"
        )
    try:
        offset = tuple(float(part) for part in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"Expected numeric offsets 'dx,dy,dz' in meters, got {value!r}"
        ) from exc
    if not all(np.isfinite(item) for item in offset):
        raise argparse.ArgumentTypeError(f"Offset contains NaN or Inf: {value!r}")
    return offset  # type: ignore[return-value]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo",
        default=str(DEFAULT_DEMO),
        help="Successful demo episode used as the output trajectory template.",
    )
    parser.add_argument(
        "--demo-root",
        default=str(DEFAULT_DEMO_ROOT),
        help="Directory containing successful demos used to aggregate the image-to-robot mapping.",
    )
    parser.add_argument(
        "--mapping-model",
        default=str(DEFAULT_MAPPING_MODEL),
        help=(
            "Validated pixel-to-robot mapping model JSON. When present, this is "
            "used instead of the old per-demo two-point similarity mapping."
        ),
    )
    parser.add_argument(
        "--require-mapping-model",
        action="store_true",
        help="Fail instead of falling back to the old two-point mapping if --mapping-model is missing.",
    )
    parser.add_argument(
        "--no-mapping-model",
        action="store_true",
        help="Disable the validated mapping model and use the old multi-demo fallback.",
    )
    parser.add_argument(
        "--max-demos",
        type=int,
        default=6,
        help="Maximum successful demos to use for mapping aggregation. Use 1 for the old single-demo behavior.",
    )
    parser.add_argument(
        "--min-valid-demos",
        type=int,
        default=2,
        help="Require at least this many usable demo mappings when --max-demos > 1.",
    )
    parser.add_argument(
        "--max-demo-spread-m",
        type=float,
        default=0.08,
        help="Reject multi-demo aggregates whose inlier pickup/place predictions spread beyond this many meters.",
    )
    parser.add_argument(
        "--current-episode",
        default=None,
        help=(
            "Optional current scene episode/directory/.pkl.gz. When omitted, "
            "the tool captures live frames from the configured RealSense cameras."
        ),
    )
    parser.add_argument(
        "--current-frame",
        type=int,
        default=0,
        help="Frame index used from --current-episode. Use an initial/static frame.",
    )
    parser.add_argument(
        "--camera-warmup-sec",
        type=float,
        default=3.0,
        help="When capturing live RealSense frames, stream for this many seconds before taking the frame.",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT),
        help="Output .pkl.gz path for the retargeted replay episode.",
    )
    parser.add_argument(
        "--cameras",
        default=",".join(DEFAULT_CAMERAS),
        help="Comma-separated side cameras to use. Default: left,middle.",
    )
    parser.add_argument(
        "--primary-camera",
        default="left",
        help="Camera used for object-role ordering and fallback. Default: left.",
    )
    parser.add_argument(
        "--require-all-cameras",
        action="store_true",
        help="Fail if any requested camera cannot provide two block detections.",
    )
    parser.add_argument(
        "--min-camera-quality",
        type=float,
        default=0.65,
        help="Minimum per-camera mapping quality to include in the averaged estimate.",
    )
    parser.add_argument(
        "--max-pick-shift-m",
        type=float,
        default=0.30,
        help="Reject mappings that move the pickup anchor farther than this from the demo.",
    )
    parser.add_argument(
        "--max-place-shift-m",
        type=float,
        default=0.30,
        help="Reject mappings that move the place anchor farther than this from the demo.",
    )
    parser.add_argument(
        "--pickup-frame",
        type=int,
        default=None,
        help="Override pickup anchor frame. Default: first inferred close frame.",
    )
    parser.add_argument(
        "--place-frame",
        type=int,
        default=None,
        help="Override place anchor frame. Default: first inferred open frame after pickup.",
    )
    parser.add_argument(
        "--no-detect-place",
        action="store_true",
        help=(
            "Only retarget pickup and keep the original pickup-to-place offset. "
            "Use this if the right Jenga is not visible in the current side views."
        ),
    )
    parser.add_argument(
        "--z-mode",
        choices=("keep-demo",),
        default="keep-demo",
        help="How to handle z coordinates. Current implementation preserves demo z.",
    )
    parser.add_argument(
        "--no-dynamic-place-z",
        action="store_true",
        help="Disable right-stack counting and keep the demo place z.",
    )
    parser.add_argument(
        "--stack-height-camera",
        default="left",
        help="Camera used to count the right-side stack height. Default: left.",
    )
    parser.add_argument(
        "--stack-count-override",
        type=int,
        default=None,
        help="Override detected right-side stack count. Useful for quick field correction.",
    )
    parser.add_argument(
        "--block-height-m",
        type=float,
        default=None,
        help=(
            "Per-Jenga Z increment in meters. Default: demo_place_z - demo_pick_z, "
            "so one right-side block preserves the original successful place height."
        ),
    )
    parser.add_argument(
        "--place-offset-m",
        type=parse_xyz_offset,
        default=(0.0, 0.0, 0.0),
        help=(
            "Final place-target offset as dx,dy,dz in meters, applied after mapping "
            "and dynamic stack-height adjustment. Example: 0,0,-0.005 lowers release by 5 mm."
        ),
    )
    parser.add_argument(
        "--max-stack-count",
        type=int,
        default=20,
        help="Clamp detected stack count to this many existing right-side blocks.",
    )
    parser.add_argument(
        "--stack-layer-pitch-px",
        type=float,
        default=None,
        help=(
            "Apparent vertical pixel increment per added stack layer. "
            "Default derives it from demo single-block height and --stack-layer-pitch-ratio."
        ),
    )
    parser.add_argument(
        "--stack-layer-pitch-ratio",
        type=float,
        default=0.17,
        help=(
            "Default apparent layer pitch as a ratio of demo right-block bbox height. "
            "For the current left view, 0.17 maps the 4-high stack silhouette correctly."
        ),
    )
    parser.add_argument(
        "--min-stack-height-quality",
        type=float,
        default=0.70,
        help="Minimum vision confidence required before dynamic place-z changes the target height.",
    )
    parser.add_argument(
        "--debug-dir",
        default=None,
        help="Optional directory for detection overlays and mapping_summary.json.",
    )
    parser.add_argument(
        "--keep-images",
        action="store_true",
        help="Keep embedded *_image fields in the output pkl. Default strips images for fast replay files.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run detection and mapping checks but do not write the output episode.",
    )
    parser.add_argument(
        "--skip-robot-check-command",
        action="store_true",
        help="Do not print the suggested 7_replay_fr3.sh dry-run command.",
    )
    return parser.parse_args()


def resolve_episode_file(path_like: str | Path) -> Path:
    path = Path(path_like).expanduser()
    if path.is_file():
        if path.name.endswith(".pkl.gz"):
            return path
        if path.name in {"metadata.json", "keyframes.json", "instruction.txt"}:
            return resolve_episode_file(path.parent)
        raise ValueError(f"Episode input file must end with .pkl.gz: {path}")
    if not path.is_dir():
        raise FileNotFoundError(f"Episode path does not exist: {path}")

    preferred = path / f"{path.name}.pkl.gz"
    if preferred.exists():
        return preferred
    candidates = sorted(path.glob("*.pkl.gz"))
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise FileNotFoundError(f"No .pkl.gz file found in episode directory: {path}")
    raise RuntimeError(f"Multiple .pkl.gz files found in {path}: {candidates}")


def discover_demo_files(template_demo: Path, demo_root: str | Path, max_demos: int) -> List[Path]:
    template_demo = resolve_episode_file(template_demo)
    if max_demos <= 1:
        return [template_demo]

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

    unique: List[Path] = [template_demo]
    for candidate in candidates:
        try:
            resolved = resolve_episode_file(candidate)
        except (FileNotFoundError, RuntimeError, ValueError):
            continue
        if resolved not in unique:
            unique.append(resolved)

    if len(unique) <= max_demos:
        return unique

    # Keep the template first, then sample the full High_Quality range evenly so
    # the mapping is not biased toward adjacent episodes.
    remainder = unique[1:]
    sample_count = max_demos - 1
    if sample_count <= 0:
        return [template_demo]
    indices = np.linspace(0, len(remainder) - 1, sample_count).round().astype(int)
    selected = [template_demo]
    for index in indices:
        candidate = remainder[int(index)]
        if candidate not in selected:
            selected.append(candidate)
    return selected[:max_demos]


def load_episode(path_like: str | Path) -> Tuple[Path, Dict[str, Any], List[MutableMapping[str, Any]], Dict[str, Any]]:
    episode_file = resolve_episode_file(path_like)
    with gzip.open(episode_file, "rb") as f:
        payload = pickle.load(f)
    if not isinstance(payload, dict):
        raise ValueError(f"Invalid episode payload in {episode_file}: expected dict")
    frames = payload.get("data") or payload.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError(f"Invalid episode payload in {episode_file}: missing non-empty data/frames")

    metadata_path = episode_file.parent / "metadata.json"
    metadata: Dict[str, Any] = {}
    if metadata_path.exists():
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            metadata = {}
    return episode_file, payload, frames, metadata


def camera_list(spec: str) -> List[str]:
    names = [name.strip() for name in str(spec).split(",") if name.strip()]
    if not names:
        raise ValueError("At least one camera must be requested")
    return names


def frame_rgb_image(frame: Mapping[str, Any], camera: str) -> np.ndarray:
    key = f"{camera}_image"
    if key not in frame:
        raise KeyError(
            f"Frame does not contain {key}. Use an episode with embedded images "
            "or live capture for current frames."
        )
    image = np.asarray(frame[key])
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"{key} must be an RGB image with shape HxWx3, got {image.shape}")
    if image.dtype != np.uint8:
        image = np.clip(image, 0, 255).astype(np.uint8)
    return image


def load_episode_images(frames: Sequence[Mapping[str, Any]], frame_index: int, cameras: Sequence[str]) -> Dict[str, np.ndarray]:
    if frame_index < 0:
        frame_index = len(frames) + frame_index
    if frame_index < 0 or frame_index >= len(frames):
        raise IndexError(f"Frame index {frame_index} is outside episode length {len(frames)}")
    frame = frames[frame_index]
    return {camera: frame_rgb_image(frame, camera) for camera in cameras}


def capture_live_images(cameras: Sequence[str], warmup_sec: float = 3.0) -> Dict[str, np.ndarray]:
    from dataclasses import replace

    from franka_capture.cameras.realsense import create_realsense_cameras
    from franka_capture.config.fr3_single import DEFAULT_CAMERAS

    requested = set(cameras)
    unknown = sorted(requested - set(DEFAULT_CAMERAS))
    if unknown:
        raise ValueError(f"Unknown configured camera(s): {unknown}. Available: {sorted(DEFAULT_CAMERAS)}")

    camera_configs = {
        name: replace(DEFAULT_CAMERAS[name], depth=False, align_depth=True)
        for name in cameras
    }
    handles = create_realsense_cameras(camera_configs, allow_missing=False)
    try:
        warmup_sec = max(0.0, float(warmup_sec))
        warmup_deadline = time.monotonic() + warmup_sec
        warmup_reads = 0
        while time.monotonic() < warmup_deadline or warmup_reads < 10:
            for handle in handles.values():
                handle.read()
            warmup_reads += 1
        images = {}
        for name in cameras:
            rgb, _ = handles[name].read()
            images[name] = np.asarray(rgb, dtype=np.uint8)
        return images
    finally:
        for handle in handles.values():
            handle.close()


def infer_anchor_frames(
    frames: Sequence[Mapping[str, Any]],
    pickup_override: Optional[int],
    place_override: Optional[int],
) -> AnchorFrames:
    widths = np.asarray([frame_gripper_width(frame) for frame in frames], dtype=float)
    if widths.ndim != 1 or widths.size != len(frames):
        raise ValueError("Invalid gripper width trajectory")
    if not np.all(np.isfinite(widths)):
        raise ValueError("Gripper width trajectory contains NaN or Inf")

    width_range = float(widths.max() - widths.min())
    if width_range < 0.002:
        raise ValueError(
            "Could not infer pickup/place anchors: gripper target width barely changes. "
            "Pass --pickup-frame and --place-frame."
        )
    threshold = float(widths.min() + 0.45 * width_range)
    closed = np.flatnonzero(widths <= threshold)
    if len(closed) == 0:
        raise ValueError("Could not find closed gripper frames; pass --pickup-frame and --place-frame")

    pickup = int(closed[0]) if pickup_override is None else int(pickup_override)
    if place_override is None:
        after_open = np.flatnonzero((np.arange(len(widths)) > closed[0]) & (widths > threshold))
        place = int(after_open[0]) if len(after_open) else int(closed[-1])
    else:
        place = int(place_override)
    if pickup < 0 or pickup >= len(frames):
        raise IndexError(f"Pickup frame {pickup} outside episode length {len(frames)}")
    if place < 0 or place >= len(frames):
        raise IndexError(f"Place frame {place} outside episode length {len(frames)}")
    if place <= pickup:
        raise ValueError(f"Place frame must be after pickup frame, got pickup={pickup}, place={place}")
    return AnchorFrames(
        pickup=pickup,
        place=place,
        closed_threshold_m=threshold,
        closed_range=(int(closed[0]), int(closed[-1])),
    )


def frame_gripper_width(frame: Mapping[str, Any]) -> float:
    for key in ("gripper_target_width", "gripper_width"):
        if key in frame:
            return float(frame[key])
    closedness = float(frame.get("gripper_closedness", frame.get("gripper_01closedness", 0.0)))
    max_width = 0.08
    return max_width * (1.0 - np.clip(closedness, 0.0, 1.0))


def frame_pose_xyz(frame: Mapping[str, Any]) -> np.ndarray:
    pose = np.asarray(frame["pose"], dtype=float)
    if pose.shape != (6,):
        raise ValueError(f"pose must have shape (6,), got {pose.shape}")
    return pose[:3].copy()


def dedupe_block_detections(
    detections: Sequence[BlockDetection],
    min_center_dist_px: float = 28.0,
    max_count: int = 8,
) -> List[BlockDetection]:
    deduped: List[BlockDetection] = []
    for det in sorted(detections, key=lambda item: item.score, reverse=True):
        center = np.asarray(det.center, dtype=float)
        if any(
            np.linalg.norm(center - np.asarray(existing.center, dtype=float)) < min_center_dist_px
            for existing in deduped
        ):
            continue
        deduped.append(det)
        if len(deduped) >= max_count:
            break
    return deduped


def detect_left_combined_fallback_blocks(camera: str, image_rgb: np.ndarray) -> List[BlockDetection]:
    if camera != "left":
        return []
    candidates: List[BlockDetection] = []
    candidates.extend(detect_left_dark_wood_blocks(camera, image_rgb))
    candidates.extend(detect_blocks_intensity_roi(camera, image_rgb))
    return dedupe_block_detections(candidates)


def detect_blocks(camera: str, image_rgb: np.ndarray) -> List[BlockDetection]:
    """Detect blue-gray Jenga blocks on the light blue board.

    The detector is intentionally classical and conservative. It favors the
    clean left side view and may skip cluttered/occluded cameras instead of
    returning low-confidence detections.
    """

    image_rgb = np.asarray(image_rgb, dtype=np.uint8)
    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    board_mask = ((h >= 78) & (h <= 126) & (s >= 8) & (v >= 65)).astype(np.uint8) * 255
    board_mask = cv2.morphologyEx(board_mask, cv2.MORPH_CLOSE, np.ones((17, 17), np.uint8))
    board_mask = cv2.morphologyEx(board_mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(board_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        if camera == "left":
            return detect_left_combined_fallback_blocks(camera, image_rgb)
        return []

    board_contour = max(contours, key=cv2.contourArea)
    board_area = float(cv2.contourArea(board_contour))
    if board_area < image_rgb.shape[0] * image_rgb.shape[1] * 0.08:
        if camera == "left":
            return detect_left_combined_fallback_blocks(camera, image_rgb)
        return []
    board = np.zeros(board_mask.shape, dtype=np.uint8)
    cv2.drawContours(board, [board_contour], -1, 255, -1)

    roi_gray = gray[board > 0]
    if roi_gray.size < 100:
        if camera == "left":
            return detect_left_combined_fallback_blocks(camera, image_rgb)
        return []
    median_gray = float(np.median(roi_gray))
    dark_delta = 18.0 if camera == "left" else 22.0
    dark = ((board > 0) & (gray < median_gray - dark_delta) & (v < 232)).astype(np.uint8) * 255
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: List[BlockDetection] = []
    image_area = float(image_rgb.shape[0] * image_rgb.shape[1])
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 250 or area > image_area * 0.04:
            continue
        rect = cv2.minAreaRect(contour)
        (cx, cy), (w, h_rect), angle = rect
        if w <= 1.0 or h_rect <= 1.0:
            continue
        long_side = max(float(w), float(h_rect))
        short_side = min(float(w), float(h_rect))
        aspect = long_side / max(short_side, 1e-6)
        if aspect < 1.35 or aspect > 8.5:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        if bw < 10 or bh < 10:
            continue
        touches_border = (
            x <= 8
            or y <= 8
            or x + bw >= image_rgb.shape[1] - 8
            or y + bh >= image_rgb.shape[0] - 8
        )
        if camera != "left":
            # The middle view is frequently occluded by the arm/gripper in the
            # upper half. Treat it as an auxiliary board-surface view only.
            if cy < image_rgb.shape[0] * 0.48 or touches_border:
                continue
        else:
            # The left view is the primary mapping view. Ignore robot/gripper
            # candidates in the upper half and partial objects at image borders.
            if cy < image_rgb.shape[0] * 0.38 or touches_border:
                continue
        extent = area / float(max(bw * bh, 1))
        if extent < 0.15:
            continue

        # Real blocks are compact, dark, and elongated. Penalize candidates near
        # the image border, where cables/robot links often touch the board mask.
        border_penalty = 0.65 if touches_border else 1.0
        aspect_score = min(aspect / 2.2, 2.2 / max(aspect, 1e-6))
        area_score = min(area / 900.0, 900.0 / max(area, 1.0))
        score = float(max(0.0, aspect_score) * max(0.0, area_score) * border_penalty)
        detections.append(
            BlockDetection(
                camera=camera,
                center=(float(cx), float(cy)),
                size=(float(w), float(h_rect)),
                angle_deg=float(angle),
                area=area,
                aspect=float(aspect),
                score=score,
                bbox=(int(x), int(y), int(bw), int(bh)),
            )
        )

    detections.sort(key=lambda det: det.score, reverse=True)
    if camera == "left":
        fallback = detect_left_combined_fallback_blocks(camera, image_rgb)
        if len(detections) < 2:
            if len(fallback) >= 2:
                return fallback
            if len(fallback) > len(detections):
                return fallback
        elif len(fallback) >= 2:
            detection_y = np.mean([det.center[1] for det in detections[:2]])
            fallback_y = np.mean([det.center[1] for det in fallback[:2]])
            if fallback_y > detection_y + image_rgb.shape[0] * 0.08:
                return fallback
    if len(detections) < 2 and camera == "middle":
        fallback = detect_middle_wood_blocks(camera, image_rgb)
        if len(fallback) >= 2:
            return fallback
    return detections


def detect_blocks_intensity_roi(camera: str, image_rgb: np.ndarray) -> List[BlockDetection]:
    """Color-agnostic fallback for the left camera.

    The live board may be natural wood while the demo board is blue. In that
    case, a local intensity contrast detector in the known board half of the
    left image is more reliable than hue thresholds.
    """

    gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
    height, width = gray.shape
    yy, xx = np.indices((height, width))
    roi = (xx > width * 0.36) & (yy > height * 0.30)
    values = gray[roi]
    if values.size < 100:
        return []
    median_gray = float(np.median(values))
    dark = (roi & (gray < median_gray - 20.0) & (gray > 40)).astype(np.uint8) * 255
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: List[BlockDetection] = []
    image_area = float(height * width)
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 350 or area > image_area * 0.035:
            continue
        rect = cv2.minAreaRect(contour)
        (cx, cy), (w, h_rect), angle = rect
        if w <= 1.0 or h_rect <= 1.0:
            continue
        long_side = max(float(w), float(h_rect))
        short_side = min(float(w), float(h_rect))
        aspect = long_side / max(short_side, 1e-6)
        if aspect < 1.45 or aspect > 6.5:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        touches_border = (
            x <= 8
            or y <= 8
            or x + bw >= width - 8
            or y + bh >= height - 8
        )
        if touches_border:
            continue
        extent = area / float(max(bw * bh, 1))
        if extent < 0.15:
            continue
        aspect_score = min(aspect / 2.3, 2.3 / max(aspect, 1e-6))
        area_score = min(area / 850.0, 850.0 / max(area, 1.0))
        score = float(max(0.0, aspect_score) * max(0.0, area_score))
        detections.append(
            BlockDetection(
                camera=camera,
                center=(float(cx), float(cy)),
                size=(float(w), float(h_rect)),
                angle_deg=float(angle),
                area=area,
                aspect=float(aspect),
                score=score,
                bbox=(int(x), int(y), int(bw), int(bh)),
            )
        )
    detections.sort(key=lambda det: det.score, reverse=True)
    return detections


def detect_left_dark_wood_blocks(camera: str, image_rgb: np.ndarray) -> List[BlockDetection]:
    """Dark-exposure fallback for the left camera.

    After RealSense auto-exposure settles, the board can become quite dark.
    The normal hue/board mask then misses the blocks entirely. This detector
    searches the known board/object half of the left view with several local
    contrast thresholds and rejects upper-left board-shadow contours.
    """

    if camera != "left":
        return []
    gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
    height, width = gray.shape
    yy, xx = np.indices((height, width))
    roi = (
        (xx > width * 0.48)
        & (yy > height * 0.42)
        & (xx < width * 0.98)
        & (yy < height * 0.98)
    )
    values = gray[roi]
    if values.size < 100:
        return []
    median_gray = float(np.median(values))

    candidates: List[BlockDetection] = []
    for delta in (5.0, 8.0, 11.0, 14.0, 18.0, 22.0):
        mask = (roi & (gray < median_gray - delta) & (gray > 8)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 250 or area > 6500:
                continue
            rect = cv2.minAreaRect(contour)
            (cx, cy), (w, h_rect), angle = rect
            if w <= 1.0 or h_rect <= 1.0:
                continue
            long_side = max(float(w), float(h_rect))
            short_side = min(float(w), float(h_rect))
            aspect = long_side / max(short_side, 1e-6)
            if aspect < 1.0 or aspect > 10.5:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < 14 or bh < 14 or bw > 150 or bh > 150:
                continue
            if x <= 8 or y <= 8 or x + bw >= width - 8 or y + bh >= height - 8:
                continue
            # Reject dark board edge/shadow regions that are not in the object
            # work area. Current left-view blocks live in the lower-right board.
            if cx < width * 0.55 or cx > width * 0.86 or cy < height * 0.50:
                continue
            extent = area / float(max(bw * bh, 1))
            if extent < 0.12:
                continue
            aspect_score = min(aspect / 2.4, 2.4 / max(aspect, 1e-6))
            area_score = min(area / 1200.0, 2400.0 / max(area, 1.0))
            lower_board_score = 1.0 - min(abs(cy - height * 0.70) / (height * 0.45), 0.45)
            score = float(max(0.0, aspect_score) * max(0.0, area_score) * lower_board_score)
            candidates.append(
                BlockDetection(
                    camera=camera,
                    center=(float(cx), float(cy)),
                    size=(float(w), float(h_rect)),
                    angle_deg=float(angle),
                    area=area,
                    aspect=float(aspect),
                    score=score,
                    bbox=(int(x), int(y), int(bw), int(bh)),
                )
            )

    candidates.sort(key=lambda det: det.score, reverse=True)
    deduped: List[BlockDetection] = []
    for det in candidates:
        center = np.asarray(det.center, dtype=float)
        if any(np.linalg.norm(center - np.asarray(existing.center, dtype=float)) < 28.0 for existing in deduped):
            continue
        deduped.append(det)
        if len(deduped) >= 8:
            break
    return deduped


def detect_middle_wood_blocks(camera: str, image_rgb: np.ndarray) -> List[BlockDetection]:
    """Fallback for the over-exposed middle side view on the natural wood board."""

    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    height, width = hsv.shape[:2]
    yy, xx = np.indices((height, width))
    roi = (xx > width * 0.25) & (xx < width * 0.98) & (yy > height * 0.25) & (yy < height * 0.98)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    mask = (
        roi
        & (h >= 10)
        & (h <= 55)
        & (s >= 5)
        & (s <= 150)
        & (v >= 100)
        & (v <= 252)
    ).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: List[BlockDetection] = []
    image_area = float(height * width)
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 150 or area > image_area * 0.03:
            continue
        rect = cv2.minAreaRect(contour)
        (cx, cy), (w, h_rect), angle = rect
        if w <= 1.0 or h_rect <= 1.0:
            continue
        long_side = max(float(w), float(h_rect))
        short_side = min(float(w), float(h_rect))
        aspect = long_side / max(short_side, 1e-6)
        if aspect < 1.15 or aspect > 8.0:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        if x <= 8 or y <= 8 or x + bw >= width - 8 or y + bh >= height - 8:
            continue
        extent = area / float(max(bw * bh, 1))
        if extent < 0.12:
            continue
        aspect_score = min(aspect / 2.2, 2.2 / max(aspect, 1e-6))
        area_score = min(area / 650.0, 650.0 / max(area, 1.0))
        center_score = 1.0 - min(abs(cx - width * 0.62) / (width * 0.55), 0.6)
        score = float(max(0.0, aspect_score) * max(0.0, area_score) * center_score)
        detections.append(
            BlockDetection(
                camera=camera,
                center=(float(cx), float(cy)),
                size=(float(w), float(h_rect)),
                angle_deg=float(angle),
                area=area,
                aspect=float(aspect),
                score=score,
                bbox=(int(x), int(y), int(bw), int(bh)),
            )
        )
    detections.sort(key=lambda det: det.score, reverse=True)
    return detections


def assign_pick_place(camera: str, detections: Sequence[BlockDetection]) -> Optional[Tuple[BlockDetection, BlockDetection]]:
    if len(detections) < 2:
        return None
    candidates = sorted(detections[:6], key=lambda det: det.score, reverse=True)

    # Pick the two candidates with enough separation to avoid duplicate fragments.
    best_pair: Optional[Tuple[BlockDetection, BlockDetection, float]] = None
    for i, first in enumerate(candidates):
        p0 = np.asarray(first.center, dtype=float)
        for second in candidates[i + 1 :]:
            p1 = np.asarray(second.center, dtype=float)
            dist = float(np.linalg.norm(p0 - p1))
            if dist < 35.0:
                continue
            score = first.score + second.score + min(dist / 120.0, 1.0)
            if best_pair is None or score > best_pair[2]:
                best_pair = (first, second, score)
    if best_pair is None:
        return None

    first, second, _ = best_pair
    if camera == "left":
        ordered = sorted([copy.copy(first), copy.copy(second)], key=lambda det: det.center[0])
        pick = ordered[0]
        place = ordered[1]
    else:
        # For auxiliary side views, the lower visible block is the best available
        # default, but users should inspect overlays before execution.
        ordered = sorted(
            [copy.copy(first), copy.copy(second)],
            key=lambda det: (det.center[1], -det.center[0]),
            reverse=True,
        )
        pick = ordered[0]
        place = ordered[1]
    pick.role = "pick"
    place.role = "place"
    return pick, place


def estimate_similarity_from_two_points(
    src_pick: np.ndarray,
    src_place: np.ndarray,
    dst_pick_xy: np.ndarray,
    dst_place_xy: np.ndarray,
) -> np.ndarray:
    src_pick = np.asarray(src_pick, dtype=float).reshape(2)
    src_place = np.asarray(src_place, dtype=float).reshape(2)
    dst_pick_xy = np.asarray(dst_pick_xy, dtype=float).reshape(2)
    dst_place_xy = np.asarray(dst_place_xy, dtype=float).reshape(2)

    src_vec = src_place - src_pick
    dst_vec = dst_place_xy - dst_pick_xy
    src_norm = float(np.linalg.norm(src_vec))
    dst_norm = float(np.linalg.norm(dst_vec))
    if src_norm < 20.0:
        raise ValueError(f"Demo image pick/place detections are too close: {src_norm:.3f} px")
    if dst_norm < 0.02:
        raise ValueError(f"Demo robot pick/place anchors are too close: {dst_norm:.4f} m")

    src_angle = math.atan2(float(src_vec[1]), float(src_vec[0]))
    dst_angle = math.atan2(float(dst_vec[1]), float(dst_vec[0]))
    theta = dst_angle - src_angle
    scale = dst_norm / src_norm
    c, s = math.cos(theta), math.sin(theta)
    rotation = np.asarray([[c, -s], [s, c]], dtype=float)
    linear = scale * rotation
    translation = dst_pick_xy - linear @ src_pick
    affine = np.eye(3, dtype=float)
    affine[:2, :2] = linear
    affine[:2, 2] = translation
    return affine


def apply_affine_2d(affine: np.ndarray, point: np.ndarray) -> np.ndarray:
    point_h = np.asarray([float(point[0]), float(point[1]), 1.0], dtype=float)
    return (np.asarray(affine, dtype=float) @ point_h)[:2]


def apply_homography_2d(matrix: Sequence[Sequence[float]], pixels: np.ndarray) -> np.ndarray:
    homography = np.asarray(matrix, dtype=np.float64)
    if homography.shape != (3, 3):
        raise ValueError(f"mapping model homography must have shape 3x3, got {homography.shape}")
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    mapped = cv2.perspectiveTransform(points, homography)
    return mapped.reshape(-1, 2)


def apply_affine_2d_matrix(matrix: Sequence[Sequence[float]], pixels: np.ndarray) -> np.ndarray:
    affine = np.asarray(matrix, dtype=np.float64)
    if affine.shape != (2, 3):
        raise ValueError(f"mapping model affine matrix must have shape 2x3, got {affine.shape}")
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    points_h = np.concatenate([points, np.ones((points.shape[0], 1), dtype=np.float64)], axis=1)
    return points_h @ affine.T


def apply_pixel_mapping_model(model_info: Mapping[str, Any], pixels: np.ndarray) -> np.ndarray:
    model_type = model_info.get("type")
    matrix = model_info.get("matrix")
    if matrix is None:
        raise ValueError("mapping model is missing matrix")
    if model_type == "homography":
        return apply_homography_2d(matrix, pixels)
    if model_type == "affine":
        return apply_affine_2d_matrix(matrix, pixels)
    raise ValueError(f"Unsupported mapping model type: {model_type!r}")


def load_mapping_model(path_like: str | Path) -> Dict[str, Any]:
    path = Path(path_like).expanduser()
    model = json.loads(path.read_text(encoding="utf-8"))
    if model.get("schema_version") != "jenga_pixel_robot_mapping_v1":
        raise ValueError(
            f"Unsupported mapping model schema {model.get('schema_version')!r} in {path}"
        )
    if not isinstance(model.get("camera_models"), dict):
        raise ValueError(f"Mapping model is missing camera_models: {path}")
    return model


def first_finite_stat(stats: Mapping[str, Any], keys: Sequence[str]) -> Optional[float]:
    for key in keys:
        value = stats.get(key)
        if value is None:
            continue
        value_float = float(value)
        if np.isfinite(value_float):
            return value_float
    return None


def camera_model_error_m(camera_model: Mapping[str, Any]) -> Optional[float]:
    cv_stats = (
        camera_model.get("cross_validation", {})
        .get("error_m", {})
    )
    cv_error = first_finite_stat(cv_stats, ("p90", "median", "mean"))
    if cv_error is not None:
        return cv_error
    train_stats = (
        camera_model.get("model", {})
        .get("train_inlier_error_m", {})
    )
    return first_finite_stat(train_stats, ("p90", "median", "mean"))


def quality_from_model_error(error_m: Optional[float]) -> float:
    if error_m is None:
        return 0.75
    # 5 cm validation error still produces a useful weight, while very noisy
    # camera models are naturally down-weighted or rejected by min_quality.
    return float(np.clip(1.0 / (1.0 + 5.0 * max(error_m, 0.0)), 0.05, 1.0))


def mapping_quality(
    demo_pick_px: np.ndarray,
    demo_place_px: np.ndarray,
    current_pick_px: np.ndarray,
    current_place_px: np.ndarray,
) -> float:
    demo_dist = float(np.linalg.norm(demo_place_px - demo_pick_px))
    current_dist = float(np.linalg.norm(current_place_px - current_pick_px))
    if demo_dist < 20.0 or current_dist < 20.0:
        return 0.0
    ratio = min(demo_dist / current_dist, current_dist / demo_dist)
    return float(np.clip(ratio, 0.0, 1.0))


def horizontal_overlap_fraction(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
    ax, _, aw, _ = a
    bx, _, bw, _ = b
    left = max(ax, bx)
    right = min(ax + aw, bx + bw)
    overlap = max(0, right - left)
    return float(overlap / max(min(aw, bw), 1))


def union_bbox(detections: Sequence[BlockDetection]) -> Tuple[int, int, int, int]:
    if not detections:
        raise ValueError("Cannot union empty detections")
    xs = [det.bbox[0] for det in detections]
    ys = [det.bbox[1] for det in detections]
    rights = [det.bbox[0] + det.bbox[2] for det in detections]
    bottoms = [det.bbox[1] + det.bbox[3] for det in detections]
    x0, y0 = min(xs), min(ys)
    x1, y1 = max(rights), max(bottoms)
    return int(x0), int(y0), int(x1 - x0), int(y1 - y0)


def grouped_vertical_centers(
    detections: Sequence[BlockDetection],
    min_separation_px: float,
) -> List[List[BlockDetection]]:
    groups: List[List[BlockDetection]] = []
    for det in sorted(detections, key=lambda item: item.center[1]):
        if not groups:
            groups.append([det])
            continue
        previous_center = float(np.mean([item.center[1] for item in groups[-1]]))
        if abs(det.center[1] - previous_center) >= min_separation_px:
            groups.append([det])
        else:
            groups[-1].append(det)
    return groups


def estimate_right_stack_height(
    camera: str,
    detections: Sequence[BlockDetection],
    pair: Optional[Tuple[BlockDetection, BlockDetection]],
    max_stack_count: int,
    reference_place: Optional[BlockDetection] = None,
    stack_layer_pitch_px: Optional[float] = None,
    stack_layer_pitch_ratio: float = 0.17,
) -> StackHeightEstimate:
    if max_stack_count < 1:
        raise ValueError("--max-stack-count must be >= 1")
    if pair is None:
        raise RuntimeError("Cannot estimate stack height without a pick/place pair")

    pick, place = pair
    midpoint_x = 0.5 * (float(pick.center[0]) + float(place.center[0]))
    place_bbox = place.bbox
    x_margin = max(50.0, float(place_bbox[2]) * 0.9)
    min_pick_separation = max(30.0, float(pick.bbox[2]) * 0.6)

    candidates: List[BlockDetection] = []
    for det in detections[:12]:
        overlap = horizontal_overlap_fraction(det.bbox, place_bbox)
        near_place_x = abs(float(det.center[0]) - float(place.center[0])) <= x_margin
        right_side = float(det.center[0]) >= midpoint_x
        separate_from_pick = abs(float(det.center[0]) - float(pick.center[0])) >= min_pick_separation
        if separate_from_pick and (right_side or overlap >= 0.35) and (near_place_x or overlap >= 0.20):
            candidates.append(det)
    if not any(det.center == place.center for det in candidates):
        candidates.append(place)

    stack_bbox = union_bbox(candidates)
    stack_height_px = float(stack_bbox[3])
    stack_area_px = float(sum(max(det.area, 1.0) for det in candidates))

    if reference_place is not None:
        single_block_bbox_height_px = float(reference_place.bbox[3])
        block_step_px = float(min(reference_place.size))
        single_block_area_px = float(max(reference_place.area, 1.0))
        reference_method = "demo_single_place_reference"
    else:
        # Fallback only. The current place detection may already be a stack, so
        # do not let it inflate the single-block baseline too much.
        single_block_bbox_height_px = float(max(pick.bbox[3], min(place.bbox[3], pick.bbox[3] * 1.25)))
        block_step_px = float(min(pick.size))
        single_block_area_px = float(max(pick.area, 1.0))
        reference_method = "current_pick_fallback_reference"
    block_step_px = max(block_step_px, single_block_bbox_height_px * 0.35, 8.0)
    if stack_layer_pitch_px is not None:
        visual_layer_pitch_px = float(stack_layer_pitch_px)
    else:
        visual_layer_pitch_px = float(single_block_bbox_height_px) * float(stack_layer_pitch_ratio)
    if not np.isfinite(visual_layer_pitch_px) or visual_layer_pitch_px <= 0.0:
        raise ValueError(f"Invalid stack layer pitch: {visual_layer_pitch_px}")
    visual_layer_pitch_px = max(4.0, visual_layer_pitch_px)

    groups = grouped_vertical_centers(candidates, min_separation_px=block_step_px * 0.55)
    cluster_count = len(groups)
    height_ratio = stack_height_px / max(single_block_bbox_height_px, 1.0)
    silhouette_count = 1 + int(np.floor(max(0.0, stack_height_px - single_block_bbox_height_px) / block_step_px + 0.5))
    if stack_height_px <= single_block_bbox_height_px + visual_layer_pitch_px * 0.55:
        calibrated_silhouette_count = 1
    else:
        calibrated_silhouette_count = 1 + int(
            np.floor(max(0.0, stack_height_px - single_block_bbox_height_px) / visual_layer_pitch_px + 0.5)
        )
    area_count = int(np.floor(stack_area_px / single_block_area_px + 0.5))
    silhouette_count = int(np.clip(silhouette_count, 1, max_stack_count))
    calibrated_silhouette_count = int(np.clip(calibrated_silhouette_count, 1, max_stack_count))
    cluster_count = int(np.clip(cluster_count, 1, max_stack_count))
    area_count = int(np.clip(area_count, 1, max_stack_count))

    if area_count >= 2 and calibrated_silhouette_count > area_count + 1:
        count = int(area_count)
        method = f"area_guarded_calibrated_silhouette_with_{reference_method}"
        quality = 0.76
    elif cluster_count > 1:
        count = int(max(cluster_count, calibrated_silhouette_count, silhouette_count))
        method = f"candidate_cluster_with_{reference_method}"
        quality = 0.9 if abs(cluster_count - calibrated_silhouette_count) <= 1 else 0.78
    elif calibrated_silhouette_count > 1:
        count = int(calibrated_silhouette_count)
        method = f"calibrated_silhouette_height_with_{reference_method}"
        # Height-only evidence is useful for merged stacks, but mark it lower
        # confidence than explicit separated block detections.
        quality = 0.74 if height_ratio >= 1.20 else 0.66
        if abs(area_count - calibrated_silhouette_count) <= 1:
            quality += 0.08
    elif silhouette_count > 1:
        count = int(silhouette_count)
        method = f"coarse_silhouette_height_with_{reference_method}"
        quality = 0.70 if height_ratio >= 1.45 else 0.64
    else:
        count = 1
        method = f"single_block_with_{reference_method}"
        quality = 0.86 if height_ratio <= 1.35 else 0.70

    return StackHeightEstimate(
        camera=camera,
        stack_count=count,
        stack_height_px=stack_height_px,
        block_step_px=block_step_px,
        visual_layer_pitch_px=visual_layer_pitch_px,
        single_block_bbox_height_px=single_block_bbox_height_px,
        pick_area_px=float(max(pick.area, 1.0)),
        stack_area_px=stack_area_px,
        silhouette_count=silhouette_count,
        calibrated_silhouette_count=calibrated_silhouette_count,
        cluster_count=cluster_count,
        area_count=area_count,
        method=method,
        quality=quality,
        place_detection=place.to_json(),
        candidate_detections=[det.to_json() for det in candidates],
    )


def write_stack_height_overlay(
    path: Path,
    image_rgb: np.ndarray,
    detections: Sequence[BlockDetection],
    pair: Optional[Tuple[BlockDetection, BlockDetection]],
    estimate: Optional[StackHeightEstimate],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    role_by_center = {}
    if pair is not None:
        role_by_center[tuple(pair[0].center)] = "pick"
        role_by_center[tuple(pair[1].center)] = "place"

    for idx, det in enumerate(detections[:8]):
        color = (0, 0, 255)
        role = role_by_center.get(tuple(det.center))
        if role == "pick":
            color = (0, 255, 0)
        elif role == "place":
            color = (255, 0, 0)
        rect = (det.center, det.size, det.angle_deg)
        points = cv2.boxPoints(rect).astype(int)
        cv2.drawContours(bgr, [points], -1, color, 2)
        label = role or str(idx)
        cv2.putText(
            bgr,
            label,
            (int(det.center[0]) + 4, int(det.center[1]) - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    if estimate is not None:
        candidate_boxes = [det["bbox"] for det in estimate.candidate_detections]
        if candidate_boxes:
            x0 = min(int(box[0]) for box in candidate_boxes)
            y0 = min(int(box[1]) for box in candidate_boxes)
            x1 = max(int(box[0]) + int(box[2]) for box in candidate_boxes)
            y1 = max(int(box[1]) + int(box[3]) for box in candidate_boxes)
            cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 255, 255), 2)
            cv2.putText(
                bgr,
                f"stack={estimate.stack_count}",
                (x0, max(20, y0 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
    cv2.imwrite(str(path), bgr)


def dynamic_place_z_adjustment(
    current_images: Mapping[str, np.ndarray],
    demo_images: Mapping[str, np.ndarray],
    target_pick_xyz: np.ndarray,
    target_place_xyz: np.ndarray,
    demo_pick_xyz: np.ndarray,
    demo_place_xyz: np.ndarray,
    stack_height_camera: str,
    block_height_m: Optional[float],
    stack_count_override: Optional[int],
    max_stack_count: int,
    stack_layer_pitch_px: Optional[float],
    stack_layer_pitch_ratio: float,
    min_stack_height_quality: float,
    debug_dir: Optional[Path],
) -> Tuple[np.ndarray, Dict[str, Any]]:
    original_place_xyz = np.asarray(target_place_xyz, dtype=float).copy()
    height_step_m = float(block_height_m) if block_height_m is not None else float(demo_place_xyz[2] - demo_pick_xyz[2])
    if not np.isfinite(height_step_m) or height_step_m <= 0.0:
        raise ValueError(f"Invalid dynamic place-z block height: {height_step_m}")

    summary: Dict[str, Any] = {
        "enabled": True,
        "camera": stack_height_camera,
        "block_height_m": height_step_m,
        "original_target_place_xyz": original_place_xyz.astype(float).tolist(),
    }

    try:
        if stack_count_override is not None:
            stack_count = int(np.clip(int(stack_count_override), 1, max_stack_count))
            summary.update(
                {
                    "method": "manual_override",
                    "stack_count": stack_count,
                    "stack_count_override": int(stack_count_override),
                }
            )
        else:
            if stack_height_camera not in current_images:
                raise KeyError(f"Camera {stack_height_camera!r} not available in current images")
            image = current_images[stack_height_camera]
            detections = detect_blocks(stack_height_camera, image)
            pair = assign_pick_place(stack_height_camera, detections)
            reference_place = None
            reference_pair = None
            if stack_height_camera in demo_images:
                reference_detections = detect_blocks(stack_height_camera, demo_images[stack_height_camera])
                reference_pair = assign_pick_place(stack_height_camera, reference_detections)
                if reference_pair is not None:
                    reference_place = reference_pair[1]
            estimate = estimate_right_stack_height(
                stack_height_camera,
                detections,
                pair,
                max_stack_count=max_stack_count,
                reference_place=reference_place,
                stack_layer_pitch_px=stack_layer_pitch_px,
                stack_layer_pitch_ratio=stack_layer_pitch_ratio,
            )
            stack_count = estimate.stack_count
            summary.update(
                {
                    "method": "vision_stack_height",
                    "stack_count": stack_count,
                    "estimate": estimate.to_json(),
                    "min_quality": float(min_stack_height_quality),
                    "reference_place_detection": None if reference_place is None else reference_place.to_json(),
                }
            )
            if estimate.quality < float(min_stack_height_quality):
                summary.update(
                    {
                        "method": "vision_low_confidence_keep_original",
                        "low_confidence_reason": (
                            f"stack height quality {estimate.quality:.3f} < "
                            f"{float(min_stack_height_quality):.3f}"
                        ),
                        "adjusted_target_place_xyz": original_place_xyz.astype(float).tolist(),
                        "place_z_delta_m": 0.0,
                    }
                )
                if debug_dir is not None:
                    write_stack_height_overlay(
                        debug_dir / f"stack_height_{stack_height_camera}.jpg",
                        image,
                        detections,
                        pair,
                        estimate,
                    )
                return original_place_xyz, summary
            if debug_dir is not None:
                write_stack_height_overlay(
                    debug_dir / f"stack_height_{stack_height_camera}.jpg",
                    image,
                    detections,
                    pair,
                    estimate,
                )

        adjusted = original_place_xyz.copy()
        adjusted[2] = float(target_pick_xyz[2]) + stack_count * height_step_m
        summary["adjusted_target_place_xyz"] = adjusted.astype(float).tolist()
        summary["place_z_delta_m"] = float(adjusted[2] - original_place_xyz[2])
        return adjusted, summary
    except Exception as exc:
        summary.update(
            {
                "method": "failed_keep_original",
                "failure": f"{type(exc).__name__}: {exc}",
                "stack_count": None,
                "adjusted_target_place_xyz": original_place_xyz.astype(float).tolist(),
                "place_z_delta_m": 0.0,
            }
        )
        return original_place_xyz, summary


def apply_place_offset(
    target_place_xyz: np.ndarray,
    place_offset_m: Sequence[float],
) -> Tuple[np.ndarray, Dict[str, Any]]:
    offset = np.asarray(place_offset_m, dtype=float).reshape(3)
    if not np.all(np.isfinite(offset)):
        raise ValueError(f"Invalid --place-offset-m: {offset}")
    before = np.asarray(target_place_xyz, dtype=float).copy()
    adjusted = before + offset
    return adjusted, {
        "offset_m": offset.astype(float).tolist(),
        "before_offset_target_place_xyz": before.astype(float).tolist(),
        "target_place_xyz": adjusted.astype(float).tolist(),
        "applied": bool(np.linalg.norm(offset) > 0.0),
    }


def predict_targets_from_mapping_model(
    cameras: Sequence[str],
    current_images: Mapping[str, np.ndarray],
    mapping_model: Mapping[str, Any],
    mapping_model_path: Path,
    demo_pick_xyz: np.ndarray,
    demo_place_xyz: np.ndarray,
    min_quality: float,
    require_all: bool,
    no_detect_place: bool,
    max_pick_shift_m: float,
    max_place_shift_m: float,
    debug_dir: Optional[Path],
) -> Tuple[np.ndarray, np.ndarray, List[ModelCameraPrediction], Dict[str, Any]]:
    summary: Dict[str, Any] = {
        "mapping_method": "pixel_to_robot_homography_model",
        "mapping_model_path": str(mapping_model_path),
        "mapping_model_schema": mapping_model.get("schema_version"),
        "mapping_model_demo_count": len(mapping_model.get("demo_files", [])),
        "cameras": {},
    }
    predictions: List[ModelCameraPrediction] = []
    failures: Dict[str, str] = {}
    camera_models = mapping_model.get("camera_models", {})

    for camera in cameras:
        current_dets = detect_blocks(camera, current_images[camera])
        current_pair = assign_pick_place(camera, current_dets)
        summary["cameras"][camera] = {
            "current_detections": [det.to_json() for det in current_dets[:8]],
        }
        if debug_dir is not None:
            write_overlay(debug_dir / f"current_{camera}.jpg", current_images[camera], current_dets, current_pair)

        camera_model = camera_models.get(camera)
        if not isinstance(camera_model, Mapping) or not camera_model.get("available"):
            failures[camera] = "mapping model has no available camera model"
            continue
        if current_pair is None:
            failures[camera] = "current image did not produce two block detections"
            continue

        model_info = camera_model.get("model", {})

        current_pick, current_place = current_pair
        current_pick_px = np.asarray(current_pick.center, dtype=float)
        current_place_px = np.asarray(current_place.center, dtype=float)
        try:
            mapped = apply_pixel_mapping_model(
                model_info,
                np.stack([current_pick_px, current_place_px], axis=0),
            )
        except Exception as exc:
            failures[camera] = f"mapping model transform failed: {type(exc).__name__}: {exc}"
            continue
        pickup_xy = mapped[0]
        if no_detect_place:
            original_offset = demo_place_xyz[:2] - demo_pick_xyz[:2]
            place_xy = pickup_xy + original_offset
        else:
            place_xy = mapped[1]

        model_error = camera_model_error_m(camera_model)
        quality = quality_from_model_error(model_error)
        if quality < min_quality:
            failures[camera] = f"camera model quality {quality:.3f} < {min_quality:.3f}"
            continue

        pick_shift = float(np.linalg.norm(pickup_xy - demo_pick_xyz[:2]))
        place_shift = float(np.linalg.norm(place_xy - demo_place_xyz[:2]))
        if pick_shift > max_pick_shift_m:
            failures[camera] = (
                f"pickup shift {pick_shift:.3f} m exceeds --max-pick-shift-m "
                f"{max_pick_shift_m:.3f}"
            )
            continue
        if place_shift > max_place_shift_m:
            failures[camera] = (
                f"place shift {place_shift:.3f} m exceeds --max-place-shift-m "
                f"{max_place_shift_m:.3f}"
            )
            continue

        predictions.append(
            ModelCameraPrediction(
                camera=camera,
                pickup_xy=pickup_xy,
                place_xy=place_xy,
                current_pick_px=current_pick_px,
                current_place_px=current_place_px,
                quality=quality,
                model_error_m=model_error,
                model_sample_count=int(camera_model.get("sample_count", model_info.get("sample_count", 0))),
            )
        )

    if require_all and failures:
        raise RuntimeError("Required camera detection/model failed: " + json.dumps(failures, indent=2, sort_keys=True))
    if not predictions:
        raise RuntimeError("No camera produced a usable model mapping: " + json.dumps(failures, indent=2, sort_keys=True))

    weights = np.asarray([max(pred.quality, 1e-6) for pred in predictions], dtype=float)
    weights = weights / weights.sum()
    pickup_xy = np.sum(np.stack([pred.pickup_xy for pred in predictions], axis=0) * weights[:, None], axis=0)
    place_xy = np.sum(np.stack([pred.place_xy for pred in predictions], axis=0) * weights[:, None], axis=0)

    pickup_xyz = np.asarray([pickup_xy[0], pickup_xy[1], demo_pick_xyz[2]], dtype=float)
    place_xyz = np.asarray([place_xy[0], place_xy[1], demo_place_xyz[2]], dtype=float)
    summary["failures"] = failures
    summary["predictions"] = [pred.to_json() for pred in predictions]
    summary["weighted_pickup_xyz"] = pickup_xyz.astype(float).tolist()
    summary["weighted_place_xyz"] = place_xyz.astype(float).tolist()
    return pickup_xyz, place_xyz, predictions, summary


def predict_targets(
    cameras: Sequence[str],
    demo_images: Mapping[str, np.ndarray],
    current_images: Mapping[str, np.ndarray],
    demo_pick_xyz: np.ndarray,
    demo_place_xyz: np.ndarray,
    min_quality: float,
    require_all: bool,
    no_detect_place: bool,
    max_pick_shift_m: float,
    max_place_shift_m: float,
    debug_dir: Optional[Path],
) -> Tuple[np.ndarray, np.ndarray, List[CameraPrediction], Dict[str, Any]]:
    summary: Dict[str, Any] = {"cameras": {}}
    predictions: List[CameraPrediction] = []
    failures: Dict[str, str] = {}

    for camera in cameras:
        demo_dets = detect_blocks(camera, demo_images[camera])
        current_dets = detect_blocks(camera, current_images[camera])
        demo_pair = assign_pick_place(camera, demo_dets)
        current_pair = assign_pick_place(camera, current_dets)

        summary["cameras"][camera] = {
            "demo_detections": [det.to_json() for det in demo_dets[:8]],
            "current_detections": [det.to_json() for det in current_dets[:8]],
        }
        if debug_dir is not None:
            write_overlay(debug_dir / f"demo_{camera}.jpg", demo_images[camera], demo_dets, demo_pair)
            write_overlay(debug_dir / f"current_{camera}.jpg", current_images[camera], current_dets, current_pair)

        if demo_pair is None:
            failures[camera] = "demo image did not produce two block detections"
            continue
        if current_pair is None:
            failures[camera] = "current image did not produce two block detections"
            continue

        demo_pick, demo_place = demo_pair
        current_pick, current_place = current_pair
        demo_pick_px = np.asarray(demo_pick.center, dtype=float)
        demo_place_px = np.asarray(demo_place.center, dtype=float)
        current_pick_px = np.asarray(current_pick.center, dtype=float)
        current_place_px = np.asarray(current_place.center, dtype=float)
        quality = mapping_quality(demo_pick_px, demo_place_px, current_pick_px, current_place_px)
        if quality < min_quality:
            failures[camera] = f"camera mapping quality {quality:.3f} < {min_quality:.3f}"
            continue

        affine = estimate_similarity_from_two_points(
            demo_pick_px,
            demo_place_px,
            demo_pick_xyz[:2],
            demo_place_xyz[:2],
        )
        pickup_xy = apply_affine_2d(affine, current_pick_px)
        if no_detect_place:
            original_offset = demo_place_xyz[:2] - demo_pick_xyz[:2]
            place_xy = pickup_xy + original_offset
        else:
            place_xy = apply_affine_2d(affine, current_place_px)
        pick_shift = float(np.linalg.norm(pickup_xy - demo_pick_xyz[:2]))
        place_shift = float(np.linalg.norm(place_xy - demo_place_xyz[:2]))
        if pick_shift > max_pick_shift_m:
            failures[camera] = (
                f"pickup shift {pick_shift:.3f} m exceeds --max-pick-shift-m "
                f"{max_pick_shift_m:.3f}"
            )
            continue
        if place_shift > max_place_shift_m:
            failures[camera] = (
                f"place shift {place_shift:.3f} m exceeds --max-place-shift-m "
                f"{max_place_shift_m:.3f}"
            )
            continue
        predictions.append(
            CameraPrediction(
                camera=camera,
                pickup_xy=pickup_xy,
                place_xy=place_xy,
                demo_pick_px=demo_pick_px,
                demo_place_px=demo_place_px,
                current_pick_px=current_pick_px,
                current_place_px=current_place_px,
                quality=quality,
            )
        )

    if require_all and failures:
        raise RuntimeError("Required camera detection failed: " + json.dumps(failures, indent=2, sort_keys=True))
    if not predictions:
        raise RuntimeError("No camera produced a usable pick/place mapping: " + json.dumps(failures, indent=2, sort_keys=True))

    weights = np.asarray([max(pred.quality, 1e-6) for pred in predictions], dtype=float)
    weights = weights / weights.sum()
    pickup_xy = np.sum(np.stack([pred.pickup_xy for pred in predictions], axis=0) * weights[:, None], axis=0)
    place_xy = np.sum(np.stack([pred.place_xy for pred in predictions], axis=0) * weights[:, None], axis=0)

    pickup_xyz = np.asarray([pickup_xy[0], pickup_xy[1], demo_pick_xyz[2]], dtype=float)
    place_xyz = np.asarray([place_xy[0], place_xy[1], demo_place_xyz[2]], dtype=float)
    summary["failures"] = failures
    summary["predictions"] = [pred.to_json() for pred in predictions]
    summary["weighted_pickup_xyz"] = pickup_xyz.astype(float).tolist()
    summary["weighted_place_xyz"] = place_xyz.astype(float).tolist()
    return pickup_xyz, place_xyz, predictions, summary


def aggregate_demo_targets(
    demo_results: Sequence[Dict[str, Any]],
    template_pick_z: float,
    template_place_z: float,
    max_spread_m: float,
    min_valid_demos: int,
) -> Tuple[np.ndarray, np.ndarray, List[Dict[str, Any]], Dict[str, Any]]:
    if not demo_results:
        raise RuntimeError("No successful demo mappings were produced")

    pick_xy = np.asarray([result["target_pick_xyz"][:2] for result in demo_results], dtype=float)
    place_xy = np.asarray([result["target_place_xyz"][:2] for result in demo_results], dtype=float)
    qualities = np.asarray([max(float(result.get("quality", 0.0)), 1e-6) for result in demo_results], dtype=float)

    pick_median = np.median(pick_xy, axis=0)
    place_median = np.median(place_xy, axis=0)
    pick_dist = np.linalg.norm(pick_xy - pick_median[None, :], axis=1)
    place_dist = np.linalg.norm(place_xy - place_median[None, :], axis=1)
    combined_dist = np.maximum(pick_dist, place_dist)
    inlier_mask = combined_dist <= float(max_spread_m)
    if len(demo_results) <= 2:
        inlier_mask[:] = True

    inlier_count = int(np.count_nonzero(inlier_mask))
    required = min_valid_demos if len(demo_results) > 1 else 1
    if inlier_count < required:
        ranked = sorted(
            [
                {
                    "demo_episode": result["demo_episode"],
                    "combined_dist_m": float(dist),
                    "target_pick_xyz": result["target_pick_xyz"],
                    "target_place_xyz": result["target_place_xyz"],
                    "quality": float(result.get("quality", 0.0)),
                }
                for result, dist in zip(demo_results, combined_dist)
            ],
            key=lambda item: item["combined_dist_m"],
        )
        raise RuntimeError(
            "Multi-demo mapping did not have enough consistent inliers. "
            f"inliers={inlier_count}, required={required}, max_demo_spread_m={max_spread_m}. "
            + json.dumps(ranked, indent=2, sort_keys=True)
        )

    weights = qualities[inlier_mask]
    weights = weights / weights.sum()
    aggregate_pick_xy = np.sum(pick_xy[inlier_mask] * weights[:, None], axis=0)
    aggregate_place_xy = np.sum(place_xy[inlier_mask] * weights[:, None], axis=0)

    inlier_results: List[Dict[str, Any]] = []
    for result, is_inlier, distance in zip(demo_results, inlier_mask, combined_dist):
        annotated = dict(result)
        annotated["aggregate_inlier"] = bool(is_inlier)
        annotated["aggregate_distance_m"] = float(distance)
        if is_inlier:
            inlier_results.append(annotated)

    pickup_xyz = np.asarray([aggregate_pick_xy[0], aggregate_pick_xy[1], float(template_pick_z)], dtype=float)
    place_xyz = np.asarray([aggregate_place_xy[0], aggregate_place_xy[1], float(template_place_z)], dtype=float)
    aggregate_summary = {
        "method": "multi_demo_weighted_inlier_mean",
        "candidate_demo_count": int(len(demo_results)),
        "inlier_demo_count": inlier_count,
        "min_valid_demos": int(required),
        "max_demo_spread_m": float(max_spread_m),
        "pickup_xy_median": pick_median.astype(float).tolist(),
        "place_xy_median": place_median.astype(float).tolist(),
        "pickup_xyz": pickup_xyz.astype(float).tolist(),
        "place_xyz": place_xyz.astype(float).tolist(),
        "demo_results": inlier_results,
    }
    return pickup_xyz, place_xyz, inlier_results, aggregate_summary


def write_overlay(
    path: Path,
    image_rgb: np.ndarray,
    detections: Sequence[BlockDetection],
    pair: Optional[Tuple[BlockDetection, BlockDetection]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    role_by_center = {}
    if pair is not None:
        role_by_center[tuple(pair[0].center)] = "pick"
        role_by_center[tuple(pair[1].center)] = "place"

    for idx, det in enumerate(detections[:8]):
        color = (0, 0, 255)
        role = role_by_center.get(tuple(det.center))
        if role == "pick":
            color = (0, 255, 0)
        elif role == "place":
            color = (255, 0, 0)
        rect = (det.center, det.size, det.angle_deg)
        points = cv2.boxPoints(rect).astype(int)
        cv2.drawContours(bgr, [points], -1, color, 2)
        label = role or str(idx)
        cv2.putText(
            bgr,
            label,
            (int(det.center[0]) + 4, int(det.center[1]) - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    cv2.imwrite(str(path), bgr)


def retarget_payload(
    payload: Mapping[str, Any],
    frames: Sequence[MutableMapping[str, Any]],
    anchors: AnchorFrames,
    target_pick_xyz: np.ndarray,
    target_place_xyz: np.ndarray,
    demo_pick_xyz: np.ndarray,
    demo_place_xyz: np.ndarray,
    mapping_metadata: Mapping[str, Any],
    keep_images: bool,
) -> Dict[str, Any]:
    output = {
        key: copy.deepcopy(value)
        for key, value in dict(payload).items()
        if key not in {"data", "frames"}
    }
    new_frames: List[MutableMapping[str, Any]] = [
        clone_frame_for_replay(frame, keep_images=keep_images) for frame in frames
    ]
    pickup_delta = np.asarray(target_pick_xyz - demo_pick_xyz, dtype=float)
    place_delta = np.asarray(target_place_xyz - demo_place_xyz, dtype=float)

    pickup_frame = anchors.pickup
    place_frame = anchors.place
    for idx, frame in enumerate(new_frames):
        if "pose" not in frame:
            continue
        pose = np.asarray(frame["pose"], dtype=float).copy()
        if pose.shape != (6,):
            raise ValueError(f"Frame {idx} pose must have shape (6,), got {pose.shape}")
        if idx <= pickup_frame:
            delta = pickup_delta
        elif idx >= place_frame:
            delta = place_delta
        else:
            alpha = (idx - pickup_frame) / float(place_frame - pickup_frame)
            delta = (1.0 - alpha) * pickup_delta + alpha * place_delta
        pose[:3] = pose[:3] + delta
        frame["pose"] = pose.astype(float).tolist()
        frame["retarget_delta_xyz"] = delta.astype(float).tolist()

    output["data"] = new_frames
    output["dagger_retarget"] = {
        "schema_version": "jenga_retarget_v1",
        "created_at_unix": time.time(),
        "anchor_frames": anchors.to_json(),
        "demo_pickup_xyz": demo_pick_xyz.astype(float).tolist(),
        "demo_place_xyz": demo_place_xyz.astype(float).tolist(),
        "target_pickup_xyz": target_pick_xyz.astype(float).tolist(),
        "target_place_xyz": target_place_xyz.astype(float).tolist(),
        "pickup_delta_xyz": pickup_delta.astype(float).tolist(),
        "place_delta_xyz": place_delta.astype(float).tolist(),
        "output_images_kept": bool(keep_images),
        **dict(mapping_metadata),
    }
    return output


def clone_frame_for_replay(frame: Mapping[str, Any], *, keep_images: bool) -> MutableMapping[str, Any]:
    cloned: MutableMapping[str, Any] = {}
    for key, value in frame.items():
        if not keep_images and (key.endswith("_image") or key.endswith("_depth")):
            continue
        cloned[key] = copy.deepcopy(value)
    return cloned


def write_episode(path_like: str | Path, payload: Mapping[str, Any]) -> Path:
    path = Path(path_like).expanduser()
    if path.suffix != ".gz" or not path.name.endswith(".pkl.gz"):
        raise ValueError(f"Output must end with .pkl.gz: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wb") as f:
        pickle.dump(dict(payload), f, protocol=pickle.HIGHEST_PROTOCOL)
    return path


def write_summary(path: Path, summary: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    cameras = camera_list(args.cameras)
    image_cameras = list(cameras)
    if not args.no_dynamic_place_z and args.stack_height_camera not in image_cameras:
        image_cameras.append(args.stack_height_camera)
    debug_dir = Path(args.debug_dir).expanduser() if args.debug_dir else None
    if debug_dir is not None:
        debug_dir.mkdir(parents=True, exist_ok=True)

    demo_file, demo_payload, demo_frames, demo_metadata = load_episode(args.demo)
    anchors = infer_anchor_frames(demo_frames, args.pickup_frame, args.place_frame)
    demo_pick_xyz = frame_pose_xyz(demo_frames[anchors.pickup])
    demo_place_xyz = frame_pose_xyz(demo_frames[anchors.place])
    demo_files = discover_demo_files(demo_file, args.demo_root, args.max_demos)
    demo_stack_images: Dict[str, np.ndarray] = {}
    if not args.no_dynamic_place_z:
        try:
            demo_stack_images = load_episode_images(demo_frames, 0, [args.stack_height_camera])
        except Exception as exc:
            print(
                f"warning: could not load demo stack-height image for {args.stack_height_camera}: "
                f"{type(exc).__name__}: {exc}",
                file=sys.stderr,
            )

    if args.current_episode:
        current_file = resolve_episode_file(args.current_episode)
        if current_file == demo_file:
            current_frames = demo_frames
        else:
            current_file, _, current_frames, _ = load_episode(current_file)
        current_images = load_episode_images(current_frames, args.current_frame, image_cameras)
        current_source = str(current_file)
    else:
        current_images = capture_live_images(image_cameras, warmup_sec=args.camera_warmup_sec)
        current_source = "live_realsense"

    mapping_model_path = Path(args.mapping_model).expanduser() if args.mapping_model else None
    if not args.no_mapping_model and mapping_model_path is not None and mapping_model_path.exists():
        mapping_model = load_mapping_model(mapping_model_path)
        target_pick_xyz, target_place_xyz, model_predictions, mapping_summary = predict_targets_from_mapping_model(
            cameras=cameras,
            current_images=current_images,
            mapping_model=mapping_model,
            mapping_model_path=mapping_model_path,
            demo_pick_xyz=demo_pick_xyz,
            demo_place_xyz=demo_place_xyz,
            min_quality=args.min_camera_quality,
            require_all=args.require_all_cameras,
            no_detect_place=args.no_detect_place,
            max_pick_shift_m=args.max_pick_shift_m,
            max_place_shift_m=args.max_place_shift_m,
            debug_dir=debug_dir,
        )
        if args.no_dynamic_place_z:
            dynamic_z_summary: Dict[str, Any] = {
                "enabled": False,
                "reason": "--no-dynamic-place-z",
                "original_target_place_xyz": target_place_xyz.astype(float).tolist(),
                "adjusted_target_place_xyz": target_place_xyz.astype(float).tolist(),
                "place_z_delta_m": 0.0,
            }
        else:
            target_place_xyz, dynamic_z_summary = dynamic_place_z_adjustment(
                current_images=current_images,
                demo_images=demo_stack_images,
                target_pick_xyz=target_pick_xyz,
                target_place_xyz=target_place_xyz,
                demo_pick_xyz=demo_pick_xyz,
                demo_place_xyz=demo_place_xyz,
                stack_height_camera=args.stack_height_camera,
                block_height_m=args.block_height_m,
                stack_count_override=args.stack_count_override,
                max_stack_count=args.max_stack_count,
                stack_layer_pitch_px=args.stack_layer_pitch_px,
                stack_layer_pitch_ratio=args.stack_layer_pitch_ratio,
                min_stack_height_quality=args.min_stack_height_quality,
                debug_dir=debug_dir,
            )
        target_place_xyz, place_offset_summary = apply_place_offset(target_place_xyz, args.place_offset_m)
        mapping_summary.update(
            {
                "demo_episode": str(demo_file),
                "demo_root": str(Path(args.demo_root).expanduser()),
                "demo_episodes_requested": [str(path) for path in demo_files],
                "current_source": current_source,
                "cameras_requested": cameras,
                "primary_camera": args.primary_camera,
                "anchor_frames": anchors.to_json(),
                "demo_pickup_xyz": demo_pick_xyz.astype(float).tolist(),
                "demo_place_xyz": demo_place_xyz.astype(float).tolist(),
                "target_pickup_xyz": target_pick_xyz.astype(float).tolist(),
                "target_place_xyz": target_place_xyz.astype(float).tolist(),
                "dynamic_place_z": dynamic_z_summary,
                "place_offset": place_offset_summary,
                "demo_metadata_schema": demo_metadata.get("schema_version"),
            }
        )

        if debug_dir is not None:
            write_summary(debug_dir / "mapping_summary.json", mapping_summary)

        print("Jenga retarget summary")
        print(f"  template demo: {demo_file}")
        print(f"  mapping model: {mapping_model_path}")
        print(f"  model demos: {len(mapping_model.get('demo_files', []))}")
        print(f"  current: {current_source}")
        print(f"  anchors: pickup={anchors.pickup}, place={anchors.place}, closed_range={anchors.closed_range}")
        print(f"  demo pickup xyz: {np.array2string(demo_pick_xyz, precision=4)}")
        print(f"  target pickup xyz: {np.array2string(target_pick_xyz, precision=4)}")
        print(f"  demo place xyz: {np.array2string(demo_place_xyz, precision=4)}")
        print(f"  target place xyz: {np.array2string(target_place_xyz, precision=4)}")
        if dynamic_z_summary.get("enabled"):
            print(
                "  dynamic place z: "
                f"stack_count={dynamic_z_summary.get('stack_count')} "
                f"block_height={float(dynamic_z_summary.get('block_height_m', 0.0)):.4f}m "
                f"delta_z={float(dynamic_z_summary.get('place_z_delta_m', 0.0)):.4f}m"
            )
        if place_offset_summary.get("applied"):
            print(f"  place offset m: {np.array2string(np.asarray(place_offset_summary['offset_m']), precision=4)}")
        print("  model camera predictions:")
        for pred in model_predictions:
            error_text = "unknown" if pred.model_error_m is None else f"{pred.model_error_m:.4f}m"
            print(
                f"    {pred.camera}: pickup_xy={np.array2string(pred.pickup_xy, precision=4)}, "
                f"place_xy={np.array2string(pred.place_xy, precision=4)}, "
                f"quality={pred.quality:.3f}, model_error={error_text}, "
                f"samples={pred.model_sample_count}"
            )

        if args.dry_run:
            print("Dry-run only; no output episode written.")
            return 0

        output_payload = retarget_payload(
            demo_payload,
            demo_frames,
            anchors,
            target_pick_xyz,
            target_place_xyz,
            demo_pick_xyz,
            demo_place_xyz,
            mapping_summary,
            keep_images=args.keep_images,
        )

        output_path = write_episode(args.output, output_payload)
        print(f"  wrote: {output_path}")
        if debug_dir is not None:
            print(f"  debug: {debug_dir}")
        if not args.skip_robot_check_command:
            print("Inspect file without robot:")
            print(f"  bash 7_replay_fr3.sh {output_path} --skip-robot-check")
            print("Cartesian dry-run command:")
            print(
                "  bash DAgger/replay_jenga_cartesian.sh "
                f"{output_path} --host 127.0.0.1 --port 6002 "
                "--gripper-host 127.0.0.1 --gripper-port 50054"
            )
            print("Do not execute this retargeted file with 7_replay_fr3.sh; script 7 sends joints.")
        return 0

    if not args.no_mapping_model and args.require_mapping_model:
        raise FileNotFoundError(
            f"Mapping model not found: {mapping_model_path}. Build it first with: "
            "bash DAgger/build_jenga_mapping_model.sh --max-demos 100 --validation-fraction 0.2"
        )
    if not args.no_mapping_model and mapping_model_path is not None:
        print(
            f"warning: mapping model not found at {mapping_model_path}; "
            "falling back to legacy multi-demo two-point mapping",
            file=sys.stderr,
        )

    demo_results: List[Dict[str, Any]] = []
    demo_failures: Dict[str, str] = {}
    all_predictions: List[CameraPrediction] = []
    first_summary: Dict[str, Any] = {}
    for demo_index, candidate_demo in enumerate(demo_files):
        try:
            if candidate_demo == demo_file:
                candidate_payload = demo_payload
                candidate_frames = demo_frames
                candidate_metadata = demo_metadata
            else:
                _, candidate_payload, candidate_frames, candidate_metadata = load_episode(candidate_demo)
            candidate_anchors = infer_anchor_frames(
                candidate_frames,
                args.pickup_frame,
                args.place_frame,
            )
            candidate_images = load_episode_images(candidate_frames, 0, cameras)
            candidate_pick_xyz = frame_pose_xyz(candidate_frames[candidate_anchors.pickup])
            candidate_place_xyz = frame_pose_xyz(candidate_frames[candidate_anchors.place])
            candidate_debug_dir = debug_dir if demo_index == 0 else None
            candidate_pick_target, candidate_place_target, candidate_predictions, candidate_summary = predict_targets(
                cameras=cameras,
                demo_images=candidate_images,
                current_images=current_images,
                demo_pick_xyz=candidate_pick_xyz,
                demo_place_xyz=candidate_place_xyz,
                min_quality=args.min_camera_quality,
                require_all=args.require_all_cameras,
                no_detect_place=args.no_detect_place,
                max_pick_shift_m=args.max_pick_shift_m,
                max_place_shift_m=args.max_place_shift_m,
                debug_dir=candidate_debug_dir,
            )
            if demo_index == 0:
                first_summary = candidate_summary
            prediction_quality = float(np.mean([prediction.quality for prediction in candidate_predictions]))
            demo_results.append(
                {
                    "demo_episode": str(candidate_demo),
                    "schema_version": candidate_metadata.get("schema_version"),
                    "anchor_frames": candidate_anchors.to_json(),
                    "demo_pickup_xyz": candidate_pick_xyz.astype(float).tolist(),
                    "demo_place_xyz": candidate_place_xyz.astype(float).tolist(),
                    "target_pick_xyz": candidate_pick_target.astype(float).tolist(),
                    "target_place_xyz": candidate_place_target.astype(float).tolist(),
                    "quality": prediction_quality,
                    "predictions": [prediction.to_json() for prediction in candidate_predictions],
                }
            )
            all_predictions.extend(candidate_predictions)
        except Exception as exc:
            demo_failures[str(candidate_demo)] = f"{type(exc).__name__}: {exc}"

    if args.max_demos > 1 and len(demo_results) < args.min_valid_demos:
        raise RuntimeError(
            "Not enough successful demo mappings. "
            f"valid={len(demo_results)}, required={args.min_valid_demos}. "
            + json.dumps(demo_failures, indent=2, sort_keys=True)
        )

    target_pick_xyz, target_place_xyz, inlier_demo_results, aggregate_summary = aggregate_demo_targets(
        demo_results,
        template_pick_z=float(demo_pick_xyz[2]),
        template_place_z=float(demo_place_xyz[2]),
        max_spread_m=args.max_demo_spread_m,
        min_valid_demos=args.min_valid_demos if args.max_demos > 1 else 1,
    )
    if args.no_dynamic_place_z:
        dynamic_z_summary = {
            "enabled": False,
            "reason": "--no-dynamic-place-z",
            "original_target_place_xyz": target_place_xyz.astype(float).tolist(),
            "adjusted_target_place_xyz": target_place_xyz.astype(float).tolist(),
            "place_z_delta_m": 0.0,
        }
    else:
        target_place_xyz, dynamic_z_summary = dynamic_place_z_adjustment(
            current_images=current_images,
            demo_images=demo_stack_images,
            target_pick_xyz=target_pick_xyz,
            target_place_xyz=target_place_xyz,
            demo_pick_xyz=demo_pick_xyz,
            demo_place_xyz=demo_place_xyz,
            stack_height_camera=args.stack_height_camera,
            block_height_m=args.block_height_m,
            stack_count_override=args.stack_count_override,
            max_stack_count=args.max_stack_count,
            stack_layer_pitch_px=args.stack_layer_pitch_px,
            stack_layer_pitch_ratio=args.stack_layer_pitch_ratio,
            min_stack_height_quality=args.min_stack_height_quality,
            debug_dir=debug_dir,
        )
    target_place_xyz, place_offset_summary = apply_place_offset(target_place_xyz, args.place_offset_m)

    mapping_summary = dict(first_summary)
    mapping_summary.update(
        {
            "demo_episode": str(demo_file),
            "demo_root": str(Path(args.demo_root).expanduser()),
            "demo_episodes_requested": [str(path) for path in demo_files],
            "demo_failures": demo_failures,
            "multi_demo_aggregate": aggregate_summary,
            "current_source": current_source,
            "cameras_requested": cameras,
            "primary_camera": args.primary_camera,
            "anchor_frames": anchors.to_json(),
            "demo_pickup_xyz": demo_pick_xyz.astype(float).tolist(),
            "demo_place_xyz": demo_place_xyz.astype(float).tolist(),
            "target_pickup_xyz": target_pick_xyz.astype(float).tolist(),
            "target_place_xyz": target_place_xyz.astype(float).tolist(),
            "dynamic_place_z": dynamic_z_summary,
            "place_offset": place_offset_summary,
            "demo_metadata_schema": demo_metadata.get("schema_version"),
        }
    )

    if debug_dir is not None:
        write_summary(debug_dir / "mapping_summary.json", mapping_summary)

    print("Jenga retarget summary")
    print(f"  template demo: {demo_file}")
    print(f"  mapping demos: {len(inlier_demo_results)}/{len(demo_files)} inliers")
    print(f"  current: {current_source}")
    print(f"  anchors: pickup={anchors.pickup}, place={anchors.place}, closed_range={anchors.closed_range}")
    print(f"  demo pickup xyz: {np.array2string(demo_pick_xyz, precision=4)}")
    print(f"  target pickup xyz: {np.array2string(target_pick_xyz, precision=4)}")
    print(f"  demo place xyz: {np.array2string(demo_place_xyz, precision=4)}")
    print(f"  target place xyz: {np.array2string(target_place_xyz, precision=4)}")
    if dynamic_z_summary.get("enabled"):
        print(
            "  dynamic place z: "
            f"stack_count={dynamic_z_summary.get('stack_count')} "
            f"block_height={float(dynamic_z_summary.get('block_height_m', 0.0)):.4f}m "
            f"delta_z={float(dynamic_z_summary.get('place_z_delta_m', 0.0)):.4f}m"
        )
    if place_offset_summary.get("applied"):
        print(f"  place offset m: {np.array2string(np.asarray(place_offset_summary['offset_m']), precision=4)}")
    print("  camera predictions:")
    for result in inlier_demo_results:
        print(f"    demo={Path(result['demo_episode']).parent.name} quality={result['quality']:.3f}")
        for pred in result.get("predictions", []):
            print(
                f"      {pred['camera']}: pickup_xy={np.array2string(np.asarray(pred['pickup_xy']), precision=4)}, "
                f"place_xy={np.array2string(np.asarray(pred['place_xy']), precision=4)}, "
                f"quality={float(pred['quality']):.3f}"
            )

    if args.dry_run:
        print("Dry-run only; no output episode written.")
        return 0

    output_payload = retarget_payload(
        demo_payload,
        demo_frames,
        anchors,
        target_pick_xyz,
        target_place_xyz,
        demo_pick_xyz,
        demo_place_xyz,
        mapping_summary,
        keep_images=args.keep_images,
    )

    output_path = write_episode(args.output, output_payload)
    print(f"  wrote: {output_path}")
    if debug_dir is not None:
        print(f"  debug: {debug_dir}")
    if not args.skip_robot_check_command:
        print("Inspect file without robot:")
        print(f"  bash 7_replay_fr3.sh {output_path} --skip-robot-check")
        print("Cartesian dry-run command:")
        print(
            "  bash DAgger/replay_jenga_cartesian.sh "
            f"{output_path} --host 127.0.0.1 --port 6002 "
            "--gripper-host 127.0.0.1 --gripper-port 50054"
        )
        print("Do not execute this retargeted file with 7_replay_fr3.sh; script 7 sends joints.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
