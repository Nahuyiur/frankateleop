from __future__ import annotations

import argparse
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np
from PIL import Image

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 180


@dataclass
class RingDetection:
    center: tuple[float, float]
    bbox: tuple[int, int, int, int]
    area: float
    circularity: float
    aspect: float
    score: float
    role: str = ""

    def to_json(self) -> dict[str, Any]:
        return {
            "center_xy": [float(value) for value in self.center],
            "bbox_xywh": [int(value) for value in self.bbox],
            "area_px": float(self.area),
            "circularity": float(self.circularity),
            "aspect": float(self.aspect),
            "score": float(self.score),
            "role": self.role,
        }


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def robust_stats(values: Iterable[float]) -> dict[str, Any]:
    array = np.asarray(list(values), dtype=np.float64)
    if not len(array):
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "p90": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": int(len(array)),
        "mean": float(array.mean()),
        "median": float(np.median(array)),
        "p90": float(np.percentile(array, 90)),
        "p95": float(np.percentile(array, 95)),
        "max": float(array.max()),
    }


def detect_rings(image_rgb: np.ndarray) -> list[RingDetection]:
    image = np.asarray(image_rgb, dtype=np.uint8)
    if image.shape != (IMAGE_HEIGHT, IMAGE_WIDTH, 3):
        raise ValueError(f"Expected 320x180 RGB, got {image.shape}")
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    hue, saturation, value = cv2.split(hsv)
    yy, xx = np.indices((IMAGE_HEIGHT, IMAGE_WIDTH))
    workspace = (
        (xx >= 120)
        & (xx <= 305)
        & (yy >= 80)
        & (yy <= 174)
    )
    yellow = (
        workspace
        & (hue >= 16)
        & (hue <= 45)
        & (saturation >= 80)
        & (value >= 70)
    ).astype(np.uint8) * 255
    yellow = cv2.morphologyEx(
        yellow, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)
    )
    contours, _ = cv2.findContours(
        yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    detections: list[RingDetection] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not 35.0 <= area <= 650.0:
            continue
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 1e-6:
            continue
        circularity = float(4.0 * math.pi * area / (perimeter * perimeter))
        x, y, width, height = cv2.boundingRect(contour)
        if not (9 <= width <= 42 and 7 <= height <= 35):
            continue
        aspect = float(width / max(height, 1))
        if not 0.55 <= aspect <= 2.0:
            continue
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-6:
            continue
        center = (
            float(moments["m10"] / moments["m00"]),
            float(moments["m01"] / moments["m00"]),
        )
        circularity_score = min(circularity / 0.72, 1.0)
        area_score = min(area / 180.0, 180.0 / max(area, 1.0))
        aspect_score = min(aspect / 1.15, 1.15 / max(aspect, 1e-6))
        score = float(
            max(circularity_score, 0.0)
            * max(area_score, 0.0)
            * max(aspect_score, 0.0)
        )
        detections.append(
            RingDetection(
                center=center,
                bbox=(x, y, width, height),
                area=area,
                circularity=circularity,
                aspect=aspect,
                score=score,
            )
        )
    return sorted(detections, key=lambda item: item.score, reverse=True)


def infer_ring_task_anchors(
    states: np.ndarray,
    minimum_closed_frames: int = 5,
) -> dict[str, Any]:
    """Find the grasp segment that actually transports the ring.

    Ring policy episodes can contain a short failed close/open attempt before the
    successful grasp. The generic first-close/first-open heuristic therefore
    chooses the closed segment with the largest planar EEF displacement.
    """
    gripper = np.asarray(states[:, 6], dtype=np.float64)
    lower = float(np.min(gripper))
    upper = float(np.max(gripper))
    if not np.isfinite([lower, upper]).all() or upper - lower < 1e-6:
        raise ValueError("Gripper signal is constant or non-finite")
    threshold = lower + 0.45 * (upper - lower)
    closed = gripper >= threshold
    segments: list[dict[str, Any]] = []
    start: int | None = None
    for index, is_closed in enumerate(closed):
        if is_closed and start is None:
            start = index
        if start is not None and (
            not is_closed or (is_closed and index == len(closed) - 1)
        ):
            end = index if is_closed and index == len(closed) - 1 else index - 1
            duration = end - start + 1
            planar_displacement = float(
                np.linalg.norm(states[end, :2] - states[start, :2])
            )
            segments.append(
                {
                    "start": int(start),
                    "end": int(end),
                    "duration_frames": int(duration),
                    "planar_displacement_m": planar_displacement,
                }
            )
            start = None
    eligible = [
        segment
        for segment in segments
        if segment["duration_frames"] >= minimum_closed_frames
    ]
    if not eligible:
        raise ValueError("No sustained gripper-closed segment found")
    selected = max(
        eligible,
        key=lambda segment: (
            segment["planar_displacement_m"],
            segment["duration_frames"],
        ),
    )
    return {
        "pickup": int(selected["start"]),
        "place": int(min(selected["end"] + 1, len(states) - 1)),
        "threshold": float(threshold),
        "selection_rule": "max_planar_displacement_among_sustained_closed_segments",
        "segments": segments,
        "selected_segment": selected,
    }


def assign_pick_place(
    detections: list[RingDetection],
) -> tuple[RingDetection, RingDetection] | None:
    candidates = detections[:6]
    best: tuple[RingDetection, RingDetection, float] | None = None
    for index, first in enumerate(candidates):
        for second in candidates[index + 1 :]:
            separation = abs(float(first.center[0] - second.center[0]))
            if separation < 35.0:
                continue
            left, right = sorted([first, second], key=lambda item: item.center[0])
            vertical_difference = abs(float(left.center[1] - right.center[1]))
            if vertical_difference > 40.0:
                continue
            score = (
                left.score
                + right.score
                + min(separation / 100.0, 1.0)
                - 0.01 * vertical_difference
            )
            if best is None or score > best[2]:
                best = (left, right, score)
    if best is None:
        return None
    pick, place, _ = best
    pick = RingDetection(**{**pick.__dict__, "role": "pick"})
    place = RingDetection(**{**place.__dict__, "role": "place"})
    return pick, place


def draw_detection_overlay(
    image_rgb: np.ndarray,
    detections: list[RingDetection],
    pair: tuple[RingDetection, RingDetection] | None,
    title: str,
) -> np.ndarray:
    canvas = np.asarray(image_rgb, dtype=np.uint8).copy()
    role_by_center = {}
    if pair is not None:
        role_by_center[pair[0].center] = "pick"
        role_by_center[pair[1].center] = "place"
    for index, detection in enumerate(detections[:6]):
        role = role_by_center.get(detection.center, "")
        color = (70, 70, 255)
        if role == "pick":
            color = (40, 230, 80)
        elif role == "place":
            color = (40, 150, 255)
        x, y, width, height = detection.bbox
        cv2.rectangle(
            canvas, (x, y), (x + width, y + height), color, 2, cv2.LINE_AA
        )
        center = tuple(np.round(detection.center).astype(int))
        cv2.circle(canvas, center, 3, color, -1, cv2.LINE_AA)
        label = role or f"candidate{index}"
        cv2.putText(
            canvas,
            label,
            (x, max(12, y - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            color,
            1,
            cv2.LINE_AA,
        )
    cv2.rectangle(canvas, (0, 0), (IMAGE_WIDTH - 1, 20), (0, 0, 0), -1)
    cv2.putText(
        canvas,
        title,
        (5, 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return canvas


def fit_affine(pixel_xy: np.ndarray, robot_xy: np.ndarray) -> dict[str, Any]:
    matrix, inliers = cv2.estimateAffine2D(
        pixel_xy.astype(np.float64),
        robot_xy.astype(np.float64),
        method=cv2.RANSAC,
        ransacReprojThreshold=0.025,
        maxIters=5000,
        confidence=0.995,
        refineIters=20,
    )
    if matrix is None:
        raise RuntimeError("cv2.estimateAffine2D failed")
    return {
        "type": "affine",
        "matrix": matrix.astype(float).tolist(),
        "inlier_count": int(np.count_nonzero(inliers)),
        "sample_count": int(len(pixel_xy)),
    }


def fit_homography(pixel_xy: np.ndarray, robot_xy: np.ndarray) -> dict[str, Any]:
    matrix, inliers = cv2.findHomography(
        pixel_xy.astype(np.float64),
        robot_xy.astype(np.float64),
        method=cv2.RANSAC,
        ransacReprojThreshold=0.025,
        maxIters=5000,
        confidence=0.995,
    )
    if matrix is None:
        raise RuntimeError("cv2.findHomography failed")
    return {
        "type": "homography",
        "matrix": matrix.astype(float).tolist(),
        "inlier_count": int(np.count_nonzero(inliers)),
        "sample_count": int(len(pixel_xy)),
    }


def apply_mapping(model: dict[str, Any], pixel_xy: np.ndarray) -> np.ndarray:
    points = np.asarray(pixel_xy, dtype=np.float64).reshape(-1, 2)
    if model["type"] == "affine":
        matrix = np.asarray(model["matrix"], dtype=np.float64)
        homogeneous = np.concatenate(
            [points, np.ones((len(points), 1), dtype=np.float64)], axis=1
        )
        return homogeneous @ matrix.T
    if model["type"] == "homography":
        return cv2.perspectiveTransform(
            points.reshape(-1, 1, 2),
            np.asarray(model["matrix"], dtype=np.float64),
        ).reshape(-1, 2)
    raise ValueError(model["type"])


def evaluate_mapping(
    model: dict[str, Any],
    samples: list[dict[str, Any]],
) -> dict[str, Any]:
    predicted = apply_mapping(
        model, np.asarray([sample["pixel_xy"] for sample in samples])
    )
    target = np.asarray([sample["robot_xy"] for sample in samples])
    errors = np.linalg.norm(predicted - target, axis=1)
    by_role = {}
    for role in ("pick", "place"):
        selected = [
            float(error)
            for error, sample in zip(errors, samples)
            if sample["role"] == role
        ]
        by_role[role] = robust_stats(selected)
    rows = []
    for sample, prediction, error in zip(samples, predicted, errors):
        rows.append(
            {
                **sample,
                "predicted_robot_xy": prediction.astype(float).tolist(),
                "error_m": float(error),
            }
        )
    return {
        "error_m": robust_stats(errors.tolist()),
        "by_role": by_role,
        "rows": rows,
    }


def fit_xyz_to_pixel(samples: list[dict[str, Any]]) -> np.ndarray:
    xyz = np.asarray([sample["robot_xyz"] for sample in samples], dtype=np.float64)
    pixels = np.asarray([sample["pixel_xy"] for sample in samples], dtype=np.float64)
    design = np.concatenate([xyz, np.ones((len(xyz), 1))], axis=1)
    matrix, *_ = np.linalg.lstsq(design, pixels, rcond=None)
    return matrix


def project_xyz(states7: np.ndarray, matrix_4x2: np.ndarray) -> np.ndarray:
    xyz = np.asarray(states7[:, :3], dtype=np.float64)
    design = np.concatenate([xyz, np.ones((len(xyz), 1))], axis=1)
    return design @ matrix_4x2


def draw_trajectory_overlay(
    image_rgb: np.ndarray,
    states7: np.ndarray,
    pickup: int,
    place: int,
    matrix_4x2: np.ndarray,
    pair: tuple[RingDetection, RingDetection],
    title: str,
) -> np.ndarray:
    canvas = draw_detection_overlay(image_rgb, list(pair), pair, title)
    pixels = project_xyz(states7, matrix_4x2)
    begin = max(0, pickup - 45)
    end = min(len(pixels) - 1, place + 45)
    segments = [
        (begin, pickup, (30, 210, 255)),
        (pickup, place, (60, 230, 90)),
        (place, end, (220, 160, 50)),
    ]
    for first, last, color in segments:
        points = np.round(pixels[first : last + 1]).astype(np.int32)
        for index in range(1, len(points)):
            if (
                -20 <= points[index - 1, 0] < IMAGE_WIDTH + 20
                and -20 <= points[index - 1, 1] < IMAGE_HEIGHT + 20
                and -20 <= points[index, 0] < IMAGE_WIDTH + 20
                and -20 <= points[index, 1] < IMAGE_HEIGHT + 20
            ):
                cv2.line(
                    canvas,
                    tuple(points[index - 1]),
                    tuple(points[index]),
                    color,
                    2,
                    cv2.LINE_AA,
                )
    for index, label in ((pickup, "PICK"), (place, "PLACE")):
        point = tuple(np.round(pixels[index]).astype(int))
        cv2.circle(canvas, point, 4, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.putText(
            canvas,
            label,
            (point[0] + 4, point[1] - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return canvas


def make_contact_sheet(
    paths: list[Path],
    output: Path,
    *,
    columns: int = 4,
    cell_scale: int = 2,
) -> None:
    images = [np.asarray(Image.open(path).convert("RGB")) for path in paths]
    cell_width = IMAGE_WIDTH * cell_scale
    cell_height = IMAGE_HEIGHT * cell_scale
    rows = int(math.ceil(len(images) / columns))
    canvas = np.zeros((rows * cell_height, columns * cell_width, 3), dtype=np.uint8)
    for index, image in enumerate(images):
        resized = cv2.resize(
            image,
            (cell_width, cell_height),
            interpolation=cv2.INTER_NEAREST,
        )
        row, column = divmod(index, columns)
        canvas[
            row * cell_height : (row + 1) * cell_height,
            column * cell_width : (column + 1) * cell_width,
        ] = resized
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(canvas).save(output)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage1-root", type=Path, required=True)
    parser.add_argument("--stage2-prepared-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=43)
    args = parser.parse_args()

    output = args.output_root.resolve()
    overlays = output / "overlays"
    trajectory_dir = output / "trajectory_overlays"
    overlays.mkdir(parents=True, exist_ok=True)
    trajectory_dir.mkdir(parents=True, exist_ok=True)

    episodes: list[dict[str, Any]] = []
    failures: dict[str, str] = {}
    sample_dirs = sorted((args.stage1_root / "samples").glob("stack_ring__*"))
    for sample_dir in sample_dirs:
        try:
            manifest = json.loads((sample_dir / "manifest.json").read_text())
            image_path = (
                sample_dir / "source/source_initial_exterior_1_320x180.png"
            )
            image = np.asarray(Image.open(image_path).convert("RGB"))
            detections = detect_rings(image)
            pair = assign_pick_place(detections)
            overlay_path = overlays / f"{sample_dir.name}.png"
            Image.fromarray(
                draw_detection_overlay(
                    image,
                    detections,
                    pair,
                    f"{sample_dir.name} | detected={pair is not None}",
                )
            ).save(overlay_path)
            if pair is None:
                raise RuntimeError("Could not assign pick/place ring pair")
            states_path = (
                args.stage2_prepared_root
                / "samples"
                / sample_dir.name
                / "actions/source_absolute_eef_xyzrpy_gripper.npy"
            )
            states = np.load(states_path).astype(np.float64)
            anchors = infer_ring_task_anchors(states)
            policy = str(manifest["source"]["policy"])
            episode = {
                "sample_id": sample_dir.name,
                "episode_index": int(manifest["source"]["episode_index"]),
                "episode_uid": str(manifest["source"]["episode_uid"]),
                "policy": policy,
                "image_path": str(image_path),
                "overlay_path": str(overlay_path),
                "states_path": str(states_path),
                "pickup_frame": int(anchors["pickup"]),
                "place_frame": int(anchors["place"]),
                "anchor_diagnostics": anchors,
                "detections": {
                    "pick": pair[0].to_json(),
                    "place": pair[1].to_json(),
                },
                "states": states,
                "pair": pair,
            }
            episodes.append(episode)
        except Exception as exc:
            failures[sample_dir.name] = f"{type(exc).__name__}: {exc}"

    collection = [episode for episode in episodes if episode["policy"] == "teleop"]
    external = [episode for episode in episodes if episode["policy"] != "teleop"]
    random_generator = random.Random(int(args.seed))
    shuffled = list(collection)
    random_generator.shuffle(shuffled)
    validation_count = max(15, int(round(0.20 * len(shuffled))))
    validation_episodes = shuffled[:validation_count]
    train_episodes = shuffled[validation_count:]

    def samples_from(selected: list[dict[str, Any]]) -> list[dict[str, Any]]:
        result = []
        for episode in selected:
            for role, detection, frame in (
                ("pick", episode["pair"][0], episode["pickup_frame"]),
                ("place", episode["pair"][1], episode["place_frame"]),
            ):
                result.append(
                    {
                        "sample_id": episode["sample_id"],
                        "episode_uid": episode["episode_uid"],
                        "episode_index": episode["episode_index"],
                        "policy": episode["policy"],
                        "role": role,
                        "pixel_xy": [float(value) for value in detection.center],
                        "robot_xy": episode["states"][frame, :2].astype(float).tolist(),
                        "robot_xyz": episode["states"][frame, :3].astype(float).tolist(),
                        "anchor_frame": int(frame),
                    }
                )
        return result

    train_samples = samples_from(train_episodes)
    validation_samples = samples_from(validation_episodes)
    external_samples = samples_from(external)
    pixel_train = np.asarray([sample["pixel_xy"] for sample in train_samples])
    robot_train = np.asarray([sample["robot_xy"] for sample in train_samples])

    candidates = {}
    for name, fit in (("affine", fit_affine), ("homography", fit_homography)):
        model = fit(pixel_train, robot_train)
        candidates[name] = {
            "model": model,
            "train": evaluate_mapping(model, train_samples),
            "validation": evaluate_mapping(model, validation_samples),
            "external_policy_eval": evaluate_mapping(model, external_samples),
        }
    selected_name = min(
        candidates,
        key=lambda name: candidates[name]["validation"]["error_m"]["p90"],
    )
    selected_model = candidates[selected_name]["model"]

    xyz_to_pixel = fit_xyz_to_pixel(train_samples)
    projection_errors = {}
    for split_name, split_samples in (
        ("train", train_samples),
        ("validation", validation_samples),
        ("external_policy_eval", external_samples),
    ):
        xyz = np.asarray([sample["robot_xyz"] for sample in split_samples])
        target = np.asarray([sample["pixel_xy"] for sample in split_samples])
        predicted = (
            np.concatenate([xyz, np.ones((len(xyz), 1))], axis=1)
            @ xyz_to_pixel
        )
        projection_errors[split_name] = robust_stats(
            np.linalg.norm(predicted - target, axis=1).tolist()
        )

    representative = sorted(
        validation_episodes + external,
        key=lambda episode: episode["sample_id"],
    )
    if len(representative) > 12:
        indices = np.linspace(0, len(representative) - 1, 12).round().astype(int)
        representative = [representative[int(index)] for index in indices]
    trajectory_paths = []
    for episode in representative:
        image = np.asarray(Image.open(episode["image_path"]).convert("RGB"))
        path = trajectory_dir / f"{episode['sample_id']}.png"
        overlay = draw_trajectory_overlay(
            image,
            episode["states"],
            episode["pickup_frame"],
            episode["place_frame"],
            xyz_to_pixel,
            episode["pair"],
            f"{episode['sample_id']} | interaction trajectory",
        )
        Image.fromarray(overlay).save(path)
        trajectory_paths.append(path)

    all_overlay_paths = [Path(episode["overlay_path"]) for episode in episodes]
    if len(all_overlay_paths) > 24:
        indices = np.linspace(0, len(all_overlay_paths) - 1, 24).round().astype(int)
        selected_overlay_paths = [all_overlay_paths[int(index)] for index in indices]
    else:
        selected_overlay_paths = all_overlay_paths
    detection_sheet = output / "ring_detection_contact_sheet.png"
    trajectory_sheet = output / "ring_trajectory_contact_sheet.png"
    make_contact_sheet(selected_overlay_paths, detection_sheet)
    make_contact_sheet(trajectory_paths, trajectory_sheet, columns=3)

    serializable_candidates = {}
    for name, candidate in candidates.items():
        serializable_candidates[name] = {
            "model": candidate["model"],
            "train": {
                key: value for key, value in candidate["train"].items() if key != "rows"
            },
            "validation": {
                key: value
                for key, value in candidate["validation"].items()
                if key != "rows"
            },
            "external_policy_eval": {
                key: value
                for key, value in candidate["external_policy_eval"].items()
                if key != "rows"
            },
        }
        write_json(
            output / f"{name}_validation_rows.json",
            candidate["validation"]["rows"],
        )
        write_json(
            output / f"{name}_external_policy_eval_rows.json",
            candidate["external_policy_eval"]["rows"],
        )
    write_json(
        output / "ring_episode_calibration_rows.json",
        [
            {
                "sample_id": episode["sample_id"],
                "episode_index": episode["episode_index"],
                "episode_uid": episode["episode_uid"],
                "policy": episode["policy"],
                "pickup_frame": episode["pickup_frame"],
                "place_frame": episode["place_frame"],
                "anchor_diagnostics": episode["anchor_diagnostics"],
                "detections": episode["detections"],
                "image_path": episode["image_path"],
                "overlay_path": episode["overlay_path"],
            }
            for episode in episodes
        ],
    )

    report = {
        "schema": "ring_pixel_robot_mapping_v1",
        "image_coordinate_contract": {
            "camera": "exterior_1",
            "width": IMAGE_WIDTH,
            "height": IMAGE_HEIGHT,
            "transform": "native_LJM_direct_resize_320x180_no_inverse_transform",
        },
        "detector": {
            "method": "yellow_HSV_contour_pair_left_pick_right_place",
            "episode_count": len(sample_dirs),
            "success_count": len(episodes),
            "failure_count": len(failures),
            "failures": failures,
        },
        "split": {
            "seed": int(args.seed),
            "train_collection_episode_count": len(train_episodes),
            "validation_collection_episode_count": len(validation_episodes),
            "external_policy_eval_episode_count": len(external),
            "train_episode_uids": [episode["episode_uid"] for episode in train_episodes],
            "validation_episode_uids": [
                episode["episode_uid"] for episode in validation_episodes
            ],
            "external_episode_uids": [episode["episode_uid"] for episode in external],
        },
        "anchor_source": {
            "signal": "LeRobot 15-fps observation.state.gripper_position",
            "closed_threshold": "min + 0.45 * (max - min)",
            "selection": (
                "maximum planar EEF displacement among sustained closed "
                "segments (at least 5 frames)"
            ),
            "pickup": "first frame of selected closed segment",
            "place": "first frame after selected closed segment",
            "reason": (
                "Robust to short failed grasp attempts observed in policy "
                "episodes 1170 and 1171."
            ),
        },
        "candidates": serializable_candidates,
        "selected_model": selected_name,
        "selected_model_payload": selected_model,
        "trajectory_projection_qa": {
            "type": "affine_xyz_to_pixel",
            "matrix_4x2": xyz_to_pixel.astype(float).tolist(),
            "errors_px": projection_errors,
            "boundary": (
                "Used only to visualize the interaction segment. Pixel-to-robot "
                "mapping is trained and evaluated independently."
            ),
        },
        "artifacts": {
            "detection_contact_sheet": str(detection_sheet),
            "trajectory_contact_sheet": str(trajectory_sheet),
            "overlay_directory": str(overlays),
            "trajectory_overlay_directory": str(trajectory_dir),
            "episode_calibration_rows": str(
                output / "ring_episode_calibration_rows.json"
            ),
        },
    }
    write_json(output / "ring_mapping_report.json", report)
    write_json(
        output / "ring_mapping_model_selected.json",
        {
            "schema": "ring_pixel_robot_mapping_v1",
            "camera": "exterior_1",
            "image_size": [IMAGE_WIDTH, IMAGE_HEIGHT],
            "model": selected_model,
            "validation": serializable_candidates[selected_name]["validation"],
            "external_policy_eval": serializable_candidates[selected_name][
                "external_policy_eval"
            ],
        },
    )
    (output / "_SUCCESS").touch()
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
