"""Solve fixed-camera eye-to-hand calibration from saved samples."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import cv2
import numpy as np

from .geometry import (
    choose_medoid_transform,
    invert_transform,
    make_transform,
    matrix_from_list,
    matrix_to_list,
    robust_stats,
    split_transform,
    transform_error_degrees,
)
from .io import SCHEMA_VERSION, load_sample_metadata, read_json, write_json


METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}
MIN_GOOD_SAMPLES = 20


def accepted_samples(
    samples: Sequence[Dict[str, Any]],
    *,
    min_corners: int = 12,
    max_reprojection_rms_px: float = 2.0,
    max_reprojection_max_px: float = 5.0,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    accepted: List[Dict[str, Any]] = []
    rejected: List[Dict[str, Any]] = []
    for sample in samples:
        detection = sample.get("detection") or {}
        reasons = []
        if not detection.get("valid"):
            reasons.append(str(detection.get("reason") or "invalid_detection"))
        if int(detection.get("corner_count") or 0) < int(min_corners):
            reasons.append("not_enough_corners")
        rms = detection.get("reprojection_rms_px")
        if rms is None or float(rms) > float(max_reprojection_rms_px):
            reasons.append("high_reprojection_rms")
        max_error = detection.get("reprojection_max_px")
        if max_error is not None and float(max_error) > float(max_reprojection_max_px):
            reasons.append("high_reprojection_max")
        if "T_base_gripper" not in sample:
            reasons.append("missing_T_base_gripper")
        stability = sample.get("robot_stability")
        if not isinstance(stability, dict) or stability.get("stable") is not True:
            reasons.append("robot_not_confirmed_still")
        if "T_camera_target" not in detection:
            reasons.append("missing_T_camera_target")
        elif detection.get("T_camera_target") is None:
            reasons.append("missing_T_camera_target")
        if reasons:
            rejected.append({"sample_id": sample.get("_sample_id", sample.get("sample_id")), "reasons": reasons})
        else:
            accepted.append(sample)
    return accepted, rejected


def _sample_id(sample: Dict[str, Any]) -> str:
    return str(sample.get("_sample_id", sample.get("sample_id")))


def _rt_lists_for_eye_to_hand(samples: Sequence[Dict[str, Any]]) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
    rotations_base2gripper = []
    translations_base2gripper = []
    rotations_target2cam = []
    translations_target2cam = []
    for sample in samples:
        transform_base_gripper = matrix_from_list(sample["T_base_gripper"], name="T_base_gripper")
        transform_gripper_base = invert_transform(transform_base_gripper)
        transform_camera_target = matrix_from_list(sample["detection"]["T_camera_target"], name="T_camera_target")
        robot_rotation, robot_translation = split_transform(transform_gripper_base)
        target_rotation, target_translation = split_transform(transform_camera_target)
        rotations_base2gripper.append(robot_rotation)
        translations_base2gripper.append(robot_translation.reshape(3, 1))
        rotations_target2cam.append(target_rotation)
        translations_target2cam.append(target_translation.reshape(3, 1))
    return rotations_base2gripper, translations_base2gripper, rotations_target2cam, translations_target2cam


def solve_method(samples: Sequence[Dict[str, Any]], method_name: str) -> np.ndarray:
    if len(samples) < 3:
        raise ValueError("At least 3 accepted samples are required")
    if method_name not in METHODS:
        raise ValueError(f"Unknown method {method_name}; choose from {sorted(METHODS)}")
    rg, tg, rt, tt = _rt_lists_for_eye_to_hand(samples)
    rotation, translation = cv2.calibrateHandEye(rg, tg, rt, tt, method=METHODS[method_name])
    return make_transform(rotation, np.asarray(translation, dtype=np.float64).reshape(3))


def _select_best_solution(
    samples: Sequence[Dict[str, Any]],
) -> Tuple[Dict[str, Any], str, np.ndarray, List[Dict[str, Any]], np.ndarray]:
    method_reports: Dict[str, Any] = {}
    best_score = float("inf")
    selected_name = None
    selected_transform = None
    selected_residuals = None
    selected_reference = None

    for method_name in METHODS:
        try:
            transform = solve_method(samples, method_name)
            residuals, reference = residuals_for_solution(transform, samples)
            summary = summarize_residuals(residuals)
            score = float(summary["translation_error_m"]["median"] or 0.0) + 0.01 * float(summary["rotation_error_deg"]["median"] or 0.0)
            method_reports[method_name] = {
                "ok": True,
                "score": score,
                "summary": summary,
                "T_base_camera": matrix_to_list(transform),
            }
            if score < best_score:
                best_score = score
                selected_name = method_name
                selected_transform = transform
                selected_residuals = residuals
                selected_reference = reference
        except Exception as exc:
            method_reports[method_name] = {"ok": False, "error": repr(exc)}

    if (
        selected_name is None
        or selected_transform is None
        or selected_residuals is None
        or selected_reference is None
    ):
        raise RuntimeError("All hand-eye methods failed")
    return method_reports, selected_name, selected_transform, selected_residuals, selected_reference


def residuals_for_solution(transform_base_camera: np.ndarray, samples: Sequence[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], np.ndarray]:
    gripper_target_transforms = []
    for sample in samples:
        transform_base_gripper = matrix_from_list(sample["T_base_gripper"], name="T_base_gripper")
        transform_camera_target = matrix_from_list(sample["detection"]["T_camera_target"], name="T_camera_target")
        gripper_target_transforms.append(invert_transform(transform_base_gripper) @ transform_base_camera @ transform_camera_target)
    reference = choose_medoid_transform(gripper_target_transforms)
    residuals = []
    for sample, transform_gripper_target in zip(samples, gripper_target_transforms):
        translation_m, rotation_deg = transform_error_degrees(reference, transform_gripper_target)
        residuals.append(
            {
                "sample_id": sample.get("_sample_id", sample.get("sample_id")),
                "translation_error_m": translation_m,
                "rotation_error_deg": rotation_deg,
                "reprojection_rms_px": sample.get("detection", {}).get("reprojection_rms_px"),
            }
        )
    return residuals, reference


def summarize_residuals(residuals: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "translation_error_m": robust_stats([float(row["translation_error_m"]) for row in residuals]),
        "rotation_error_deg": robust_stats([float(row["rotation_error_deg"]) for row in residuals]),
        "reprojection_rms_px": robust_stats(
            [
                float(row["reprojection_rms_px"])
                for row in residuals
                if row.get("reprojection_rms_px") is not None
            ]
        ),
    }


def outlier_ids(residuals: Sequence[Dict[str, Any]], *, max_translation_m: float = 0.02, max_rotation_deg: float = 3.0) -> List[str]:
    trans_values = np.asarray([float(row["translation_error_m"]) for row in residuals], dtype=np.float64)
    rot_values = np.asarray([float(row["rotation_error_deg"]) for row in residuals], dtype=np.float64)
    if trans_values.size == 0:
        return []
    trans_median = float(np.median(trans_values))
    trans_mad = float(np.median(np.abs(trans_values - trans_median)))
    rot_median = float(np.median(rot_values))
    rot_mad = float(np.median(np.abs(rot_values - rot_median)))
    trans_limit = max(float(max_translation_m), trans_median + 3.0 * max(trans_mad, 1e-9))
    rot_limit = max(float(max_rotation_deg), rot_median + 3.0 * max(rot_mad, 1e-9))
    ids = []
    for row in residuals:
        if float(row["translation_error_m"]) > trans_limit or float(row["rotation_error_deg"]) > rot_limit:
            ids.append(str(row["sample_id"]))
    return ids


def solve_session(
    session_dir: Path,
    *,
    min_corners: int = 12,
    max_reprojection_rms_px: float = 2.0,
    max_reprojection_max_px: float = 5.0,
    max_outlier_iterations: int = 3,
    allow_low_samples: bool = False,
) -> Dict[str, Any]:
    samples = load_sample_metadata(session_dir)
    accepted, rejected = accepted_samples(
        samples,
        min_corners=min_corners,
        max_reprojection_rms_px=max_reprojection_rms_px,
        max_reprojection_max_px=max_reprojection_max_px,
    )
    if len(accepted) < 3:
        raise RuntimeError(f"Need at least 3 accepted samples, got {len(accepted)}")
    if len(accepted) < MIN_GOOD_SAMPLES and not allow_low_samples:
        raise RuntimeError(
            f"Need at least {MIN_GOOD_SAMPLES} accepted samples for a formal calibration, "
            f"got {len(accepted)}. Re-run with --allow-low-samples only for debugging."
        )

    session_meta: Dict[str, Any] = {}
    session_json = Path(session_dir).expanduser() / "session.json"
    if session_json.exists():
        session_meta = read_json(session_json)

    excluded_ids: List[str] = []
    current_samples = list(accepted)

    for _iteration in range(max(1, int(max_outlier_iterations))):
        _, _, _, selected_residuals, _ = _select_best_solution(current_samples)
        ids = outlier_ids(selected_residuals)
        new_ids = [sample_id for sample_id in ids if sample_id not in excluded_ids]
        if not new_ids:
            break
        if len(excluded_ids) + len(new_ids) > max(1, int(0.25 * len(accepted))):
            break
        excluded_ids.extend(new_ids)
        current_samples = [sample for sample in accepted if _sample_id(sample) not in set(excluded_ids)]
        if len(current_samples) < 3:
            break

    method_reports, selected_name, selected_transform, selected_residuals, selected_reference = _select_best_solution(current_samples)
    result = {
        "schema_version": SCHEMA_VERSION,
        "calibration_type": "eye_to_hand",
        "world_frame": "robot_base",
        "camera_frame": "camera_color_optical",
        "camera_name": session_meta.get("camera_name"),
        "camera_metadata": session_meta.get("camera_metadata"),
        "transform_name": "T_base_camera_color_optical",
        "T_base_camera": matrix_to_list(selected_transform),
        "T_gripper_target_reference": matrix_to_list(selected_reference),
        "selected_method": selected_name,
        "provisional": bool(len(current_samples) < MIN_GOOD_SAMPLES),
        "method_reports": method_reports,
        "sample_counts": {
            "total": len(samples),
            "accepted_initial": len(accepted),
            "used_final": len(current_samples),
            "rejected_gate": len(rejected),
            "excluded_outliers": len(excluded_ids),
        },
        "used_sample_ids": [_sample_id(sample) for sample in current_samples],
        "rejected_samples": rejected,
        "excluded_outlier_ids": excluded_ids,
        "residuals": selected_residuals,
        "residual_summary": summarize_residuals(selected_residuals),
        "notes": [
            "For eye-to-hand, OpenCV receives inv(T_base_gripper) and T_camera_target.",
            "The returned transform is interpreted as T_base_camera.",
        ],
    }
    return result


def validation_report_from_result(result: Dict[str, Any], session_dir: Path) -> Dict[str, Any]:
    summary = result["residual_summary"]
    translation_median = _metric_value(summary, "translation_error_m", "median")
    translation_p90 = _metric_value(summary, "translation_error_m", "p90")
    rotation_median = _metric_value(summary, "rotation_error_deg", "median")
    rotation_p90 = _metric_value(summary, "rotation_error_deg", "p90")
    sample_counts = result.get("sample_counts", {})
    used_final = int(sample_counts.get("used_final") or 0)
    accepted_initial = int(sample_counts.get("accepted_initial") or 0)
    excluded_outliers = int(sample_counts.get("excluded_outliers") or 0)
    outlier_ratio = float(excluded_outliers / accepted_initial) if accepted_initial else 1.0
    pass_checks = {
        "good_samples": used_final >= MIN_GOOD_SAMPLES,
        "outlier_ratio": outlier_ratio <= 0.25,
        "translation_median": translation_median < 0.010,
        "translation_p90": translation_p90 < 0.020,
        "rotation_median": rotation_median < 1.5,
        "rotation_p90": rotation_p90 < 3.0,
    }
    report = {
        "session_dir": str(session_dir),
        "selected_method": result.get("selected_method"),
        "sample_counts": result.get("sample_counts", {}),
        "residual_summary": summary,
        "residuals": result.get("residuals", []),
        "thresholds": {
            "translation_median_target_m": 0.010,
            "translation_p90_target_m": 0.020,
            "rotation_median_target_deg": 1.5,
            "rotation_p90_target_deg": 3.0,
        },
        "pass_checks": pass_checks,
        "pass": bool(all(pass_checks.values())),
        "notes": [
            "This report checks internal hand-eye consistency. Real hardware still needs held-out board and world-frame pointcloud checks.",
            "Do not use a calibration for formal collection unless this report passes and visual overlays look physically correct.",
        ],
    }
    return report


def _metric_value(summary: Dict[str, Any], group: str, key: str) -> float:
    value = summary.get(group, {}).get(key)
    if value is None:
        return 1e9
    return float(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir")
    parser.add_argument("--min-corners", type=int, default=12)
    parser.add_argument("--max-reprojection-rms-px", type=float, default=2.0)
    parser.add_argument("--max-reprojection-max-px", type=float, default=5.0)
    parser.add_argument("--max-outlier-iterations", type=int, default=3)
    parser.add_argument("--allow-low-samples", action="store_true", help="Debug only: allow fewer than 20 accepted samples.")
    parser.add_argument("--output", default=None, help="Defaults to session_dir/calibration_result.json")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    session_dir = Path(args.session_dir).expanduser()
    result = solve_session(
        session_dir,
        min_corners=args.min_corners,
        max_reprojection_rms_px=args.max_reprojection_rms_px,
        max_reprojection_max_px=args.max_reprojection_max_px,
        max_outlier_iterations=args.max_outlier_iterations,
        allow_low_samples=args.allow_low_samples,
    )
    output = Path(args.output).expanduser() if args.output else session_dir / "calibration_result.json"
    write_json(output, result)
    report_path = output.parent / "validation_report.json"
    write_json(report_path, validation_report_from_result(result, session_dir))
    print(f"Wrote {output}")
    print(f"Wrote {report_path}")
    print(f"Selected method: {result['selected_method']}")
    print(f"Used samples: {result['sample_counts']['used_final']} / {result['sample_counts']['total']}")
    print(f"Residual summary: {result['residual_summary']}")


if __name__ == "__main__":
    main()
