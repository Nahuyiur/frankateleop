"""Calibration environment and session diagnostics."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence

import cv2
import numpy as np

from .geometry import matrix_from_list, robust_stats, rotation_angle_rad
from .io import load_sample_metadata, read_json, write_json
from .solve_eye_to_hand import accepted_samples


THRESHOLDS = {
    "min_good_samples": 20,
    "preferred_good_samples": 25,
    "min_corners": 12,
    "preferred_corners": 16,
    "reprojection_rms_good_px": 1.5,
    "reprojection_rms_accept_px": 2.0,
    "reprojection_max_accept_px": 5.0,
    "bbox_area_min": 0.03,
    "bbox_area_max": 0.60,
    "border_margin_min_px": 8.0,
    "depth_valid_ratio_min": 0.70,
    "depth_plane_median_max_m": 0.010,
    "depth_plane_p90_max_m": 0.020,
    "laplacian_warn": 50.0,
    "translation_span_preferred_m": 0.20,
    "rotation_span_preferred_deg": 25.0,
    "handeye_translation_median_m": 0.010,
    "handeye_translation_p90_m": 0.020,
    "handeye_rotation_median_deg": 1.5,
    "handeye_rotation_p90_deg": 3.0,
}


def environment_report() -> Dict[str, Any]:
    aruco_names = []
    if hasattr(cv2, "aruco"):
        aruco_names = [
            name
            for name in (
                "ArucoDetector",
                "CharucoBoard",
                "CharucoDetector",
                "getPredefinedDictionary",
            )
            if hasattr(cv2.aruco, name)
        ]
    report = {
        "opencv_version": cv2.__version__,
        "has_aruco": hasattr(cv2, "aruco"),
        "aruco_api": aruco_names,
        "has_solvePnP": hasattr(cv2, "solvePnP"),
        "has_calibrateHandEye": hasattr(cv2, "calibrateHandEye"),
        "handeye_methods": [name for name in dir(cv2) if name.startswith("CALIB_HAND_EYE_")],
        "display": os.environ.get("DISPLAY"),
        "xauthority": os.environ.get("XAUTHORITY"),
    }
    report["pass"] = bool(
        report["has_aruco"]
        and "CharucoBoard" in aruco_names
        and "CharucoDetector" in aruco_names
        and report["has_solvePnP"]
        and report["has_calibrateHandEye"]
    )
    return report


def session_report(session_dir: Path, *, strict: bool = False) -> Dict[str, Any]:
    session_dir = Path(session_dir).expanduser()
    samples = load_sample_metadata(session_dir)
    accepted, rejected = accepted_samples(samples)
    result_path = session_dir / "calibration_result.json"
    validation_path = session_dir / "validation_report.json"

    report: Dict[str, Any] = {
        "session_dir": str(session_dir),
        "thresholds": THRESHOLDS,
        "sample_counts": {
            "total": len(samples),
            "accepted": len(accepted),
            "rejected_gate": len(rejected),
        },
        "gate_rejected_samples": rejected,
        "sample_quality": _sample_quality(samples),
        "pose_diversity": _pose_diversity(accepted),
        "calibration_result": None,
        "validation_report": None,
        "warnings": [],
        "errors": [],
        "status": "UNKNOWN",
    }

    if len(accepted) < THRESHOLDS["min_good_samples"]:
        report["errors"].append(
            f"good samples too few: {len(accepted)} < {THRESHOLDS['min_good_samples']}"
        )
    elif len(accepted) < THRESHOLDS["preferred_good_samples"]:
        report["warnings"].append(
            f"good samples below preferred count: {len(accepted)} < {THRESHOLDS['preferred_good_samples']}"
        )

    _append_quality_warnings(report)

    if result_path.exists():
        result = read_json(result_path)
        report["calibration_result"] = {
            "selected_method": result.get("selected_method"),
            "sample_counts": result.get("sample_counts"),
            "residual_summary": result.get("residual_summary"),
            "excluded_outlier_ids": result.get("excluded_outlier_ids", []),
        }
        _append_residual_warnings(report, result.get("residual_summary") or {})
    else:
        message = "calibration_result.json not found; run solve first"
        if strict:
            report["errors"].append(message)
        else:
            report["warnings"].append(message)

    if validation_path.exists():
        validation = read_json(validation_path)
        report["validation_report"] = {
            "pass": validation.get("pass"),
            "residual_summary": validation.get("residual_summary"),
            "sample_counts": validation.get("sample_counts"),
        }
        if not validation.get("pass"):
            report["errors"].append("validation_report pass=false")
    else:
        message = "validation_report.json not found; run solve/validate first"
        if strict:
            report["errors"].append(message)
        else:
            report["warnings"].append(message)

    if report["errors"]:
        report["status"] = "FAIL"
    elif result_path.exists() and validation_path.exists():
        report["status"] = "PASS_WITH_WARNINGS" if report["warnings"] else "PASS"
    else:
        report["status"] = "READY_TO_SOLVE_WITH_WARNINGS" if report["warnings"] else "READY_TO_SOLVE"
    return report


def _sample_quality(samples: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    detections = [sample.get("detection") or {} for sample in samples]
    quality = [detection.get("quality") or {} for detection in detections]
    depth_plane = [
        item.get("depth_plane") or {}
        for item in quality
        if isinstance(item.get("depth_plane"), dict)
    ]
    return {
        "corner_count": robust_stats(int(detection.get("corner_count") or 0) for detection in detections),
        "reprojection_rms_px": robust_stats(
            float(detection["reprojection_rms_px"])
            for detection in detections
            if detection.get("reprojection_rms_px") is not None
        ),
        "reprojection_max_px": robust_stats(
            float(detection["reprojection_max_px"])
            for detection in detections
            if detection.get("reprojection_max_px") is not None
        ),
        "bbox_area_ratio": robust_stats(
            float(item["bbox_area_ratio"]) for item in quality if item.get("bbox_area_ratio") is not None
        ),
        "border_margin_px": robust_stats(
            float(item["border_margin_px"]) for item in quality if item.get("border_margin_px") is not None
        ),
        "laplacian_variance": robust_stats(
            float(item["laplacian_variance"]) for item in quality if item.get("laplacian_variance") is not None
        ),
        "depth_valid_ratio": robust_stats(
            float(item["valid_ratio"]) for item in depth_plane if item.get("valid_ratio") is not None
        ),
        "depth_plane_median_abs_error_m": robust_stats(
            float(item["median_abs_plane_error_m"])
            for item in depth_plane
            if item.get("median_abs_plane_error_m") is not None
        ),
        "depth_plane_p90_abs_error_m": robust_stats(
            float(item["p90_abs_plane_error_m"])
            for item in depth_plane
            if item.get("p90_abs_plane_error_m") is not None
        ),
    }


def _pose_diversity(samples: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    transforms = []
    for sample in samples:
        try:
            transforms.append(matrix_from_list(sample["T_base_gripper"], name="T_base_gripper"))
        except Exception:
            pass
    if not transforms:
        return {
            "count": 0,
            "translation_span_xyz_m": None,
            "translation_span_norm_m": None,
            "rotation_span_deg": None,
            "rotation_axis_rank": 0,
            "rotation_axis_singular_values": [],
        }
    translations = np.asarray([transform[:3, 3] for transform in transforms], dtype=np.float64)
    span_xyz = np.max(translations, axis=0) - np.min(translations, axis=0)
    reference = transforms[0][:3, :3]
    angles = []
    for transform in transforms:
        angles.append(math.degrees(rotation_angle_rad(reference.T @ transform[:3, :3])))
    axes = []
    for transform in transforms[1:]:
        rotvec, _ = cv2.Rodrigues(reference.T @ transform[:3, :3])
        angle = float(np.linalg.norm(rotvec))
        if angle > math.radians(2.0):
            axes.append((rotvec.reshape(3) / angle).astype(np.float64))
    if len(axes) >= 2:
        singular_values = np.linalg.svd(np.asarray(axes), compute_uv=False)
        axis_rank = int(np.sum(singular_values > 0.25 * max(float(singular_values[0]), 1e-9)))
        singular_values_list = [float(x) for x in singular_values]
    else:
        axis_rank = len(axes)
        singular_values_list = []
    return {
        "count": len(transforms),
        "translation_span_xyz_m": [float(x) for x in span_xyz],
        "translation_span_norm_m": float(np.linalg.norm(span_xyz)),
        "rotation_span_deg": float(max(angles) - min(angles)) if angles else 0.0,
        "rotation_axis_rank": axis_rank,
        "rotation_axis_singular_values": singular_values_list,
    }


def _stat_value(stats: Dict[str, Any], key: str, default: float = 1e9) -> float:
    value = stats.get(key)
    if value is None:
        return default
    return float(value)


def _append_quality_warnings(report: Dict[str, Any]) -> None:
    quality = report["sample_quality"]
    pose = report["pose_diversity"]
    if _stat_value(quality["corner_count"], "median", 0.0) < THRESHOLDS["preferred_corners"]:
        report["warnings"].append("median detected ChArUco corners below preferred count")
    if _stat_value(quality["reprojection_rms_px"], "p90") > THRESHOLDS["reprojection_rms_accept_px"]:
        report["warnings"].append("reprojection RMS p90 is high")
    if _stat_value(quality["reprojection_max_px"], "p90") > THRESHOLDS["reprojection_max_accept_px"]:
        report["warnings"].append("reprojection max error p90 is high")
    bbox_median = _stat_value(quality["bbox_area_ratio"], "median", default=0.0)
    if bbox_median and (bbox_median < THRESHOLDS["bbox_area_min"] or bbox_median > THRESHOLDS["bbox_area_max"]):
        report["warnings"].append("board image coverage is outside recommended range")
    border_min = _stat_value(quality["border_margin_px"], "min", default=1e9)
    if border_min < THRESHOLDS["border_margin_min_px"]:
        report["warnings"].append("some board detections are too close to image border")
    lap_median = _stat_value(quality["laplacian_variance"], "median", default=1e9)
    if lap_median < THRESHOLDS["laplacian_warn"]:
        report["warnings"].append("images may be blurry: low Laplacian variance")
    depth_count = quality["depth_valid_ratio"]["count"]
    if depth_count:
        if _stat_value(quality["depth_valid_ratio"], "median", default=0.0) < THRESHOLDS["depth_valid_ratio_min"]:
            report["warnings"].append("board depth ROI valid ratio is low")
        if _stat_value(quality["depth_plane_median_abs_error_m"], "median") > THRESHOLDS["depth_plane_median_max_m"]:
            report["warnings"].append("depth points are not close to the detected board plane")
        if _stat_value(quality["depth_plane_p90_abs_error_m"], "p90") > THRESHOLDS["depth_plane_p90_max_m"]:
            report["warnings"].append("depth-plane p90 error is high")
    else:
        report["warnings"].append("no depth-plane consistency metrics found; capture new samples with current capture_samples")

    span = pose.get("translation_span_norm_m")
    if span is not None and span < THRESHOLDS["translation_span_preferred_m"]:
        report["warnings"].append("robot pose translation span is small; cover more workspace")
    rotation_span = pose.get("rotation_span_deg")
    if rotation_span is not None and rotation_span < THRESHOLDS["rotation_span_preferred_deg"]:
        report["warnings"].append("robot pose rotation span is small; rotate gripper around multiple axes")
    axis_rank = pose.get("rotation_axis_rank")
    if axis_rank is not None and axis_rank < 2:
        report["warnings"].append("robot rotation axes are not diverse enough; avoid single-axis-only motion")


def _append_residual_warnings(report: Dict[str, Any], summary: Dict[str, Any]) -> None:
    trans = summary.get("translation_error_m") or {}
    rot = summary.get("rotation_error_deg") or {}
    if _stat_value(trans, "median") > THRESHOLDS["handeye_translation_median_m"]:
        report["errors"].append("hand-eye translation median residual is too high")
    if _stat_value(trans, "p90") > THRESHOLDS["handeye_translation_p90_m"]:
        report["errors"].append("hand-eye translation p90 residual is too high")
    if _stat_value(rot, "median") > THRESHOLDS["handeye_rotation_median_deg"]:
        report["errors"].append("hand-eye rotation median residual is too high")
    if _stat_value(rot, "p90") > THRESHOLDS["handeye_rotation_p90_deg"]:
        report["errors"].append("hand-eye rotation p90 residual is too high")


def print_environment(report: Dict[str, Any]) -> None:
    print("Environment check:")
    print(f"  OpenCV: {report['opencv_version']}")
    print(f"  cv2.aruco: {report['has_aruco']} {report['aruco_api']}")
    print(f"  solvePnP: {report['has_solvePnP']}")
    print(f"  calibrateHandEye: {report['has_calibrateHandEye']}")
    print(f"  DISPLAY: {report['display']}")
    print(f"  status: {'PASS' if report['pass'] else 'FAIL'}")


def print_session(report: Dict[str, Any]) -> None:
    print("Calibration session check:")
    print(f"  session: {report['session_dir']}")
    print(f"  status: {report['status']}")
    counts = report["sample_counts"]
    print(f"  samples: total={counts['total']} accepted={counts['accepted']} rejected={counts['rejected_gate']}")
    quality = report["sample_quality"]
    print(f"  reprojection RMS px: {quality['reprojection_rms_px']}")
    print(f"  corners: {quality['corner_count']}")
    print(f"  pose diversity: {report['pose_diversity']}")
    if report["calibration_result"]:
        print(f"  selected method: {report['calibration_result']['selected_method']}")
        print(f"  residual summary: {report['calibration_result']['residual_summary']}")
    if report["warnings"]:
        print("  warnings:")
        for warning in report["warnings"]:
            print(f"    - {warning}")
    if report["errors"]:
        print("  errors:")
        for error in report["errors"]:
            print(f"    - {error}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", nargs="?", default=None)
    parser.add_argument("--env-only", action="store_true", help="Only check OpenCV/environment")
    parser.add_argument("--strict", action="store_true", help="Fail if solved result/report files are missing.")
    parser.add_argument("--output", default=None, help="Write JSON report. Defaults to session_dir/doctor_report.json when session_dir is given.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    env = environment_report()
    if args.env_only or not args.session_dir:
        print_environment(env)
        if args.output:
            write_json(Path(args.output).expanduser(), {"environment": env})
        if not env["pass"]:
            raise SystemExit(1)
        return

    session = session_report(Path(args.session_dir).expanduser(), strict=args.strict)
    report = {"environment": env, "session": session}
    print_environment(env)
    print_session(session)
    output = Path(args.output).expanduser() if args.output else Path(args.session_dir).expanduser() / "doctor_report.json"
    write_json(output, report)
    print(f"Wrote {output}")
    if not env["pass"] or session["status"] == "FAIL":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
