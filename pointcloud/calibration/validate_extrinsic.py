"""Validate a saved eye-to-hand calibration result against its samples."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict

from .geometry import matrix_from_list
from .io import load_sample_metadata, read_json, write_json
from .solve_eye_to_hand import accepted_samples, residuals_for_solution, summarize_residuals


MIN_GOOD_SAMPLES = 20


def validate_session(session_dir: Path, result_path: Path, *, all_accepted: bool = False) -> Dict[str, Any]:
    result = read_json(result_path)
    samples = load_sample_metadata(session_dir)
    accepted, rejected = accepted_samples(samples)
    accepted_by_id = {
        str(sample.get("_sample_id", sample.get("sample_id"))): sample
        for sample in accepted
    }
    used_ids = [str(sample_id) for sample_id in result.get("used_sample_ids", [])]
    missing_used_ids = [sample_id for sample_id in used_ids if sample_id not in accepted_by_id]
    if all_accepted or not used_ids:
        validation_samples = list(accepted)
        validation_sample_mode = "all_accepted"
    else:
        validation_samples = [accepted_by_id[sample_id] for sample_id in used_ids if sample_id in accepted_by_id]
        validation_sample_mode = "result_used_sample_ids"
    if missing_used_ids:
        raise RuntimeError(f"Result references sample ids that are not accepted in this session: {missing_used_ids}")
    transform = matrix_from_list(result["T_base_camera"], name="T_base_camera")
    residuals, reference = residuals_for_solution(transform, validation_samples)
    summary = summarize_residuals(residuals)
    translation_median = _metric_value(summary, "translation_error_m", "median")
    translation_p90 = _metric_value(summary, "translation_error_m", "p90")
    rotation_median = _metric_value(summary, "rotation_error_deg", "median")
    rotation_p90 = _metric_value(summary, "rotation_error_deg", "p90")
    sample_counts = result.get("sample_counts", {})
    accepted_initial = int(sample_counts.get("accepted_initial") or len(accepted))
    excluded_outliers = int(sample_counts.get("excluded_outliers") or 0)
    outlier_ratio = float(excluded_outliers / accepted_initial) if accepted_initial else 1.0
    report = {
        "session_dir": str(session_dir),
        "result_path": str(result_path),
        "selected_method": result.get("selected_method"),
        "validation_sample_mode": validation_sample_mode,
        "sample_counts": {
            "total": len(samples),
            "accepted_initial": len(accepted),
            "used_for_validation": len(validation_samples),
            "rejected_gate": len(rejected),
            "excluded_outliers": excluded_outliers,
        },
        "gate_rejected_samples": rejected,
        "excluded_outlier_ids": result.get("excluded_outlier_ids", []),
        "residuals": residuals,
        "residual_summary": summary,
        "thresholds": {
            "translation_median_target_m": 0.010,
            "translation_p90_target_m": 0.020,
            "rotation_median_target_deg": 1.5,
            "rotation_p90_target_deg": 3.0,
        },
        "pass_checks": {
            "good_samples": len(validation_samples) >= MIN_GOOD_SAMPLES,
            "outlier_ratio": outlier_ratio <= 0.25,
            "translation_median": translation_median < 0.010,
            "translation_p90": translation_p90 < 0.020,
            "rotation_median": rotation_median < 1.5,
            "rotation_p90": rotation_p90 < 3.0,
        },
    }
    report["pass"] = bool(all(report["pass_checks"].values()))
    return report


def _metric_value(summary: Dict[str, Any], group: str, key: str) -> float:
    value = summary.get(group, {}).get(key)
    if value is None:
        return 1e9
    return float(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir")
    parser.add_argument("--result", default=None, help="Defaults to session_dir/calibration_result.json")
    parser.add_argument("--output", default=None, help="Defaults to session_dir/validation_report.json")
    parser.add_argument(
        "--all-accepted",
        action="store_true",
        help="Diagnostic mode: validate against all gate-accepted samples. Defaults to validation_all_accepted_report.json.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    session_dir = Path(args.session_dir).expanduser()
    result_path = Path(args.result).expanduser() if args.result else session_dir / "calibration_result.json"
    report = validate_session(session_dir, result_path, all_accepted=args.all_accepted)
    if args.output:
        output = Path(args.output).expanduser()
    elif args.all_accepted:
        output = session_dir / "validation_all_accepted_report.json"
    else:
        output = session_dir / "validation_report.json"
    write_json(output, report)
    print(f"Wrote {output}")
    print(f"Pass: {report['pass']}")
    print(f"Residual summary: {report['residual_summary']}")


if __name__ == "__main__":
    main()
