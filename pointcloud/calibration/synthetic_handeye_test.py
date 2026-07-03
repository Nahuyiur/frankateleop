"""Synthetic checks for transform conventions and OpenCV hand-eye direction."""

from __future__ import annotations

import argparse
import math
import tempfile
from pathlib import Path
from typing import List

import cv2
import numpy as np

from .geometry import (
    euler_xyz_to_matrix,
    invert_transform,
    make_transform,
    matrix_to_list,
    transform_error_degrees,
)
from .io import write_json
from .solve_eye_to_hand import solve_method


def _random_rotation(rng: np.random.Generator, max_angle_rad: float = 1.2) -> np.ndarray:
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = rng.uniform(-max_angle_rad, max_angle_rad)
    rotation, _ = cv2.Rodrigues(axis * angle)
    return rotation


def _random_transform(rng: np.random.Generator, translation_scale: float = 0.7, max_angle_rad: float = 1.2) -> np.ndarray:
    return make_transform(_random_rotation(rng, max_angle_rad=max_angle_rad), rng.uniform(-translation_scale, translation_scale, 3))


def _synthetic_samples(count: int = 24) -> tuple:
    rng = np.random.default_rng(42)
    transform_base_camera = _random_transform(rng, translation_scale=1.0)
    transform_gripper_target = _random_transform(rng, translation_scale=0.15)
    samples = []
    for index in range(count):
        transform_base_gripper = _random_transform(rng, translation_scale=0.55, max_angle_rad=2.2)
        transform_camera_target = invert_transform(transform_base_camera) @ transform_base_gripper @ transform_gripper_target
        samples.append(
            {
                "_sample_id": f"sample_{index:06d}",
                "T_base_gripper": matrix_to_list(transform_base_gripper),
                "robot_stability": {
                    "stable": True,
                    "checks": 3,
                    "max_translation_m": 0.0,
                    "max_rotation_deg": 0.0,
                },
                "detection": {
                    "valid": True,
                    "corner_count": 24,
                    "reprojection_rms_px": 0.2,
                    "reprojection_max_px": 0.8,
                    "T_camera_target": matrix_to_list(transform_camera_target),
                },
            }
        )
    return transform_base_camera, samples


def test_euler_convention() -> None:
    expected = np.array(
        [
            [0.749596, -0.660349, 0.045215],
            [0.631376, 0.692859, -0.348296],
            [0.198669, 0.289629, 0.936293],
        ],
        dtype=np.float64,
    )
    actual = euler_xyz_to_matrix(0.3, -0.2, 0.7)
    err = float(np.max(np.abs(actual - expected)))
    if err > 1e-6:
        raise AssertionError(f"Euler xyz convention mismatch: max error {err}")


def test_eye_to_hand_direction() -> None:
    truth, samples = _synthetic_samples()
    estimated = solve_method(samples, "PARK")
    translation_m, rotation_deg = transform_error_degrees(truth, estimated)
    if translation_m > 1e-8 or rotation_deg > 1e-6:
        raise AssertionError(f"eye-to-hand solve failed: {translation_m} m, {rotation_deg} deg")


def test_wrong_direction_fails() -> None:
    truth, samples = _synthetic_samples()
    wrong_samples = []
    for sample in samples:
        wrong_sample = dict(sample)
        wrong_sample["T_base_gripper"] = matrix_to_list(invert_transform(np.asarray(sample["T_base_gripper"], dtype=np.float64)))
        wrong_samples.append(wrong_sample)
    estimated = solve_method(wrong_samples, "PARK")
    translation_m, rotation_deg = transform_error_degrees(truth, estimated)
    if translation_m < 0.05 and rotation_deg < 5.0:
        raise AssertionError("wrong hand-eye input direction unexpectedly passed")


def test_solve_session_json(tmp_dir: Path) -> None:
    from .solve_eye_to_hand import solve_session

    truth, samples = _synthetic_samples()
    session = tmp_dir / "synthetic_session"
    samples_dir = session / "samples"
    samples_dir.mkdir(parents=True)
    for sample in samples:
        sample_dir = samples_dir / str(sample["_sample_id"])
        sample_dir.mkdir()
        payload = dict(sample)
        payload.pop("_sample_id", None)
        write_json(sample_dir / "metadata.json", payload)
    result = solve_session(session)
    estimated = np.asarray(result["T_base_camera"], dtype=np.float64)
    translation_m, rotation_deg = transform_error_degrees(truth, estimated)
    if translation_m > 1e-8 or rotation_deg > 1e-6:
        raise AssertionError(f"solve_session failed: {translation_m} m, {rotation_deg} deg")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    test_euler_convention()
    test_eye_to_hand_direction()
    test_wrong_direction_fails()
    with tempfile.TemporaryDirectory(prefix="frankateleop_calib_test_") as tmp:
        test_solve_session_json(Path(tmp))
    print("SYNTHETIC_HAND_EYE_TEST_OK")


if __name__ == "__main__":
    main()
