"""Small transform helpers used by the calibration tools.

Conventions:
    T_A_B maps points from frame B into frame A:
        p_A = T_A_B @ p_B

Robot poses stored as ``ee_pose_euler`` are interpreted as
``[x, y, z, roll, pitch, yaw]`` where the angles are SciPy
``Rotation.as_euler("xyz", degrees=False)`` values.  Reconstructing that
rotation is equivalent to ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``.
"""

from __future__ import annotations

import math
from typing import Any, Iterable, List, Sequence, Tuple

import cv2
import numpy as np


def ensure_transform(value: Any, *, name: str = "transform") -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"{name} must have shape (4, 4), got {matrix.shape}")
    return matrix


def make_transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64).reshape(3)
    if rotation.shape != (3, 3):
        raise ValueError(f"rotation must have shape (3, 3), got {rotation.shape}")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def split_transform(transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    matrix = ensure_transform(transform)
    return matrix[:3, :3].copy(), matrix[:3, 3].copy()


def invert_transform(transform: np.ndarray) -> np.ndarray:
    matrix = ensure_transform(transform)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def compose_transforms(*transforms: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    for transform in transforms:
        result = result @ ensure_transform(transform)
    return result


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    matrix = ensure_transform(transform)
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must have shape (N, 3), got {points.shape}")
    return (points @ matrix[:3, :3].T) + matrix[:3, 3]


def euler_xyz_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the matrix equivalent to SciPy ``Rotation.from_euler("xyz")``."""

    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def pose_euler_xyz_to_transform(pose: Sequence[float]) -> np.ndarray:
    pose = np.asarray(pose, dtype=np.float64).reshape(-1)
    if pose.shape != (6,):
        raise ValueError(f"pose must have 6 values, got shape {pose.shape}")
    rotation = euler_xyz_to_matrix(pose[3], pose[4], pose[5])
    return make_transform(rotation, pose[:3])


def rodrigues_to_transform(rvec: Sequence[float], tvec: Sequence[float]) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return make_transform(rotation, np.asarray(tvec, dtype=np.float64).reshape(3))


def transform_to_rodrigues(transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    rotation, translation = split_transform(transform)
    rvec, _ = cv2.Rodrigues(rotation)
    return rvec.reshape(3), translation.reshape(3)


def rotation_angle_rad(rotation: np.ndarray) -> float:
    rotation = np.asarray(rotation, dtype=np.float64)
    rvec, _ = cv2.Rodrigues(rotation)
    return float(np.linalg.norm(rvec))


def transform_delta(reference: np.ndarray, candidate: np.ndarray) -> np.ndarray:
    return invert_transform(reference) @ ensure_transform(candidate)


def transform_error(reference: np.ndarray, candidate: np.ndarray) -> Tuple[float, float]:
    delta = transform_delta(reference, candidate)
    return float(np.linalg.norm(delta[:3, 3])), rotation_angle_rad(delta[:3, :3])


def transform_error_degrees(reference: np.ndarray, candidate: np.ndarray) -> Tuple[float, float]:
    translation_m, rotation_rad = transform_error(reference, candidate)
    return translation_m, math.degrees(rotation_rad)


def matrix_to_list(matrix: np.ndarray) -> List[List[float]]:
    return [[float(value) for value in row] for row in np.asarray(matrix, dtype=np.float64)]


def matrix_from_list(value: Any, *, name: str = "matrix") -> np.ndarray:
    return ensure_transform(np.asarray(value, dtype=np.float64), name=name)


def choose_medoid_transform(transforms: Sequence[np.ndarray], *, rotation_scale_m_per_deg: float = 0.01) -> np.ndarray:
    """Choose the transform with the lowest median distance to the others."""

    if not transforms:
        raise ValueError("cannot choose medoid from an empty transform list")
    matrices = [ensure_transform(transform) for transform in transforms]
    best_index = 0
    best_score = float("inf")
    for index, candidate in enumerate(matrices):
        scores = []
        for other in matrices:
            translation_m, rotation_deg = transform_error_degrees(candidate, other)
            scores.append(translation_m + rotation_scale_m_per_deg * rotation_deg)
        score = float(np.median(np.asarray(scores, dtype=np.float64)))
        if score < best_score:
            best_score = score
            best_index = index
    return matrices[best_index].copy()


def robust_stats(values: Iterable[float]) -> dict:
    array = np.asarray(list(values), dtype=np.float64)
    if array.size == 0:
        return {
            "count": 0,
            "min": None,
            "median": None,
            "mean": None,
            "p90": None,
            "max": None,
            "mad": None,
        }
    median = float(np.median(array))
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "median": median,
        "mean": float(np.mean(array)),
        "p90": float(np.percentile(array, 90)),
        "max": float(np.max(array)),
        "mad": float(np.median(np.abs(array - median))),
    }
