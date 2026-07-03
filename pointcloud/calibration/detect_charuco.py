"""Detect a ChArUco target and estimate target pose in the camera frame."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

from .geometry import matrix_to_list, rodrigues_to_transform, transform_to_rodrigues
from .io import read_json, write_json
from .targets import camera_matrix_from_intrinsics, create_charuco_board, dist_coeffs_from_intrinsics, normalize_board_config


def detect_charuco_pose(
    bgr_image: np.ndarray,
    intrinsics: Dict[str, Any],
    board_config: Optional[Dict[str, Any]] = None,
    *,
    min_corners: int = 12,
    depth_m: Optional[np.ndarray] = None,
) -> Dict[str, Any]:
    board_config = normalize_board_config(board_config)
    board = create_charuco_board(board_config)
    camera_matrix = camera_matrix_from_intrinsics(intrinsics)
    dist_coeffs = dist_coeffs_from_intrinsics(intrinsics)
    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY) if bgr_image.ndim == 3 else bgr_image

    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)

    result: Dict[str, Any] = {
        "valid": False,
        "reason": None,
        "corner_count": 0,
        "marker_count": 0,
        "reprojection_rms_px": None,
        "reprojection_max_px": None,
        "T_camera_target": None,
        "rvec_target2cam": None,
        "tvec_target2cam": None,
        "board_config": board_config,
        "quality": {},
    }

    if marker_ids is not None:
        result["marker_count"] = int(len(marker_ids))
    if charuco_ids is None or charuco_corners is None:
        result["reason"] = "no_charuco_corners"
        return result

    corner_count = int(len(charuco_ids))
    result["corner_count"] = corner_count
    if corner_count < int(min_corners):
        result["reason"] = f"not_enough_corners:{corner_count}<{min_corners}"
        return result

    object_points, image_points = board.matchImagePoints(charuco_corners, charuco_ids)
    object_points = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    image_points = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    result["quality"] = image_quality_metrics(gray, image_points)

    flags = getattr(cv2, "SOLVEPNP_IPPE", cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs, flags=flags)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        result["reason"] = "solve_pnp_failed"
        return result

    if hasattr(cv2, "solvePnPRefineLM"):
        try:
            rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec)
        except cv2.error:
            pass

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - image_points, axis=1)
    transform = rodrigues_to_transform(rvec, tvec)

    result.update(
        {
            "valid": True,
            "reason": "ok",
            "reprojection_rms_px": float(math.sqrt(float(np.mean(errors ** 2)))),
            "reprojection_max_px": float(np.max(errors)),
            "T_camera_target": matrix_to_list(transform),
            "rvec_target2cam": [float(x) for x in np.asarray(rvec).reshape(3)],
            "tvec_target2cam": [float(x) for x in np.asarray(tvec).reshape(3)],
            "charuco_ids": [int(x) for x in np.asarray(charuco_ids).reshape(-1)],
            "charuco_corners": [[float(x), float(y)] for x, y in image_points],
        }
    )
    if depth_m is not None:
        result["quality"]["depth_plane"] = depth_plane_metrics(
            depth_m=np.asarray(depth_m, dtype=np.float32),
            image_points=image_points,
            intrinsics=intrinsics,
            transform_camera_target=transform,
        )
    return result


def image_quality_metrics(gray: np.ndarray, image_points: np.ndarray) -> Dict[str, Any]:
    height, width = gray.shape[:2]
    x0 = float(np.min(image_points[:, 0]))
    x1 = float(np.max(image_points[:, 0]))
    y0 = float(np.min(image_points[:, 1]))
    y1 = float(np.max(image_points[:, 1]))
    bbox_area = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    border_margin = min(x0, y0, float(width - 1) - x1, float(height - 1) - y1)
    return {
        "bbox_area_ratio": float(bbox_area / max(1.0, float(width * height))),
        "border_margin_px": float(border_margin),
        "laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
    }


def depth_plane_metrics(
    *,
    depth_m: np.ndarray,
    image_points: np.ndarray,
    intrinsics: Dict[str, Any],
    transform_camera_target: np.ndarray,
) -> Dict[str, Any]:
    if depth_m.ndim == 3 and depth_m.shape[2] == 1:
        depth_m = depth_m[:, :, 0]
    height, width = depth_m.shape[:2]
    hull = cv2.convexHull(image_points.astype(np.float32)).astype(np.int32)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(mask, hull, 1)
    ys, xs = np.where(mask > 0)
    if xs.size == 0:
        return {"valid_ratio": 0.0, "median_abs_plane_error_m": None, "p90_abs_plane_error_m": None}
    depth_values = depth_m[ys, xs].astype(np.float64)
    valid = np.isfinite(depth_values) & (depth_values > 0)
    valid_ratio = float(valid.mean()) if valid.size else 0.0
    if not np.any(valid):
        return {"valid_ratio": valid_ratio, "median_abs_plane_error_m": None, "p90_abs_plane_error_m": None}

    z = depth_values[valid]
    u = xs[valid].astype(np.float64)
    v = ys[valid].astype(np.float64)
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    ppx = float(intrinsics["ppx"])
    ppy = float(intrinsics["ppy"])
    points = np.column_stack(((u - ppx) * z / fx, (v - ppy) * z / fy, z))
    normal = transform_camera_target[:3, 2]
    normal = normal / max(1e-12, float(np.linalg.norm(normal)))
    point_on_plane = transform_camera_target[:3, 3]
    distances = np.abs((points - point_on_plane.reshape(1, 3)) @ normal)
    return {
        "valid_ratio": valid_ratio,
        "median_abs_plane_error_m": float(np.median(distances)),
        "p90_abs_plane_error_m": float(np.percentile(distances, 90)),
    }


def draw_detection_overlay(
    bgr_image: np.ndarray,
    detection: Dict[str, Any],
    intrinsics: Dict[str, Any],
    *,
    axis_length_m: float = 0.08,
) -> np.ndarray:
    overlay = bgr_image.copy()
    if detection.get("charuco_corners") and detection.get("charuco_ids"):
        corners = np.asarray(detection["charuco_corners"], dtype=np.float32).reshape(-1, 1, 2)
        ids = np.asarray(detection["charuco_ids"], dtype=np.int32).reshape(-1, 1)
        try:
            cv2.aruco.drawDetectedCornersCharuco(overlay, corners, ids)
        except Exception:
            for point in corners.reshape(-1, 2):
                cv2.circle(overlay, (int(point[0]), int(point[1])), 3, (0, 255, 255), -1)

    if detection.get("valid") and detection.get("T_camera_target") is not None:
        rvec, tvec = transform_to_rodrigues(np.asarray(detection["T_camera_target"], dtype=np.float64))
        cv2.drawFrameAxes(
            overlay,
            camera_matrix_from_intrinsics(intrinsics),
            dist_coeffs_from_intrinsics(intrinsics),
            rvec.reshape(3, 1),
            tvec.reshape(3, 1),
            float(axis_length_m),
        )

    status = "valid" if detection.get("valid") else f"invalid:{detection.get('reason')}"
    text = (
        f"ChArUco {status} corners={detection.get('corner_count', 0)} "
        f"rms={detection.get('reprojection_rms_px')}"
    )
    cv2.putText(overlay, text, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(overlay, text, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
    return overlay


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", help="Input RGB image")
    parser.add_argument("--intrinsics-json", required=True, help="JSON containing fx/fy/ppx/ppy")
    parser.add_argument("--output-json", default=None)
    parser.add_argument("--overlay", default=None)
    parser.add_argument("--min-corners", type=int, default=12)
    parser.add_argument("--dictionary", default=None)
    parser.add_argument("--squares-x", type=int, default=None)
    parser.add_argument("--squares-y", type=int, default=None)
    parser.add_argument("--square-length-m", type=float, default=None)
    parser.add_argument("--marker-length-m", type=float, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    image = cv2.imread(str(Path(args.image).expanduser()), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(args.image)
    intrinsics = read_json(Path(args.intrinsics_json).expanduser())
    board_config = normalize_board_config(
        None,
        dictionary=args.dictionary,
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length_m=args.square_length_m,
        marker_length_m=args.marker_length_m,
    )
    detection = detect_charuco_pose(image, intrinsics, board_config, min_corners=args.min_corners)
    if args.output_json:
        write_json(Path(args.output_json), detection)
    if args.overlay:
        overlay = draw_detection_overlay(image, detection, intrinsics)
        Path(args.overlay).expanduser().parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(Path(args.overlay).expanduser()), overlay)
    print(detection)


if __name__ == "__main__":
    main()
