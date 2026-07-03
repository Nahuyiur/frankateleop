"""Interactively capture ChArUco hand-eye calibration samples.

This script only reads robot state. It never sends robot motion commands.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np

from franka_capture.cameras.realsense import create_realsense_cameras
from franka_capture.config.fr3_single import DEFAULT_CAMERAS
from franka_capture.core.robot_zmq_client import RobotZMQClient
from pointcloud.depth_proof import write_metric_depth_png

from .detect_charuco import detect_charuco_pose, draw_detection_overlay
from .geometry import matrix_to_list, pose_euler_xyz_to_transform, transform_error_degrees
from .io import SCHEMA_VERSION, create_session_dir, next_sample_dir, write_json
from .targets import normalize_board_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="middle")
    parser.add_argument("--output-root", default=str(Path.home() / "Desktop" / "franka_record_data" / "calibration"))
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=6001)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--min-corners", type=int, default=12)
    parser.add_argument("--dictionary", default=None)
    parser.add_argument("--squares-x", type=int, default=None)
    parser.add_argument("--squares-y", type=int, default=None)
    parser.add_argument("--square-length-m", type=float, default=None)
    parser.add_argument("--marker-length-m", type=float, default=None)
    return parser.parse_args()


def _camera_config(camera_name: str, fps: int, width: int, height: int):
    if camera_name not in DEFAULT_CAMERAS:
        raise ValueError(f"Unknown camera {camera_name}; available: {sorted(DEFAULT_CAMERAS)}")
    return {
        camera_name: replace(
            DEFAULT_CAMERAS[camera_name],
            fps=int(fps),
            dim=(int(width), int(height)),
            depth=True,
            align_depth=True,
        )
    }


def _read_robot_pose(robot: RobotZMQClient) -> Dict[str, Any]:
    observations = robot.get_observations()
    if "ee_pose_euler" not in observations:
        raise RuntimeError("Robot node does not expose ee_pose_euler")
    pose = np.asarray(observations["ee_pose_euler"], dtype=np.float64).reshape(6)
    return {
        "ee_pose_euler": [float(x) for x in pose],
        "T_base_gripper": matrix_to_list(pose_euler_xyz_to_transform(pose)),
    }


def _read_stable_robot_pose(
    robot: RobotZMQClient,
    *,
    checks: int = 3,
    interval_sec: float = 0.05,
    max_translation_m: float = 0.001,
    max_rotation_deg: float = 0.2,
) -> Dict[str, Any]:
    poses = []
    for index in range(max(2, int(checks))):
        poses.append(_read_robot_pose(robot))
        if index + 1 < checks:
            time.sleep(float(interval_sec))
    first = np.asarray(poses[0]["T_base_gripper"], dtype=np.float64)
    max_translation = 0.0
    max_rotation = 0.0
    for pose in poses[1:]:
        translation_m, rotation_deg = transform_error_degrees(first, np.asarray(pose["T_base_gripper"], dtype=np.float64))
        max_translation = max(max_translation, translation_m)
        max_rotation = max(max_rotation, rotation_deg)
    stable = max_translation <= float(max_translation_m) and max_rotation <= float(max_rotation_deg)
    result = dict(poses[-1])
    result["robot_stability"] = {
        "checks": int(checks),
        "interval_sec": float(interval_sec),
        "max_translation_m": float(max_translation),
        "max_rotation_deg": float(max_rotation),
        "stable": bool(stable),
        "threshold_translation_m": float(max_translation_m),
        "threshold_rotation_deg": float(max_rotation_deg),
    }
    if not stable:
        raise RuntimeError(
            "Robot is not still enough to save sample: "
            f"translation={max_translation:.4f}m rotation={max_rotation:.3f}deg"
        )
    return result


def _bracket_stability(
    before_pose: Dict[str, Any],
    after_pose: Dict[str, Any],
    *,
    max_translation_m: float = 0.001,
    max_rotation_deg: float = 0.2,
) -> Dict[str, Any]:
    translation_m, rotation_deg = transform_error_degrees(
        np.asarray(before_pose["T_base_gripper"], dtype=np.float64),
        np.asarray(after_pose["T_base_gripper"], dtype=np.float64),
    )
    stable = translation_m <= float(max_translation_m) and rotation_deg <= float(max_rotation_deg)
    return {
        "frame_bracket_translation_m": float(translation_m),
        "frame_bracket_rotation_deg": float(rotation_deg),
        "frame_bracket_stable": bool(stable),
    }


def main() -> None:
    args = parse_args()
    board_config = normalize_board_config(
        None,
        dictionary=args.dictionary,
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length_m=args.square_length_m,
        marker_length_m=args.marker_length_m,
    )
    session_dir = create_session_dir(Path(args.output_root).expanduser(), args.camera)
    cameras = create_realsense_cameras(_camera_config(args.camera, args.camera_fps, args.width, args.height))
    camera = cameras[args.camera]
    robot: Optional[RobotZMQClient] = None
    try:
        robot = RobotZMQClient(host=args.robot_host, port=args.robot_port)
        camera_metadata = camera.metadata()
        write_json(
            session_dir / "session.json",
            {
                "schema_version": SCHEMA_VERSION,
                "calibration_type": "eye_to_hand",
                "camera_name": args.camera,
                "camera_metadata": camera_metadata,
                "board_config": board_config,
                "robot_pose_convention": "ee_pose_euler=[x,y,z,roll,pitch,yaw], scipy xyz radians",
                "instructions": "Manually move robot; press s to save a still sample; q to quit.",
            },
        )
        print(f"Session: {session_dir}")
        print("Keys: s=save sample, q=quit")
        latest_rgb = None
        latest_depth = None
        latest_detection: Dict[str, Any] = {}
        while True:
            rgb, depth = camera.read()
            latest_rgb = rgb
            latest_depth = depth
            latest_detection = detect_charuco_pose(
                rgb,
                camera_metadata["intrinsics"],
                board_config,
                min_corners=args.min_corners,
                depth_m=depth,
            )
            overlay = draw_detection_overlay(rgb, latest_detection, camera_metadata["intrinsics"])
            cv2.imshow("ChArUco eye-to-hand calibration", overlay)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key not in (ord("s"), ord("S")):
                continue
            if latest_rgb is None or latest_depth is None:
                print("No camera frame yet")
                continue
            try:
                robot_pose_before = _read_stable_robot_pose(robot)
                sample_rgb, sample_depth = camera.read()
                sample_detection = detect_charuco_pose(
                    sample_rgb,
                    camera_metadata["intrinsics"],
                    board_config,
                    min_corners=args.min_corners,
                    depth_m=sample_depth,
                )
                robot_pose_after = _read_robot_pose(robot)
                bracket = _bracket_stability(robot_pose_before, robot_pose_after)
            except RuntimeError as exc:
                print(f"Sample not saved: {exc}")
                continue
            robot_stability = dict(robot_pose_before["robot_stability"])
            robot_stability.update(bracket)
            robot_stability["stable"] = bool(robot_stability.get("stable") and bracket["frame_bracket_stable"])
            if not robot_stability["stable"]:
                print(
                    "Sample not saved: robot moved while bracketing frame "
                    f"translation={bracket['frame_bracket_translation_m']:.4f}m "
                    f"rotation={bracket['frame_bracket_rotation_deg']:.3f}deg"
                )
                continue
            if not sample_detection.get("valid"):
                print(f"Sample not saved: detection invalid after stillness check ({sample_detection.get('reason')})")
                continue
            sample_overlay = draw_detection_overlay(sample_rgb, sample_detection, camera_metadata["intrinsics"])
            robot_pose = dict(robot_pose_after)
            robot_pose["robot_stability"] = robot_stability
            sample_dir = next_sample_dir(session_dir)
            rgb_path = sample_dir / "rgb.png"
            depth_path = sample_dir / "depth.png"
            overlay_path = sample_dir / "detection_overlay.png"
            cv2.imwrite(str(rgb_path), sample_rgb)
            if sample_depth.ndim == 3 and sample_depth.shape[2] == 1:
                sample_depth = sample_depth[:, :, 0]
            depth_scale = float(camera_metadata.get("depth_scale") or 0.001)
            write_metric_depth_png(depth_path, np.asarray(sample_depth, dtype=np.float32), depth_scale)
            cv2.imwrite(str(overlay_path), sample_overlay)
            metadata = {
                "schema_version": SCHEMA_VERSION,
                "sample_id": sample_dir.name,
                "timestamp": time.time(),
                "camera_name": args.camera,
                "rgb_path": rgb_path.name,
                "depth_path": depth_path.name,
                "overlay_path": overlay_path.name,
                "camera_intrinsics": camera_metadata["intrinsics"],
                "depth_scale": depth_scale,
                "board_config": board_config,
                "detection": sample_detection,
                **robot_pose,
            }
            write_json(sample_dir / "metadata.json", metadata)
            quality = sample_detection.get("quality") or {}
            depth_plane = quality.get("depth_plane") or {}
            print(
                f"Saved {sample_dir.name}: corners={sample_detection.get('corner_count')} "
                f"rms={sample_detection.get('reprojection_rms_px')} "
                f"depth_plane_median={depth_plane.get('median_abs_plane_error_m')} "
                f"robot_stable={robot_pose['robot_stability']['stable']}"
            )
    finally:
        cv2.destroyAllWindows()
        for cam in cameras.values():
            cam.close()
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
