"""Synthetic checks for multi-camera world-frame point-cloud fusion."""

from __future__ import annotations

import gzip
import pickle
import tempfile
from pathlib import Path

import numpy as np

from .fuse_world_pointclouds import fuse_episode
from .io import write_json


def _write_episode(root: Path) -> Path:
    episode = root / "episode" / "0"
    episode.mkdir(parents=True)
    intrinsics = {"width": 4, "height": 3, "ppx": 0.0, "ppy": 0.0, "fx": 2.0, "fy": 2.0}
    metadata = {
        "camera_names": ["cam_a", "cam_b"],
        "depth_recording": {"aligned_to": "color"},
        "cameras": {
            "cam_a": {
                "name": "cam_a",
                "serial_number": "A",
                "depth_scale": 0.01,
                "align_depth": True,
                "flip": False,
                "intrinsics": dict(intrinsics),
            },
            "cam_b": {
                "name": "cam_b",
                "serial_number": "B",
                "depth_scale": 0.01,
                "align_depth": True,
                "flip": False,
                "intrinsics": dict(intrinsics),
            },
        },
    }
    write_json(episode / "metadata.json", metadata)

    depth = np.ones((3, 4), dtype=np.float32)
    frame = {
        "frame_index": 0,
        "cam_a_image": np.zeros((3, 4, 3), dtype=np.uint8) + np.array([10, 20, 30], dtype=np.uint8),
        "cam_a_depth": depth,
        "cam_a_depth_scale": 0.01,
        "cam_b_image": np.zeros((3, 4, 3), dtype=np.uint8) + np.array([40, 50, 60], dtype=np.uint8),
        "cam_b_depth": depth,
        "cam_b_depth_scale": 0.01,
    }
    with gzip.open(episode / "0.pkl.gz", "wb") as f:
        pickle.dump({"data": [frame]}, f)
    return episode


def _write_extrinsics(root: Path) -> dict[str, Path]:
    extrinsics = {}
    for camera_name, serial, tx in (("cam_a", "A", 0.0), ("cam_b", "B", 2.0)):
        session = root / f"{camera_name}_calib"
        session.mkdir(parents=True)
        result = session / "calibration_result.json"
        write_json(
            result,
            {
                "calibration_type": "eye_to_hand",
                "world_frame": "robot_base",
                "camera_frame": "camera_color_optical",
                "camera_name": camera_name,
                "camera_metadata": {
                    "serial_number": serial,
                    "intrinsics": {"width": 4, "height": 3, "ppx": 0.0, "ppy": 0.0, "fx": 2.0, "fy": 2.0},
                },
                "provisional": False,
                "T_base_camera": [
                    [1.0, 0.0, 0.0, tx],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
        )
        write_json(session / "validation_report.json", {"pass": True, "validation_sample_mode": "synthetic"})
        write_json(session / "doctor_report.json", {"session": {"status": "PASS"}})
        extrinsics[camera_name] = result
    return extrinsics


def test_fusion() -> None:
    with tempfile.TemporaryDirectory(prefix="frankateleop_fusion_test_") as tmp:
        root = Path(tmp)
        episode = _write_episode(root)
        extrinsics = _write_extrinsics(root)
        summary = fuse_episode(
            episode,
            extrinsics,
            frame_spec="0",
            stride=1,
            voxel_size_m=0.0,
            max_points_per_camera=1000,
            max_fused_points=1000,
        )
        if summary["point_count"] != 24:
            raise AssertionError(f"expected 24 fused points, got {summary['point_count']}")
        if summary["camera_names_fused"] != ["cam_a", "cam_b"]:
            raise AssertionError(summary["camera_names_fused"])
        x_bounds = summary["bounds_xyz_m"]["x"]
        if x_bounds[0] < -1e-6 or x_bounds[1] < 3.4:
            raise AssertionError(f"unexpected fused x bounds: {x_bounds}")
        ply = Path(summary["ply"])
        lines = ply.read_text(encoding="ascii").splitlines()
        if f"element vertex {summary['point_count']}" not in lines[:8]:
            raise AssertionError("PLY header point count mismatch")


def test_camera_mismatch_fails() -> None:
    with tempfile.TemporaryDirectory(prefix="frankateleop_fusion_test_") as tmp:
        root = Path(tmp)
        episode = _write_episode(root)
        extrinsics = _write_extrinsics(root)
        wrong = root / "wrong.json"
        write_json(
            wrong,
            {
                "world_frame": "robot_base",
                "camera_frame": "camera_color_optical",
                "camera_name": "not_cam_a",
                "T_base_camera": [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
        )
        try:
            fuse_episode(
                episode,
                {"cam_a": wrong},
                camera_names_spec="cam_a",
                require_validated_extrinsics=False,
            )
        except RuntimeError as exc:
            if "not_cam_a" not in str(exc):
                raise
        else:
            raise AssertionError("camera mismatch unexpectedly passed")


def main() -> None:
    test_fusion()
    test_camera_mismatch_fails()
    print("SYNTHETIC_FUSION_TEST_OK")


if __name__ == "__main__":
    main()
