from __future__ import annotations

import gzip
import json
from pathlib import Path
import pickle

import numpy as np
import pytest

pytest.importorskip("cv2")

from data_review.frame_provider import EpisodeFrameProvider
import data_review.model as review_model
from data_review.model import load_episode_review


CAMERAS = ("left_wrist", "left", "middle")


def _write_episode(
    root: Path,
    frames: list[dict],
    *,
    metadata: dict,
    keyframes=None,
) -> Path:
    episode_dir = root / "task" / "High_Quality" / "0"
    episode_dir.mkdir(parents=True)
    pkl_path = episode_dir / "0.pkl.gz"
    with gzip.open(pkl_path, "wb") as handle:
        pickle.dump({"data": frames, "keyframes": keyframes or []}, handle)
    (episode_dir / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8"
    )
    return episode_dir


def _images(value: int) -> dict[str, np.ndarray]:
    return {
        f"{name}_image": np.full((12, 16, 3), value, dtype=np.uint8)
        for name in CAMERAS
    }


def test_single_episode_review_keeps_views_operator_and_diagnostics(tmp_path: Path) -> None:
    frames = []
    for index in range(3):
        joint = np.zeros(7, dtype=np.float64)
        joint[3] = -1.5
        joint[0] = (0.0, 0.05, 0.30)[index]
        frames.append(
            {
                "timestamp": 10.0 + index / 30.0,
                "joint": joint,
                "pose": np.asarray([index * 0.01, 0.1, 0.2, 0, 0, 0]),
                "gripper_width": 0.07 - index * 0.01,
                "gripper_target_width": 0.06,
                **_images(20 + index),
            }
        )
    episode_dir = _write_episode(
        tmp_path,
        frames,
        metadata={
            "schema_version": "franka_single_v2",
            "camera_names": list(CAMERAS),
            "arm_side": "left",
            "operator_name": "Alice",
            "video_fps": 30,
        },
        keyframes=[1, "bad", 99],
    )

    review = load_episode_review(episode_dir)
    assert review.camera_names == CAMERAS
    assert review.metadata["operator_name"] == "Alice"
    assert review.validator_report.path == str(episode_dir)
    assert review.keyframes == (1,)
    assert tuple(review.arms) == ("left",)
    assert any(event.kind == "joint_delta" and event.severity == "fail" for event in review.events)

    provider = EpisodeFrameProvider(review)
    try:
        images = provider.read(2)
        assert tuple(images) == CAMERAS
        assert all(image.shape == (12, 16, 3) for image in images.values())
    finally:
        provider.close()


def test_dual_episode_review_exposes_both_joint_trajectories(tmp_path: Path) -> None:
    frames = []
    for index in range(2):
        frames.append(
            {
                "timestamp": 20.0 + index / 30.0,
                "left_joint": np.full(7, index * 0.01),
                "right_joint": np.full(7, index * 0.02),
                "left_pose": np.asarray([0.1, 0.2, 0.3, 0, 0, 0]),
                "right_pose": np.asarray([0.2, 0.1, 0.3, 0, 0, 0]),
                **_images(40 + index),
            }
        )
    episode_dir = _write_episode(
        tmp_path,
        frames,
        metadata={"schema_version": "franka_dual_v2", "camera_names": list(CAMERAS)},
    )

    review = load_episode_review(episode_dir)
    assert tuple(review.arms) == ("left", "right")
    np.testing.assert_allclose(review.arms["right"].joints[1], 0.02)


def test_review_rejects_declared_camera_without_video_or_embedded_frames(tmp_path: Path) -> None:
    frames = [{"timestamp": 1.0, "joint": np.zeros(7)}]
    episode_dir = _write_episode(
        tmp_path,
        frames,
        metadata={"schema_version": "franka_single_v3", "camera_names": ["middle"]},
    )
    with pytest.raises(ValueError, match="missing declared camera streams: middle"):
        load_episode_review(episode_dir)


def test_review_displays_full_validator_failure_separately_from_joint_events(
    tmp_path: Path,
) -> None:
    valid_joint = np.asarray([0, 0, 0, -1.5, 0, 1.5, 0], dtype=np.float64)
    frames = [
        {"timestamp": 1.0, "joint": valid_joint.copy(), **_images(10)},
        {"timestamp": 1.0 + 1 / 30, "joint": valid_joint.copy(), **_images(11)},
    ]
    episode_dir = _write_episode(
        tmp_path,
        frames,
        metadata={
            "schema_version": "franka_single_v2",
            "camera_names": list(CAMERAS),
            "arm_side": "left",
            "frame_count": 99,
        },
    )
    review = load_episode_review(episode_dir)
    assert review.events == ()
    assert review.validator_report.status == "FAIL"
    assert any(
        issue.code == "metadata_frame_count_mismatch"
        for issue in review.validator_report.issues
    )


def test_embedded_camera_corruption_never_reuses_the_previous_frame(tmp_path: Path) -> None:
    frames = [
        {"timestamp": 1.0, "joint": np.zeros(7), **_images(10)},
        {"timestamp": 1.0 + 1 / 30, "joint": np.zeros(7), **_images(11)},
    ]
    frames[1]["middle_image"] = None
    episode_dir = _write_episode(
        tmp_path,
        frames,
        metadata={
            "schema_version": "franka_single_v2",
            "camera_names": list(CAMERAS),
            "arm_side": "left",
        },
    )
    review = load_episode_review(episode_dir)
    provider = EpisodeFrameProvider(review)
    try:
        provider.read(0)
        with pytest.raises(RuntimeError, match="middle.*frame 1"):
            provider.read(1)
    finally:
        provider.close()


def test_large_legacy_payload_is_rejected_even_without_metadata(
    tmp_path: Path, monkeypatch
) -> None:
    episode_dir = _write_episode(
        tmp_path,
        [{"timestamp": 1.0, "joint": np.zeros(7), **_images(10)}],
        metadata={},
    )
    monkeypatch.setattr(
        review_model,
        "_gzip_uncompressed_size",
        lambda _path, **_kwargs: review_model.MAX_SAFE_LEGACY_UNCOMPRESSED_BYTES + 1,
    )
    with pytest.raises(MemoryError, match="PKL 解压数据"):
        load_episode_review(episode_dir)


def test_stale_decoded_frame_is_not_rendered(monkeypatch) -> None:
    pytest.importorskip("PyQt6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt6 import QtWidgets
    from data_review.window import DataReviewWindow

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = DataReviewWindow()
    submitted = []
    window._decode_generation = 4
    window._decode_running = True
    window._pending_frame = 8
    monkeypatch.setattr(window, "_submit_decode", lambda index: submitted.append(index))
    window._apply_decoded_frame(4, 7, {"middle": np.zeros((2, 2, 3), dtype=np.uint8)}, "")
    assert submitted == [8]
    window.close()
    app.processEvents()


def test_review_window_keeps_brand_status_and_playback_contract(monkeypatch) -> None:
    pytest.importorskip("PyQt6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt6 import QtWidgets
    from data_review.window import DataReviewWindow

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = DataReviewWindow()
    window.show()
    app.processEvents()

    assert window.findChild(QtWidgets.QFrame, "AuxHeader") is not None
    assert window.summary_badge.property("status") == "idle"
    assert window.play_btn.accessibleName() == "播放或暂停"
    assert window.step_back_btn.accessibleName() == "上一帧"
    assert window.step_next_btn.accessibleName() == "下一帧"
    assert window.camera_empty.isVisibleTo(window)

    window.close()
    app.processEvents()
