from __future__ import annotations

import gzip
import pickle
import sys
import unittest
from copy import deepcopy
from pathlib import Path

import numpy as np
try:
    import pytest
except ImportError as exc:
    raise unittest.SkipTest("pytest is not installed") from exc

from franka_replay import replay_fr3, replay_fr3_dual


def _single_frames() -> list[dict[str, object]]:
    return [
        {
            "pose": np.zeros(6, dtype=float),
            "joint": np.zeros(7, dtype=float),
            "gripper_closedness": 0.25,
            "gripper_01closedness": 0.0,
            "gripper_width": 0.07,
            "gripper_target_width": 0.06,
            "timestamp": 10.0 + index / 30.0,
        }
        for index in range(2)
    ]


def _dual_frames() -> list[dict[str, object]]:
    frames: list[dict[str, object]] = []
    for index in range(2):
        frame: dict[str, object] = {"timestamp": 20.0 + index / 30.0}
        for arm in ("left", "right"):
            frame.update(
                {
                    f"{arm}_pose": np.zeros(6, dtype=float),
                    f"{arm}_joint": np.zeros(7, dtype=float),
                    f"{arm}_gripper_closedness": 0.25,
                    f"{arm}_gripper_width": 0.07,
                    f"{arm}_gripper_target_width": 0.06,
                }
            )
        frames.append(frame)
    return frames


@pytest.mark.parametrize("bad_value", [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize(
    "field,index",
    [
        ("pose", 2),
        ("joint", 3),
        ("timestamp", None),
        ("gripper_closedness", None),
        ("gripper_01closedness", None),
        ("gripper_width", None),
        ("gripper_target_width", None),
        ("gripper_command_raw", None),
        ("gripper", None),
    ],
)
def test_single_replay_rejects_nonfinite_recorded_values(
    field: str, index: int | None, bad_value: float
) -> None:
    frames = deepcopy(_single_frames())
    if index is None:
        frames[1][field] = bad_value
    else:
        value = np.asarray(frames[1][field], dtype=float).copy()
        value[index] = bad_value
        frames[1][field] = value

    with pytest.raises(ValueError, match="NaN or Inf"):
        replay_fr3.extract_trajectory(frames)


@pytest.mark.parametrize("arm", ["left", "right"])
@pytest.mark.parametrize("field,index", [("pose", 1), ("joint", 4), ("gripper_width", None)])
def test_dual_replay_rejects_nonfinite_arm_values(
    arm: str, field: str, index: int | None
) -> None:
    frames = deepcopy(_dual_frames())
    key = f"{arm}_{field}"
    if index is None:
        frames[0][key] = np.nan
    else:
        value = np.asarray(frames[0][key], dtype=float).copy()
        value[index] = np.inf
        frames[0][key] = value

    timestamps = replay_fr3_dual.extract_timestamps(frames)
    with pytest.raises(ValueError, match="NaN or Inf"):
        replay_fr3_dual.extract_dual_trajectories(frames, timestamps, 0.01)


def test_dual_replay_rejects_nonfinite_timestamp() -> None:
    frames = _dual_frames()
    frames[0]["timestamp"] = np.nan

    with pytest.raises(ValueError, match="NaN or Inf"):
        replay_fr3_dual.extract_timestamps(frames)


@pytest.mark.parametrize(
    "module,frames",
    [(replay_fr3, _single_frames()), (replay_fr3_dual, _dual_frames())],
)
def test_main_rejects_nonfinite_episode_before_robot_connection(
    module: object,
    frames: list[dict[str, object]],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    frames[0]["timestamp"] = np.nan
    episode_path = tmp_path / "0.pkl.gz"
    with gzip.open(episode_path, "wb") as handle:
        pickle.dump({"data": frames}, handle, protocol=pickle.HIGHEST_PROTOCOL)

    connected = False

    class ForbiddenRobotClient:
        def __init__(self, *args: object, **kwargs: object) -> None:
            nonlocal connected
            connected = True
            raise AssertionError("robot connection must not be attempted")

    monkeypatch.setattr(module, "RobotZMQReplayClient", ForbiddenRobotClient)
    monkeypatch.setattr(sys, "argv", [module.__name__, str(episode_path)])

    with pytest.raises(SystemExit) as exc_info:
        module.main()

    assert exc_info.value.code == 1
    assert not connected
