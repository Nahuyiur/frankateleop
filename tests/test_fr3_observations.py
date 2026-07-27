from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TELEOP_ROOT = REPO_ROOT / "teleop"
if str(TELEOP_ROOT) not in sys.path:
    sys.path.insert(0, str(TELEOP_ROOT))

from teleop.robots.fr3 import fr3Robot  # noqa: E402


class _FakeTensor:
    def __init__(self, value: np.ndarray) -> None:
        self._value = np.asarray(value)

    def detach(self) -> "_FakeTensor":
        return self

    def cpu(self) -> "_FakeTensor":
        return self

    def numpy(self) -> np.ndarray:
        return self._value.copy()


class _FakeRobotInterface:
    def get_joint_velocities(self) -> _FakeTensor:
        return _FakeTensor(np.arange(7, dtype=np.float64) + 0.25)

    def get_ee_pose(self) -> tuple[_FakeTensor, _FakeTensor]:
        return (
            _FakeTensor(np.array([0.1, 0.2, 0.3], dtype=np.float64)),
            _FakeTensor(np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)),
        )


def test_fr3_observations_report_real_arm_velocity() -> None:
    robot = fr3Robot.__new__(fr3Robot)
    robot.robot = _FakeRobotInterface()
    robot._max_gripper_width = 0.08
    robot._last_gripper_command_raw = 0.2
    robot._last_gripper_command = 0.2
    robot._last_gripper_target_width = 0.064
    robot._last_gripper_command_timestamp = 123.0
    robot._last_gripper_command_source = "test"
    joint_state = np.arange(8, dtype=np.float32) / 10.0
    robot.get_joint_state = lambda: joint_state.copy()

    observations = robot.get_observations()

    np.testing.assert_array_equal(observations["joint_positions"], joint_state)
    np.testing.assert_allclose(
        observations["joint_velocities"],
        np.arange(7, dtype=np.float32) + 0.25,
    )
    assert observations["joint_velocities"].dtype == np.float32
    assert observations["joint_velocities"].shape == (7,)


def test_right_runtime_sync_includes_robot_node_dispatch() -> None:
    launcher = (REPO_ROOT / "15_record_bi_arm_pipeline.sh").read_text(encoding="utf-8")
    assert "teleop/teleop/zmq_core/robot_node.py" in launcher
