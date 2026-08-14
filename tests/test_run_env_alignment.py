import sys
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "teleop"))

try:
    from experiments.run_env import (  # noqa: E402
        arm_joint_indices,
        limit_arm_step,
        wrap_arm_action_to_nearest,
    )
except ModuleNotFoundError as exc:
    raise unittest.SkipTest(
        f"run_env optional runtime dependency is unavailable: {exc.name}"
    ) from exc


class RunEnvAlignmentTest(unittest.TestCase):
    def test_single_arm_indices_exclude_gripper(self):
        np.testing.assert_array_equal(arm_joint_indices(8), np.arange(7))

    def test_gripper_mismatch_does_not_limit_arm_step(self):
        current = np.array([0.0] * 7 + [1.0])
        command = np.array([0.01] * 7 + [0.197])

        limited = limit_arm_step(command, current, max_delta=0.05)

        np.testing.assert_allclose(limited[:7], command[:7])
        self.assertEqual(limited[-1], command[-1])

    def test_wrap_does_not_modify_gripper(self):
        current = np.array([0.0] * 7 + [1.0])
        action = np.array([2 * np.pi + 0.1] + [0.0] * 6 + [0.197])

        wrapped = wrap_arm_action_to_nearest(action, current)

        self.assertAlmostEqual(wrapped[0], 0.1)
        self.assertEqual(wrapped[-1], action[-1])

    def test_bimanual_indices_exclude_both_grippers(self):
        expected = np.array(list(range(7)) + list(range(8, 15)))
        np.testing.assert_array_equal(
            arm_joint_indices(16, bimanual=True), expected
        )


if __name__ == "__main__":
    unittest.main()
