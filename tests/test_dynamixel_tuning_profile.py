import tempfile
import unittest
from pathlib import Path

from teleop.dynamixel.tuning_profile import (
    BinarySearchBounds,
    ProfileError,
    load_profile,
    nearest_safe_goals,
    save_profile,
    validate_profile,
)


def profile():
    return {
        "version": 1,
        "torque_enable": [1, 0],
        "goal_current_ma": [80, 120],
        "kp_p": [30, 60],
        "kp_i": [0, 0],
        "kp_d": [250, 100],
        "goal_position": [0.1, -0.2],
    }


class DynamixelTuningProfileTest(unittest.TestCase):
    def test_profile_round_trip_is_validated_and_atomic(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.json"
            save_profile(str(path), profile(), joint_count=2)
            self.assertEqual(load_profile(str(path), joint_count=2), profile())
            self.assertTrue(path.read_text(encoding="utf-8").endswith("\n"))

    def test_rejects_current_over_connected_device_limit(self):
        with self.assertRaisesRegex(ProfileError, "exceeds device Current Limit"):
            validate_profile(profile(), joint_count=2, max_current_ma=[50, 200])

    def test_nearest_goal_wraps_turn_and_rejects_large_start_error(self):
        wrapped = nearest_safe_goals([6.2], [-0.05], max_delta_rad=0.2)
        self.assertAlmostEqual(wrapped[0], 2.0 * 3.141592653589793 - 0.05)
        with self.assertRaisesRegex(ProfileError, "capture the current pose"):
            nearest_safe_goals([0.0], [0.5], max_delta_rad=0.2)

    def test_binary_search_updates_bounds_without_skipping_endpoints(self):
        bounds = BinarySearchBounds(0, 100)
        self.assertEqual(bounds.midpoint, 50)
        self.assertEqual(bounds.too_strong(), 24)
        self.assertEqual((bounds.low, bounds.high), (0, 49))
        self.assertEqual(bounds.too_weak(), 37)
        self.assertEqual((bounds.low, bounds.high), (25, 49))

if __name__ == "__main__":
    unittest.main()
