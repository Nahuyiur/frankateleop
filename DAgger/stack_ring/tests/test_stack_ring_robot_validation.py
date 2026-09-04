from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

import stack_ring_hover_validate as hover
import stack_ring_retarget as mapping


class MappingMathTest(unittest.TestCase):
    def test_affine_relative_mapping_cancels_bias(self):
        model = {"matrix": [[2.0, 0.0, 100.0], [0.0, 3.0, -50.0]]}
        source = np.asarray([[1.0, 2.0], [4.0, 5.0]])
        current = source + np.asarray([[0.5, -0.25], [-0.5, 0.25]])
        source_xy = mapping.apply_mapping(model, source)
        current_xy = mapping.apply_mapping(model, current)
        np.testing.assert_allclose(current_xy - source_xy, [[1.0, -0.75], [-1.0, 0.75]])

    def test_hover_plan_preserves_current_orientation_by_default(self):
        current = np.asarray([0.56, 0.00, 0.62, 3.0, 0.1, -0.1])
        mapped = np.asarray([0.52, 0.20, 0.28, 2.8, -0.3, 0.2])
        plan, phases = hover.build_hover_plan(
            current, mapped, clearance=0.12, minimum_transit_z=0.55,
            xyz_step=0.005, rotation_step=0.04, use_anchor_orientation=False,
        )
        np.testing.assert_allclose(plan[-1, :2], mapped[:2])
        self.assertAlmostEqual(plan[-1, 2], 0.40)
        np.testing.assert_allclose(plan[-1, 3:], current[3:])
        self.assertEqual([phase["name"] for phase in phases], [
            "vertical_lift", "planar_transit", "orientation", "hover_descent"
        ])

    def test_execute_requires_confirmation_token(self):
        self.assertEqual(hover.CONFIRM_TOKEN, "STACK_RING_HOVER")


if __name__ == "__main__":
    unittest.main()
