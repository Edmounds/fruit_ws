"""Stateless mathematical unit tests for camera target and offset transforms."""

from __future__ import annotations

import math
import unittest

from fruit_mission.camera_target_transform import (
    apply_target_offset_mm,
)


class TestCameraTargetTransform(unittest.TestCase):
    """Verify end-effector relative target offset and base frame projection."""

    def test_zero_offset(self) -> None:
        """Zero offset should preserve base coordinates exactly."""
        base_xyz = (100.0, 200.0, 300.0)
        offset_m = (0.0, 0.0, 0.0)
        result = apply_target_offset_mm(base_xyz, offset_m, 0.5)
        self.assertAlmostEqual(result[0], 100.0, places=5)
        self.assertAlmostEqual(result[1], 200.0, places=5)
        self.assertAlmostEqual(result[2], 300.0, places=5)

    def test_acceptance_left_target(self) -> None:
        """Left target (100, 200, 300) mm with pitch=0 and offset (0.03, 0, -0.015) m."""
        base_xyz = (100.0, 200.0, 300.0)
        offset_m = (0.03, 0.0, -0.015)
        # yaw = atan2(200, 100) -> cos = 1/sqrt(5), sin = 2/sqrt(5)
        # pitch = 0 -> delta_x = 30 * cos(yaw), delta_y = 30 * sin(yaw), delta_z = -15
        expected_x = 100.0 + 30.0 / math.sqrt(5.0)  # ~113.416408
        expected_y = 200.0 + 60.0 / math.sqrt(5.0)  # ~226.832816
        expected_z = 300.0 - 15.0                   # 285.0

        result = apply_target_offset_mm(base_xyz, offset_m, 0.0)
        self.assertAlmostEqual(result[0], expected_x, places=5)
        self.assertAlmostEqual(result[1], expected_y, places=5)
        self.assertAlmostEqual(result[2], expected_z, places=5)

        self.assertAlmostEqual(result[0], 113.416408, places=4)
        self.assertAlmostEqual(result[1], 226.832816, places=4)
        self.assertAlmostEqual(result[2], 285.0, places=4)

    def test_acceptance_right_target(self) -> None:
        """Symmetric right target (100, -200, 300) mm with same offset."""
        base_xyz = (100.0, -200.0, 300.0)
        offset_m = (0.03, 0.0, -0.015)
        expected_x = 100.0 + 30.0 / math.sqrt(5.0)   # ~113.416408
        expected_y = -200.0 - 60.0 / math.sqrt(5.0)  # ~-226.832816
        expected_z = 285.0

        result = apply_target_offset_mm(base_xyz, offset_m, 0.0)
        self.assertAlmostEqual(result[0], expected_x, places=5)
        self.assertAlmostEqual(result[1], expected_y, places=5)
        self.assertAlmostEqual(result[2], expected_z, places=5)

        self.assertAlmostEqual(result[0], 113.416408, places=4)
        self.assertAlmostEqual(result[1], -226.832816, places=4)
        self.assertAlmostEqual(result[2], 285.0, places=4)

    def test_acceptance_pitch_coupling(self) -> None:
        """Coupling test: yaw=0, pitch=-45 deg rotates X and Z offset together."""
        base_xyz = (100.0, 0.0, 0.0)
        offset_m = (0.03, 0.0, -0.015)
        pitch_rad = -math.pi / 4.0  # -45 degrees

        # Hand calculation:
        # cp = cos(-pi/4) = 1/sqrt(2), sp = sin(-pi/4) = -1/sqrt(2)
        # radial = cp * 30 - sp * (-15) = (30 - 15) / sqrt(2) = 15 / sqrt(2) ~ 10.606602
        # delta_x = radial ~ 10.606602
        # delta_y = 0.0
        # delta_z = sp * 30 + cp * (-15) = (-30 - 15) / sqrt(2) = -45 / sqrt(2) ~ -31.819805
        expected_delta_x = 15.0 / math.sqrt(2.0)
        expected_delta_y = 0.0
        expected_delta_z = -45.0 / math.sqrt(2.0)

        result = apply_target_offset_mm(base_xyz, offset_m, pitch_rad)
        self.assertAlmostEqual(result[0] - base_xyz[0], expected_delta_x, places=5)
        self.assertAlmostEqual(result[1] - base_xyz[1], expected_delta_y, places=5)
        self.assertAlmostEqual(result[2] - base_xyz[2], expected_delta_z, places=5)

        self.assertAlmostEqual(result[0] - base_xyz[0], 10.606602, places=4)
        self.assertAlmostEqual(result[1] - base_xyz[1], 0.0, places=4)
        self.assertAlmostEqual(result[2] - base_xyz[2], -31.819805, places=4)

    def test_positive_pitch_rotation(self) -> None:
        """Positive pitch (+30 deg) rotates forward +X down into -Z and up into +X."""
        base_xyz = (200.0, 0.0, 50.0)
        offset_m = (0.03, 0.0, -0.015)
        pitch_rad = math.pi / 6.0  # +30 deg: cp = sqrt(3)/2, sp = 0.5
        # radial = 30 * (sqrt(3)/2) - (-15) * 0.5 = 15 * sqrt(3) + 7.5
        # delta_z = 30 * 0.5 + (-15) * (sqrt(3)/2) = 15 - 7.5 * sqrt(3)
        expected_x = 200.0 + 15.0 * math.sqrt(3.0) + 7.5
        expected_y = 0.0
        expected_z = 50.0 + 15.0 - 7.5 * math.sqrt(3.0)

        result = apply_target_offset_mm(base_xyz, offset_m, pitch_rad)
        self.assertAlmostEqual(result[0], expected_x, places=5)
        self.assertAlmostEqual(result[1], expected_y, places=5)
        self.assertAlmostEqual(result[2], expected_z, places=5)

    def test_signed_lateral_y_not_auto_flipped(self) -> None:
        """Local +Y represents gripper left without automatic sign inversion."""
        offset_m = (0.0, 0.03, 0.0)  # 30 mm local left

        # Target at +90 deg (yaw = pi/2): gripper facing +Y in base.
        # Local +Y (left) should point towards robot -X.
        base_left = (0.0, 100.0, 50.0)
        result_left = apply_target_offset_mm(base_left, offset_m, 0.0)
        self.assertAlmostEqual(result_left[0], -30.0, places=5)
        self.assertAlmostEqual(result_left[1], 100.0, places=5)
        self.assertAlmostEqual(result_left[2], 50.0, places=5)

        # Target at -90 deg (yaw = -pi/2): gripper facing -Y in base.
        # Local +Y (left) should point towards robot +X.
        base_right = (0.0, -100.0, 50.0)
        result_right = apply_target_offset_mm(base_right, offset_m, 0.0)
        self.assertAlmostEqual(result_right[0], 30.0, places=5)
        self.assertAlmostEqual(result_right[1], -100.0, places=5)
        self.assertAlmostEqual(result_right[2], 50.0, places=5)

    def test_negative_x_and_z(self) -> None:
        """Retracting (-X) and pushing down (-Z) on forward target."""
        base_xyz = (300.0, 0.0, 100.0)
        offset_m = (-0.02, 0.0, -0.01)  # -20 mm X, -10 mm Z
        result = apply_target_offset_mm(base_xyz, offset_m, 0.0)
        self.assertAlmostEqual(result[0], 280.0, places=5)
        self.assertAlmostEqual(result[1], 0.0, places=5)
        self.assertAlmostEqual(result[2], 90.0, places=5)

    def test_non_orthogonal_target_direction(self) -> None:
        """Target at (300, 400, 100) mm with non-zero yaw, pitch, and full 3D offset."""
        base_xyz = (300.0, 400.0, 100.0)
        offset_m = (0.05, 0.02, -0.01)
        pitch_rad = -0.2

        # Independent analytical computation:
        # yaw = atan2(400, 300) -> cy = 3/5 = 0.6, sy = 4/5 = 0.8
        cy, sy = 0.6, 0.8
        cp = math.cos(-0.2)
        sp = math.sin(-0.2)
        xl, yl, zl = 50.0, 20.0, -10.0
        radial = cp * xl - sp * zl
        dx = cy * radial - sy * yl
        dy = sy * radial + cy * yl
        dz = sp * xl + cp * zl

        expected = (300.0 + dx, 400.0 + dy, 100.0 + dz)
        result = apply_target_offset_mm(base_xyz, offset_m, pitch_rad)
        self.assertAlmostEqual(result[0], expected[0], places=5)
        self.assertAlmostEqual(result[1], expected[1], places=5)
        self.assertAlmostEqual(result[2], expected[2], places=5)

    def test_no_duplicate_translation(self) -> None:
        """Base position is used only as translation origin, not added twice."""
        base_xyz = (500.0, 500.0, 500.0)
        offset_m = (0.0, 0.0, 0.0)
        result = apply_target_offset_mm(base_xyz, offset_m, 0.0)
        self.assertEqual(result, (500.0, 500.0, 500.0))

    def test_invalid_inputs_rejected(self) -> None:
        """Validate rejection of malformed, non-finite, and boolean inputs."""
        valid_base = (100.0, 100.0, 100.0)
        valid_offset = (0.0, 0.0, 0.0)
        valid_pitch = 0.0

        # Wrong length or type
        with self.assertRaises(ValueError):
            apply_target_offset_mm((100.0, 100.0), valid_offset, valid_pitch)
        with self.assertRaises(ValueError):
            apply_target_offset_mm(valid_base, (0.0, 0.0), valid_pitch)
        with self.assertRaises(ValueError):
            apply_target_offset_mm(None, valid_offset, valid_pitch)
        with self.assertRaises(ValueError):
            apply_target_offset_mm(valid_base, None, valid_pitch)

        # Booleans must not pass as numbers
        with self.assertRaises(ValueError):
            apply_target_offset_mm((True, 100.0, 100.0), valid_offset, valid_pitch)
        with self.assertRaises(ValueError):
            apply_target_offset_mm(valid_base, (0.0, False, 0.0), valid_pitch)
        with self.assertRaises(ValueError):
            apply_target_offset_mm(valid_base, valid_offset, True)

        # NaN and Inf
        with self.assertRaises(ValueError):
            apply_target_offset_mm((float('nan'), 0.0, 0.0), valid_offset, valid_pitch)
        with self.assertRaises(ValueError):
            apply_target_offset_mm(valid_base, (float('inf'), 0.0, 0.0), valid_pitch)
        with self.assertRaises(ValueError):
            apply_target_offset_mm(valid_base, valid_offset, float('nan'))
        with self.assertRaises(ValueError):
            apply_target_offset_mm(valid_base, valid_offset, float('inf'))

        # Non-numeric pitch
        with self.assertRaises(ValueError):
            apply_target_offset_mm(valid_base, valid_offset, '0.0')


if __name__ == '__main__':
    unittest.main()
