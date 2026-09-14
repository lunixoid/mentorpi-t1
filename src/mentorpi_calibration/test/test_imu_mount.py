#!/usr/bin/env python3
"""IMU mount roll/pitch from gravity (SD012 D3.2.5)."""
import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.calibration_file import FACTORY_POSES, mat_vec3, rotation_matrix_from_rpy  # noqa: E402
from mentorpi_calibration.stages.imu_mount import (  # noqa: E402
    DETAIL_IMU_NOT_STILL,
    GRAVITY_MPS2,
    imu_mount_from_gravity,
)
from mentorpi_calibration.stages.tilt import rpy_from_up_keep_yaw  # noqa: E402


def _transpose(
    matrix: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(matrix[j][i] for j in range(3)) for i in range(3))


def _accel_for_mount_rpy(
    mount_rpy: tuple[float, float, float],
    *,
    magnitude: float = GRAVITY_MPS2,
) -> tuple[float, float, float]:
    """Simulate at-rest accel: g * (R^T e_z) in imu_link."""
    r_mount = rotation_matrix_from_rpy(mount_rpy)
    e_z = (0.0, 0.0, 1.0)
    direction = mat_vec3(_transpose(r_mount), e_z)
    return tuple(magnitude * component for component in direction)


def _perturbed_up_from_factory(
    factory_rpy: tuple[float, float, float],
    perturbation: tuple[float, float, float],
) -> np.ndarray:
    up = np.array(_accel_for_mount_rpy(factory_rpy), dtype=np.float64) / GRAVITY_MPS2
    perturbed = up + np.asarray(perturbation, dtype=np.float64)
    return perturbed / np.linalg.norm(perturbed)


class ImuMountFromGravityTest(unittest.TestCase):
    def _factory_pose(self):
        pose = FACTORY_POSES["imu"]
        return pose["xyz"], pose["rpy"]

    def test_recovers_factory_roll_pitch(self):
        factory_xyz, factory_rpy = self._factory_pose()
        accel = _accel_for_mount_rpy(factory_rpy)

        result = imu_mount_from_gravity(accel, (factory_xyz, factory_rpy))
        self.assertTrue(result.ok, result.detail)
        self.assertIsNotNone(result.rpy)
        self.assertIsNotNone(result.xyz)

        recovered_rpy = result.rpy
        recovered_xyz = result.xyz
        self.assertEqual(recovered_xyz, factory_xyz)
        self.assertAlmostEqual(recovered_rpy[0], factory_rpy[0], delta=1e-3)
        self.assertAlmostEqual(recovered_rpy[1], factory_rpy[1], delta=1e-3)
        self.assertAlmostEqual(recovered_rpy[2], factory_rpy[2], delta=1e-6)

    def test_recovers_known_extra_tilt(self):
        factory_xyz, factory_rpy = self._factory_pose()
        perturbed_up = _perturbed_up_from_factory(factory_rpy, (0.03, -0.04, 0.02))
        expected_rpy = rpy_from_up_keep_yaw(perturbed_up, factory_rpy)
        accel = tuple(GRAVITY_MPS2 * component for component in perturbed_up)

        result = imu_mount_from_gravity(accel, (factory_xyz, factory_rpy))
        self.assertTrue(result.ok, result.detail)
        self.assertIsNotNone(result.rpy)

        recovered_rpy = result.rpy
        self.assertAlmostEqual(recovered_rpy[0], expected_rpy[0], delta=1e-3)
        self.assertAlmostEqual(recovered_rpy[1], expected_rpy[1], delta=1e-3)
        self.assertAlmostEqual(recovered_rpy[2], factory_rpy[2], delta=1e-6)
        self.assertNotAlmostEqual(recovered_rpy[0], factory_rpy[0], delta=1e-3)
        self.assertNotAlmostEqual(recovered_rpy[1], factory_rpy[1], delta=1e-3)

    def test_rejects_non_gravity_magnitude(self):
        factory_xyz, factory_rpy = self._factory_pose()
        direction = _accel_for_mount_rpy(factory_rpy, magnitude=1.0)

        too_small = imu_mount_from_gravity(direction, (factory_xyz, factory_rpy))
        self.assertFalse(too_small.ok)
        self.assertEqual(too_small.detail, DETAIL_IMU_NOT_STILL)

        too_large = imu_mount_from_gravity(
            _accel_for_mount_rpy(factory_rpy, magnitude=20.0),
            (factory_xyz, factory_rpy),
        )
        self.assertFalse(too_large.ok)
        self.assertEqual(too_large.detail, DETAIL_IMU_NOT_STILL)


if __name__ == "__main__":
    unittest.main()
