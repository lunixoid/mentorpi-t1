#!/usr/bin/env python3
"""URDF roll/pitch from an up vector (SD012 T5)."""
import os
import sys
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.calibration_file import FACTORY_POSES  # noqa: E402
from mentorpi_calibration.stages.planes import camera_tilt_from_vertical  # noqa: E402
from mentorpi_calibration.stages.tilt import rpy_from_up_keep_yaw  # noqa: E402
from mentorpi_calibration.stages.tilt import up_in_link_from_rpy  # noqa: E402
from mentorpi_calibration.stages.transforms import world_up_in_optical  # noqa: E402


class RpyFromUpKeepYawTest(unittest.TestCase):
    def test_recovers_identity(self):
        up = up_in_link_from_rpy((0.0, 0.0, 0.0))
        roll, pitch, yaw = rpy_from_up_keep_yaw(up, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(roll, 0.0, places=9)
        self.assertAlmostEqual(pitch, 0.0, places=9)
        self.assertAlmostEqual(yaw, 0.0, places=9)

    def test_recovers_generating_rpy_independent_of_yaw(self):
        true_rpy = (0.04, -0.05, 0.12)
        up = up_in_link_from_rpy(true_rpy)
        roll, pitch, yaw = rpy_from_up_keep_yaw(up, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(roll, true_rpy[0], places=9)
        self.assertAlmostEqual(pitch, true_rpy[1], places=9)
        self.assertAlmostEqual(yaw, 0.0, places=9)

        roll, pitch, yaw = rpy_from_up_keep_yaw(up, (0.0, 0.0, true_rpy[2]))
        self.assertAlmostEqual(roll, true_rpy[0], places=9)
        self.assertAlmostEqual(pitch, true_rpy[1], places=9)
        self.assertAlmostEqual(yaw, true_rpy[2], places=9)

    def test_recovers_factory_imu_chip_orientation(self):
        factory = FACTORY_POSES["imu"]["rpy"]
        up = up_in_link_from_rpy(factory)
        roll, pitch, yaw = rpy_from_up_keep_yaw(up, factory)
        self.assertAlmostEqual(roll, factory[0], places=9)
        self.assertAlmostEqual(pitch, factory[1], places=9)
        self.assertAlmostEqual(yaw, factory[2], places=9)


class CameraTiltFromVerticalTest(unittest.TestCase):
    def test_optical_up_recovers_mount_roll_pitch(self):
        xyz = FACTORY_POSES["depth_cam"]["xyz"]
        true_rpy = (0.04, -0.05, 0.12)
        up_opt = world_up_in_optical(true_rpy)
        recovered_xyz, recovered_rpy = camera_tilt_from_vertical(
            up_opt,
            (xyz, (0.0, 0.0, 0.0)),
        )
        self.assertEqual(recovered_xyz, xyz)
        self.assertAlmostEqual(recovered_rpy[0], true_rpy[0], places=9)
        self.assertAlmostEqual(recovered_rpy[1], true_rpy[1], places=9)
        self.assertAlmostEqual(recovered_rpy[2], 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
