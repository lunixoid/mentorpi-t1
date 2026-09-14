#!/usr/bin/env python3
"""2-D scan matching helpers (SD012 T7)."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.stages.drive import DriveScan  # noqa: E402
from mentorpi_calibration.stages.scan_match import (  # noqa: E402
    apply_se2,
    build_scan_trajectory,
    compose_se2,
    downsample_xy,
    icp_se2,
)


def _se2_inverse(x, y, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (-cosine * x - sine * y, sine * x - cosine * y, -yaw)


class ScanMatchTest(unittest.TestCase):
    def test_icp_recovers_known_se2(self):
        rng = np.random.default_rng(0)
        xs = np.linspace(-1.5, 1.5, 40)
        wall_a = np.column_stack((np.full(40, 1.6), xs))
        wall_b = np.column_stack((xs, np.full(40, -1.2)))
        target = np.concatenate((wall_a, wall_b), axis=0)
        target += rng.normal(0.0, 0.002, size=target.shape)
        true_pose = (0.06, -0.03, 0.18)
        source = apply_se2(target, _se2_inverse(*true_pose))
        match = icp_se2(source, target)
        self.assertTrue(match.ok)
        self.assertAlmostEqual(match.pose[0], true_pose[0], delta=0.01)
        self.assertAlmostEqual(match.pose[1], true_pose[1], delta=0.01)
        self.assertAlmostEqual(match.pose[2], true_pose[2], delta=0.02)

    def test_icp_identity_on_same_cloud(self):
        xy = np.column_stack((np.linspace(-1.0, 1.0, 50), np.full(50, 1.4)))
        match = icp_se2(xy, xy.copy())
        self.assertTrue(match.ok)
        self.assertAlmostEqual(match.pose[0], 0.0, delta=1e-3)
        self.assertAlmostEqual(match.pose[1], 0.0, delta=1e-3)
        self.assertAlmostEqual(match.pose[2], 0.0, delta=1e-3)

    def test_trajectory_travel_and_turn(self):
        rng = np.random.default_rng(1)
        xs = np.linspace(-1.8, 1.8, 50)
        cloud = np.concatenate(
            (
                np.column_stack((np.full(50, 1.8), xs)),
                np.column_stack((xs, np.full(50, 1.8))),
            ),
            axis=0,
        )
        poses = [(0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.10, 0.0, 0.0), (0.10, 0.0, 0.2)]
        scans = []
        for index, pose in enumerate(poses):
            xy = apply_se2(cloud, _se2_inverse(*pose))
            xy = xy + rng.normal(0.0, 0.001, size=xy.shape)
            scans.append(DriveScan(xy=xy, stamp=index * 0.1))
        trajectory = build_scan_trajectory(scans)
        self.assertTrue(trajectory.ok, trajectory.detail)
        self.assertGreater(trajectory.travel_m, 0.08)
        self.assertGreater(trajectory.turn_deg, 8.0)
        last = trajectory.poses[-1]
        self.assertAlmostEqual(last.x, 0.10, delta=0.03)
        self.assertAlmostEqual(last.yaw, 0.2, delta=0.05)

    def test_downsample_caps_points(self):
        pts = np.random.default_rng(2).uniform(-2.0, 2.0, size=(4000, 2))
        down = downsample_xy(pts)
        self.assertLessEqual(down.shape[0], 120)
        self.assertGreater(down.shape[0], 10)

    def test_compose_se2_associative_on_axis(self):
        left = (0.1, 0.0, 0.3)
        right = (0.2, 0.0, -0.1)
        composed = compose_se2(left, right)
        self.assertAlmostEqual(composed[2], 0.2, delta=1e-9)


if __name__ == "__main__":
    unittest.main()
