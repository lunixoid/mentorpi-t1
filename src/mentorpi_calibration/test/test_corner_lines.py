#!/usr/bin/env python3
"""2-D scan line extraction (SD012 T5)."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.stages.scan_lines import DETAIL_ONE_WALL, extract_orthogonal_lines, laserscan_xy  # noqa: E402


def _line_points(normal, rho, count, rng, span=0.8, noise=0.004):
    normal = np.asarray(normal, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    tangent = np.array([-normal[1], normal[0]])
    t = rng.uniform(-span, span, size=count)
    xy = rho * normal + t[:, None] * tangent
    xy += rng.normal(0.0, noise, size=xy.shape)
    z = np.full(count, 0.04)
    return np.column_stack((xy, z))


class ScanLinesTest(unittest.TestCase):
    def setUp(self):
        np.random.seed(7)
        self._rng = np.random.default_rng(7)

    def test_two_orthogonal_lines(self):
        a = _line_points((1.0, 0.0), 1.4, 40, self._rng)
        b = _line_points((0.0, 1.0), 0.9, 40, self._rng)
        result = extract_orthogonal_lines(np.vstack((a, b)))
        self.assertTrue(result.ok, result.detail)
        self.assertIsNotNone(result.first)
        self.assertIsNotNone(result.second)
        dot = abs(float(np.dot(result.first.normal, result.second.normal)))
        self.assertLess(dot, 0.2)

    def test_one_line_fails(self):
        a = _line_points((1.0, 0.0), 1.4, 50, self._rng)
        result = extract_orthogonal_lines(a)
        self.assertFalse(result.ok)
        self.assertEqual(result.detail, DETAIL_ONE_WALL)

    def test_laserscan_xy_drops_beyond_max(self):
        class _Scan:
            ranges = [0.5, 26.0, 1.2, float("inf")]
            angle_min = 0.0
            angle_increment = math.pi / 2.0
            range_min = 0.05
            range_max = 25.0

        xy = laserscan_xy(_Scan())
        self.assertEqual(xy.shape[0], 2)
        self.assertAlmostEqual(xy[0, 0], 0.5)
        self.assertAlmostEqual(xy[1, 0], -1.2, places=5)


if __name__ == "__main__":
    unittest.main()
