#!/usr/bin/env python3
"""Orthogonal wall RANSAC and floor-strip 1-D peak (SD012 T5)."""
import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.stages.planes import (  # noqa: E402
    DETAIL_ONE_WALL,
    DETAIL_WALLS_NOT_ORTHOGONAL,
    estimate_floor_strip,
    extract_orthogonal_walls,
    fit_plane_ransac,
    wall_mask_union,
)


def _plane_grid(normal, d, count, rng, span=0.8, noise=0.002):
    normal = np.asarray(normal, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    anchor = -d * normal
    if abs(normal[2]) < 0.9:
        u = np.cross(normal, np.array([0.0, 0.0, 1.0]))
    else:
        u = np.cross(normal, np.array([1.0, 0.0, 0.0]))
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    offsets = rng.uniform(-span, span, size=(count, 2))
    points = anchor + offsets[:, 0:1] * u + offsets[:, 1:2] * v
    points += rng.normal(0.0, noise, size=points.shape)
    return points


class OrthogonalWallsTest(unittest.TestCase):
    def setUp(self):
        np.random.seed(42)
        self._rng = np.random.default_rng(42)

    def test_two_orthogonal_walls_recover_vertical(self):
        wall_a = _plane_grid((1.0, 0.0, 0.0), -1.4, 700, self._rng)
        wall_b = _plane_grid((0.0, 1.0, 0.0), -0.9, 700, self._rng)
        floor = _plane_grid((0.0, 0.0, 1.0), 0.177, 200, self._rng)
        points = np.vstack((wall_a, wall_b, floor))
        up_prior = np.array([0.0, 0.0, 1.0])

        walls = extract_orthogonal_walls(points, up_prior=up_prior)

        self.assertTrue(walls.ok, walls.detail)
        self.assertIsNotNone(walls.up)
        self.assertGreater(float(np.dot(walls.up, up_prior)), 0.95)

    def test_one_wall_fails(self):
        wall_a = _plane_grid((1.0, 0.0, 0.0), -1.4, 900, self._rng)
        walls = extract_orthogonal_walls(
            wall_a,
            up_prior=np.array([0.0, 0.0, 1.0]),
        )
        self.assertFalse(walls.ok)
        self.assertEqual(walls.detail, DETAIL_ONE_WALL)

    def test_non_orthogonal_walls_fail(self):
        n60 = (np.cos(np.deg2rad(60.0)), np.sin(np.deg2rad(60.0)), 0.0)
        wall_a = _plane_grid((1.0, 0.0, 0.0), -1.4, 700, self._rng)
        wall_b = _plane_grid(n60, -1.0, 700, self._rng)
        points = np.vstack((wall_a, wall_b))
        walls = extract_orthogonal_walls(
            points,
            up_prior=np.array([0.0, 0.0, 1.0]),
        )
        self.assertFalse(walls.ok)
        self.assertEqual(walls.detail, DETAIL_WALLS_NOT_ORTHOGONAL)

    def test_floor_strip_along_known_vertical(self):
        up = np.array([0.0, 0.0, 1.0])
        wall_a = _plane_grid((1.0, 0.0, 0.0), -1.4, 500, self._rng)
        floor = _plane_grid(up, 0.177, 250, self._rng)
        points = np.vstack((wall_a, floor))
        first = fit_plane_ransac(points, up=up)
        self.assertTrue(first.ok)
        mask = np.zeros(points.shape[0], dtype=bool)
        mask[:500] = True
        strip = estimate_floor_strip(points, up, wall_mask=mask)
        self.assertTrue(strip.observable)
        self.assertAlmostEqual(strip.height_h, 0.177, delta=0.02)

    def test_missing_floor_strip_not_observable(self):
        wall_a = _plane_grid((1.0, 0.0, 0.0), -1.4, 400, self._rng)
        wall_b = _plane_grid((0.0, 1.0, 0.0), -0.9, 400, self._rng)
        points = np.vstack((wall_a, wall_b))
        up = np.array([0.0, 0.0, 1.0])
        walls = extract_orthogonal_walls(points, up_prior=up)
        self.assertTrue(walls.ok, walls.detail)
        mask = wall_mask_union(walls, points.shape[0])
        strip = estimate_floor_strip(points, walls.up, wall_mask=mask)
        self.assertFalse(strip.observable)


if __name__ == "__main__":
    unittest.main()
