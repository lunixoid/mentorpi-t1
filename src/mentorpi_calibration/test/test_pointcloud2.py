#!/usr/bin/env python3
"""PointCloud2 → xyz parsing (SD012 D3.2.2)."""
import os
import struct
import sys
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.pointcloud2 import (  # noqa: E402
    DETAIL_CLOUD_LAYOUT_UNSUPPORTED,
    MIN_FLOOR_POINTS,
    POINTFIELD_FLOAT32,
    CloudLayoutError,
    clouds_to_xyz,
    pointcloud2_xyz,
)


def _make_cloud(
    xyz_rows,
    *,
    x_datatype=POINTFIELD_FLOAT32,
    y_datatype=POINTFIELD_FLOAT32,
    z_datatype=POINTFIELD_FLOAT32,
    point_step=12,
):
    n = len(xyz_rows)
    data = bytearray(n * point_step)
    for index, (x, y, z) in enumerate(xyz_rows):
        struct.pack_into("fff", data, index * point_step, x, y, z)

    fields = [
        SimpleNamespace(name="x", offset=0, datatype=x_datatype, count=1),
        SimpleNamespace(name="y", offset=4, datatype=y_datatype, count=1),
        SimpleNamespace(name="z", offset=8, datatype=z_datatype, count=1),
    ]
    return SimpleNamespace(
        fields=fields,
        data=bytes(data),
        point_step=point_step,
        width=n,
        height=1,
        row_step=n * point_step,
        is_dense=True,
    )


class PointCloud2Test(unittest.TestCase):
    def test_known_xyz(self):
        cloud = _make_cloud([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
        xyz = pointcloud2_xyz(cloud)
        self.assertEqual(xyz.shape, (2, 3))
        np.testing.assert_allclose(xyz, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])

    def test_drops_non_finite(self):
        cloud = _make_cloud([(1.0, 2.0, 3.0), (np.nan, 0.0, 0.0), (4.0, 5.0, 6.0)])
        xyz = pointcloud2_xyz(cloud)
        self.assertEqual(xyz.shape, (2, 3))
        np.testing.assert_allclose(xyz, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])

    def test_wrong_datatype_raises(self):
        cloud = _make_cloud([(1.0, 2.0, 3.0)], x_datatype=8)
        with self.assertRaises(CloudLayoutError) as ctx:
            pointcloud2_xyz(cloud)
        self.assertEqual(ctx.exception.detail, DETAIL_CLOUD_LAYOUT_UNSUPPORTED)

    def test_clouds_to_xyz_concatenates(self):
        first = _make_cloud([(1.0, 0.0, 0.0)])
        second = np.array([[0.0, 1.0, 0.0]], dtype=np.float32)
        xyz = clouds_to_xyz([first, second])
        np.testing.assert_allclose(xyz, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

    def test_min_floor_points_constant_exposed(self):
        self.assertGreater(MIN_FLOOR_POINTS, 0)


if __name__ == "__main__":
    unittest.main()
