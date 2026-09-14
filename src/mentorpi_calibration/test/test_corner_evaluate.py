#!/usr/bin/env python3
"""Corner stage evaluation on a synthetic room corner (SD012 T5)."""
import math
import os
import sys
import tempfile
import unittest
from dataclasses import replace
from unittest import mock

import numpy as np

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.calibration_file import (  # noqa: E402
    BASE_LINK_OFFSET_Z,
    FACTORY_POSES,
    SensorPose,
    draft_path,
    factory_calibration,
    mat_vec3,
    rotation_matrix_from_rpy,
)
from mentorpi_calibration.stages.corner import (  # noqa: E402
    DETAIL_NO_CAMERA_DEPTH,
    STAGE_NAME,
    CornerSamples,
    evaluate_corner_samples,
    run_corner,
)
from mentorpi_calibration.stages.imu_mount import GRAVITY_MPS2  # noqa: E402
from mentorpi_calibration.stages.planes import DETAIL_ONE_WALL  # noqa: E402
from mentorpi_calibration.stages.planes import DETAIL_WALLS_NOT_ORTHOGONAL  # noqa: E402
from mentorpi_calibration.stages.transforms import (  # noqa: E402
    camera_optical_from_base_link,
    lidar_frame_from_base_link,
    np_rotation,
)


def _as_np(matrix):
    return np.asarray(matrix, dtype=np.float64)


def _transpose(matrix):
    return tuple(tuple(matrix[j][i] for j in range(3)) for i in range(3))


def _accel_for_mount_rpy(mount_rpy, magnitude=GRAVITY_MPS2):
    r_mount = rotation_matrix_from_rpy(mount_rpy)
    direction = mat_vec3(_transpose(r_mount), (0.0, 0.0, 1.0))
    return tuple(magnitude * component for component in direction)


def _wall_grid(axis, value, count, rng, *, u_span, v_span, noise=0.002):
    side = int(math.ceil(math.sqrt(count)))
    us = np.linspace(u_span[0], u_span[1], side)
    vs = np.linspace(v_span[0], v_span[1], side)
    points = []
    for u in us:
        for v in vs:
            if axis == "x":
                points.append((value, u, v))
            else:
                points.append((u, value, v))
    grid = np.asarray(points[:count], dtype=np.float64)
    grid += rng.normal(0.0, noise, size=grid.shape)
    return grid


def _floor_grid(count, rng, *, x_span, y_span, z, noise=0.002):
    side = int(math.ceil(math.sqrt(count)))
    xs = np.linspace(x_span[0], x_span[1], side)
    ys = np.linspace(y_span[0], y_span[1], side)
    grid = np.array([[x, y, z] for x in xs for y in ys], dtype=np.float64)
    grid = grid[:count]
    grid += rng.normal(0.0, noise, size=grid.shape)
    return grid


def _scan_line_bl(axis, value, z, count, rng, span, noise=0.004):
    t = np.linspace(span[0], span[1], count)
    if axis == "x":
        pts = np.column_stack((np.full(count, value), t, np.full(count, z)))
    else:
        pts = np.column_stack((t, np.full(count, value), np.full(count, z)))
    pts += rng.normal(0.0, noise, size=pts.shape)
    return pts


def build_corner_scene(
    camera_xyz,
    camera_rpy,
    lidar_xyz,
    lidar_rpy,
    *,
    include_floor=True,
    second_wall="orthogonal",
    scan_omits_optical=False,
    rng=None,
):
    rng = np.random.default_rng(17) if rng is None else rng
    z_floor = -BASE_LINK_OFFSET_Z
    z_scan = lidar_xyz[2]
    x_wall = 1.35
    y_wall = 0.85

    wall_a = _wall_grid(
        "x",
        x_wall,
        800,
        rng,
        u_span=(0.05, 0.8),
        v_span=(z_floor + 0.02, 0.55),
    )
    if second_wall == "orthogonal":
        wall_b = _wall_grid(
            "y",
            y_wall,
            800,
            rng,
            u_span=(0.25, 1.3),
            v_span=(z_floor + 0.02, 0.55),
        )
        scan_b_bl = _scan_line_bl("y", y_wall, z_scan, 35, rng, (0.25, 1.3))
    elif second_wall == "skew":
        n = np.array([math.cos(math.radians(55.0)), math.sin(math.radians(55.0)), 0.0])
        d = -1.1
        # Plane n·p + d = 0 through a 55° wall.
        wall_b = []
        for u in np.linspace(-0.5, 0.5, 28):
            for v in np.linspace(z_floor + 0.02, 0.55, 28):
                # p = -d n + u t + v ez, t horizontal tangent
                tangent = np.array([-n[1], n[0], 0.0])
                wall_b.append(-d * n + u * tangent + np.array([0.0, 0.0, v - z_floor]))
        wall_b = np.asarray(wall_b, dtype=np.float64)
        wall_b += rng.normal(0.0, 0.002, size=wall_b.shape)
        scan_b_bl = None
    else:
        wall_b = np.empty((0, 3), dtype=np.float64)
        scan_b_bl = None

    clouds_bl = [wall_a]
    if wall_b.shape[0]:
        clouds_bl.append(wall_b)
    if include_floor:
        clouds_bl.append(
            _floor_grid(
                280,
                rng,
                x_span=(0.35, 1.15),
                y_span=(0.0, 0.7),
                z=z_floor,
            )
        )
    points_bl = np.vstack(clouds_bl)
    points_opt = camera_optical_from_base_link(points_bl, camera_xyz, camera_rpy)
    in_front = points_opt[:, 2] > 0.12
    points_opt = points_opt[in_front]

    scan_a_bl = _scan_line_bl("x", x_wall, z_scan, 35, rng, (0.05, 0.8))
    scan_parts = [scan_a_bl]
    if scan_b_bl is not None:
        scan_parts.append(scan_b_bl)
    scan_bl = np.vstack(scan_parts)
    if scan_omits_optical:
        r_mount = np_rotation(lidar_rpy)
        t_mount = np.asarray(lidar_xyz, dtype=np.float64)
        scan_link = (r_mount.T @ (scan_bl - t_mount).T).T
        return points_opt.astype(np.float32), scan_link[:, :2]
    scan_frame = lidar_frame_from_base_link(scan_bl, lidar_xyz, lidar_rpy)
    return points_opt.astype(np.float32), scan_frame[:, :2]


def _ok_samples(cloud, scan_xy, imu_accels):
    return CornerSamples(
        clouds=(cloud,),
        scans=(scan_xy,),
        imu_accels=imu_accels,
        frames=1,
        cloud_points=cloud.shape[0],
        scan_rays=scan_xy.shape[0],
        imu_samples=len(imu_accels),
        ok=True,
        details=(),
    )


class CornerEvaluateTest(unittest.TestCase):
    def setUp(self):
        np.random.seed(42)
        self._calib = factory_calibration()
        self._tmpdir = tempfile.TemporaryDirectory()
        self._env_patch = mock.patch.dict(
            os.environ,
            {"T1_CALIBRATION_DIR": self._tmpdir.name},
        )
        self._env_patch.start()
        self._true_cam_xyz = (0.11, 0.02, 0.055)
        self._true_cam_rpy = (0.04, -0.05, 0.12)
        self._lidar_xyz = FACTORY_POSES["lidar"]["xyz"]
        self._lidar_rpy = FACTORY_POSES["lidar"]["rpy"]
        self._imu_rpy = FACTORY_POSES["imu"]["rpy"]

    def tearDown(self):
        self._env_patch.stop()
        self._tmpdir.cleanup()

    def _evaluate(self, samples):
        with mock.patch(
            "mentorpi_calibration.stages.corner.commit_corner_pending",
            return_value=(),
        ) as commit_pending:
            result = evaluate_corner_samples(samples, self._calib)
        self.assertFalse(os.path.isfile(draft_path(self._tmpdir.name)))
        if result.ok:
            commit_pending.assert_called_once()
        else:
            commit_pending.assert_not_called()
        return result

    def test_run_corner_is_alias(self):
        self.assertIs(run_corner, evaluate_corner_samples)

    def test_recovers_six_camera_values_and_imu(self):
        cloud, scan = build_corner_scene(
            self._true_cam_xyz,
            self._true_cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
        )
        accel = _accel_for_mount_rpy(self._imu_rpy)
        result = self._evaluate(_ok_samples(cloud, scan, (accel, accel)))

        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.stage, STAGE_NAME)
        self.assertTrue(result.include_camera_z)
        self.assertLess(result.residual_after, result.residual_before)
        self.assertAlmostEqual(result.depth_cam_xyz[0], self._true_cam_xyz[0], delta=0.04)
        self.assertAlmostEqual(result.depth_cam_xyz[1], self._true_cam_xyz[1], delta=0.04)
        self.assertAlmostEqual(result.depth_cam_xyz[2], self._true_cam_xyz[2], delta=0.03)
        self.assertAlmostEqual(result.depth_cam_rpy[0], self._true_cam_rpy[0], delta=0.05)
        self.assertAlmostEqual(result.depth_cam_rpy[1], self._true_cam_rpy[1], delta=0.05)
        self.assertAlmostEqual(result.depth_cam_rpy[2], self._true_cam_rpy[2], delta=0.08)
        self.assertAlmostEqual(result.imu_rpy[0], self._imu_rpy[0], delta=0.02)
        self.assertAlmostEqual(result.imu_rpy[1], self._imu_rpy[1], delta=0.02)
        self.assertEqual(result.cause, "")
        self.assertAlmostEqual(result.lidar_rpy[2], self._lidar_rpy[2], delta=0.05)

    def test_no_floor_omits_z(self):
        cloud, scan = build_corner_scene(
            self._true_cam_xyz,
            self._true_cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            include_floor=False,
        )
        accel = _accel_for_mount_rpy(self._imu_rpy)
        result = self._evaluate(_ok_samples(cloud, scan, (accel, accel)))

        self.assertTrue(result.ok, result.details)
        self.assertFalse(result.include_camera_z)
        self.assertIn("calib camera --height", result.details[0])
        self.assertAlmostEqual(
            result.depth_cam_xyz[2],
            self._calib.sensors["depth_cam"].xyz[2],
            delta=1e-9,
        )
        self.assertAlmostEqual(result.depth_cam_rpy[0], self._true_cam_rpy[0], delta=0.05)
        self.assertAlmostEqual(result.depth_cam_rpy[1], self._true_cam_rpy[1], delta=0.05)

    def test_one_wall_refuses(self):
        cloud, scan = build_corner_scene(
            self._true_cam_xyz,
            self._true_cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            second_wall=None,
        )
        accel = _accel_for_mount_rpy(self._imu_rpy)
        result = self._evaluate(_ok_samples(cloud, scan, (accel, accel)))
        self.assertFalse(result.ok)
        self.assertEqual(result.details, (DETAIL_ONE_WALL,))

    def test_skew_walls_refuse(self):
        cloud, scan = build_corner_scene(
            FACTORY_POSES["depth_cam"]["xyz"],
            (0.0, 0.0, 0.0),
            self._lidar_xyz,
            self._lidar_rpy,
            second_wall="skew",
        )
        accel = _accel_for_mount_rpy(self._imu_rpy)
        result = self._evaluate(_ok_samples(cloud, scan, (accel, accel)))
        self.assertFalse(result.ok)
        self.assertIn(
            result.details[0],
            (DETAIL_WALLS_NOT_ORTHOGONAL, DETAIL_ONE_WALL),
        )

    def test_axis_swap_yaw_pi(self):
        true_rpy = (0.02, -0.03, math.pi)
        # Corner in -X so a yaw=π camera still looks at the walls.
        rng = np.random.default_rng(3)
        z_floor = -BASE_LINK_OFFSET_Z
        x_wall = -1.35
        y_wall = 0.85
        wall_a = _wall_grid(
            "x",
            x_wall,
            800,
            rng,
            u_span=(0.05, 0.8),
            v_span=(z_floor + 0.02, 0.55),
        )
        wall_b = _wall_grid(
            "y",
            y_wall,
            800,
            rng,
            u_span=(-1.3, -0.25),
            v_span=(z_floor + 0.02, 0.55),
        )
        floor = _floor_grid(
            250,
            rng,
            x_span=(-1.15, -0.35),
            y_span=(0.0, 0.7),
            z=z_floor,
        )
        points_bl = np.vstack((wall_a, wall_b, floor))
        true_xyz = (0.11, 0.02, 0.055)
        cloud = camera_optical_from_base_link(points_bl, true_xyz, true_rpy)
        cloud = cloud[cloud[:, 2] > 0.12].astype(np.float32)
        z_scan = self._lidar_xyz[2]
        scan_bl = np.vstack(
            (
                _scan_line_bl("x", x_wall, z_scan, 35, rng, (0.05, 0.8)),
                _scan_line_bl("y", y_wall, z_scan, 35, rng, (-1.3, -0.25)),
            )
        )
        scan = lidar_frame_from_base_link(scan_bl, self._lidar_xyz, self._lidar_rpy)[:, :2]
        accel = _accel_for_mount_rpy(self._imu_rpy)
        result = self._evaluate(_ok_samples(cloud, scan, (accel, accel)))

        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.cause, "axis swap")
        yaw = result.depth_cam_rpy[2]
        snapped = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
        self.assertAlmostEqual(yaw, snapped, delta=1e-6)
        self.assertGreater(abs(snapped), math.pi / 2.0 - 1e-6)
        self.assertAlmostEqual(result.lidar_rpy[2], self._lidar_rpy[2], delta=0.05)

    def test_scan_missing_optical_pi_yaws_lidar(self):
        cloud, scan = build_corner_scene(
            self._true_cam_xyz,
            (0.04, -0.05, 0.0),
            self._lidar_xyz,
            self._lidar_rpy,
            scan_omits_optical=True,
        )
        accel = _accel_for_mount_rpy(self._imu_rpy)
        result = self._evaluate(_ok_samples(cloud, scan, (accel, accel)))

        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.cause, "axis swap")
        self.assertAlmostEqual(abs(result.lidar_rpy[2]), math.pi, delta=0.1)
        self.assertAlmostEqual(result.depth_cam_xyz[0], self._true_cam_xyz[0], delta=0.08)
        self.assertAlmostEqual(result.depth_cam_xyz[1], self._true_cam_xyz[1], delta=0.08)
        self.assertGreater(result.depth_cam_xyz[0], 0.0)

    def test_scan_missing_optical_recovers_from_bad_camera_xy(self):
        cam = self._calib.sensors["depth_cam"]
        self._calib = replace(
            self._calib,
            sensors={
                **self._calib.sensors,
                "depth_cam": SensorPose(
                    parent=cam.parent,
                    xyz=(-0.838, 0.143, cam.xyz[2]),
                    rpy=cam.rpy,
                    source=cam.source,
                ),
            },
        )
        cloud, scan = build_corner_scene(
            self._true_cam_xyz,
            (0.04, -0.05, 0.0),
            self._lidar_xyz,
            self._lidar_rpy,
            scan_omits_optical=True,
        )
        accel = _accel_for_mount_rpy(self._imu_rpy)
        result = self._evaluate(_ok_samples(cloud, scan, (accel, accel)))

        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.cause, "axis swap")
        self.assertAlmostEqual(abs(result.lidar_rpy[2]), math.pi, delta=0.1)
        self.assertGreater(result.depth_cam_xyz[0], 0.0)
        self.assertAlmostEqual(result.depth_cam_xyz[0], self._true_cam_xyz[0], delta=0.08)

    def test_refuse_no_camera_depth(self):
        samples = CornerSamples(
            clouds=(),
            scans=(np.zeros((10, 2)),),
            imu_accels=(_accel_for_mount_rpy(self._imu_rpy),),
            frames=0,
            cloud_points=None,
            scan_rays=10,
            imu_samples=1,
            ok=False,
            details=(DETAIL_NO_CAMERA_DEPTH,),
        )
        result = self._evaluate(samples)
        self.assertFalse(result.ok)
        self.assertEqual(result.details, (DETAIL_NO_CAMERA_DEPTH,))


if __name__ == "__main__":
    unittest.main()
