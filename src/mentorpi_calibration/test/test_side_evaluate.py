#!/usr/bin/env python3
"""Side stage evaluation on a synthetic side object (SD022 T2)."""
import math
import os
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.calibration_file import (  # noqa: E402
    FACTORY_POSES,
    draft_path,
    factory_calibration,
    load_draft,
)
from mentorpi_calibration.pointcloud2 import POINTFIELD_FLOAT32  # noqa: E402
from mentorpi_calibration.stages.side import (  # noqa: E402
    DETAIL_LIDAR_SIDE_DISAGREES,
    DETAIL_NO_CAMERA_DEPTH,
    DETAIL_OBJECT_NOT_FOUND_CLOUD,
    DETAIL_OBJECT_NOT_FOUND_SCAN,
    LAYER_CLOUD_GEOMETRY,
    LAYER_LIDAR_SUSPECT,
    LAYER_NONE,
    SIDE_LEFT,
    SIDE_RIGHT,
    STAGE_NAME,
    SideSamples,
    commit_side_pending,
    evaluate_side_samples,
    run_side,
)
from mentorpi_calibration.stages.transforms import (  # noqa: E402
    camera_optical_from_base_link,
    lidar_frame_from_base_link,
)

AURORA_W = 640
AURORA_H = 400
# /aurora/rgb/camera_info as measured on the stand (SD022 ### As-built п. 8).
AURORA_FX = 417.4124755859375
AURORA_FY = 418.379150390625
_RGB_FILL = (20, 40, 60)


def _blob_xy(cx, cy, count, rng, sigma=0.04):
    return np.column_stack(
        (
            rng.normal(cx, sigma, count),
            rng.normal(cy, sigma, count),
        )
    )


def _blob_xyz(cx, cy, cz, count, rng, sigma=0.04):
    xy = _blob_xy(cx, cy, count, rng, sigma=sigma)
    z = rng.normal(cz, sigma, count)
    return np.column_stack((xy, z))


def _front_wall_xyz(x, y_span, z_span, count, rng, noise=0.002):
    side = int(math.ceil(math.sqrt(count)))
    ys = np.linspace(y_span[0], y_span[1], side)
    zs = np.linspace(z_span[0], z_span[1], side)
    points = np.array([[x, y, z] for y in ys for z in zs], dtype=np.float64)
    points = points[:count]
    points += rng.normal(0.0, noise, size=points.shape)
    return points


def build_side_scene(
    camera_xyz,
    camera_rpy,
    lidar_xyz,
    lidar_rpy,
    *,
    object_xy=(1.2, 0.55),
    mirror_cloud=False,
    walls_only=False,
    include_object_in_cloud=True,
    rng=None,
):
    rng = np.random.default_rng(17) if rng is None else rng
    object_x, object_y = object_xy
    object_z = 0.18

    if walls_only:
        wall_bl = _front_wall_xyz(2.2, (-1.3, 1.3), (-0.05, 0.35), 900, rng)
        scan_bl = np.column_stack(
            (
                np.full(60, 2.2),
                np.linspace(-1.3, 1.3, 60),
                np.full(60, lidar_xyz[2]),
            )
        )
        cloud_bl = wall_bl
    else:
        scan_xy = _blob_xy(object_x, object_y, 40, rng)
        scan_bl = np.column_stack((scan_xy, np.full(scan_xy.shape[0], lidar_xyz[2])))
        if include_object_in_cloud:
            cloud_bl = _blob_xyz(object_x, object_y, object_z, 80, rng)
        else:
            cloud_bl = _front_wall_xyz(2.2, (-1.3, 1.3), (-0.05, 0.35), 900, rng)

    cloud_opt = camera_optical_from_base_link(cloud_bl, camera_xyz, camera_rpy)
    cloud_opt = cloud_opt[cloud_opt[:, 2] > 0.05]
    if mirror_cloud:
        cloud_opt = np.asarray(cloud_opt, dtype=np.float64).copy()
        cloud_opt[:, 0] *= -1.0
    scan_frame = lidar_frame_from_base_link(scan_bl, lidar_xyz, lidar_rpy)[:, :2]
    return cloud_opt.astype(np.float32), scan_frame.astype(np.float64)


def _rgb_image(width=AURORA_W, height=AURORA_H, fill=_RGB_FILL, encoding="rgb8"):
    data = bytes(fill) * (width * height)
    return SimpleNamespace(
        width=width,
        height=height,
        encoding=encoding,
        step=width * 3,
        data=data,
    )


def flat_cloud(xyz, point_step=32, capacity=None):
    """Aurora as-built: compacted list of valid points, zero tail (SD022 D6.1).

    No grid: capacity is not width*height of any frame, so a slot index cannot
    stand in for a pixel.
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    n = int(xyz.shape[0])
    slots = int(capacity if capacity is not None else max(n * 2, 64))
    buf = bytearray(slots * point_step)
    for index, (x, y, z) in enumerate(xyz):
        struct.pack_into("<fff", buf, index * point_step, float(x), float(y), float(z))
    fields = [
        SimpleNamespace(name="x", offset=0, datatype=POINTFIELD_FLOAT32, count=1),
        SimpleNamespace(name="y", offset=4, datatype=POINTFIELD_FLOAT32, count=1),
        SimpleNamespace(name="z", offset=8, datatype=POINTFIELD_FLOAT32, count=1),
    ]
    return SimpleNamespace(
        fields=fields,
        data=bytes(buf),
        point_step=point_step,
        width=slots,
        height=1,
        row_step=slots * point_step,
        is_dense=True,
    )


def camera_info(width=AURORA_W, height=AURORA_H, fx=AURORA_FX, fy=AURORA_FY, cx=None, cy=None):
    """CameraInfo with the stand's as-built intrinsics unless overridden."""
    cx = (width * 0.5) if cx is None else cx
    cy = (height * 0.5) if cy is None else cy
    return SimpleNamespace(
        width=width,
        height=height,
        k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
    )


def _ok_samples(cloud, scan_xy, *, organized=False, info=None, rgb_size=(AURORA_W, AURORA_H)):
    """Samples for the evaluator. ``organized`` now means "frame + intrinsics attached"."""
    out_cloud = cloud
    images = (object(),)
    infos = ()
    n_cloud = cloud.shape[0] if isinstance(cloud, np.ndarray) else None
    if organized:
        width, height = rgb_size
        if isinstance(cloud, np.ndarray):
            n_cloud = cloud.shape[0]
            out_cloud = flat_cloud(cloud)
        images = (_rgb_image(width, height),)
        infos = (camera_info(width, height) if info is None else info,)
    return SideSamples(
        clouds=(out_cloud,),
        scans=(scan_xy,),
        images=images,
        frames=1,
        cloud_points=n_cloud,
        scan_rays=scan_xy.shape[0],
        rgb_frames=1,
        ok=True,
        details=(),
        camera_infos=infos,
    )


class SideEvaluateTest(unittest.TestCase):
    def setUp(self):
        np.random.seed(42)
        self._calib = factory_calibration()
        self._tmpdir = tempfile.TemporaryDirectory()
        self._env_patch = mock.patch.dict(
            os.environ,
            {"T1_CALIBRATION_DIR": self._tmpdir.name},
        )
        self._env_patch.start()
        self._cam_xyz = FACTORY_POSES["depth_cam"]["xyz"]
        self._cam_rpy = FACTORY_POSES["depth_cam"]["rpy"]
        self._lidar_xyz = FACTORY_POSES["lidar"]["xyz"]
        self._lidar_rpy = FACTORY_POSES["lidar"]["rpy"]

    def tearDown(self):
        self._env_patch.stop()
        self._tmpdir.cleanup()

    def _evaluate(self, samples, operator_side=SIDE_LEFT):
        with mock.patch(
            "mentorpi_calibration.stages.side.commit_side_pending",
            return_value=(),
        ) as commit_pending:
            result = evaluate_side_samples(samples, self._calib, operator_side)
        self.assertFalse(os.path.isfile(draft_path(self._tmpdir.name)))
        if result.ok:
            commit_pending.assert_called_once()
        else:
            commit_pending.assert_not_called()
        return result

    def test_run_side_is_alias(self):
        self.assertIs(run_side, evaluate_side_samples)

    def test_matching_sides_layer_none(self):
        cloud, scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
        )
        result = self._evaluate(_ok_samples(cloud, scan, organized=True), SIDE_LEFT)
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.stage, STAGE_NAME)
        self.assertEqual(result.operator_side, SIDE_LEFT)
        self.assertEqual(result.scan_side, SIDE_LEFT)
        self.assertEqual(result.cloud_side, SIDE_LEFT)
        self.assertEqual(result.layer, LAYER_NONE)
        self.assertFalse(result.propose_transverse_mirror)
        self.assertGreater(result.scan_bearing_deg, 15.0)
        self.assertGreater(result.cloud_bearing_deg, 15.0)

    def test_mirrored_cloud_layer_cloud_geometry(self):
        cloud, scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            mirror_cloud=True,
        )
        result = self._evaluate(_ok_samples(cloud, scan, organized=True), SIDE_LEFT)
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.scan_side, SIDE_LEFT)
        self.assertEqual(result.cloud_side, SIDE_RIGHT)
        self.assertEqual(result.layer, LAYER_CLOUD_GEOMETRY)
        self.assertTrue(result.propose_transverse_mirror)
        self.assertGreater(result.scan_bearing_deg, 15.0)
        self.assertLess(result.cloud_bearing_deg, -15.0)

    def test_walls_only_object_not_found(self):
        cloud, scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            walls_only=True,
        )
        result = self._evaluate(_ok_samples(cloud, scan), SIDE_LEFT)
        self.assertFalse(result.ok)
        self.assertIn(DETAIL_OBJECT_NOT_FOUND_SCAN, result.details)
        self.assertIn(DETAIL_OBJECT_NOT_FOUND_CLOUD, result.details)
        self.assertIsNone(result.scan_side)
        self.assertIsNone(result.cloud_side)
        self.assertIsNone(result.layer)
        self.assertIsNone(result.propose_transverse_mirror)

    def test_object_missing_in_cloud_only(self):
        cloud, scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            include_object_in_cloud=False,
        )
        result = self._evaluate(_ok_samples(cloud, scan), SIDE_LEFT)
        self.assertFalse(result.ok)
        self.assertEqual(result.details, (DETAIL_OBJECT_NOT_FOUND_CLOUD,))
        self.assertIsNone(result.scan_side)
        self.assertIsNone(result.cloud_side)
        self.assertIsNone(result.layer)
        self.assertIsNone(result.propose_transverse_mirror)

    def test_scan_disagrees_with_operator_lidar_suspect(self):
        cloud, scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            object_xy=(1.2, -0.55),
        )
        result = self._evaluate(_ok_samples(cloud, scan), SIDE_LEFT)
        self.assertFalse(result.ok)
        self.assertEqual(result.details, (DETAIL_LIDAR_SIDE_DISAGREES,))
        self.assertEqual(result.scan_side, SIDE_RIGHT)
        self.assertEqual(result.cloud_side, SIDE_RIGHT)
        self.assertEqual(result.layer, LAYER_LIDAR_SUSPECT)
        self.assertIsNone(result.propose_transverse_mirror)

    def test_lidar_suspect_does_not_write_draft(self):
        cloud, scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            object_xy=(1.2, -0.55),
        )
        result = evaluate_side_samples(_ok_samples(cloud, scan), self._calib, SIDE_LEFT)
        self.assertEqual(result.layer, LAYER_LIDAR_SUSPECT)
        self.assertIsNone(result.propose_transverse_mirror)
        self.assertEqual(commit_side_pending(result, self._calib, dir=self._tmpdir.name), ())
        self.assertFalse(os.path.isfile(draft_path(self._tmpdir.name)))

    def test_cloud_geometry_commit_proposes_mirror(self):
        cloud, scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            mirror_cloud=True,
        )
        result = evaluate_side_samples(
            _ok_samples(cloud, scan, organized=True),
            self._calib,
            SIDE_LEFT,
        )
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.layer, LAYER_CLOUD_GEOMETRY)
        self.assertTrue(result.propose_transverse_mirror)
        draft = load_draft(self._tmpdir.name)
        self.assertFalse(draft.absent)
        self.assertIsNotNone(draft.pending)
        self.assertEqual(draft.pending.stage, STAGE_NAME)
        self.assertEqual(draft.pending.values["depth_cam"]["transverse_mirror"], True)

    def test_object_in_front_of_nearby_wall_is_found(self):
        rng = np.random.default_rng(17)
        object_xy = (1.2, 0.55)
        scan_obj = _blob_xy(*object_xy, 40, rng)
        scan_wall = np.column_stack((np.full(50, 1.55), np.linspace(-1.0, 1.0, 50)))
        scan_bl = np.vstack((scan_obj, scan_wall))
        scan_bl = np.column_stack((scan_bl, np.full(scan_bl.shape[0], self._lidar_xyz[2])))
        cloud_obj = _blob_xyz(object_xy[0], object_xy[1], 0.18, 80, rng)
        cloud_opt = camera_optical_from_base_link(cloud_obj, self._cam_xyz, self._cam_rpy)
        cloud_opt = cloud_opt[cloud_opt[:, 2] > 0.05]
        scan_frame = lidar_frame_from_base_link(scan_bl, self._lidar_xyz, self._lidar_rpy)[:, :2]
        result = self._evaluate(
            _ok_samples(cloud_opt.astype(np.float32), scan_frame.astype(np.float64), organized=True),
        )
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.scan_side, SIDE_LEFT)
        self.assertEqual(result.cloud_side, SIDE_LEFT)

    def test_floor_points_do_not_hide_object(self):
        rng = np.random.default_rng(17)
        object_xy = (1.2, 0.55)
        xs = np.linspace(0.5, 2.0, 16)
        ys = np.linspace(-0.8, 0.8, 16)
        floor = np.array([[x, y, 0.0] for x in xs for y in ys], dtype=np.float64)
        cloud_obj = _blob_xyz(object_xy[0], object_xy[1], 0.18, 80, rng)
        cloud_bl = np.vstack((floor, cloud_obj))
        scan_xy = _blob_xy(*object_xy, 40, rng)
        scan_bl = np.column_stack((scan_xy, np.full(scan_xy.shape[0], self._lidar_xyz[2])))
        cloud_opt = camera_optical_from_base_link(cloud_bl, self._cam_xyz, self._cam_rpy)
        cloud_opt = cloud_opt[cloud_opt[:, 2] > 0.05]
        scan_frame = lidar_frame_from_base_link(scan_bl, self._lidar_xyz, self._lidar_rpy)[:, :2]
        result = self._evaluate(
            _ok_samples(cloud_opt.astype(np.float32), scan_frame.astype(np.float64), organized=True),
        )
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.cloud_side, SIDE_LEFT)

    def test_object_a_few_cm_in_front_of_wall_is_found(self):
        rng = np.random.default_rng(17)
        object_xy = (1.2, 0.55)
        scan_obj = _blob_xy(*object_xy, 40, rng, sigma=0.03)
        scan_wall = np.column_stack((np.full(80, 1.28), np.linspace(-1.0, 1.0, 80)))
        scan_bl = np.vstack((scan_obj, scan_wall))
        scan_bl = np.column_stack((scan_bl, np.full(scan_bl.shape[0], self._lidar_xyz[2])))
        cloud_obj = _blob_xyz(object_xy[0], object_xy[1], 0.18, 80, rng, sigma=0.03)
        cloud_opt = camera_optical_from_base_link(cloud_obj, self._cam_xyz, self._cam_rpy)
        cloud_opt = cloud_opt[cloud_opt[:, 2] > 0.05]
        scan_frame = lidar_frame_from_base_link(scan_bl, self._lidar_xyz, self._lidar_rpy)[:, :2]
        result = self._evaluate(
            _ok_samples(cloud_opt.astype(np.float32), scan_frame.astype(np.float64), organized=True),
        )
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.scan_side, SIDE_LEFT)

    def test_ceiling_table_points_do_not_hide_object(self):
        rng = np.random.default_rng(17)
        object_xy = (1.2, 0.55)
        xs = np.linspace(0.6, 1.8, 12)
        ys = np.linspace(-0.7, 0.7, 12)
        table = np.array([[x, y, 0.70] for x in xs for y in ys], dtype=np.float64)
        cloud_obj = _blob_xyz(object_xy[0], object_xy[1], 0.18, 80, rng)
        cloud_bl = np.vstack((table, cloud_obj))
        scan_xy = _blob_xy(*object_xy, 40, rng)
        scan_bl = np.column_stack((scan_xy, np.full(scan_xy.shape[0], self._lidar_xyz[2])))
        cloud_opt = camera_optical_from_base_link(cloud_bl, self._cam_xyz, self._cam_rpy)
        cloud_opt = cloud_opt[cloud_opt[:, 2] > 0.05]
        scan_frame = lidar_frame_from_base_link(scan_bl, self._lidar_xyz, self._lidar_rpy)[:, :2]
        result = self._evaluate(
            _ok_samples(cloud_opt.astype(np.float32), scan_frame.astype(np.float64), organized=True),
        )
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.cloud_side, SIDE_LEFT)

    def test_evaluate_uses_last_cloud_snapshot(self):
        wall_cloud, _wall_scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            walls_only=True,
        )
        obj_cloud, obj_scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
        )
        wall_samples = _ok_samples(wall_cloud, obj_scan)
        obj_samples = _ok_samples(obj_cloud, obj_scan, organized=True)
        samples = SideSamples(
            clouds=wall_samples.clouds + obj_samples.clouds,
            scans=obj_samples.scans,
            images=obj_samples.images,
            frames=2,
            cloud_points=obj_samples.cloud_points,
            scan_rays=obj_samples.scan_rays,
            rgb_frames=1,
            ok=True,
            details=(),
            camera_infos=obj_samples.camera_infos,
        )
        result = self._evaluate(samples)
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.layer, LAYER_NONE)

    def test_sparse_last_scan_still_finds_object(self):
        obj_cloud, obj_scan = build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
        )
        sparse = np.asarray(obj_scan[:2], dtype=np.float64)
        obj_samples = _ok_samples(obj_cloud, obj_scan, organized=True)
        samples = SideSamples(
            clouds=obj_samples.clouds,
            scans=(obj_scan, sparse),
            images=obj_samples.images,
            frames=1,
            cloud_points=obj_samples.cloud_points,
            scan_rays=obj_scan.shape[0] + sparse.shape[0],
            rgb_frames=1,
            ok=True,
            details=(),
            camera_infos=obj_samples.camera_infos,
        )
        result = self._evaluate(samples)
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.scan_side, SIDE_LEFT)

    def test_refuse_collect_failure(self):
        samples = SideSamples(
            clouds=(),
            scans=(),
            images=(),
            frames=0,
            cloud_points=None,
            scan_rays=None,
            rgb_frames=0,
            ok=False,
            details=(DETAIL_NO_CAMERA_DEPTH,),
        )
        result = self._evaluate(samples, SIDE_LEFT)
        self.assertFalse(result.ok)
        self.assertEqual(result.details, (DETAIL_NO_CAMERA_DEPTH,))
        self.assertIsNone(result.layer)


if __name__ == "__main__":
    unittest.main()
