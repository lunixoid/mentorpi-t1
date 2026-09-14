#!/usr/bin/env python3
"""Cloud cluster → RGB frame projection and PNG (SD022 T3 / D2.3)."""
import os
import struct
import sys
import tempfile
import unittest
import zlib
from types import SimpleNamespace
from unittest import mock

import numpy as np

sys.path.insert(
    0,
    os.path.abspath(os.path.dirname(__file__)),
)
sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from test_side_evaluate import (  # noqa: E402
    _RGB_FILL,
    AURORA_FX,
    AURORA_FY,
    AURORA_H,
    AURORA_W,
    _ok_samples,
    build_side_scene,
    camera_info,
    flat_cloud,
)

from mentorpi_calibration.calibration_file import (  # noqa: E402
    FACTORY_POSES,
    draft_path,
    factory_calibration,
    load_draft,
    transverse_mirror,
)
from mentorpi_calibration.stages.side import (  # noqa: E402
    DETAIL_CANNOT_PROJECT,
    LAYER_CLOUD_GEOMETRY,
    LAYER_NONE,
    LAYER_PIXEL_MAPPING,
    SIDE_FRAME_FILENAME,
    SIDE_LEFT,
    SIDE_RIGHT,
    CameraIntrinsics,
    evaluate_side_samples,
    intrinsics_from_camera_info,
    project_point,
    side_frame_path,
    write_rgb_png,
)

_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_PACKAGE_XML = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "package.xml"),
)


def _parse_png(path):
    with open(path, "rb") as handle:
        blob = handle.read()
    if blob[:8] != _PNG_SIGNATURE:
        raise AssertionError("missing PNG signature")
    pos = 8
    idat = b""
    ihdr = None
    while pos + 12 <= len(blob):
        length, tag = struct.unpack(">I4s", blob[pos : pos + 8])
        payload = blob[pos + 8 : pos + 8 + length]
        pos += 12 + length
        if tag == b"IHDR":
            ihdr = struct.unpack(">IIBBBBB", payload)
        elif tag == b"IDAT":
            idat += payload
        elif tag == b"IEND":
            break
    if ihdr is None:
        raise AssertionError("missing IHDR")
    width, height, bit_depth, color_type, _comp, _filt, _inter = ihdr
    raw = zlib.decompress(idat)
    stride = width * 3 + 1
    pixels = bytearray()
    for row in range(height):
        start = row * stride
        line = raw[start : start + stride]
        if line[0] != 0:
            raise AssertionError("unexpected PNG filter {}".format(line[0]))
        pixels.extend(line[1:])
    return width, height, bit_depth, color_type, bytes(pixels)


class RgbCloudMappingTest(unittest.TestCase):
    """Pixel <-> point is projection, never a slot index (SD022 D6.1, D6.2)."""

    def _cam(self):
        return intrinsics_from_camera_info(camera_info())

    def test_camera_info_parsed_into_intrinsics(self):
        cam = self._cam()
        self.assertIsNotNone(cam)
        self.assertTrue(cam.valid)
        self.assertAlmostEqual(cam.fx, AURORA_FX)
        self.assertAlmostEqual(cam.fy, AURORA_FY)
        self.assertAlmostEqual(cam.cx, AURORA_W / 2.0)
        self.assertAlmostEqual(cam.cy, AURORA_H / 2.0)

    def test_camera_info_without_focal_is_rejected(self):
        self.assertIsNone(intrinsics_from_camera_info(camera_info(fx=0.0)))
        self.assertIsNone(intrinsics_from_camera_info(None))
        self.assertIsNone(intrinsics_from_camera_info(SimpleNamespace(width=640, height=400)))
        self.assertFalse(CameraIntrinsics().valid)

    def test_projection_matches_perception_law(self):
        cam = self._cam()
        # Same law as person_geometry.hpp project_point: u = fx*x/z + cx.
        self.assertEqual(project_point(cam, (0.0, 0.0, 2.0)), (cam.cx, cam.cy))
        left = project_point(cam, (-0.5, 0.0, 2.0))
        right = project_point(cam, (0.5, 0.0, 2.0))
        self.assertIsNotNone(left)
        self.assertIsNotNone(right)
        self.assertLess(left[0], cam.cx)
        self.assertGreater(right[0], cam.cx)
        expected_u = cam.fx * (-0.5 / 2.0) + cam.cx
        self.assertAlmostEqual(left[0], expected_u)

    def test_projection_rejects_points_behind_and_no_intrinsics(self):
        cam = self._cam()
        self.assertIsNone(project_point(cam, (0.1, 0.0, -1.0)))
        self.assertIsNone(project_point(cam, (0.1, 0.0, 0.0)))
        self.assertIsNone(project_point(CameraIntrinsics(), (0.1, 0.0, 2.0)))

    def test_flat_cloud_has_no_grid_to_index(self):
        # The as-built buffer is a compacted list with a zero tail: its capacity
        # is not the frame size, so slot arithmetic has nothing to stand on.
        cloud = flat_cloud(np.array([[0.1, 0.0, 1.0]], dtype=np.float32))
        self.assertEqual(cloud.height, 1)
        self.assertNotEqual(cloud.width, AURORA_W * AURORA_H)
        self.assertGreater(cloud.width, 1)


class SideFramePngTest(unittest.TestCase):
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

    def _scene(self, **kwargs):
        return build_side_scene(
            self._cam_xyz,
            self._cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            **kwargs,
        )

    def test_write_rgb_png_signature_and_ihdr(self):
        path = os.path.join(self._tmpdir.name, "plain.png")
        fill = bytes(_RGB_FILL) * (AURORA_W * AURORA_H)
        write_rgb_png(path, AURORA_W, AURORA_H, fill)
        width, height, bit_depth, color_type, pixels = _parse_png(path)
        self.assertEqual((width, height), (AURORA_W, AURORA_H))
        self.assertEqual(bit_depth, 8)
        self.assertEqual(color_type, 2)
        self.assertEqual(pixels[:3], bytes(_RGB_FILL))

    def test_matching_sides_writes_png_layer_none(self):
        cloud, scan = self._scene()
        result = evaluate_side_samples(
            _ok_samples(cloud, scan, organized=True),
            self._calib,
            SIDE_LEFT,
        )
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.layer, LAYER_NONE)
        self.assertFalse(result.propose_transverse_mirror)
        self.assertEqual(result.frame_side, SIDE_LEFT)
        self.assertIsNotNone(result.frame_path)
        self.assertEqual(os.path.basename(result.frame_path), SIDE_FRAME_FILENAME)
        self.assertEqual(result.frame_path, side_frame_path(self._tmpdir.name))
        self.assertTrue(os.path.isfile(result.frame_path))

        with open(result.frame_path, "rb") as handle:
            signature = handle.read(8)
        self.assertEqual(signature, _PNG_SIGNATURE)
        width, height, bit_depth, color_type, pixels = _parse_png(result.frame_path)
        self.assertEqual((width, height), (AURORA_W, AURORA_H))
        self.assertEqual(bit_depth, 8)
        self.assertEqual(color_type, 2)
        mark_col = max(0, min(width - 1, int(round(result.frame_u_px))))
        mark_i = (10 * width + mark_col) * 3
        self.assertEqual(pixels[mark_i : mark_i + 3], bytes((255, 255, 0)))
        self.assertNotEqual(pixels, bytes(_RGB_FILL) * (width * height))
        self.assertLess(result.frame_u_px, width / 2.0)

    def test_mark_on_opposite_half_is_pixel_mapping(self):
        # Cloud says left, but the frame's optical axis sits far to the right of
        # the picture centre, so the projected mark lands on the other half —
        # exactly the D1.2(b) signature, now expressed in intrinsics.
        cloud, scan = self._scene()
        result = evaluate_side_samples(
            _ok_samples(cloud, scan, organized=True, info=camera_info(cx=620.0)),
            self._calib,
            SIDE_LEFT,
        )
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.scan_side, SIDE_LEFT)
        self.assertEqual(result.cloud_side, SIDE_LEFT)
        self.assertEqual(result.layer, LAYER_PIXEL_MAPPING)
        self.assertFalse(result.propose_transverse_mirror)
        self.assertEqual(result.frame_side, SIDE_RIGHT)
        self.assertGreaterEqual(result.frame_u_px, AURORA_W / 2.0)
        self.assertTrue(os.path.isfile(result.frame_path))

        draft = load_draft(self._tmpdir.name)
        self.assertFalse(draft.absent)
        self.assertIsNotNone(draft.pending)
        self.assertEqual(draft.pending.values["depth_cam"]["transverse_mirror"], False)
        self.assertFalse(transverse_mirror(self._calib))

    def test_pixel_mapping_does_not_overwrite_geometry_when_sides_differ(self):
        cloud, scan = self._scene(mirror_cloud=True)
        result = evaluate_side_samples(
            _ok_samples(cloud, scan, organized=True),
            self._calib,
            SIDE_LEFT,
        )
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.layer, LAYER_CLOUD_GEOMETRY)
        self.assertTrue(result.propose_transverse_mirror)
        # A mirrored cloud honestly projects onto the mirrored half of the frame,
        # so frame_side follows it; the geometry verdict must survive that and
        # not be relabelled pixel_mapping.
        self.assertEqual(result.frame_side, SIDE_RIGHT)

    def test_compacted_buffer_with_zero_tail_still_projects(self):
        # Aurora publishes height=1 with a zero tail; the evaluator must read the
        # points it has and project them, not infer a 640x400 grid (D6.1).
        cloud, scan = self._scene()
        samples = _ok_samples(cloud, scan, organized=True)
        padded = flat_cloud(cloud, capacity=AURORA_W * AURORA_H)
        self.assertEqual(padded.height, 1)
        self.assertEqual(padded.width, AURORA_W * AURORA_H)
        samples = samples.__class__(
            clouds=(padded,),
            scans=samples.scans,
            images=samples.images,
            frames=samples.frames,
            cloud_points=samples.cloud_points,
            scan_rays=samples.scan_rays,
            rgb_frames=samples.rgb_frames,
            ok=True,
            details=(),
            camera_infos=samples.camera_infos,
        )
        result = evaluate_side_samples(samples, self._calib, SIDE_LEFT)
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.layer, LAYER_NONE)
        self.assertEqual(result.frame_side, SIDE_LEFT)

    def test_missing_projection_does_not_name_layer(self):
        cloud, scan = self._scene()
        result = evaluate_side_samples(_ok_samples(cloud, scan), self._calib, SIDE_LEFT)
        self.assertFalse(result.ok)
        self.assertEqual(result.details, (DETAIL_CANNOT_PROJECT,))
        self.assertIsNone(result.layer)
        self.assertIsNone(result.propose_transverse_mirror)
        self.assertIsNone(result.frame_path)
        self.assertFalse(os.path.isfile(draft_path(self._tmpdir.name)))

    def test_package_xml_has_no_opencv(self):
        with open(_PACKAGE_XML, encoding="utf-8") as handle:
            text = handle.read()
        lowered = text.lower()
        self.assertNotIn("opencv", lowered)
        self.assertNotIn("cv2", lowered)
        self.assertNotIn("cv_bridge", lowered)
        self.assertNotIn("pillow", lowered)
        self.assertIn("rclpy", lowered)
        self.assertIn("sensor_msgs", lowered)


if __name__ == "__main__":
    unittest.main()
