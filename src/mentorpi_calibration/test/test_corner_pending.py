#!/usr/bin/env python3
"""Corner pending draft write (SD012 T5)."""
import os
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)
sys.path.insert(0, os.path.dirname(__file__))

from test_corner_evaluate import _accel_for_mount_rpy, _ok_samples, build_corner_scene  # noqa: E402

from mentorpi_calibration.calibration_file import (  # noqa: E402
    FACTORY_POSES,
    draft_path,
    drop_pending,
    factory_calibration,
    load,
    load_draft,
    save,
)
from mentorpi_calibration.stages.corner import (  # noqa: E402
    DETAIL_NO_IMU,
    CornerEvaluation,
    commit_corner_pending,
    evaluate_corner_samples,
)


class CornerCommitPendingTest(unittest.TestCase):
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

    def tearDown(self):
        self._env_patch.stop()
        self._tmpdir.cleanup()

    def _successful_evaluation(self, *, include_floor=True):
        cloud, scan = build_corner_scene(
            self._true_cam_xyz,
            self._true_cam_rpy,
            self._lidar_xyz,
            self._lidar_rpy,
            include_floor=include_floor,
        )
        accel = _accel_for_mount_rpy(FACTORY_POSES["imu"]["rpy"])
        return evaluate_corner_samples(
            _ok_samples(cloud, scan, (accel, accel)),
            self._calib,
        )

    def test_success_writes_pending_and_leaves_main_unchanged(self):
        save(self._calib, self._tmpdir.name)
        before_main = load(self._tmpdir.name)
        evaluation = self._successful_evaluation()
        self.assertTrue(evaluation.ok, evaluation.details)
        draft = load_draft(self._tmpdir.name)
        after_main = load(self._tmpdir.name)
        self.assertFalse(draft.absent)
        self.assertIsNotNone(draft.pending)
        self.assertEqual(draft.pending.stage, "corner")
        self.assertEqual(draft.pending.sensor, "depth_cam,lidar,imu")
        self.assertEqual(before_main.sensors, after_main.sensors)

    def test_no_z_in_pending_without_floor(self):
        evaluation = self._successful_evaluation(include_floor=False)
        self.assertTrue(evaluation.ok, evaluation.details)
        draft = load_draft(self._tmpdir.name)
        depth_cam = draft.pending.values.get("depth_cam", {})
        self.assertNotIn("z", depth_cam)

    def test_no_pending_when_evaluation_not_ok(self):
        failed = CornerEvaluation(
            ok=False,
            stage="corner",
            details=(DETAIL_NO_IMU,),
        )
        commit_corner_pending(failed, self._calib, dir=self._tmpdir.name)
        draft = load_draft(self._tmpdir.name)
        self.assertTrue(draft.absent)

    def test_drop_pending_after_commit(self):
        self._successful_evaluation()
        drop_pending(self._tmpdir.name)
        draft = load_draft(self._tmpdir.name)
        self.assertIsNone(draft.pending)
        self.assertFalse(draft.absent)

    def test_main_file_absent_still_writes_draft(self):
        evaluation = self._successful_evaluation()
        self.assertTrue(evaluation.ok, evaluation.details)
        self.assertTrue(os.path.isfile(draft_path(self._tmpdir.name)))
        self.assertFalse(os.path.isfile(os.path.join(self._tmpdir.name, "sensor_calibration.yaml")))


if __name__ == "__main__":
    unittest.main()
