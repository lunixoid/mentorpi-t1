#!/usr/bin/env python3
"""Drive pending draft write (SD012 T7)."""
import math
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)
sys.path.insert(0, os.path.dirname(__file__))

from test_drive_evaluate import build_drive_run  # noqa: E402

from mentorpi_calibration.calibration_file import (  # noqa: E402
    FACTORY_POSES,
    draft_path,
    factory_calibration,
    load,
    load_draft,
    save,
)
from mentorpi_calibration.stages.drive import (  # noqa: E402
    IMU_SIGN_OK,
    STAGE_NAME,
    DriveEvaluation,
    commit_drive_pending,
    evaluate_drive_trajectory,
)
from mentorpi_calibration.stages.scan_match import ScanTrajectory  # noqa: E402
from mentorpi_calibration.stages.tilt import wrap_pi  # noqa: E402


class DriveCommitPendingTest(unittest.TestCase):
    def setUp(self):
        self._calib = factory_calibration()
        self._tmpdir = tempfile.TemporaryDirectory()
        self._env_patch = mock.patch.dict(
            os.environ,
            {"T1_CALIBRATION_DIR": self._tmpdir.name},
        )
        self._env_patch.start()

    def tearDown(self):
        self._env_patch.stop()
        self._tmpdir.cleanup()

    def test_success_writes_pending_and_leaves_main_unchanged(self):
        save(self._calib, self._tmpdir.name)
        before_main = load(self._tmpdir.name)
        poses, imu, _scans = build_drive_run(
            (0.09, 0.02, FACTORY_POSES["lidar"]["xyz"][2]),
            0.2,
            FACTORY_POSES["imu"]["rpy"],
        )
        trajectory = ScanTrajectory(
            ok=True,
            poses=tuple(poses),
            travel_m=sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses[:-1], poses[1:])),
            turn_deg=sum(abs(math.degrees(wrap_pi(b.yaw - a.yaw))) for a, b in zip(poses[:-1], poses[1:])),
            mean_residual=0.0,
        )
        evaluation = evaluate_drive_trajectory(trajectory, imu, self._calib)
        self.assertTrue(evaluation.ok, evaluation.details)
        fields = commit_drive_pending(evaluation, self._calib, dir=self._tmpdir.name)
        draft = load_draft(self._tmpdir.name)
        after_main = load(self._tmpdir.name)
        self.assertFalse(draft.absent)
        self.assertIsNotNone(draft.pending)
        self.assertEqual(draft.pending.stage, STAGE_NAME)
        self.assertEqual(draft.pending.sensor, "lidar")
        self.assertIn("yaw", draft.pending.values.get("lidar", {}))
        self.assertEqual(after_main.sensors["lidar"].xyz, before_main.sensors["lidar"].xyz)
        names = [field.name for field in fields]
        self.assertEqual(names[:3], ["lidar_x", "lidar_y", "lidar_yaw"])
        self.assertEqual(fields[-1].name, "imu_yaw_sign")
        self.assertEqual(fields[-1].after, IMU_SIGN_OK)
        self.assertEqual(fields[-1].source, "drive")

    def test_failure_does_not_write_draft(self):
        save(self._calib, self._tmpdir.name)
        evaluation = DriveEvaluation(
            ok=False,
            stage=STAGE_NAME,
            details=("travel too short",),
        )
        fields = commit_drive_pending(evaluation, self._calib, dir=self._tmpdir.name)
        self.assertEqual(fields, ())
        self.assertFalse(os.path.isfile(draft_path(self._tmpdir.name)))


if __name__ == "__main__":
    unittest.main()
