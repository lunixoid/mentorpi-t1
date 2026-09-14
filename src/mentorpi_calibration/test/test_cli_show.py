#!/usr/bin/env python3
"""calib CLI contract (SD012 T3/T4)."""
import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest import mock

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.calibration_file import CALIBRATION_FILENAME  # noqa: E402
from mentorpi_calibration.calibration_file import ENV_CALIBRATION_DIR  # noqa: E402
from mentorpi_calibration.calibration_file import accept_pending  # noqa: E402
from mentorpi_calibration.calibration_file import draft_path  # noqa: E402
from mentorpi_calibration.calibration_file import load  # noqa: E402
from mentorpi_calibration.calibration_file import load_draft  # noqa: E402
from mentorpi_calibration.calibration_file import write_draft  # noqa: E402
from mentorpi_calibration.cli import cmd_abort  # noqa: E402
from mentorpi_calibration.cli import cmd_accept  # noqa: E402
from mentorpi_calibration.cli import cmd_reject  # noqa: E402
from mentorpi_calibration.cli import cmd_save  # noqa: E402
from mentorpi_calibration.cli import cmd_show  # noqa: E402
from mentorpi_calibration.cli import main  # noqa: E402

CORNER_VALUES = {
    "depth_cam": {"x": 0.11, "y": 0.01, "z": 0.06, "roll": 0.01, "pitch": 0.02, "yaw": 0.05},
    "imu": {"roll": 0.005, "pitch": -0.003},
}


def _write_corner_pending(directory, **kwargs):
    defaults = {
        "stage": "corner",
        "sensor": "depth_cam,imu",
        "values": CORNER_VALUES,
        "residual_before": 0.05,
        "residual_after": 0.01,
        "imu_residual_before": 0.02,
        "imu_residual_after": 0.005,
        "cause": "",
        "dir": directory,
    }
    defaults.update(kwargs)
    write_draft(**defaults)


class _EnvDirTestCase(unittest.TestCase):
    def _run_in(self, directory, func, *args, **kwargs):
        old = os.environ.get(ENV_CALIBRATION_DIR)
        os.environ[ENV_CALIBRATION_DIR] = directory
        try:
            buf = io.StringIO()
            with redirect_stdout(buf):
                code = func(*args, **kwargs)
        finally:
            if old is None:
                os.environ.pop(ENV_CALIBRATION_DIR, None)
            else:
                os.environ[ENV_CALIBRATION_DIR] = old
        return code, buf.getvalue()

    def _main_in(self, directory, argv):
        return self._run_in(directory, main, argv)


class CalibShowTest(_EnvDirTestCase):
    def test_show_missing_file_prints_factory_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._run_in(tmp, cmd_show)
        self.assertEqual(code, 0)
        self.assertIn("T1CTL_CALIB_OK=1", text)
        self.assertIn("source: factory", text)
        self.assertNotIn("reason:", text)
        self.assertNotIn("draft:", text)
        self.assertIn("pose: camera xyz", text)
        self.assertIn("pose: lidar rpy", text)
        self.assertIn("pose: imu xyz", text)
        self.assertIn(" factory\n", text)
        self.assertIn("180", text)

    def test_show_broken_yaml_is_unused_with_reason(self):
        with tempfile.TemporaryDirectory() as tmp:
            with open(os.path.join(tmp, CALIBRATION_FILENAME), "w", encoding="utf-8") as handle:
                handle.write("this is not: [yaml: {{\n")
            code, text = self._run_in(tmp, cmd_show)
        self.assertEqual(code, 0)
        self.assertIn("T1CTL_CALIB_OK=1", text)
        self.assertIn("source: unused", text)
        self.assertIn("reason: unreadable yaml", text)
        self.assertIn("pose: camera xyz", text)

    def test_show_pending_only_has_no_draft_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            code, text = self._run_in(tmp, cmd_show)
        self.assertEqual(code, 0)
        self.assertNotIn("draft:", text)

    def test_show_accepted_corner_prints_draft_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            accept_pending(tmp)
            code, text = self._run_in(tmp, cmd_show)
        self.assertEqual(code, 0)
        self.assertIn("draft: corner", text)


class CalibDraftCommandsTest(_EnvDirTestCase):
    def test_accept_pending_prints_stage(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            code, text = self._run_in(tmp, cmd_accept)
        self.assertEqual(code, 0)
        self.assertIn("T1CTL_CALIB_OK=1", text)
        self.assertIn("stage: corner", text)

    def test_accept_without_pending_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            accept_pending(tmp)
            code, text = self._run_in(tmp, cmd_accept)
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("detail: no pending", text)

    def test_reject_pending_prints_stage(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            code, text = self._run_in(tmp, cmd_reject)
        self.assertEqual(code, 0)
        self.assertIn("T1CTL_CALIB_OK=1", text)
        self.assertIn("stage: corner", text)
        draft = load_draft(tmp)
        self.assertIsNone(draft.pending)

    def test_reject_without_pending_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._run_in(tmp, cmd_reject)
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("detail: no draft", text)

    def test_save_after_accept_promotes_and_prints_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            accept_pending(tmp)
            code, text = self._run_in(tmp, cmd_save)
            main_calib = load(tmp)
            draft = load_draft(tmp)
        self.assertEqual(code, 0)
        self.assertIn("T1CTL_CALIB_OK=1", text)
        self.assertIn("field: camera_z", text)
        self.assertTrue(draft.absent)
        self.assertFalse(main_calib.unused)
        self.assertAlmostEqual(main_calib.sensors["depth_cam"].xyz[2], 0.06)

    def test_save_without_changes_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            code, text = self._run_in(tmp, cmd_save)
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("detail: no changes", text)

    def test_abort_deletes_draft(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            code, text = self._run_in(tmp, cmd_abort)
            draft = load_draft(tmp)
        self.assertEqual(code, 0)
        self.assertIn("T1CTL_CALIB_OK=1", text)
        self.assertTrue(draft.absent)
        self.assertFalse(os.path.isfile(draft_path(tmp)))


class CalibStageStubsTest(_EnvDirTestCase):
    def test_main_floor_is_unknown(self):
        code, text = self._run_in(".", main, ["floor"])
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("detail: unknown command", text)

    def test_main_corner_collect_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch(
                "mentorpi_calibration.cli.collect_corner_samples",
            ) as collect_mock:
                collect_mock.return_value = mock.Mock(
                    ok=False,
                    details=("no camera depth",),
                )
                code, text = self._main_in(tmp, ["corner"])
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("stage: corner", text)
        self.assertIn("detail: no camera depth", text)
        self.assertNotIn("detail: unknown command", text)

    def test_main_drive_collect_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch(
                "mentorpi_calibration.cli.collect_drive_samples",
            ) as collect_mock:
                collect_mock.return_value = mock.Mock(
                    ok=False,
                    details=("travel too short",),
                )
                code, text = self._main_in(tmp, ["drive"])
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("stage: drive", text)
        self.assertIn("detail: travel too short", text)
        self.assertNotIn("detail: unknown command", text)

    def test_main_drive_proposal(self):
        evaluation = mock.Mock(
            ok=True,
            residual_before=0.04,
            residual_after=0.01,
            imu_yaw_sign="ok",
            details=(),
        )
        yaw_field = mock.Mock()
        yaw_field.name = "lidar_yaw"
        yaw_field.before = 180.0
        yaw_field.after = 2.4
        yaw_field.source = "computed"
        sign_field = mock.Mock()
        sign_field.name = "imu_yaw_sign"
        sign_field.before = "ok"
        sign_field.after = "ok"
        sign_field.source = "drive"
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch(
                "mentorpi_calibration.cli.collect_drive_samples",
            ) as collect_mock, mock.patch(
                "mentorpi_calibration.cli.evaluate_drive_samples",
                return_value=evaluation,
            ), mock.patch(
                "mentorpi_calibration.cli.drive_operator_fields",
                return_value=(yaw_field, sign_field),
            ):
                collect_mock.return_value = mock.Mock(ok=True, details=())
                code, text = self._main_in(tmp, ["drive"])
        self.assertEqual(code, 0)
        self.assertIn("T1CTL_CALIB_OK=1", text)
        self.assertIn("stage: drive", text)
        self.assertIn("residual_before: 0.04", text)
        self.assertIn("residual_after: 0.01", text)
        self.assertIn("field: lidar_yaw 180.0 2.4 computed", text)
        self.assertIn("field: imu_yaw_sign ok ok drive", text)

    def test_main_lidar_proposal(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(
                tmp,
                ["lidar", "--height", "0.168", "--pitch", "0", "--roll", "0"],
            )
            draft = load_draft(tmp)
        self.assertEqual(code, 0)
        self.assertIn("T1CTL_CALIB_OK=1", text)
        self.assertIn("stage: lidar", text)
        self.assertIn("field: lidar_z", text)
        self.assertIn(" measured\n", text)
        self.assertEqual(draft.pending.stage, "lidar")

    def test_main_camera_proposal(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(tmp, ["camera", "--height", "0.18"])
            draft = load_draft(tmp)
        self.assertEqual(code, 0)
        self.assertIn("stage: camera", text)
        self.assertIn("field: camera_z", text)
        self.assertNotIn("field: camera_x", text)
        self.assertEqual(list(draft.pending.values["depth_cam"].keys()), ["z"])

    def test_main_unknown_command_fails_without_poses(self):
        buf = io.StringIO()
        with redirect_stdout(buf):
            code = main(["rotate"])
        self.assertEqual(code, 1)
        text = buf.getvalue()
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("detail: unknown command", text)
        self.assertNotIn("pose:", text)


if __name__ == "__main__":
    unittest.main()
