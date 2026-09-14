#!/usr/bin/env python3
"""calib side CLI contract (SD022 T4 / I1)."""
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from test_cli_show import _EnvDirTestCase  # noqa: E402

from mentorpi_calibration.calibration_file import load_draft  # noqa: E402
from mentorpi_calibration.calibration_file import transverse_mirror  # noqa: E402
from mentorpi_calibration.calibration_file import transverse_mirror_source  # noqa: E402
from mentorpi_calibration.stages.side import FIELD_CAMERA_TRANSVERSE_MIRROR  # noqa: E402
from mentorpi_calibration.stages.side import LAYER_CLOUD_GEOMETRY  # noqa: E402
from mentorpi_calibration.stages.side import SIDE_LEFT  # noqa: E402
from mentorpi_calibration.stages.side import SIDE_RIGHT  # noqa: E402
from mentorpi_calibration.stages.side import STAGE_NAME  # noqa: E402
from mentorpi_calibration.stages.side import SideEvaluation  # noqa: E402
from mentorpi_calibration.stages.side import commit_side_pending  # noqa: E402

SIDE_FRAME_PATH = "/home/ubuntu/mentorpi_t1_ws/config/platform/t1/side_frame.png"

SUCCESS_STDOUT = (
    "T1CTL_CALIB_OK=1\n"
    "stage: side\n"
    "observed: operator left\n"
    "observed: scan left 24.6\n"
    "observed: cloud right -25.1\n"
    "layer: cloud_geometry\n"
    "frame: {}\n"
    "field: {} 0 1 computed\n"
).format(SIDE_FRAME_PATH, FIELD_CAMERA_TRANSVERSE_MIRROR)


def _success_evaluation():
    return SideEvaluation(
        ok=True,
        stage=STAGE_NAME,
        details=(),
        operator_side=SIDE_LEFT,
        scan_side=SIDE_LEFT,
        cloud_side=SIDE_RIGHT,
        scan_bearing_deg=24.6,
        cloud_bearing_deg=-25.1,
        layer=LAYER_CLOUD_GEOMETRY,
        propose_transverse_mirror=True,
        frame_path=SIDE_FRAME_PATH,
    )


class CalibSideCliTest(_EnvDirTestCase):
    def _patch_stage(self, evaluation, *, commit=False):
        collect_patch = mock.patch("mentorpi_calibration.cli.collect_side_samples")
        evaluate_patch = mock.patch("mentorpi_calibration.cli.evaluate_side_samples")
        collect_mock = collect_patch.start()
        evaluate_mock = evaluate_patch.start()
        self.addCleanup(collect_patch.stop)
        self.addCleanup(evaluate_patch.stop)
        collect_mock.return_value = mock.Mock(ok=True, details=())
        if commit:

            def _evaluate(samples, calib, operator_side, dir=None):
                commit_side_pending(evaluation, calib, dir=dir)
                return evaluation

            evaluate_mock.side_effect = _evaluate
        else:
            evaluate_mock.return_value = evaluation
        return collect_mock, evaluate_mock

    def test_side_success_prints_i1_contract(self):
        self._patch_stage(_success_evaluation())
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(tmp, ["side", "--side", "left"])
        self.assertEqual(code, 0)
        self.assertEqual(text, SUCCESS_STDOUT)

    def test_side_collect_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch("mentorpi_calibration.cli.collect_side_samples") as collect_mock:
                collect_mock.return_value = mock.Mock(
                    ok=False,
                    details=("no camera depth", "no /scan"),
                )
                code, text = self._main_in(tmp, ["side", "--side", "left"])
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("stage: side", text)
        self.assertIn("detail: no camera depth", text)
        self.assertIn("detail: no /scan", text)
        self.assertNotIn("detail: unknown command", text)

    def test_side_evaluate_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch(
                "mentorpi_calibration.cli.collect_side_samples",
            ) as collect_mock, mock.patch(
                "mentorpi_calibration.cli.evaluate_side_samples",
            ) as evaluate_mock:
                collect_mock.return_value = mock.Mock(ok=True, details=())
                evaluate_mock.return_value = mock.Mock(
                    ok=False,
                    details=("object not found in cloud",),
                )
                code, text = self._main_in(tmp, ["side", "--side", "right"])
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("stage: side", text)
        self.assertIn("detail: object not found in cloud", text)

    def test_side_missing_argument_is_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(tmp, ["side"])
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("stage: side", text)
        self.assertIn("detail: incomplete options", text)

    def test_side_invalid_choice(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(tmp, ["side", "--side", "front"])
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("stage: side", text)
        self.assertIn("detail: invalid value", text)

    def test_show_and_accept_after_side(self):
        self._patch_stage(_success_evaluation(), commit=True)
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(tmp, ["side", "--side", "left"])
            self.assertEqual(code, 0)
            self.assertEqual(text, SUCCESS_STDOUT)
            pending = load_draft(tmp)
            self.assertEqual(pending.pending.stage, STAGE_NAME)
            self.assertTrue(pending.pending.values["depth_cam"]["transverse_mirror"])

            code, text = self._main_in(tmp, ["show"])
            self.assertEqual(code, 0)
            self.assertNotIn("draft:", text)

            code, text = self._main_in(tmp, ["accept"])
            self.assertEqual(code, 0)
            self.assertIn("T1CTL_CALIB_OK=1", text)
            self.assertIn("stage: side", text)
            accepted = load_draft(tmp)
            self.assertIsNone(accepted.pending)
            self.assertTrue(accepted.depth_cam_transverse_mirror)
            self.assertEqual(accepted.depth_cam_transverse_mirror_source, "computed")
            self.assertTrue(transverse_mirror(accepted))
            self.assertEqual(transverse_mirror_source(accepted), "computed")

            code, text = self._main_in(tmp, ["show"])
        self.assertEqual(code, 0)
        self.assertIn("draft: side", text)


if __name__ == "__main__":
    unittest.main()
