#!/usr/bin/env python3
"""Manual lidar/camera input (SD012 T6)."""
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from test_cli_show import _EnvDirTestCase  # noqa: E402

from mentorpi_calibration.calibration_file import (  # noqa: E402
    BASE_LINK_OFFSET_Z,
    FACTORY_POSES,
    Calibration,
    SensorPose,
    accept_pending,
    factory_calibration,
    load,
    load_draft,
    operator_display,
    operator_height_to_mount_z,
    operator_rpy_deg_to_mount,
    pending_field_source,
    write_draft,
)
from mentorpi_calibration.cli import cmd_accept  # noqa: E402
from mentorpi_calibration.cli import cmd_save  # noqa: E402
from mentorpi_calibration.stages.manual import (  # noqa: E402
    DETAIL_INVALID_VALUE,
    DETAIL_OUT_OF_RANGE,
    SOURCE_MEASURED,
    evaluate_camera,
    evaluate_lidar,
)

CORNER_VALUES = {
    "depth_cam": {"x": 0.11, "y": 0.01, "z": 0.06, "roll": 0.01, "pitch": 0.02, "yaw": 0.05},
    "imu": {"roll": 0.005, "pitch": -0.003},
}


class OperatorMountRoundtripTest(unittest.TestCase):
    def test_pending_source_by_stage(self):
        self.assertEqual(pending_field_source("lidar"), "measured")
        self.assertEqual(pending_field_source("camera"), "measured")
        self.assertEqual(pending_field_source("corner"), "computed")
        self.assertEqual(pending_field_source("drive"), "computed")

    def test_height_subtracts_base_link_offset(self):
        self.assertAlmostEqual(operator_height_to_mount_z(0.168), 0.168 - BASE_LINK_OFFSET_Z)

    def test_lidar_rpy_roundtrip_preserves_operator_angles(self):
        calib = factory_calibration()
        _, factory_rpy, _, _ = operator_display(calib, "lidar")
        mount = operator_rpy_deg_to_mount("lidar", (2.0, -5.0, factory_rpy[2]))
        sensors = dict(calib.sensors)
        pose = sensors["lidar"]
        sensors["lidar"] = SensorPose(
            parent=pose.parent,
            xyz=pose.xyz,
            rpy=(mount[0], mount[1], pose.rpy[2]),
            source=pose.source,
        )
        proposed = Calibration(sensors=sensors, unused=False, reason="")
        _, after_rpy, _, _ = operator_display(proposed, "lidar")
        self.assertAlmostEqual(after_rpy[0], 2.0, places=7)
        self.assertAlmostEqual(after_rpy[1], -5.0, places=7)
        self.assertAlmostEqual(after_rpy[2], factory_rpy[2], places=7)


class ManualEvaluateTest(unittest.TestCase):
    def setUp(self):
        self.calib = factory_calibration()

    def test_lidar_proposal_uses_operator_height_and_measured(self):
        evaluation = evaluate_lidar(0.168, 0.0, 0.0, self.calib)
        self.assertTrue(evaluation.ok, evaluation.details)
        self.assertEqual(evaluation.stage, "lidar")
        self.assertEqual(evaluation.sensor, "lidar")
        self.assertEqual(set(evaluation.values["lidar"].keys()), {"z", "roll", "pitch"})
        self.assertNotIn("x", evaluation.values["lidar"])
        self.assertNotIn("yaw", evaluation.values["lidar"])
        names = [field.name for field in evaluation.fields]
        self.assertEqual(names, ["lidar_z", "lidar_pitch", "lidar_roll"])
        self.assertTrue(all(field.source == SOURCE_MEASURED for field in evaluation.fields))
        z_field = evaluation.fields[0]
        self.assertAlmostEqual(z_field.after, 0.168, places=7)
        self.assertAlmostEqual(z_field.before, FACTORY_POSES["lidar"]["xyz"][2] + BASE_LINK_OFFSET_Z)

    def test_camera_proposal_only_height(self):
        evaluation = evaluate_camera(0.18, self.calib)
        self.assertTrue(evaluation.ok, evaluation.details)
        self.assertEqual(evaluation.stage, "camera")
        self.assertEqual(list(evaluation.values["depth_cam"].keys()), ["z"])
        self.assertEqual([field.name for field in evaluation.fields], ["camera_z"])
        self.assertAlmostEqual(evaluation.fields[0].after, 0.18, places=7)
        self.assertAlmostEqual(evaluation.values["depth_cam"]["z"], 0.18 - BASE_LINK_OFFSET_Z, places=7)

    def test_out_of_range_height(self):
        evaluation = evaluate_lidar(9.0, 0.0, 0.0, self.calib)
        self.assertFalse(evaluation.ok)
        self.assertEqual(evaluation.details, (DETAIL_OUT_OF_RANGE,))

    def test_out_of_range_pitch(self):
        evaluation = evaluate_lidar(0.168, 400.0, 0.0, self.calib)
        self.assertFalse(evaluation.ok)
        self.assertEqual(evaluation.details, (DETAIL_OUT_OF_RANGE,))

    def test_non_finite_is_invalid(self):
        evaluation = evaluate_camera(math.nan, self.calib)
        self.assertFalse(evaluation.ok)
        self.assertEqual(evaluation.details, (DETAIL_INVALID_VALUE,))
        evaluation = evaluate_lidar(0.168, math.inf, 0.0, self.calib)
        self.assertFalse(evaluation.ok)
        self.assertEqual(evaluation.details, (DETAIL_INVALID_VALUE,))

    def test_zero_height_is_out_of_range(self):
        evaluation = evaluate_camera(0.0, self.calib)
        self.assertFalse(evaluation.ok)
        self.assertEqual(evaluation.details, (DETAIL_OUT_OF_RANGE,))


class ManualCliTest(_EnvDirTestCase):
    def test_lidar_writes_pending_measured_and_prints_delta(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(
                tmp,
                ["lidar", "--height", "0.168", "--pitch", "0", "--roll", "0"],
            )
            draft = load_draft(tmp)
            main_calib = load(tmp)
        self.assertEqual(code, 0)
        self.assertIn("T1CTL_CALIB_OK=1", text)
        self.assertIn("stage: lidar", text)
        self.assertIn("field: lidar_z", text)
        self.assertIn(" measured\n", text)
        self.assertIsNotNone(draft.pending)
        self.assertEqual(draft.pending.stage, "lidar")
        self.assertEqual(set(draft.pending.values["lidar"].keys()), {"z", "roll", "pitch"})
        self.assertTrue(main_calib.unused)
        self.assertEqual(main_calib.reason, "no file")

    def test_camera_writes_only_z(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(tmp, ["camera", "--height", "0.18"])
            draft = load_draft(tmp)
            current = factory_calibration().sensors["depth_cam"]
        self.assertEqual(code, 0)
        self.assertIn("stage: camera", text)
        self.assertIn("field: camera_z", text)
        self.assertNotIn("field: camera_x", text)
        self.assertNotIn("field: camera_pitch", text)
        self.assertEqual(list(draft.pending.values["depth_cam"].keys()), ["z"])
        self.assertEqual(draft.sensors["depth_cam"].xyz, current.xyz)
        self.assertEqual(draft.sensors["depth_cam"].rpy, current.rpy)

    def test_accept_save_keeps_measured_and_computed_sources(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_draft(
                stage="corner",
                sensor="depth_cam,imu",
                values=CORNER_VALUES,
                residual_before=0.05,
                residual_after=0.01,
                imu_residual_before=0.02,
                imu_residual_after=0.005,
                dir=tmp,
            )
            accept_pending(tmp)
            code_lidar, _ = self._main_in(
                tmp,
                ["lidar", "--height", "0.170", "--pitch", "0", "--roll", "0"],
            )
            self.assertEqual(code_lidar, 0)
            code_accept, _ = self._run_in(tmp, cmd_accept)
            self.assertEqual(code_accept, 0)
            code_cam, _ = self._main_in(tmp, ["camera", "--height", "0.18"])
            self.assertEqual(code_cam, 0)
            code_accept2, _ = self._run_in(tmp, cmd_accept)
            self.assertEqual(code_accept2, 0)
            code_save, text = self._run_in(tmp, cmd_save)
            main_calib = load(tmp)
        self.assertEqual(code_save, 0)
        self.assertIn("field: lidar_z", text)
        self.assertIn(" measured\n", text)
        lidar = main_calib.sensors["lidar"]
        cam = main_calib.sensors["depth_cam"]
        self.assertEqual(lidar.source["z"], "measured")
        self.assertEqual(lidar.source["roll"], "measured")
        self.assertEqual(lidar.source["pitch"], "measured")
        self.assertEqual(lidar.source["x"], "factory")
        self.assertEqual(lidar.source["yaw"], "factory")
        self.assertEqual(cam.source["z"], "measured")
        self.assertEqual(cam.source["x"], "computed")
        self.assertEqual(cam.source["pitch"], "computed")
        self.assertAlmostEqual(cam.xyz[0], 0.11)
        self.assertAlmostEqual(cam.xyz[2], 0.18 - BASE_LINK_OFFSET_Z)

    def test_incomplete_lidar_options_leave_draft_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(tmp, ["lidar", "--height", "0.168", "--pitch", "0"])
            draft = load_draft(tmp)
        self.assertEqual(code, 1)
        self.assertIn("T1CTL_CALIB_OK=0", text)
        self.assertIn("stage: lidar", text)
        self.assertIn("detail: incomplete options", text)
        self.assertTrue(draft.absent)

    def test_missing_camera_height_is_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(tmp, ["camera"])
            draft = load_draft(tmp)
        self.assertEqual(code, 1)
        self.assertIn("detail: incomplete options", text)
        self.assertTrue(draft.absent)

    def test_unit_suffix_is_invalid_value(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(
                tmp,
                ["lidar", "--height", "0.168m", "--pitch", "0", "--roll", "0"],
            )
            draft = load_draft(tmp)
        self.assertEqual(code, 1)
        self.assertIn("detail: invalid value", text)
        self.assertTrue(draft.absent)

    def test_out_of_range_does_not_write_draft(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, text = self._main_in(
                tmp,
                ["camera", "--height", "9"],
            )
            draft = load_draft(tmp)
        self.assertEqual(code, 1)
        self.assertIn("detail: out of range", text)
        self.assertTrue(draft.absent)


if __name__ == "__main__":
    unittest.main()
