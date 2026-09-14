#!/usr/bin/env python3
"""Draft calibration file API (SD012 T4)."""
import os
import sys
import tempfile
import unittest

import yaml

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.calibration_file import (  # noqa: E402
    DRAFT_FILENAME,
    FIELD_NAMES,
    REASON_NO_CHANGES,
    REASON_NO_DRAFT,
    SCHEMA_VERSION,
    SENSOR_NAMES,
    Calibration,
    SensorPose,
    abort_draft,
    accept_pending,
    draft_path,
    drop_pending,
    factory_calibration,
    load,
    load_draft,
    promote_draft,
    save,
    transverse_mirror,
    transverse_mirror_source,
    write_draft,
)


def _assert_vec(test, actual, expected, places=7):
    test.assertEqual(len(actual), len(expected))
    for got, want in zip(actual, expected):
        test.assertAlmostEqual(got, want, places=places)


CORNER_VALUES = {
    "depth_cam": {"roll": 0.01, "pitch": 0.02, "z": 0.06},
    "imu": {"roll": 0.005, "pitch": -0.003},
}


def _write_corner_pending(tmp, **kwargs):
    defaults = {
        "stage": "corner",
        "sensor": "depth_cam,imu",
        "values": CORNER_VALUES,
        "residual_before": 0.05,
        "residual_after": 0.01,
        "imu_residual_before": 0.02,
        "imu_residual_after": 0.005,
        "cause": "",
    }
    defaults.update(kwargs)
    write_draft(dir=tmp, **defaults)


class DraftCalibrationFileTest(unittest.TestCase):
    def test_load_draft_absent(self):
        with tempfile.TemporaryDirectory() as tmp:
            draft = load_draft(tmp)
        self.assertTrue(draft.absent)
        self.assertTrue(draft.unused)
        self.assertEqual(draft.reason, REASON_NO_DRAFT)
        self.assertIsNone(draft.pending)

    def test_load_draft_broken_yaml(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = draft_path(tmp)
            with open(path, "w", encoding="utf-8") as handle:
                handle.write("not: [yaml: {{\n")
            draft = load_draft(tmp)
        self.assertFalse(draft.absent)
        self.assertTrue(draft.unused)
        self.assertEqual(draft.reason, "unreadable yaml")

    def test_write_draft_creates_from_current_calibration(self):
        original = factory_calibration()
        sensors = dict(original.sensors)
        sensors["depth_cam"] = SensorPose(
            parent="base_link",
            xyz=(0.11, 0.0, 0.05),
            rpy=(0.0, 0.0, 0.0),
            source={name: "computed" for name in FIELD_NAMES},
        )
        written = Calibration(sensors=sensors, unused=False, reason="")
        with tempfile.TemporaryDirectory() as tmp:
            save(written, tmp)
            _write_corner_pending(tmp)
            draft = load_draft(tmp)
            main = load(tmp)
        self.assertFalse(draft.absent)
        self.assertFalse(draft.unused)
        self.assertIsNotNone(draft.pending)
        self.assertEqual(draft.pending.stage, "corner")
        self.assertEqual(draft.pending.sensor, "depth_cam,imu")
        self.assertEqual(draft.pending.cause, "")
        for name in SENSOR_NAMES:
            self.assertEqual(
                draft.sensors[name].xyz,
                main.sensors[name].xyz,
            )
            self.assertEqual(
                dict(draft.sensors[name].source),
                dict(main.sensors[name].source),
            )
        self.assertFalse(os.path.isfile(os.path.join(tmp, DRAFT_FILENAME + ".tmp")))

    def test_write_draft_from_factory_when_no_main_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            draft = load_draft(tmp)
            factory = factory_calibration()
        self.assertFalse(draft.absent)
        for name in SENSOR_NAMES:
            _assert_vec(self, draft.sensors[name].xyz, factory.sensors[name].xyz)
            _assert_vec(self, draft.sensors[name].rpy, factory.sensors[name].rpy)

    def test_write_draft_replaces_only_pending(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp, residual_before=0.1)
            _write_corner_pending(tmp, residual_before=0.2, residual_after=0.02)
            draft = load_draft(tmp)
        self.assertAlmostEqual(draft.pending.residual_before, 0.2)
        self.assertAlmostEqual(draft.pending.residual_after, 0.02)
        factory = factory_calibration()
        for name in SENSOR_NAMES:
            _assert_vec(self, draft.sensors[name].xyz, factory.sensors[name].xyz)

    def test_main_file_unchanged_after_write_draft(self):
        with tempfile.TemporaryDirectory() as tmp:
            save(factory_calibration(), tmp)
            before = load(tmp)
            _write_corner_pending(tmp)
            after = load(tmp)
        self.assertEqual(before.sensors["depth_cam"].xyz, after.sensors["depth_cam"].xyz)
        self.assertFalse(after.unused)

    def test_drop_pending_removes_pending_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            drop_pending(tmp)
            draft = load_draft(tmp)
        self.assertIsNone(draft.pending)
        self.assertFalse(draft.absent)
        self.assertFalse(draft.unused)

    def test_accept_pending_merges_values_with_computed_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            accept_pending(tmp)
            draft = load_draft(tmp)
        self.assertIsNone(draft.pending)
        cam = draft.sensors["depth_cam"]
        _assert_vec(self, cam.rpy, (0.01, 0.02, 0.0))
        self.assertAlmostEqual(cam.xyz[2], 0.06)
        self.assertEqual(cam.source["roll"], "computed")
        self.assertEqual(cam.source["pitch"], "computed")
        self.assertEqual(cam.source["z"], "computed")
        self.assertEqual(cam.source["x"], "factory")
        imu = draft.sensors["imu"]
        self.assertEqual(imu.source["roll"], "computed")
        self.assertEqual(imu.source["pitch"], "computed")
        self.assertEqual(imu.source["yaw"], "factory")

    def test_abort_draft_deletes_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            self.assertTrue(os.path.isfile(draft_path(tmp)))
            abort_draft(tmp)
            self.assertFalse(os.path.isfile(draft_path(tmp)))
            draft = load_draft(tmp)
        self.assertTrue(draft.absent)

    def test_promote_draft_writes_accepted_and_deletes_draft(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            accept_pending(tmp)
            promote_draft(tmp)
            main = load(tmp)
            draft = load_draft(tmp)
        self.assertTrue(draft.absent)
        self.assertFalse(main.unused)
        self.assertAlmostEqual(main.sensors["depth_cam"].xyz[2], 0.06)
        self.assertEqual(main.sensors["depth_cam"].source["z"], "computed")

    def test_promote_draft_ignores_hanging_pending(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            accept_pending(tmp)
            _write_corner_pending(
                tmp,
                values={"depth_cam": {"x": 0.99, "y": 0.99, "z": 0.99}},
            )
            promote_draft(tmp)
            main = load(tmp)
        self.assertFalse(main.unused)
        self.assertAlmostEqual(main.sensors["depth_cam"].xyz[2], 0.06)
        self.assertAlmostEqual(main.sensors["depth_cam"].xyz[0], 0.1)

    def test_promote_draft_fails_without_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            save(factory_calibration(), tmp)
            _write_corner_pending(tmp)
            drop_pending(tmp)
            with self.assertRaises(ValueError) as ctx:
                promote_draft(tmp)
            self.assertEqual(str(ctx.exception), REASON_NO_CHANGES)
            self.assertTrue(os.path.isfile(draft_path(tmp)))

    def test_promote_draft_fails_when_factory_and_no_accepted_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            with self.assertRaises(ValueError) as ctx:
                promote_draft(tmp)
        self.assertEqual(str(ctx.exception), REASON_NO_CHANGES)

    def test_accept_pending_raises_without_pending(self):
        with tempfile.TemporaryDirectory() as tmp:
            _write_corner_pending(tmp)
            accept_pending(tmp)
            with self.assertRaises(ValueError):
                accept_pending(tmp)

    def test_draft_cycle_promotes_transverse_mirror_computed(self):
        self.assertEqual(SCHEMA_VERSION, 1)
        with tempfile.TemporaryDirectory() as tmp:
            write_draft(
                stage="side",
                sensor="depth_cam",
                values={"depth_cam": {"transverse_mirror": True}},
                residual_before=0.0,
                residual_after=0.0,
                imu_residual_before=0.0,
                imu_residual_after=0.0,
                dir=tmp,
            )
            pending_draft = load_draft(tmp)
            self.assertFalse(pending_draft.unused)
            self.assertTrue(pending_draft.pending.values["depth_cam"]["transverse_mirror"])
            accept_pending(tmp)
            accepted = load_draft(tmp)
            self.assertIsNone(accepted.pending)
            self.assertTrue(accepted.depth_cam_transverse_mirror)
            self.assertEqual(accepted.depth_cam_transverse_mirror_source, "computed")
            promote_draft(tmp)
            main = load(tmp)
            path = os.path.join(tmp, "sensor_calibration.yaml")
            with open(path, encoding="utf-8") as handle:
                raw = yaml.safe_load(handle)
        self.assertFalse(main.unused)
        self.assertTrue(transverse_mirror(main))
        self.assertEqual(transverse_mirror_source(main), "computed")
        self.assertEqual(raw["version"], 1)
        self.assertTrue(raw["conventions"]["depth_cam"]["transverse_mirror"])
        self.assertEqual(raw["conventions"]["depth_cam"]["source"], "computed")


if __name__ == "__main__":
    unittest.main()
