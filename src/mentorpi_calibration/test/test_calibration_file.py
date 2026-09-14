#!/usr/bin/env python3
"""Calibration file load/save and TF apply helpers (SD012 T1–T2)."""
import math
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
    BASE_LINK_OFFSET_Z,
    FALLBACK_CHILD_FRAMES,
    FALLBACK_PARENT_FRAME,
    FIELD_NAMES,
    SCHEMA_VERSION,
    SENSOR_NAMES,
    Calibration,
    SensorPose,
    factory_calibration,
    fallback_tf_args,
    load,
    operator_display,
    operator_source,
    save,
    transverse_mirror,
    transverse_mirror_source,
    xacro_mappings,
)

# Exact origin strings from mentorpi_description xacro (AC1, 7 decimal places).
XACRO_LIDAR_XYZ = (0.0900034859353204, 0.0, 0.0405195774179554)
XACRO_LIDAR_RPY = (0.0, 0.0, 0.0)
XACRO_IMU_XYZ = (0.0048416, 0.011168, -0.0057398)
XACRO_M_PI = 3.1415926535897931
XACRO_IMU_RPY = (XACRO_M_PI, 0.0, -XACRO_M_PI / 2.0)
XACRO_DEPTH_XYZ = (0.10, 0.0, 0.05)
XACRO_DEPTH_RPY = (0.0, 0.0, 0.0)

I11_EXAMPLE = """\
version: 1
sensors:
  lidar:   {parent: base_link, xyz: [0.0900035, 0.0, 0.0405196], rpy: [0.0, 0.0, 0.0],
            source: {x: computed, y: computed, z: measured, roll: measured, pitch: measured, yaw: computed}}
  imu:     {parent: base_link, xyz: [0.0048416, 0.011168, -0.0057398], rpy: [3.1415927, 0.0, -1.5707963],
            source: {roll: computed, pitch: computed, yaw: factory, x: factory, y: factory, z: factory}}
  depth_cam: {parent: base_link, xyz: [0.10, 0.0, 0.05], rpy: [0.0, 0.0, 0.0],
            source: {x: computed, y: computed, z: computed, roll: computed, pitch: computed, yaw: computed}}
"""


def _assert_vec(test, actual, expected, places=7):
    test.assertEqual(len(actual), len(expected))
    for got, want in zip(actual, expected):
        test.assertAlmostEqual(got, want, places=places)


class CalibrationFileTest(unittest.TestCase):
    def test_missing_file_returns_factory_unused_no_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            calib = load(tmp)
        self.assertTrue(calib.unused)
        self.assertEqual(calib.reason, "no file")
        self._assert_factory_poses(calib)
        for name in SENSOR_NAMES:
            for field in FIELD_NAMES:
                self.assertEqual(calib.sensors[name].source[field], "factory")

    def test_factory_matches_xacro_to_seven_places(self):
        calib = factory_calibration()
        self._assert_factory_poses(calib)

    def test_save_load_roundtrip_values_and_sources(self):
        sources = {
            "lidar": {
                "x": "computed",
                "y": "computed",
                "z": "measured",
                "roll": "measured",
                "pitch": "measured",
                "yaw": "computed",
            },
            "imu": {
                "x": "factory",
                "y": "factory",
                "z": "factory",
                "roll": "computed",
                "pitch": "computed",
                "yaw": "factory",
            },
            "depth_cam": {name: "computed" for name in FIELD_NAMES},
        }
        original = factory_calibration()
        sensors = {}
        for name in SENSOR_NAMES:
            pose = original.sensors[name]
            if name == "depth_cam":
                xyz = (0.11, 0.01, 0.06)
                rpy = (0.02, -0.03, 0.04)
            else:
                xyz = pose.xyz
                rpy = pose.rpy
            sensors[name] = SensorPose(
                parent=pose.parent,
                xyz=xyz,
                rpy=rpy,
                source=sources[name],
            )
        written = Calibration(sensors=sensors, unused=False, reason="")
        with tempfile.TemporaryDirectory() as tmp:
            save(written, tmp)
            loaded = load(tmp)
        self.assertFalse(loaded.unused)
        self.assertEqual(loaded.reason, "")
        for name in SENSOR_NAMES:
            _assert_vec(self, loaded.sensors[name].xyz, written.sensors[name].xyz)
            _assert_vec(self, loaded.sensors[name].rpy, written.sensors[name].rpy)
            self.assertEqual(
                dict(loaded.sensors[name].source),
                dict(written.sensors[name].source),
            )
            self.assertEqual(loaded.sensors[name].parent, "base_link")

    def test_factory_save_load_keeps_seven_places(self):
        with tempfile.TemporaryDirectory() as tmp:
            save(factory_calibration(), tmp)
            loaded = load(tmp)
        self.assertFalse(loaded.unused)
        self._assert_factory_poses(loaded)

    def test_i11_example_loads(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "sensor_calibration.yaml")
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(I11_EXAMPLE)
            calib = load(tmp)
        self.assertFalse(calib.unused)
        self.assertEqual(calib.reason, "")
        self.assertEqual(calib.sensors["lidar"].source["z"], "measured")
        self.assertEqual(calib.sensors["imu"].source["roll"], "computed")
        _assert_vec(self, calib.sensors["lidar"].xyz, (0.0900035, 0.0, 0.0405196))

    def test_file_without_conventions_is_false_factory_and_used(self):
        self.assertEqual(SCHEMA_VERSION, 1)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "sensor_calibration.yaml")
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(I11_EXAMPLE)
            calib = load(tmp)
            with open(path, encoding="utf-8") as handle:
                raw = yaml.safe_load(handle)
        self.assertNotIn("conventions", raw)
        self.assertFalse(calib.unused)
        self.assertEqual(calib.reason, "")
        self.assertFalse(transverse_mirror(calib))
        self.assertEqual(transverse_mirror_source(calib), "factory")

    def test_conventions_roundtrip_keeps_schema_version_one(self):
        original = factory_calibration()
        written = Calibration(
            sensors=original.sensors,
            unused=False,
            reason="",
            depth_cam_transverse_mirror=True,
            depth_cam_transverse_mirror_source="computed",
        )
        with tempfile.TemporaryDirectory() as tmp:
            save(written, tmp)
            path = os.path.join(tmp, "sensor_calibration.yaml")
            with open(path, encoding="utf-8") as handle:
                raw = yaml.safe_load(handle)
            loaded = load(tmp)
        self.assertEqual(raw["version"], 1)
        self.assertTrue(raw["conventions"]["depth_cam"]["transverse_mirror"])
        self.assertEqual(raw["conventions"]["depth_cam"]["source"], "computed")
        self.assertFalse(loaded.unused)
        self.assertTrue(transverse_mirror(loaded))
        self.assertEqual(transverse_mirror_source(loaded), "computed")

    def test_broken_yaml_returns_factory_without_raising(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "sensor_calibration.yaml")
            with open(path, "w", encoding="utf-8") as handle:
                handle.write("this is not: [yaml: {{\n")
            calib = load(tmp)
        self.assertTrue(calib.unused)
        self.assertEqual(calib.reason, "unreadable yaml")
        self._assert_factory_poses(calib)

    def test_unknown_version_returns_factory_without_raising(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "sensor_calibration.yaml")
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(I11_EXAMPLE.replace("version: 1", "version: 99"))
            calib = load(tmp)
        self.assertTrue(calib.unused)
        self.assertEqual(calib.reason, "unknown version")
        self._assert_factory_poses(calib)

    def test_out_of_range_returns_factory(self):
        with tempfile.TemporaryDirectory() as tmp:
            save(factory_calibration(), tmp)
            path = os.path.join(tmp, "sensor_calibration.yaml")
            with open(path, encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
            data["sensors"]["lidar"]["xyz"][0] = 9.0
            with open(path, "w", encoding="utf-8") as handle:
                yaml.safe_dump(data, handle)
            calib = load(tmp)
        self.assertTrue(calib.unused)
        self.assertEqual(calib.reason, "out of range")
        self._assert_factory_poses(calib)

    def test_env_dir_overrides_default(self):
        previous = os.environ.get("T1_CALIBRATION_DIR")
        with tempfile.TemporaryDirectory() as tmp:
            os.environ["T1_CALIBRATION_DIR"] = tmp
            try:
                calib = load()
                self.assertTrue(calib.unused)
                self.assertEqual(calib.reason, "no file")
            finally:
                if previous is None:
                    os.environ.pop("T1_CALIBRATION_DIR", None)
                else:
                    os.environ["T1_CALIBRATION_DIR"] = previous

    def _assert_factory_poses(self, calib):
        _assert_vec(self, calib.sensors["lidar"].xyz, XACRO_LIDAR_XYZ)
        _assert_vec(self, calib.sensors["lidar"].rpy, XACRO_LIDAR_RPY)
        _assert_vec(self, calib.sensors["imu"].xyz, XACRO_IMU_XYZ)
        _assert_vec(self, calib.sensors["imu"].rpy, XACRO_IMU_RPY)
        _assert_vec(self, calib.sensors["depth_cam"].xyz, XACRO_DEPTH_XYZ)
        _assert_vec(self, calib.sensors["depth_cam"].rpy, XACRO_DEPTH_RPY)
        for name in SENSOR_NAMES:
            self.assertEqual(calib.sensors[name].parent, "base_link")


# Composed factory fallback as published today by _TEMP_*_TF (8-arg yaw pitch roll).
_FACTORY_FALLBACK = {
    "lidar": (
        0.0900034859353204,
        0.0,
        0.1675195774179554,
        3.1415926535897931,
        0.0,
        0.0,
    ),
    "depth_cam": (
        0.10,
        0.0,
        0.177,
        -1.5707963267948966,
        0.0,
        -1.5707963267948966,
    ),
    "imu": (
        0.0048416,
        0.011168,
        0.1212602,
        -1.5707963267948966,
        0.0,
        3.1415926535897931,
    ),
}

_XACRO_MAPPING_KEYS = (
    "lidar_x",
    "lidar_y",
    "lidar_z",
    "lidar_roll",
    "lidar_pitch",
    "lidar_yaw",
    "imu_x",
    "imu_y",
    "imu_z",
    "imu_roll",
    "imu_pitch",
    "imu_yaw",
    "depth_cam_x",
    "depth_cam_y",
    "depth_cam_z",
    "depth_cam_roll",
    "depth_cam_pitch",
    "depth_cam_yaw",
)


def _pose_with(name, xyz=None, rpy=None):
    original = factory_calibration()
    sensors = dict(original.sensors)
    pose = sensors[name]
    sensors[name] = SensorPose(
        parent=pose.parent,
        xyz=xyz if xyz is not None else pose.xyz,
        rpy=rpy if rpy is not None else pose.rpy,
        source=pose.source,
    )
    return Calibration(sensors=sensors, unused=False, reason="")


class CalibrationApplyTest(unittest.TestCase):
    def test_xacro_mappings_factory_keys_and_values(self):
        mappings = xacro_mappings(factory_calibration())
        self.assertEqual(tuple(mappings.keys()), _XACRO_MAPPING_KEYS)
        for value in mappings.values():
            self.assertIsInstance(value, str)
        self.assertAlmostEqual(float(mappings["lidar_x"]), XACRO_LIDAR_XYZ[0], places=7)
        self.assertAlmostEqual(float(mappings["lidar_z"]), XACRO_LIDAR_XYZ[2], places=7)
        self.assertAlmostEqual(float(mappings["imu_x"]), XACRO_IMU_XYZ[0], places=7)
        self.assertAlmostEqual(float(mappings["imu_roll"]), XACRO_IMU_RPY[0], places=7)
        self.assertAlmostEqual(float(mappings["imu_yaw"]), XACRO_IMU_RPY[2], places=7)
        self.assertAlmostEqual(float(mappings["depth_cam_x"]), XACRO_DEPTH_XYZ[0], places=7)
        self.assertAlmostEqual(float(mappings["depth_cam_z"]), XACRO_DEPTH_XYZ[2], places=7)

    def test_xacro_mappings_follows_calibration(self):
        calib = _pose_with("lidar", xyz=(0.2, 0.01, 0.03), rpy=(0.0, 0.0, 0.4))
        mappings = xacro_mappings(calib)
        self.assertAlmostEqual(float(mappings["lidar_x"]), 0.2, places=7)
        self.assertAlmostEqual(float(mappings["lidar_y"]), 0.01, places=7)
        self.assertAlmostEqual(float(mappings["lidar_z"]), 0.03, places=7)
        self.assertAlmostEqual(float(mappings["lidar_yaw"]), 0.4, places=7)
        self.assertAlmostEqual(float(mappings["depth_cam_x"]), XACRO_DEPTH_XYZ[0], places=7)

    def test_fallback_factory_matches_current_temp_tf(self):
        calib = factory_calibration()
        self.assertEqual(BASE_LINK_OFFSET_Z, 0.127)
        for sensor, expected in _FACTORY_FALLBACK.items():
            args = fallback_tf_args(calib, sensor)
            self.assertEqual(len(args), 8)
            got = tuple(float(item) for item in args[:6])
            _assert_vec(self, got, expected, places=12)
            self.assertEqual(args[6], FALLBACK_PARENT_FRAME)
            self.assertEqual(args[7], FALLBACK_CHILD_FRAMES[sensor])

    def test_fallback_adds_base_link_offset_and_lidar_optical_yaw(self):
        calib = _pose_with("lidar", xyz=(0.2, 0.01, 0.03), rpy=(0.0, 0.0, 0.4))
        args = fallback_tf_args(calib, "lidar")
        self.assertAlmostEqual(float(args[0]), 0.2, places=12)
        self.assertAlmostEqual(float(args[1]), 0.01, places=12)
        self.assertAlmostEqual(float(args[2]), 0.03 + BASE_LINK_OFFSET_Z, places=12)
        expected_yaw = math.atan2(
            math.sin(0.4 + XACRO_M_PI),
            math.cos(0.4 + XACRO_M_PI),
        )
        self.assertAlmostEqual(float(args[3]), expected_yaw, places=12)
        self.assertAlmostEqual(float(args[4]), 0.0, places=12)
        self.assertAlmostEqual(float(args[5]), 0.0, places=12)
        self.assertEqual(args[6], "base_footprint")
        self.assertEqual(args[7], "lidar_frame")

    def test_fallback_depth_keeps_optical_when_mount_identity(self):
        calib = _pose_with("depth_cam", xyz=(0.2, 0.01, 0.08))
        args = fallback_tf_args(calib, "depth_cam")
        self.assertAlmostEqual(float(args[0]), 0.2, places=12)
        self.assertAlmostEqual(float(args[1]), 0.01, places=12)
        self.assertAlmostEqual(float(args[2]), 0.08 + BASE_LINK_OFFSET_Z, places=12)
        self.assertAlmostEqual(float(args[3]), -XACRO_M_PI / 2.0, places=12)
        self.assertAlmostEqual(float(args[4]), 0.0, places=12)
        self.assertAlmostEqual(float(args[5]), -XACRO_M_PI / 2.0, places=12)
        self.assertEqual(args[7], "depth_camera_link")

    def test_fallback_imu_has_no_optical_and_offsets_z(self):
        calib = factory_calibration()
        args = fallback_tf_args(calib, "imu")
        pose = calib.sensors["imu"]
        self.assertAlmostEqual(float(args[0]), pose.xyz[0], places=12)
        self.assertAlmostEqual(float(args[1]), pose.xyz[1], places=12)
        self.assertAlmostEqual(float(args[2]), pose.xyz[2] + BASE_LINK_OFFSET_Z, places=12)
        self.assertAlmostEqual(float(args[3]), pose.rpy[2], places=12)
        self.assertAlmostEqual(float(args[4]), pose.rpy[1], places=12)
        self.assertAlmostEqual(float(args[5]), pose.rpy[0], places=12)
        self.assertEqual(args[7], "imu_link")

    def test_fallback_unknown_sensor_raises(self):
        with self.assertRaises(ValueError):
            fallback_tf_args(factory_calibration(), "gps")

    def test_operator_source_missing_file_is_factory(self):
        with tempfile.TemporaryDirectory() as tmp:
            calib = load(tmp)
        self.assertEqual(operator_source(calib), "factory")

    def test_operator_source_applied_file_is_file(self):
        calib = factory_calibration()
        calib = Calibration(sensors=calib.sensors, unused=False, reason="")
        self.assertEqual(operator_source(calib), "file")

    def test_operator_source_unreadable_is_unused(self):
        calib = factory_calibration(unused=True, reason="invalid schema")
        self.assertEqual(operator_source(calib), "unused")

    def test_operator_display_factory_matches_cli_mock(self):
        calib = factory_calibration()
        cam_xyz, cam_rpy, cam_xyz_src, cam_rpy_src = operator_display(calib, "depth_cam")
        self.assertAlmostEqual(cam_xyz[0], 0.10, places=7)
        self.assertAlmostEqual(cam_xyz[1], 0.0, places=7)
        self.assertAlmostEqual(cam_xyz[2], 0.05 + BASE_LINK_OFFSET_Z, places=7)
        _assert_vec(self, cam_rpy, (0.0, 0.0, 0.0), places=7)
        self.assertEqual(cam_xyz_src, "factory")
        self.assertEqual(cam_rpy_src, "factory")

        lidar_xyz, lidar_rpy, lidar_xyz_src, lidar_rpy_src = operator_display(calib, "lidar")
        self.assertAlmostEqual(lidar_xyz[0], XACRO_LIDAR_XYZ[0], places=7)
        self.assertAlmostEqual(lidar_xyz[2], XACRO_LIDAR_XYZ[2] + BASE_LINK_OFFSET_Z, places=7)
        self.assertAlmostEqual(round(lidar_xyz[2], 3), 0.168, places=3)
        _assert_vec(self, lidar_rpy, (0.0, 0.0, 180.0), places=7)
        self.assertEqual(lidar_xyz_src, "factory")
        self.assertEqual(lidar_rpy_src, "factory")

        imu_xyz, imu_rpy, imu_xyz_src, imu_rpy_src = operator_display(calib, "imu")
        self.assertAlmostEqual(imu_xyz[0], XACRO_IMU_XYZ[0], places=7)
        self.assertAlmostEqual(imu_xyz[2], XACRO_IMU_XYZ[2] + BASE_LINK_OFFSET_Z, places=7)
        self.assertAlmostEqual(round(imu_xyz[0], 3), 0.005, places=3)
        self.assertAlmostEqual(round(imu_xyz[1], 3), 0.011, places=3)
        self.assertAlmostEqual(round(imu_xyz[2], 3), 0.121, places=3)
        _assert_vec(self, imu_rpy, (0.0, 0.0, 0.0), places=7)
        self.assertEqual(imu_xyz_src, "factory")
        self.assertEqual(imu_rpy_src, "factory")


if __name__ == "__main__":
    unittest.main()
