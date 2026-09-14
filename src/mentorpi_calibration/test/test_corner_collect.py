#!/usr/bin/env python3
"""Corner stage collection (SD012 T5)."""
import io
import os
import sys
import time
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.stages.corner import (  # noqa: E402
    DEFAULT_TIMEOUT,
    DETAIL_NO_CAMERA_DEPTH,
    DETAIL_NO_IMU,
    DETAIL_NO_SCAN,
    HINT,
    STAGE_NAME,
    RosUnavailableError,
    collect_corner_samples,
    collect_ros,
)


class FakeCollector:
    def __init__(
        self,
        *,
        points2=True,
        scan=True,
        imu=True,
        clouds=None,
        scans=None,
        imu_accels=None,
        run_delay=0.0,
    ):
        self._points2 = points2
        self._scan = scan
        self._imu = imu
        self._clouds = list(clouds or [])
        self._scans = list(scans or [])
        self._imu_accels = list(imu_accels or [])
        self._run_delay = run_delay
        self.closed = False

    def topics_available(self):
        return self._points2, self._scan, self._imu

    def run(self, timeout, on_progress):
        del timeout
        if self._run_delay:
            time.sleep(self._run_delay)
        frames = max(len(self._clouds), len(self._scans), 1)
        for index in range(frames):
            on_progress(
                min(index + 1, len(self._clouds)),
                None,
                None,
                min(index + 1, len(self._imu_accels)),
            )
        on_progress(len(self._clouds), None, None, len(self._imu_accels))
        return list(self._clouds), list(self._scans), list(self._imu_accels)

    def close(self):
        self.closed = True


class CornerCollectTest(unittest.TestCase):
    def _collect(self, collector, timeout=0.05):
        stdout = io.StringIO()
        samples = collect_corner_samples(timeout, collector=collector, stdout=stdout)
        return samples, stdout.getvalue()

    def test_default_timeout_constant(self):
        self.assertEqual(DEFAULT_TIMEOUT, 5.0)

    def test_success_collects_samples(self):
        clouds = [object(), object()]
        scans = [object()]
        imu_accels = [(0.0, 0.0, 9.8), (0.1, 0.0, 9.7)]
        collector = FakeCollector(clouds=clouds, scans=scans, imu_accels=imu_accels)
        samples, output = self._collect(collector)
        self.assertTrue(samples.ok)
        self.assertEqual(samples.details, ())
        self.assertEqual(samples.frames, 2)
        self.assertEqual(samples.imu_samples, 2)
        self.assertEqual(samples.clouds, tuple(clouds))
        self.assertEqual(samples.scans, tuple(scans))
        self.assertFalse(collector.closed)
        self.assertIn("stage: {}".format(STAGE_NAME), output)
        self.assertIn("hint: {}".format(HINT), output)
        self.assertIn("frames: 2", output)
        self.assertIn("cloud_points:", output)
        self.assertIn("scan_rays:", output)
        self.assertIn("imu_samples: 2", output)

    def test_missing_scan_fails_immediately(self):
        collector = FakeCollector(
            scan=False,
            clouds=[object()],
            scans=[object()],
            imu_accels=[(0.0, 0.0, 9.8)],
        )
        started = time.monotonic()
        samples, _output = self._collect(collector)
        elapsed = time.monotonic() - started
        self.assertLess(elapsed, 0.2)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_SCAN,))

    def test_missing_points2_fails_immediately(self):
        collector = FakeCollector(
            points2=False,
            clouds=[object()],
            scans=[object()],
            imu_accels=[(0.0, 0.0, 9.8)],
        )
        samples, _output = self._collect(collector)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_CAMERA_DEPTH,))

    def test_missing_imu_fails_immediately(self):
        collector = FakeCollector(
            imu=False,
            clouds=[object()],
            scans=[object()],
            imu_accels=[(0.0, 0.0, 9.8)],
        )
        samples, _output = self._collect(collector)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_IMU,))

    def test_collect_ros_without_rclpy_raises(self):
        with self.assertRaises(RosUnavailableError):
            collect_ros(timeout=0.01)

    def test_injected_collector_not_closed(self):
        collector = FakeCollector(
            clouds=[object()],
            scans=[object()],
            imu_accels=[(0.0, 0.0, 9.8)],
        )
        collect_corner_samples(0.01, collector=collector)
        self.assertFalse(collector.closed)

    def test_no_publishers_in_stage_sources(self):
        stages_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "mentorpi_calibration"))
        banned = ("create_publisher", "/vehicle/cmd_vel", "/hiwonder_controller/cmd_vel")
        for root, _dirs, files in os.walk(stages_dir):
            for name in files:
                if not name.endswith(".py"):
                    continue
                path = os.path.join(root, name)
                with open(path, encoding="utf-8") as handle:
                    text = handle.read()
                for token in banned:
                    self.assertNotIn(token, text, path)


if __name__ == "__main__":
    unittest.main()
