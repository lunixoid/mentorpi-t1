#!/usr/bin/env python3
"""Drive stage collection (SD012 T7)."""
import io
import os
import sys
import time
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

import numpy as np  # noqa: E402

from mentorpi_calibration.stages.drive import (  # noqa: E402
    DEFAULT_TIMEOUT,
    DETAIL_NO_IMU,
    DETAIL_NO_ROTATION,
    DETAIL_NO_SCAN,
    HINTS_FORWARD,
    HINTS_TURN,
    MIN_TURN_DEG,
    STAGE_NAME,
    DriveImuSample,
    DriveScan,
    RosUnavailableError,
    collect_drive_samples,
    collect_ros,
)


class FakeCollector:
    def __init__(
        self,
        *,
        scan=True,
        imu=True,
        scans=None,
        imu_samples=None,
        travel=None,
        turn=None,
        run_delay=0.0,
    ):
        self._scan = scan
        self._imu = imu
        self._scans = list(scans or [])
        self._imu_samples = list(imu_samples or [])
        self._travel = list(travel or [])
        self._turn = list(turn or [])
        self._run_delay = run_delay
        self.closed = False

    def topics_available(self):
        return self._scan, self._imu

    def run(self, timeout, on_progress, should_stop):
        del timeout
        if self._run_delay:
            time.sleep(self._run_delay)
        n_frames = max(len(self._scans), len(self._imu_samples), 1)
        last_scan = None
        last_imu = None
        for index in range(n_frames):
            travel = self._travel[index] if index < len(self._travel) else (self._travel[-1] if self._travel else 0.0)
            turn = self._turn[index] if index < len(self._turn) else (self._turn[-1] if self._turn else 0.0)
            n_scan = min(index + 1, len(self._scans))
            n_imu = min(index + 1, len(self._imu_samples))
            on_progress(travel, turn, n_scan, n_imu)
            last_scan = n_scan
            last_imu = n_imu
            if should_stop(travel, turn):
                return self._scans[:n_scan], self._imu_samples[:n_imu]
        on_progress(
            self._travel[-1] if self._travel else 0.0,
            self._turn[-1] if self._turn else 0.0,
            len(self._scans),
            len(self._imu_samples),
        )
        del last_scan, last_imu
        return list(self._scans), list(self._imu_samples)

    def close(self):
        self.closed = True


class DriveCollectTest(unittest.TestCase):
    def _collect(self, collector, timeout=0.05):
        stdout = io.StringIO()
        samples = collect_drive_samples(timeout, collector=collector, stdout=stdout)
        return samples, stdout.getvalue()

    def test_default_timeout_is_longer_than_snapshot(self):
        self.assertGreaterEqual(DEFAULT_TIMEOUT, 30.0)

    def test_success_collects_samples_and_prints_hold_contract(self):
        scans = [
            DriveScan(xy=np.zeros((12, 2)), stamp=0.0),
            DriveScan(xy=np.zeros((12, 2)), stamp=0.1),
            DriveScan(xy=np.zeros((12, 2)), stamp=0.2),
        ]
        imu = [
            DriveImuSample(0.0, (0.0, 0.0, 0.1)),
            DriveImuSample(0.1, (0.0, 0.0, 0.1)),
            DriveImuSample(0.2, (0.0, 0.0, 0.4)),
        ]
        collector = FakeCollector(
            scans=scans,
            imu_samples=imu,
            travel=[0.0, 0.4, 0.4],
            turn=[0.0, 5.0, 5.0 + MIN_TURN_DEG],
        )
        samples, output = self._collect(collector)
        self.assertTrue(samples.ok)
        self.assertEqual(samples.details, ())
        self.assertEqual(len(samples.scans), 3)
        self.assertFalse(collector.closed)
        self.assertIn("stage: {}".format(STAGE_NAME), output)
        for hint in HINTS_FORWARD:
            self.assertIn("hint: {}".format(hint), output)
        self.assertIn("hint:\n", output)
        for hint in HINTS_TURN:
            self.assertIn("hint: {}".format(hint), output)
        self.assertIn("travel: 0.400", output)
        self.assertLess(output.find(HINTS_FORWARD[0]), output.find(HINTS_TURN[0]))

    def test_travel_alone_does_not_finish_collection(self):
        scans = [
            DriveScan(xy=np.zeros((12, 2)), stamp=0.0),
            DriveScan(xy=np.zeros((12, 2)), stamp=0.1),
            DriveScan(xy=np.zeros((12, 2)), stamp=0.2),
        ]
        imu = [
            DriveImuSample(0.0, (0.0, 0.0, 0.0)),
            DriveImuSample(0.1, (0.0, 0.0, 0.0)),
            DriveImuSample(0.2, (0.0, 0.0, 0.0)),
        ]
        collector = FakeCollector(
            scans=scans,
            imu_samples=imu,
            travel=[0.0, 0.5, 0.5],
            turn=[12.0, 40.0, 40.0],
        )
        samples, output = self._collect(collector)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_ROTATION,))
        self.assertEqual(len(samples.scans), 3)
        self.assertIn("hint: {}".format(HINTS_TURN[0]), output)
        self.assertIn("turn: 0.0", output)

    def test_missing_scan_fails_immediately(self):
        collector = FakeCollector(
            scan=False,
            scans=[DriveScan(xy=np.zeros((8, 2)), stamp=0.0)],
            imu_samples=[DriveImuSample(0.0, (0.0, 0.0, 0.0))],
        )
        started = time.monotonic()
        samples, output = self._collect(collector)
        elapsed = time.monotonic() - started
        self.assertLess(elapsed, 0.2)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_SCAN,))
        self.assertIn("stage: {}".format(STAGE_NAME), output)

    def test_missing_imu_fails_immediately(self):
        collector = FakeCollector(
            imu=False,
            scans=[DriveScan(xy=np.zeros((8, 2)), stamp=0.0)],
            imu_samples=[DriveImuSample(0.0, (0.0, 0.0, 0.0))],
        )
        samples, _output = self._collect(collector)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_IMU,))

    def test_collect_ros_without_rclpy_raises(self):
        with self.assertRaises(RosUnavailableError):
            collect_ros(timeout=0.01)

    def test_injected_collector_not_closed(self):
        collector = FakeCollector(
            scans=[DriveScan(xy=np.zeros((8, 2)), stamp=0.0)],
            imu_samples=[DriveImuSample(0.0, (0.0, 0.0, 0.0))],
        )
        collect_drive_samples(0.01, collector=collector)
        self.assertFalse(collector.closed)

    def test_no_publishers_in_drive_sources(self):
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
