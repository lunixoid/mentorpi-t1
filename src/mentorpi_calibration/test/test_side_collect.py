#!/usr/bin/env python3
"""Side stage collection (SD022 T2)."""
import io
import os
import sys
import time
import unittest

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from mentorpi_calibration.stages.side import (  # noqa: E402
    DEFAULT_TIMEOUT,
    DETAIL_NO_CAMERA_DEPTH,
    DETAIL_NO_CAMERA_INFO,
    DETAIL_NO_CAMERA_RGB,
    DETAIL_NO_SCAN,
    HINT,
    STAGE_NAME,
    RosUnavailableError,
    collect_ros,
    collect_side_samples,
)


class FakeCollector:
    def __init__(
        self,
        *,
        points2=True,
        scan=True,
        rgb=True,
        camera_info=True,
        clouds=None,
        scans=None,
        images=None,
        infos=None,
        run_delay=0.0,
    ):
        self._points2 = points2
        self._scan = scan
        self._rgb = rgb
        self._camera_info = camera_info
        self._clouds = list(clouds or [])
        self._scans = list(scans or [])
        self._images = list(images or [])
        self._infos = list(infos if infos is not None else [object()])
        self._run_delay = run_delay
        self.closed = False
        self.run_called = False

    def topics_available(self):
        return self._points2, self._scan, self._rgb, self._camera_info

    def run(self, timeout, on_progress):
        del timeout
        self.run_called = True
        if self._run_delay:
            time.sleep(self._run_delay)
        frames = max(len(self._clouds), len(self._scans), len(self._images), 1)
        for index in range(frames):
            on_progress(
                min(index + 1, len(self._clouds)),
                None,
                None,
                min(index + 1, len(self._images)),
            )
        on_progress(len(self._clouds), None, None, len(self._images))
        return list(self._clouds), list(self._scans), list(self._images), list(self._infos)

    def close(self):
        self.closed = True


class SideCollectTest(unittest.TestCase):
    def _collect(self, collector, timeout=0.05):
        stdout = io.StringIO()
        samples = collect_side_samples(timeout, collector=collector, stdout=stdout)
        return samples, stdout.getvalue()

    def test_default_timeout_constant(self):
        self.assertEqual(DEFAULT_TIMEOUT, 5.0)

    def test_success_collects_samples(self):
        clouds = [object(), object()]
        scans = [object()]
        images = [object(), object(), object()]
        collector = FakeCollector(clouds=clouds, scans=scans, images=images)
        samples, output = self._collect(collector)
        self.assertTrue(samples.ok)
        self.assertEqual(samples.details, ())
        self.assertEqual(samples.frames, 2)
        self.assertEqual(samples.rgb_frames, 3)
        self.assertEqual(samples.clouds, tuple(clouds))
        self.assertEqual(samples.scans, tuple(scans))
        self.assertEqual(samples.images, tuple(images))
        self.assertTrue(collector.run_called)
        self.assertFalse(collector.closed)
        self.assertIn("stage: {}".format(STAGE_NAME), output)
        self.assertIn("hint: {}".format(HINT), output)
        self.assertIn("frames: 2", output)
        self.assertIn("cloud_points:", output)
        self.assertIn("scan_rays:", output)
        self.assertIn("rgb_frames: 3", output)

    def test_missing_scan_fails_immediately(self):
        collector = FakeCollector(
            scan=False,
            clouds=[object()],
            scans=[object()],
            images=[object()],
            run_delay=1.0,
        )
        started = time.monotonic()
        samples, _output = self._collect(collector, timeout=2.0)
        elapsed = time.monotonic() - started
        self.assertLess(elapsed, 0.2)
        self.assertFalse(collector.run_called)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_SCAN,))

    def test_missing_points2_fails_immediately(self):
        collector = FakeCollector(
            points2=False,
            clouds=[object()],
            scans=[object()],
            images=[object()],
            run_delay=1.0,
        )
        started = time.monotonic()
        samples, _output = self._collect(collector, timeout=2.0)
        elapsed = time.monotonic() - started
        self.assertLess(elapsed, 0.2)
        self.assertFalse(collector.run_called)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_CAMERA_DEPTH,))

    def test_missing_rgb_fails_immediately(self):
        collector = FakeCollector(
            rgb=False,
            clouds=[object()],
            scans=[object()],
            images=[object()],
            run_delay=1.0,
        )
        started = time.monotonic()
        samples, _output = self._collect(collector, timeout=2.0)
        elapsed = time.monotonic() - started
        self.assertLess(elapsed, 0.2)
        self.assertFalse(collector.run_called)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_CAMERA_RGB,))

    def test_missing_camera_info_fails_immediately(self):
        # D2.3: intrinsics are the only pixel<->point law, so their absence is
        # a graph failure, not a silent fallback to slot arithmetic.
        collector = FakeCollector(
            camera_info=False,
            clouds=[object()],
            scans=[object()],
            images=[object()],
            run_delay=1.0,
        )
        started = time.monotonic()
        samples, _output = self._collect(collector, timeout=2.0)
        elapsed = time.monotonic() - started
        self.assertLess(elapsed, 0.2)
        self.assertFalse(collector.run_called)
        self.assertFalse(samples.ok)
        self.assertEqual(samples.details, (DETAIL_NO_CAMERA_INFO,))

    def test_collect_ros_without_rclpy_raises(self):
        with self.assertRaises(RosUnavailableError):
            collect_ros(timeout=0.01)

    def test_injected_collector_not_closed(self):
        collector = FakeCollector(
            clouds=[object()],
            scans=[object()],
            images=[object()],
        )
        collect_side_samples(0.01, collector=collector)
        self.assertFalse(collector.closed)


if __name__ == "__main__":
    unittest.main()
