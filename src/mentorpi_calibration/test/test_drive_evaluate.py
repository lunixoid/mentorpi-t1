#!/usr/bin/env python3
"""Drive stage evaluation on a synthetic nonholonomic run (SD012 T7)."""
import math
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

from mentorpi_calibration.calibration_file import (  # noqa: E402
    FACTORY_POSES,
    SensorPose,
    factory_calibration,
    rotation_matrix_from_rpy,
)
from mentorpi_calibration.stages.drive import (  # noqa: E402
    DETAIL_NO_ROTATION,
    DETAIL_TRAVEL_TOO_SHORT,
    IMU_SIGN_MISMATCH,
    IMU_SIGN_OK,
    STAGE_NAME,
    DriveImuSample,
    DriveSamples,
    ScanPose,
    check_imu_yaw_sign,
    evaluate_drive_samples,
    evaluate_drive_trajectory,
    segment_motion,
)
from mentorpi_calibration.stages.scan_match import ScanTrajectory  # noqa: E402
from mentorpi_calibration.stages.tilt import wrap_pi  # noqa: E402
from mentorpi_calibration.stages.transforms import LIDAR_OPTICAL_RPY  # noqa: E402

_OPTICAL = LIDAR_OPTICAL_RPY[2]


class _Stamp:
    def __init__(self, t):
        self.sec = int(math.floor(t))
        self.nanosec = int(round((t - self.sec) * 1e9))


class _Header:
    def __init__(self, t):
        self.stamp = _Stamp(t)


class FakeLaserScan:
    def __init__(self, ranges, stamp, *, angle_min=-math.pi, range_max=25.0):
        self.ranges = list(ranges)
        self.angle_min = angle_min
        self.angle_increment = (2.0 * math.pi) / max(len(ranges), 1)
        self.range_min = 0.05
        self.range_max = range_max
        self.header = _Header(stamp)


def _compose(left, right):
    x1, y1, yaw1 = left
    x2, y2, yaw2 = right
    cosine = math.cos(yaw1)
    sine = math.sin(yaw1)
    return (
        x1 + cosine * x2 - sine * y2,
        y1 + sine * x2 + cosine * y2,
        wrap_pi(yaw1 + yaw2),
    )


def _inverse(pose):
    x, y, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (-cosine * x - sine * y, sine * x - cosine * y, wrap_pi(-yaw))


def _ray_segment(ox, oy, dx, dy, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    det = dx * vy - dy * vx
    if abs(det) < 1e-12:
        return None
    t_hit = ((ax - ox) * vy - (ay - oy) * vx) / det
    u_hit = ((ax - ox) * dy - (ay - oy) * dx) / det
    if t_hit > 1e-4 and 0.0 <= u_hit <= 1.0:
        return t_hit
    return None


def _ranges_at_frame(frame_pose, n_rays=180):
    x, y, yaw = frame_pose
    walls = (
        ((-2.4, -2.4), (-2.4, 2.4)),
        ((2.4, -2.4), (2.4, 2.4)),
        ((-2.4, -2.4), (2.4, -2.4)),
        ((-2.4, 2.4), (2.4, 2.4)),
    )
    angle_min = -math.pi
    increment = 2.0 * math.pi / n_rays
    ranges = []
    for index in range(n_rays):
        angle = angle_min + index * increment
        dx = math.cos(yaw + angle)
        dy = math.sin(yaw + angle)
        best = 25.0
        for (ax, ay), (bx, by) in walls:
            hit = _ray_segment(x, y, dx, dy, ax, ay, bx, by)
            if hit is not None and hit < best:
                best = hit
        ranges.append(26.0 if best >= 24.9 else best)
    return ranges


def _frame_pose_from_base(base_pose, lidar_xyz, lidar_yaw):
    psi_frame = wrap_pi(lidar_yaw + _OPTICAL)
    t_base_frame = (lidar_xyz[0], lidar_xyz[1], psi_frame)
    return _compose(base_pose, t_base_frame)


def _imu_gyro_for_base_yaw_rate(imu_rpy, yaw_rate):
    rotation = np.asarray(rotation_matrix_from_rpy(imu_rpy), dtype=np.float64)
    omega_base = np.array([0.0, 0.0, yaw_rate], dtype=np.float64)
    gyro = rotation.T @ omega_base
    return (float(gyro[0]), float(gyro[1]), float(gyro[2]))


def build_drive_run(
    lidar_xyz,
    lidar_yaw,
    imu_rpy,
    *,
    travel_m=0.80,
    turn_rad=math.pi / 2.0,
    dt=0.1,
    as_scans=False,
):
    """Straight +x then in-place left turn. Returns poses, imu, optional scans."""
    t_base_frame = (lidar_xyz[0], lidar_xyz[1], wrap_pi(lidar_yaw + _OPTICAL))
    base0 = _inverse(t_base_frame)
    n_trans = max(8, int(round(travel_m / 0.04)))
    n_rot = max(8, int(round(abs(turn_rad) / 0.08)))
    poses = []
    imu = []
    stamp = 0.0

    def emit_base(base_pose, yaw_rate):
        nonlocal stamp
        frame = _frame_pose_from_base(base_pose, lidar_xyz, lidar_yaw)
        poses.append(ScanPose(stamp, frame[0], frame[1], frame[2]))
        imu.append(DriveImuSample(stamp, _imu_gyro_for_base_yaw_rate(imu_rpy, yaw_rate)))
        stamp += dt

    for _ in range(4):
        emit_base(base0, 0.0)
    for index in range(n_trans + 1):
        s = travel_m * index / float(n_trans)
        emit_base(_compose(base0, (s, 0.0, 0.0)), 0.0)
    base_mid = _compose(base0, (travel_m, 0.0, 0.0))
    for _ in range(3):
        emit_base(base_mid, 0.0)
    yaw_rate = turn_rad / max(n_rot * dt, dt)
    for index in range(n_rot + 1):
        alpha = turn_rad * index / float(n_rot)
        emit_base(_compose(base_mid, (0.0, 0.0, alpha)), yaw_rate)
    for _ in range(3):
        emit_base(_compose(base_mid, (0.0, 0.0, turn_rad)), 0.0)

    scans = None
    if as_scans:
        scans = []
        for pose in poses:
            ranges = _ranges_at_frame((pose.x, pose.y, pose.yaw))
            scans.append(FakeLaserScan(ranges, pose.stamp))
    return poses, tuple(imu), scans


def _ok_samples(scans, imu):
    return DriveSamples(
        scans=tuple(scans),
        imu=tuple(imu),
        travel_m=1.0,
        turn_deg=90.0,
        ok=True,
        details=(),
    )


class DriveEvaluateTest(unittest.TestCase):
    def setUp(self):
        self._calib = factory_calibration()
        self._tmpdir = tempfile.TemporaryDirectory()
        self._env_patch = mock.patch.dict(
            os.environ,
            {"T1_CALIBRATION_DIR": self._tmpdir.name},
        )
        self._env_patch.start()
        self._true_xyz = (0.0900034859353204, 0.025, 0.0405195774179554)
        self._true_yaw = 0.35
        self._imu_rpy = FACTORY_POSES["imu"]["rpy"]

    def tearDown(self):
        self._env_patch.stop()
        self._tmpdir.cleanup()

    def _evaluate_traj(self, poses, imu):
        trajectory = ScanTrajectory(
            ok=True,
            poses=tuple(poses),
            travel_m=sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses[:-1], poses[1:])),
            turn_deg=sum(abs(math.degrees(wrap_pi(b.yaw - a.yaw))) for a, b in zip(poses[:-1], poses[1:])),
            mean_residual=0.0,
        )
        return evaluate_drive_trajectory(trajectory, imu, self._calib)

    def test_segments_split_translation_and_rotation(self):
        poses, _imu, _scans = build_drive_run(
            self._true_xyz,
            self._true_yaw,
            self._imu_rpy,
        )
        segments = segment_motion(poses)
        self.assertGreaterEqual(len(segments.translation), 1)
        self.assertGreaterEqual(len(segments.rotation), 1)
        self.assertGreater(segments.translation[0].length_m, 0.5)
        self.assertGreater(segments.rotation[0].yaw_span_rad, 0.6)

    def test_recovers_known_lidar_yaw_xy_from_trajectory(self):
        poses, imu, _scans = build_drive_run(
            self._true_xyz,
            self._true_yaw,
            self._imu_rpy,
        )
        result = self._evaluate_traj(poses, imu)
        self.assertTrue(result.ok, result.details)
        self.assertEqual(result.stage, STAGE_NAME)
        self.assertLess(result.residual_after, result.residual_before)
        self.assertAlmostEqual(result.lidar_rpy[2], self._true_yaw, delta=0.05)
        self.assertAlmostEqual(result.lidar_xyz[0], self._true_xyz[0], delta=0.03)
        self.assertAlmostEqual(result.lidar_xyz[1], self._true_xyz[1], delta=0.03)
        self.assertEqual(result.lidar_xyz[2], self._calib.sensors["lidar"].xyz[2])
        self.assertEqual(result.imu_yaw_sign, IMU_SIGN_OK)

    def test_recovers_pi_lidar_yaw(self):
        true_yaw = math.pi
        poses, imu, _scans = build_drive_run(
            self._true_xyz,
            true_yaw,
            self._imu_rpy,
        )
        result = self._evaluate_traj(poses, imu)
        self.assertTrue(result.ok, result.details)
        err = abs(wrap_pi(result.lidar_rpy[2] - true_yaw))
        self.assertLess(err, 0.08)

    def test_imu_sign_mismatch_when_gyro_flipped(self):
        poses, imu, _scans = build_drive_run(
            self._true_xyz,
            self._true_yaw,
            self._imu_rpy,
        )
        flipped = tuple(DriveImuSample(sample.stamp, (-g[0], -g[1], -g[2])) for sample in imu for g in [sample.gyro])
        segments = segment_motion(poses)
        sign = check_imu_yaw_sign(segments.rotation, flipped, self._imu_rpy)
        self.assertEqual(sign, IMU_SIGN_MISMATCH)

    def test_short_travel_refuses(self):
        poses, imu, _scans = build_drive_run(
            self._true_xyz,
            self._true_yaw,
            self._imu_rpy,
            travel_m=0.05,
            turn_rad=math.pi / 2.0,
        )
        result = self._evaluate_traj(poses, imu)
        self.assertFalse(result.ok)
        self.assertIn(DETAIL_TRAVEL_TOO_SHORT, result.details)

    def test_no_rotation_segment_refuses(self):
        poses, imu, _scans = build_drive_run(
            self._true_xyz,
            self._true_yaw,
            self._imu_rpy,
            travel_m=0.80,
            turn_rad=0.0,
        )
        # turn_rad=0 still emits a degenerate rotate loop of zeros; drop those poses.
        trans_only = [pose for pose in poses if abs(pose.yaw - poses[0].yaw) < 0.02]
        result = self._evaluate_traj(trans_only, imu)
        self.assertFalse(result.ok)
        self.assertIn(DETAIL_NO_ROTATION, result.details)

    def test_full_scan_pipeline_recovers_mount(self):
        poses, imu, scans = build_drive_run(
            self._true_xyz,
            self._true_yaw,
            self._imu_rpy,
            as_scans=True,
        )
        self.assertIsNotNone(scans)
        with mock.patch(
            "mentorpi_calibration.stages.drive.commit_drive_pending",
            return_value=(),
        ) as commit_pending:
            result = evaluate_drive_samples(_ok_samples(scans, imu), self._calib)
        self.assertTrue(result.ok, result.details)
        commit_pending.assert_called_once()
        self.assertAlmostEqual(result.lidar_rpy[2], self._true_yaw, delta=0.12)
        self.assertAlmostEqual(result.lidar_xyz[0], self._true_xyz[0], delta=0.05)
        self.assertAlmostEqual(result.lidar_xyz[1], self._true_xyz[1], delta=0.05)
        self.assertEqual(result.imu_yaw_sign, IMU_SIGN_OK)
        self.assertLess(result.residual_after, result.residual_before)

    def test_failure_does_not_commit(self):
        samples = DriveSamples(
            scans=(),
            imu=(),
            travel_m=0.0,
            turn_deg=0.0,
            ok=False,
            details=(DETAIL_TRAVEL_TOO_SHORT,),
        )
        with mock.patch(
            "mentorpi_calibration.stages.drive.commit_drive_pending",
            return_value=(),
        ) as commit_pending:
            result = evaluate_drive_samples(samples, self._calib)
        self.assertFalse(result.ok)
        commit_pending.assert_not_called()

    def test_keeps_current_z_roll_pitch(self):
        lidar = self._calib.sensors["lidar"]
        self._calib.sensors["lidar"] = SensorPose(
            parent=lidar.parent,
            xyz=(lidar.xyz[0], lidar.xyz[1], 0.051),
            rpy=(0.02, -0.01, lidar.rpy[2]),
            source=lidar.source,
        )
        poses, imu, _scans = build_drive_run(
            self._true_xyz,
            self._true_yaw,
            self._imu_rpy,
        )
        result = self._evaluate_traj(poses, imu)
        self.assertTrue(result.ok, result.details)
        self.assertAlmostEqual(result.lidar_xyz[2], 0.051)
        self.assertAlmostEqual(result.lidar_rpy[0], 0.02)
        self.assertAlmostEqual(result.lidar_rpy[1], -0.01)


if __name__ == "__main__":
    unittest.main()
