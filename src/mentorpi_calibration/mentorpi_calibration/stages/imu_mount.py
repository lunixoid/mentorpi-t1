"""IMU mount roll/pitch from mean linear acceleration (SD012 D3.2.5)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from mentorpi_calibration.calibration_file import mat_vec3, rotation_matrix_from_rpy
from mentorpi_calibration.stages.tilt import rpy_from_up_keep_yaw

DETAIL_IMU_NOT_STILL = "imu not still"

# Implementation tuning constants, not an accuracy SLA.
GRAVITY_MPS2 = 9.80665
GRAVITY_MAG_TOLERANCE = 2.0

_UP_EPS = 1e-12


@dataclass(frozen=True)
class ImuMountFromGravity:
    """Result of estimating IMU mount rpy from mean linear acceleration."""

    ok: bool
    xyz: Optional[tuple[float, float, float]] = None
    rpy: Optional[tuple[float, float, float]] = None
    detail: Optional[str] = None


def imu_mount_from_gravity(
    accel_mean: tuple[float, float, float],
    current_imu_pose: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
    ],
) -> ImuMountFromGravity:
    """Derive imu_link mount on base_link from mean linear acceleration at rest.

    At rest ``linear_acceleration`` in ``imu_link`` points along world up
    (ROS: ≈ +g along vertical). ``current_imu_pose`` is mount ``(xyz, rpy)`` in
    file units (m, rad); ``xyz`` and ``yaw`` are preserved, ``roll`` and
    ``pitch`` are updated. Stored ``rpy`` is absolute mount including factory
    chip orientation.
    """
    failure = ImuMountFromGravity(ok=False, detail=DETAIL_IMU_NOT_STILL)

    current_xyz, current_rpy = current_imu_pose
    ax, ay, az = accel_mean
    mag = math.hypot(ax, math.hypot(ay, az))
    if abs(mag - GRAVITY_MPS2) > GRAVITY_MAG_TOLERANCE:
        return failure
    if mag < _UP_EPS:
        return failure

    up = np.array([ax / mag, ay / mag, az / mag], dtype=np.float64)
    roll, pitch, yaw = rpy_from_up_keep_yaw(up, current_rpy)
    return ImuMountFromGravity(
        ok=True,
        xyz=current_xyz,
        rpy=(roll, pitch, yaw),
        detail=None,
    )


def imu_gravity_residual_deg(
    accel_mean: tuple[float, float, float],
    mount_rpy: tuple[float, float, float],
) -> float:
    """Angle (deg) between mean accel direction and R_mount^T e_z."""
    ax, ay, az = accel_mean
    mag = math.hypot(ax, math.hypot(ay, az))
    if mag < _UP_EPS:
        return float("inf")

    observed = np.array([ax / mag, ay / mag, az / mag], dtype=np.float64)
    r_mount = rotation_matrix_from_rpy(mount_rpy)
    r_t = tuple(tuple(r_mount[j][i] for j in range(3)) for i in range(3))
    expected = np.array(
        mat_vec3(r_t, (0.0, 0.0, 1.0)),
        dtype=np.float64,
    )
    dot = float(np.clip(np.dot(observed, expected), -1.0, 1.0))
    return math.degrees(math.acos(dot))
