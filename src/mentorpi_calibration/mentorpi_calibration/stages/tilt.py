"""Roll/pitch from a known up vector, keeping yaw (SD012 D3.2)."""

from __future__ import annotations

import math

import numpy as np

from mentorpi_calibration.calibration_file import rotation_matrix_from_rpy


def unwrap_angle_near(reference: float, value: float) -> float:
    delta = value - reference
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta <= -math.pi:
        delta += 2.0 * math.pi
    return reference + delta


def wrap_pi(angle: float) -> float:
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def rpy_from_up_keep_yaw(
    up_in_link: np.ndarray,
    current_rpy: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Roll/pitch that align ``up_in_link`` with base +Z while keeping ``yaw``.

    ``up_in_link`` is world +Z in the mount link: ``R(rpy)^T e_z`` for URDF
    ``R = Rz(yaw) Ry(pitch) Rx(roll)``. That vector does not depend on yaw, so
    yaw is copied through and is not mixed into roll/pitch.
    """
    current_roll, current_pitch, yaw = current_rpy
    up = np.asarray(up_in_link, dtype=np.float64).reshape(3)
    # Third row of R: (-sin(pitch), cos(pitch) sin(roll), cos(pitch) cos(roll)).
    roll = math.atan2(float(up[1]), float(up[2]))
    pitch = math.atan2(
        -float(up[0]),
        math.hypot(float(up[1]), float(up[2])),
    )
    roll = unwrap_angle_near(current_roll, roll)
    pitch = unwrap_angle_near(current_pitch, pitch)
    return (roll, pitch, yaw)


def up_in_link_from_rpy(rpy: tuple[float, float, float]) -> np.ndarray:
    """World +Z expressed in the mount link for a URDF rpy."""
    matrix = np.asarray(rotation_matrix_from_rpy(rpy), dtype=np.float64)
    up = matrix.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return up / np.linalg.norm(up)
