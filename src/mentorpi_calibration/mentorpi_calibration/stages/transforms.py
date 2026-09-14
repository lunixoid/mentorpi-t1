"""URDF-convention frame maps for camera optical and lidar_frame (SD012 D3.2)."""

from __future__ import annotations

import math

import numpy as np

from mentorpi_calibration.calibration_file import depth_cam_optical_rpy, rotation_matrix_from_rpy

# lidar_frame_joint rpy in lidar_frame.urdf.xacro (not calibrated, D2.3).
LIDAR_OPTICAL_RPY = (0.0, 0.0, 3.1415926535897931)

_Vector3 = tuple[float, float, float]


def np_rotation(rpy: _Vector3) -> np.ndarray:
    return np.asarray(rotation_matrix_from_rpy(rpy), dtype=np.float64)


def camera_optical_from_base_link(
    points_bl: np.ndarray,
    mount_xyz: _Vector3,
    mount_rpy: _Vector3,
) -> np.ndarray:
    """Map base_link points into depth_camera_link (optical).

    URDF: p_parent = R_mount R_opt p_optical + t.
    """
    pts = np.asarray(points_bl, dtype=np.float64)
    r_mount = np_rotation(mount_rpy)
    r_opt = np_rotation(depth_cam_optical_rpy())
    t_mount = np.asarray(mount_xyz, dtype=np.float64)
    return (r_opt.T @ r_mount.T @ (pts - t_mount).T).T


def camera_base_link_from_optical(
    points_opt: np.ndarray,
    mount_xyz: _Vector3,
    mount_rpy: _Vector3,
) -> np.ndarray:
    """Map depth_camera_link (optical) points into base_link."""
    pts = np.asarray(points_opt, dtype=np.float64)
    r_mount = np_rotation(mount_rpy)
    r_opt = np_rotation(depth_cam_optical_rpy())
    t_mount = np.asarray(mount_xyz, dtype=np.float64)
    return (r_mount @ r_opt @ pts.T).T + t_mount


def lidar_frame_from_base_link(
    points_bl: np.ndarray,
    mount_xyz: _Vector3,
    mount_rpy: _Vector3,
) -> np.ndarray:
    """Map base_link points into lidar_frame."""
    pts = np.asarray(points_bl, dtype=np.float64)
    r_mount = np_rotation(mount_rpy)
    r_opt = np_rotation(LIDAR_OPTICAL_RPY)
    t_mount = np.asarray(mount_xyz, dtype=np.float64)
    return (r_opt.T @ r_mount.T @ (pts - t_mount).T).T


def lidar_base_link_from_frame(
    points_frame: np.ndarray,
    mount_xyz: _Vector3,
    mount_rpy: _Vector3,
) -> np.ndarray:
    """Map lidar_frame points into base_link."""
    pts = np.asarray(points_frame, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(1, -1)
    if pts.shape[1] == 2:
        pts = np.column_stack((pts, np.zeros(pts.shape[0])))
    r_mount = np_rotation(mount_rpy)
    r_opt = np_rotation(LIDAR_OPTICAL_RPY)
    t_mount = np.asarray(mount_xyz, dtype=np.float64)
    return (r_mount @ r_opt @ pts.T).T + t_mount


def world_up_in_optical(mount_rpy: _Vector3) -> np.ndarray:
    """Unit +Z of base_link expressed in depth_camera_link."""
    r_mount = np_rotation(mount_rpy)
    r_opt = np_rotation(depth_cam_optical_rpy())
    up = r_opt.T @ r_mount.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    norm = np.linalg.norm(up)
    if norm < 1e-12:
        return np.array([0.0, -1.0, 0.0], dtype=np.float64)
    return up / norm


def plane_optical_to_base_link(
    normal_opt: np.ndarray,
    d_opt: float,
    mount_xyz: _Vector3,
    mount_rpy: _Vector3,
) -> tuple[np.ndarray, float]:
    """Transform n·p + d = 0 from optical into base_link."""
    r_mount = np_rotation(mount_rpy)
    r_opt = np_rotation(depth_cam_optical_rpy())
    n_bl = r_mount @ r_opt @ np.asarray(normal_opt, dtype=np.float64).reshape(3)
    n_norm = np.linalg.norm(n_bl)
    if n_norm < 1e-12:
        return n_bl, d_opt
    n_bl = n_bl / n_norm
    t_mount = np.asarray(mount_xyz, dtype=np.float64)
    d_bl = (d_opt * n_norm) - float(np.dot(n_bl, t_mount))
    return n_bl, d_bl


def signed_plane_distance(normal: np.ndarray, d: float, points: np.ndarray) -> np.ndarray:
    """n·p + d for each point (||n|| = 1)."""
    pts = np.asarray(points, dtype=np.float64)
    return pts @ np.asarray(normal, dtype=np.float64).reshape(3) + float(d)


def nearest_right_angle(delta: float) -> float:
    """Nearest multiple of π/2 to ``delta`` (radians)."""
    quarter = math.pi / 2.0
    return round(delta / quarter) * quarter
