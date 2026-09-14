"""2-D scan lines and closed-form camera x, y, yaw (SD012 D3.2).

Line extraction is sequential RANSAC in the scan plane. The SE(2) of the
camera relative to a known lidar pose follows Choi et al. (T-RO 2016) /
Fernández-Moral et al. (IJRR 2015): wall normals give yaw, signed distances
give translation. Zhang & Pless (IROS 2004) supplies the residual — RMS of
lidar points to the camera wall planes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import permutations
from typing import Any, Optional, Sequence

import numpy as np

from mentorpi_calibration.calibration_file import FACTORY_POSES, depth_cam_optical_rpy
from mentorpi_calibration.stages.planes import ORTHOGONALITY_DEG, PlaneFit
from mentorpi_calibration.stages.tilt import unwrap_angle_near, wrap_pi
from mentorpi_calibration.stages.transforms import (
    camera_optical_from_base_link,
    lidar_base_link_from_frame,
    nearest_right_angle,
    np_rotation,
    plane_optical_to_base_link,
    signed_plane_distance,
)

DETAIL_TOO_FEW_SCAN_POINTS = "too few scan points"
DETAIL_ONE_WALL = "one wall, need a corner or two walls"
DETAIL_UNCERTAIN_ALIGNMENT = "uncertain alignment"

# Implementation tuning constants, not an accuracy SLA.
LINE_RANSAC_ITERATIONS = 250
LINE_INLIER_DISTANCE_M = 0.03
MIN_LINE_INLIERS = 8
YAW_AGREE_DEG = 12.0
AXIS_SWAP_DEG = 20.0
AMBIGUOUS_SCORE_RATIO = 0.85
# Camera must stay on the chassis relative to factory, not the current
# (possibly already-wrong) pose. A π lidar-frame mismatch otherwise
# "explains" the walls by putting the camera behind the robot.
CAMERA_XY_MAX_DELTA_M = 0.25
LIDAR_YAW_EXTRAS = (0.0, math.pi)

_Vector3 = tuple[float, float, float]
_Pose = tuple[_Vector3, _Vector3]
_NORMAL_EPS = 1e-12
_LINE_COLLINEAR_EPS = 1e-8


@dataclass(frozen=True)
class ScanLine:
    """2-D line n·(x, y) = rho in base_link XY, plus originating scan points."""

    normal: np.ndarray
    rho: float
    points_bl: np.ndarray
    inliers_mask: np.ndarray


@dataclass(frozen=True)
class ScanLines:
    ok: bool
    first: Optional[ScanLine] = None
    second: Optional[ScanLine] = None
    detail: Optional[str] = None


@dataclass(frozen=True)
class Se2Match:
    ok: bool
    xyz: Optional[_Vector3] = None
    rpy: Optional[_Vector3] = None
    lidar_xyz: Optional[_Vector3] = None
    lidar_rpy: Optional[_Vector3] = None
    cause: str = ""
    residual_before: Optional[float] = None
    residual_after: Optional[float] = None
    detail: Optional[str] = None


def laserscan_xy(msg: Any) -> np.ndarray:
    """Valid LaserScan returns as (N, 2) points in the scan frame."""
    ranges = np.asarray(getattr(msg, "ranges", ()), dtype=np.float64)
    if ranges.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    angle_min = float(getattr(msg, "angle_min", 0.0))
    angle_increment = float(getattr(msg, "angle_increment", 0.0))
    range_min = float(getattr(msg, "range_min", 0.0))
    range_max = float(getattr(msg, "range_max", np.inf))
    angles = angle_min + np.arange(ranges.size) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max) & (ranges > 0.0)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def scans_to_xy(scans: Sequence[Any]) -> np.ndarray:
    """Concatenate LaserScan-like messages or pre-parsed (N, 2) arrays."""
    if not scans:
        return np.empty((0, 2), dtype=np.float64)
    parts: list[np.ndarray] = []
    for scan in scans:
        if isinstance(scan, np.ndarray):
            arr = np.asarray(scan, dtype=np.float64)
            if arr.ndim != 2 or arr.shape[1] not in (2, 3):
                continue
            parts.append(arr[:, :2])
        elif hasattr(scan, "ranges"):
            parts.append(laserscan_xy(scan))
    if not parts:
        return np.empty((0, 2), dtype=np.float64)
    return np.concatenate(parts, axis=0)


def count_scan_rays(scans: Sequence[Any]) -> int:
    return int(scans_to_xy(scans).shape[0])


def _fit_line_svd(points_xy: np.ndarray) -> Optional[tuple[np.ndarray, float]]:
    if points_xy.shape[0] < 2:
        return None
    centroid = points_xy.mean(axis=0)
    centered = points_xy - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    direction = vh[0]
    normal = np.array([-direction[1], direction[0]], dtype=np.float64)
    norm = np.linalg.norm(normal)
    if norm < _NORMAL_EPS:
        return None
    normal = normal / norm
    rho = float(np.dot(normal, centroid))
    if rho < 0.0:
        normal = -normal
        rho = -rho
    return normal, rho


def _xy(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return np.empty((0, 2), dtype=np.float64)
    return pts[:, :2]


def fit_line_ransac(points_bl: np.ndarray) -> Optional[ScanLine]:
    pts = np.asarray(points_bl, dtype=np.float64)
    xy = _xy(pts)
    n_points = xy.shape[0]
    if n_points < 2:
        return None
    best_count = 0
    best_mask: Optional[np.ndarray] = None
    for _ in range(LINE_RANSAC_ITERATIONS):
        pair = np.random.choice(n_points, size=2, replace=False)
        p0, p1 = xy[pair[0]], xy[pair[1]]
        delta = p1 - p0
        length = np.linalg.norm(delta)
        if length < _LINE_COLLINEAR_EPS:
            continue
        normal = np.array([-delta[1], delta[0]], dtype=np.float64) / length
        rho = float(np.dot(normal, p0))
        distances = np.abs(xy @ normal - rho)
        mask = distances < LINE_INLIER_DISTANCE_M
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < MIN_LINE_INLIERS:
        return None
    refit = _fit_line_svd(xy[best_mask])
    if refit is None:
        return None
    normal, rho = refit
    distances = np.abs(xy @ normal - rho)
    inliers = distances < LINE_INLIER_DISTANCE_M
    if int(inliers.sum()) < MIN_LINE_INLIERS:
        return None
    points_in = pts[inliers]
    if points_in.shape[1] == 2:
        points_in = np.column_stack((points_in, np.zeros(points_in.shape[0])))
    return ScanLine(
        normal=normal,
        rho=float(rho),
        points_bl=points_in,
        inliers_mask=inliers,
    )


def extract_orthogonal_lines(points_bl: np.ndarray) -> ScanLines:
    pts = np.asarray(points_bl, dtype=np.float64)
    if pts.shape[0] < MIN_LINE_INLIERS * 2:
        return ScanLines(ok=False, detail=DETAIL_TOO_FEW_SCAN_POINTS)
    first = fit_line_ransac(pts)
    if first is None:
        return ScanLines(ok=False, detail=DETAIL_TOO_FEW_SCAN_POINTS)
    remaining = pts[~first.inliers_mask]
    second = fit_line_ransac(remaining)
    if second is None:
        return ScanLines(ok=False, first=first, detail=DETAIL_ONE_WALL)
    dot = abs(float(np.dot(first.normal, second.normal)))
    angle_deg = math.degrees(math.acos(min(1.0, max(0.0, dot))))
    if abs(90.0 - angle_deg) > ORTHOGONALITY_DEG:
        return ScanLines(
            ok=False,
            first=first,
            second=second,
            detail=DETAIL_ONE_WALL,
        )
    return ScanLines(ok=True, first=first, second=second, detail=None)


def _horizontal_normal(n_bl: np.ndarray) -> Optional[np.ndarray]:
    n_h = np.array([float(n_bl[0]), float(n_bl[1])], dtype=np.float64)
    norm = np.linalg.norm(n_h)
    if norm < _NORMAL_EPS:
        return None
    return n_h / norm


def _mean_angle(a: float, b: float) -> float:
    return math.atan2(
        math.sin(a) + math.sin(b),
        math.cos(a) + math.cos(b),
    )


def _angle_diff(a: float, b: float) -> float:
    return abs(wrap_pi(a - b))


def _wall_normal_at_yaw_zero(
    n_opt: np.ndarray,
    tilt_rpy: _Vector3,
) -> Optional[np.ndarray]:
    r_tilt = np_rotation((tilt_rpy[0], tilt_rpy[1], 0.0))
    r_opt = np_rotation(depth_cam_optical_rpy())
    n_bl = r_tilt @ r_opt @ np.asarray(n_opt, dtype=np.float64).reshape(3)
    return _horizontal_normal(n_bl)


def scan_plane_residual(
    wall_planes: tuple[tuple[np.ndarray, float], tuple[np.ndarray, float]],
    line_points_bl: tuple[np.ndarray, np.ndarray],
    camera_pose: _Pose,
    assignment: tuple[int, int],
) -> float:
    """RMS |n·p + d| of lidar points (in base_link) to camera planes."""
    distances: list[np.ndarray] = []
    xyz, rpy = camera_pose
    for wall_index, line_index in enumerate(assignment):
        n_opt, d_opt = wall_planes[wall_index]
        pts = line_points_bl[line_index]
        if pts.shape[0] == 0:
            continue
        pts_opt = camera_optical_from_base_link(pts, xyz, rpy)
        dist = np.abs(signed_plane_distance(n_opt, d_opt, pts_opt))
        distances.append(dist)
    if not distances:
        return float("inf")
    stacked = np.concatenate(distances)
    return float(np.sqrt(np.mean(stacked * stacked)))


def _lidar_rpy_with_extra(lidar_rpy: _Vector3, extra: float) -> _Vector3:
    yaw = unwrap_angle_near(lidar_rpy[2], lidar_rpy[2] + extra)
    return (lidar_rpy[0], lidar_rpy[1], yaw)


def match_camera_yaw_xy(
    walls: tuple[PlaneFit, PlaneFit],
    scan_xy: np.ndarray,
    current_camera_pose: _Pose,
    lidar_pose: _Pose,
    proposed_tilt_xyz: _Vector3,
    proposed_tilt_rpy: _Vector3,
) -> Se2Match:
    """Closed-form camera x, y, yaw; lidar yaw extras for a π frame mismatch.

    D2.3: a 180° lidar optical-frame error is expressed as mount yaw, not by
    dragging the camera behind the chassis. Candidates with camera x < 0 or
    more than ``CAMERA_XY_MAX_DELTA_M`` from factory xy are dropped.
    """
    failure = Se2Match(ok=False, detail=DETAIL_UNCERTAIN_ALIGNMENT)
    if walls[0].normal is None or walls[1].normal is None or walls[0].d is None or walls[1].d is None:
        return failure

    wall_planes = (
        (walls[0].normal, float(walls[0].d)),
        (walls[1].normal, float(walls[1].d)),
    )
    _, current_rpy = current_camera_pose
    lidar_xyz, lidar_rpy = lidar_pose
    factory_xy = FACTORY_POSES["depth_cam"]["xyz"]
    n_cam0 = (
        _wall_normal_at_yaw_zero(walls[0].normal, proposed_tilt_rpy),
        _wall_normal_at_yaw_zero(walls[1].normal, proposed_tilt_rpy),
    )
    if n_cam0[0] is None or n_cam0[1] is None:
        return failure

    xy = np.asarray(scan_xy, dtype=np.float64)
    if xy.ndim != 2 or xy.shape[0] == 0:
        return Se2Match(ok=False, detail=DETAIL_TOO_FEW_SCAN_POINTS)

    yaw_agree = math.radians(YAW_AGREE_DEG)
    axis_swap_tol = math.radians(AXIS_SWAP_DEG)
    candidates: list[tuple[float, float, float, float, float, _Vector3, tuple[int, int], str]] = []

    baseline_bl = lidar_base_link_from_frame(xy, lidar_xyz, lidar_rpy)
    baseline_lines = extract_orthogonal_lines(baseline_bl)

    for extra in LIDAR_YAW_EXTRAS:
        lidar_rpy_try = _lidar_rpy_with_extra(lidar_rpy, extra)
        scan_bl = lidar_base_link_from_frame(xy, lidar_xyz, lidar_rpy_try)
        lines = extract_orthogonal_lines(scan_bl)
        if not lines.ok or lines.first is None or lines.second is None:
            continue
        line_pair = (lines.first, lines.second)
        line_points = (lines.first.points_bl, lines.second.points_bl)
        extra_is_swap = abs(wrap_pi(extra)) >= math.pi / 2.0 - 1e-9

        for perm in permutations((0, 1)):
            for s0 in (1.0, -1.0):
                for s1 in (1.0, -1.0):
                    signs = (s0, s1)
                    yaw_est: list[float] = []
                    skip = False
                    for wall_i in (0, 1):
                        n_c = n_cam0[wall_i] * signs[wall_i]
                        n_l = line_pair[perm[wall_i]].normal
                        yaw_i = math.atan2(float(n_l[1]), float(n_l[0])) - math.atan2(
                            float(n_c[1]),
                            float(n_c[0]),
                        )
                        yaw_est.append(yaw_i)
                    if _angle_diff(yaw_est[0], yaw_est[1]) > yaw_agree:
                        skip = True
                    if skip:
                        continue
                    yaw = _mean_angle(yaw_est[0], yaw_est[1])
                    delta = wrap_pi(yaw - current_rpy[2])
                    snapped = nearest_right_angle(delta)
                    cause = ""
                    if extra_is_swap:
                        cause = "axis swap"
                    elif abs(snapped) >= math.pi / 2.0 - 1e-9 and abs(wrap_pi(delta - snapped)) <= axis_swap_tol:
                        yaw = wrap_pi(current_rpy[2] + snapped)
                        cause = "axis swap"
                    yaw = unwrap_angle_near(current_rpy[2], yaw)
                    rpy = (proposed_tilt_rpy[0], proposed_tilt_rpy[1], yaw)
                    solved = _solve_xy(
                        wall_planes,
                        (line_pair[perm[0]], line_pair[perm[1]]),
                        proposed_tilt_xyz,
                        rpy,
                        signs,
                    )
                    if solved is None:
                        continue
                    x, y = solved
                    xy_err = math.hypot(x - factory_xy[0], y - factory_xy[1])
                    if x < 0.0 or xy_err > CAMERA_XY_MAX_DELTA_M:
                        continue
                    xyz = (x, y, proposed_tilt_xyz[2])
                    residual = scan_plane_residual(
                        wall_planes,
                        line_points,
                        (xyz, rpy),
                        perm,
                    )
                    candidates.append(
                        (
                            residual,
                            xy_err,
                            x,
                            y,
                            yaw,
                            lidar_rpy_try,
                            perm,
                            cause,
                        )
                    )

    if not candidates:
        return failure
    candidates.sort(
        key=lambda item: (
            item[0],
            abs(wrap_pi(item[4] - current_rpy[2])),
            item[1],
            abs(wrap_pi(item[5][2] - lidar_rpy[2])),
        )
    )
    best = candidates[0]
    if len(candidates) > 1:
        second = candidates[1]
        extra_best = wrap_pi(best[5][2] - lidar_rpy[2])
        extra_second = wrap_pi(second[5][2] - lidar_rpy[2])
        same_lidar_extra = abs(wrap_pi(extra_best - extra_second)) <= math.radians(AXIS_SWAP_DEG)
        if (
            second[0] < float("inf")
            and best[0] > 0.0
            and second[0] <= best[0] / AMBIGUOUS_SCORE_RATIO
            and same_lidar_extra
            and second[6] != best[6]
        ):
            return Se2Match(ok=False, detail=DETAIL_UNCERTAIN_ALIGNMENT)

    residual_after = best[0]
    before_perm = best[6]
    if baseline_lines.ok and baseline_lines.first is not None and baseline_lines.second is not None:
        residual_before = scan_plane_residual(
            wall_planes,
            (baseline_lines.first.points_bl, baseline_lines.second.points_bl),
            current_camera_pose,
            before_perm,
        )
    else:
        residual_before = residual_after
    xyz = (best[2], best[3], proposed_tilt_xyz[2])
    rpy = (proposed_tilt_rpy[0], proposed_tilt_rpy[1], best[4])
    return Se2Match(
        ok=True,
        xyz=xyz,
        rpy=rpy,
        lidar_xyz=lidar_xyz,
        lidar_rpy=best[5],
        cause=best[7],
        residual_before=residual_before,
        residual_after=residual_after,
        detail=None,
    )


def _solve_xy(
    wall_planes: tuple[tuple[np.ndarray, float], tuple[np.ndarray, float]],
    matched_lines: tuple[ScanLine, ScanLine],
    xyz: _Vector3,
    rpy: _Vector3,
    signs: tuple[float, float],
) -> Optional[tuple[float, float]]:
    rows = []
    rhs = []
    for index, (n_opt, d_opt) in enumerate(wall_planes):
        n_signed = np.asarray(n_opt, dtype=np.float64) * signs[index]
        n_bl, _d_bl = plane_optical_to_base_link(n_signed, float(d_opt) * signs[index], xyz, rpy)
        pts = matched_lines[index].points_bl
        if pts.shape[0] == 0:
            return None
        p_mean = pts.mean(axis=0)
        # n_bl · t_cam = n_bl · p + d_opt_signed
        rows.append([float(n_bl[0]), float(n_bl[1])])
        rhs.append(float(np.dot(n_bl, p_mean)) + float(d_opt) * signs[index] - float(n_bl[2]) * xyz[2])
    matrix = np.asarray(rows, dtype=np.float64)
    vec = np.asarray(rhs, dtype=np.float64)
    if abs(float(np.linalg.det(matrix))) < 1e-8:
        return None
    try:
        xy = np.linalg.solve(matrix, vec)
    except np.linalg.LinAlgError:
        return None
    return float(xy[0]), float(xy[1])


def scan_points_in_base_link(
    scans: Sequence[Any],
    lidar_pose: _Pose,
) -> np.ndarray:
    xy = scans_to_xy(scans)
    if xy.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)
    xyz, rpy = lidar_pose
    return lidar_base_link_from_frame(xy, xyz, rpy)
