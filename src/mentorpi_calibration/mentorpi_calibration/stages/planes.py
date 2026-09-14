"""RANSAC planes, orthogonal walls, and a 1-D floor strip (SD012 D3.2)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from mentorpi_calibration.calibration_file import BASE_LINK_OFFSET_Z, MAX_TRANSLATION_M, depth_cam_optical_rpy
from mentorpi_calibration.stages.tilt import rpy_from_up_keep_yaw
from mentorpi_calibration.stages.transforms import np_rotation

DETAIL_ONE_WALL = "one wall, need a corner or two walls"
DETAIL_WALLS_NOT_ORTHOGONAL = "walls not orthogonal"
DETAIL_TOO_FEW_WALL_POINTS = "too few wall points"
DETAIL_CAMERA_HEIGHT_NOT_OBSERVABLE = "camera height not observable, use calib camera --height"

# Implementation tuning constants, not an accuracy SLA.
RANSAC_SAMPLE_CAP = 4000
RANSAC_ITERATIONS = 400
RANSAC_INLIER_DISTANCE_M = 0.02
MIN_WALL_INLIERS = 150
ORTHOGONALITY_DEG = 15.0
WALL_UP_DOT_MAX = 0.45
MIN_FLOOR_STRIP_POINTS = 80
FLOOR_STRIP_BIN_M = 0.04
MAX_FLOOR_HEIGHT_M = MAX_TRANSLATION_M
ORIGIN_EPS_M = 1e-4

_COLLINEAR_EPS = 1e-8
_NORMAL_EPS = 1e-12
_Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class PlaneFit:
    """A plane n·p + d = 0 with ||n|| = 1, in the cloud's frame."""

    ok: bool
    normal: Optional[np.ndarray] = None
    d: Optional[float] = None
    inliers_mask: Optional[np.ndarray] = None
    detail: Optional[str] = None


@dataclass(frozen=True)
class OrthogonalWalls:
    """Two orthogonal wall planes in the camera optical frame."""

    ok: bool
    first: Optional[PlaneFit] = None
    second: Optional[PlaneFit] = None
    up: Optional[np.ndarray] = None
    detail: Optional[str] = None


@dataclass(frozen=True)
class FloorStrip:
    """1-D height along a known vertical; missing strip is not a failure."""

    observable: bool
    height_h: Optional[float] = None
    inliers_mask: Optional[np.ndarray] = None


def finite_nonzero_points(points: np.ndarray) -> np.ndarray:
    """Drop non-finite rows and Aurora holes at the origin."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if pts.shape[0] == 0:
        return pts
    radii = np.linalg.norm(pts, axis=1)
    return pts[radii > ORIGIN_EPS_M]


def _fit_plane_svd(points: np.ndarray) -> Optional[tuple[np.ndarray, float]]:
    if points.shape[0] < 3:
        return None
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    norm = np.linalg.norm(normal)
    if norm < _NORMAL_EPS:
        return None
    normal = normal / norm
    d = -float(np.dot(normal, centroid))
    return normal, d


def fit_plane_ransac(
    points: np.ndarray,
    *,
    min_inliers: int = MIN_WALL_INLIERS,
    up: Optional[np.ndarray] = None,
    max_up_dot: float = WALL_UP_DOT_MAX,
) -> PlaneFit:
    """Fit n·p + d = 0 via RANSAC and SVD refit. No inlier-ratio gate.

    When ``up`` is set, candidate planes whose normal is aligned with ``up``
    (floor/ceiling) are skipped so sequential extraction prefers walls.
    """
    failure = PlaneFit(ok=False, detail=DETAIL_TOO_FEW_WALL_POINTS)
    pts = finite_nonzero_points(points)
    n_points = pts.shape[0]
    if n_points < 3:
        return failure

    sample_count = min(n_points, RANSAC_SAMPLE_CAP)
    if sample_count < 3:
        return failure
    sample_idx = (
        np.arange(n_points)
        if sample_count == n_points
        else np.random.choice(n_points, size=sample_count, replace=False)
    )
    sample = pts[sample_idx]
    up_vec = None if up is None else np.asarray(up, dtype=np.float64).reshape(3)
    if up_vec is not None:
        up_norm = np.linalg.norm(up_vec)
        if up_norm < _NORMAL_EPS:
            up_vec = None
        else:
            up_vec = up_vec / up_norm

    best_inlier_count = 0
    best_inlier_idx: Optional[np.ndarray] = None

    for _ in range(RANSAC_ITERATIONS):
        triplet = np.random.choice(sample_count, size=3, replace=False)
        p0, p1, p2 = sample[triplet[0]], sample[triplet[1]], sample[triplet[2]]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < _COLLINEAR_EPS:
            continue
        normal = normal / norm
        if up_vec is not None and abs(float(np.dot(normal, up_vec))) > max_up_dot:
            continue
        d = -float(np.dot(normal, p0))
        distances = np.abs(pts @ normal + d)
        inlier_mask = distances < RANSAC_INLIER_DISTANCE_M
        inlier_count = int(inlier_mask.sum())
        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_inlier_idx = inlier_mask

    if best_inlier_idx is None or best_inlier_count < min_inliers:
        return failure

    refit = _fit_plane_svd(pts[best_inlier_idx])
    if refit is None:
        return failure
    normal, d = refit
    if up_vec is not None and abs(float(np.dot(normal, up_vec))) > max_up_dot:
        return failure
    distances = np.abs(pts @ normal + d)
    inliers_mask = distances < RANSAC_INLIER_DISTANCE_M
    if int(inliers_mask.sum()) < min_inliers:
        return failure
    return PlaneFit(
        ok=True,
        normal=normal.astype(np.float64),
        d=float(d),
        inliers_mask=inliers_mask,
        detail=None,
    )


def extract_orthogonal_walls(
    points: np.ndarray,
    *,
    up_prior: Optional[np.ndarray] = None,
) -> OrthogonalWalls:
    """Sequential RANSAC of two wall planes; normals must be orthogonal."""
    pts = finite_nonzero_points(points)
    if pts.shape[0] < MIN_WALL_INLIERS * 2:
        return OrthogonalWalls(ok=False, detail=DETAIL_TOO_FEW_WALL_POINTS)

    first = fit_plane_ransac(pts, up=up_prior)
    if not first.ok or first.inliers_mask is None:
        return OrthogonalWalls(ok=False, detail=first.detail or DETAIL_ONE_WALL)

    remaining = pts[~first.inliers_mask]
    second = fit_plane_ransac(remaining, up=up_prior)
    if not second.ok or second.normal is None or first.normal is None:
        return OrthogonalWalls(
            ok=False,
            first=first,
            detail=DETAIL_ONE_WALL,
        )

    # Second-plane inliers are indexed into ``remaining``; expand to ``pts``.
    second_mask = np.zeros(pts.shape[0], dtype=bool)
    second_mask[~first.inliers_mask] = second.inliers_mask
    second = PlaneFit(
        ok=True,
        normal=second.normal,
        d=second.d,
        inliers_mask=second_mask,
        detail=None,
    )

    dot = abs(float(np.dot(first.normal, second.normal)))
    angle_deg = math.degrees(math.acos(min(1.0, dot)))
    if abs(90.0 - angle_deg) > ORTHOGONALITY_DEG:
        return OrthogonalWalls(
            ok=False,
            first=first,
            second=second,
            detail=DETAIL_WALLS_NOT_ORTHOGONAL,
        )

    up = np.cross(first.normal, second.normal)
    up_norm = np.linalg.norm(up)
    if up_norm < _NORMAL_EPS:
        return OrthogonalWalls(
            ok=False,
            first=first,
            second=second,
            detail=DETAIL_WALLS_NOT_ORTHOGONAL,
        )
    up = up / up_norm
    if up_prior is not None:
        prior = np.asarray(up_prior, dtype=np.float64).reshape(3)
        prior_norm = np.linalg.norm(prior)
        if prior_norm > _NORMAL_EPS and float(np.dot(up, prior / prior_norm)) < 0.0:
            up = -up
    return OrthogonalWalls(
        ok=True,
        first=first,
        second=second,
        up=up,
        detail=None,
    )


def estimate_floor_strip(
    points: np.ndarray,
    up: np.ndarray,
    *,
    wall_mask: Optional[np.ndarray] = None,
) -> FloorStrip:
    """1-D peak along ``up`` below the camera; no dominance requirement."""
    missing = FloorStrip(observable=False)
    pts = finite_nonzero_points(points)
    if pts.shape[0] == 0:
        return missing
    up_vec = np.asarray(up, dtype=np.float64).reshape(3)
    up_norm = np.linalg.norm(up_vec)
    if up_norm < _NORMAL_EPS:
        return missing
    up_vec = up_vec / up_norm
    if wall_mask is not None and wall_mask.shape[0] == pts.shape[0]:
        candidates = pts[~wall_mask]
    else:
        candidates = pts
    if candidates.shape[0] < MIN_FLOOR_STRIP_POINTS:
        return missing

    proj = candidates @ up_vec
    below = proj < 0.0
    if int(below.sum()) < MIN_FLOOR_STRIP_POINTS:
        return missing
    s_below = proj[below]
    # Densest 1-D window among points below the optical origin.
    order = np.argsort(s_below)
    sorted_s = s_below[order]
    best_count = 0
    best_lo = 0
    best_hi = 0
    lo = 0
    for hi in range(sorted_s.shape[0]):
        while sorted_s[hi] - sorted_s[lo] > FLOOR_STRIP_BIN_M:
            lo += 1
        count = hi - lo + 1
        if count > best_count:
            best_count = count
            best_lo = lo
            best_hi = hi
    if best_count < MIN_FLOOR_STRIP_POINTS:
        return missing
    cluster = sorted_s[best_lo : best_hi + 1]
    height_h = -float(np.median(cluster))
    if height_h <= 0.0 or height_h > MAX_FLOOR_HEIGHT_M:
        return missing
    full_proj = pts @ up_vec
    inliers = np.abs(full_proj + height_h) < RANSAC_INLIER_DISTANCE_M
    return FloorStrip(
        observable=True,
        height_h=height_h,
        inliers_mask=inliers,
    )


def camera_tilt_from_vertical(
    up_opt: np.ndarray,
    current_depth_cam_pose: tuple[_Vector3, _Vector3],
) -> tuple[_Vector3, _Vector3]:
    """Roll/pitch from wall vertical; x, y, z, yaw kept from current pose."""
    current_xyz, current_rpy = current_depth_cam_pose
    r_opt = np_rotation(depth_cam_optical_rpy())
    n_mount = r_opt @ np.asarray(up_opt, dtype=np.float64).reshape(3)
    roll, pitch, yaw = rpy_from_up_keep_yaw(n_mount, current_rpy)
    return current_xyz, (roll, pitch, yaw)


def camera_z_from_height(height_h: float, current_xyz: _Vector3) -> _Vector3:
    """Mount z from optical-origin height above floor in base_footprint."""
    z = height_h - BASE_LINK_OFFSET_Z
    return (current_xyz[0], current_xyz[1], z)


def wall_mask_union(walls: OrthogonalWalls, n_points: int) -> np.ndarray:
    mask = np.zeros(n_points, dtype=bool)
    if walls.first is not None and walls.first.inliers_mask is not None:
        if walls.first.inliers_mask.shape[0] == n_points:
            mask |= walls.first.inliers_mask
    if walls.second is not None and walls.second.inliers_mask is not None:
        if walls.second.inliers_mask.shape[0] == n_points:
            mask |= walls.second.inliers_mask
    return mask
