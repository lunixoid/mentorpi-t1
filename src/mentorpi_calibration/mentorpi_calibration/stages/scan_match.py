"""2-D scan matching for the drive calibration stage (SD012 D3.4)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np

from mentorpi_calibration.stages.scan_lines import laserscan_xy
from mentorpi_calibration.stages.tilt import wrap_pi

# Implementation tuning constants, not an accuracy SLA.
ICP_ITERATIONS = 25
ICP_MAX_DIST_M = 0.35
ICP_MIN_INLIERS = 12
ICP_CONVERGE_TRANS_M = 1e-4
ICP_CONVERGE_YAW_RAD = 1e-4
DOWNSAMPLE_LEAF_M = 0.04
DOWNSAMPLE_CAP = 120
MIN_SCAN_POINTS = 8
MATCH_FAIL_STREAK = 3
SKIP_FRACTION_MAX = 0.5

DETAIL_MATCH_FAILED = "scan matching did not converge"
DETAIL_SCENE_CHANGED = "scene changed"
DETAIL_TOO_FEW_SCANS = "travel too short"

_NN_CHUNK = 64


@dataclass(frozen=True)
class ScanPose:
    """Lidar-frame pose in the first-scan frame (x m, y m, yaw rad)."""

    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Se2Match:
    ok: bool
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    residual: float = float("inf")
    inliers: int = 0


@dataclass(frozen=True)
class ScanTrajectory:
    ok: bool
    poses: tuple[ScanPose, ...] = ()
    travel_m: float = 0.0
    turn_deg: float = 0.0
    mean_residual: float = float("inf")
    detail: Optional[str] = None


def scan_xy(scan: Any) -> np.ndarray:
    """XY returns from a LaserScan, DriveScan, or (N, 2) array."""
    if isinstance(scan, np.ndarray):
        arr = np.asarray(scan, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] < 2:
            return np.empty((0, 2), dtype=np.float64)
        return arr[:, :2]
    xy = getattr(scan, "xy", None)
    if isinstance(xy, np.ndarray):
        arr = np.asarray(xy, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] < 2:
            return np.empty((0, 2), dtype=np.float64)
        return arr[:, :2]
    if hasattr(scan, "ranges"):
        return laserscan_xy(scan)
    return np.empty((0, 2), dtype=np.float64)


def scan_stamp(scan: Any, fallback: float = 0.0) -> float:
    """Message timestamp in seconds, or ``fallback`` when absent."""
    stamp = getattr(scan, "stamp", None)
    if isinstance(stamp, (int, float)):
        return float(stamp)
    header = getattr(scan, "header", None)
    if header is None:
        return fallback
    value = getattr(header, "stamp", None)
    if value is None:
        return fallback
    if isinstance(value, (int, float)):
        return float(value)
    sec = float(getattr(value, "sec", 0.0))
    nanosec = float(getattr(value, "nanosec", 0.0))
    return sec + 1e-9 * nanosec


def downsample_xy(xy: np.ndarray, leaf: float = DOWNSAMPLE_LEAF_M) -> np.ndarray:
    """Voxel-grid downsample in the scan plane, capped at ``DOWNSAMPLE_CAP``."""
    pts = np.asarray(xy, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return np.empty((0, 2), dtype=np.float64)
    pts = pts[:, :2]
    if pts.shape[0] <= DOWNSAMPLE_CAP and leaf <= 0.0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int32)
    _, index = np.unique(keys, axis=0, return_index=True)
    index.sort()
    down = pts[index]
    if down.shape[0] > DOWNSAMPLE_CAP:
        step = int(math.ceil(down.shape[0] / float(DOWNSAMPLE_CAP)))
        down = down[::step][:DOWNSAMPLE_CAP]
    return down


def apply_se2(xy: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    """Apply ``p' = R(yaw) p + (x, y)``."""
    x, y, yaw = pose
    pts = np.asarray(xy, dtype=np.float64)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=np.float64)
    return pts @ rotation.T + np.array([x, y], dtype=np.float64)


def compose_se2(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Compose ``T_left * T_right`` as (x, y, yaw)."""
    x1, y1, yaw1 = left
    x2, y2, yaw2 = right
    cosine = math.cos(yaw1)
    sine = math.sin(yaw1)
    return (
        x1 + cosine * x2 - sine * y2,
        y1 + sine * x2 + cosine * y2,
        wrap_pi(yaw1 + yaw2),
    )


def _nearest_neighbours(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Brute-force nearest neighbour of each source point in ``target``."""
    if source.shape[0] == 0 or target.shape[0] == 0:
        empty = np.empty((0,), dtype=np.int64)
        return empty, np.empty((0,), dtype=np.float64)
    best_dist = np.full(source.shape[0], np.inf, dtype=np.float64)
    best_idx = np.zeros(source.shape[0], dtype=np.int64)
    for start in range(0, source.shape[0], _NN_CHUNK):
        chunk = source[start : start + _NN_CHUNK]
        delta = chunk[:, None, :] - target[None, :, :]
        dist2 = np.einsum("ijk,ijk->ij", delta, delta)
        best_idx[start : start + chunk.shape[0]] = dist2.argmin(axis=1)
        best_dist[start : start + chunk.shape[0]] = np.sqrt(dist2.min(axis=1))
    return best_idx, best_dist


def _se2_from_correspondences(src: np.ndarray, dst: np.ndarray) -> Optional[tuple[float, float, float]]:
    """Umeyama SE(2): ``R src + t ≈ dst``."""
    if src.shape[0] < 2:
        return None
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst
    covariance = src_c.T @ dst_c
    u_mat, _singular, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u_mat.T
    if float(np.linalg.det(rotation)) < 0.0:
        vt = vt.copy()
        vt[1, :] *= -1.0
        rotation = vt.T @ u_mat.T
    translation = mu_dst - rotation @ mu_src
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return (float(translation[0]), float(translation[1]), float(yaw))


def _line_normals(points: np.ndarray) -> np.ndarray:
    """Unit normals from each point to its nearest neighbour tangent."""
    pts = np.asarray(points, dtype=np.float64)
    n_points = pts.shape[0]
    normals = np.zeros_like(pts)
    if n_points == 0:
        return normals
    for index in range(n_points):
        delta = pts - pts[index]
        dist2 = np.einsum("ij,ij->i", delta, delta)
        dist2[index] = np.inf
        nearest = int(np.argmin(dist2))
        tangent = pts[nearest] - pts[index]
        length = math.hypot(float(tangent[0]), float(tangent[1]))
        if length < 1e-9:
            normals[index] = np.array([1.0, 0.0], dtype=np.float64)
            continue
        normals[index] = np.array(
            [-tangent[1] / length, tangent[0] / length],
            dtype=np.float64,
        )
    return normals


def _point_to_line_delta(
    src: np.ndarray,
    dst: np.ndarray,
    normals: np.ndarray,
) -> Optional[tuple[float, float, float]]:
    """Small-angle point-to-line SE(2) step: n·(R p + t - q) ≈ 0."""
    if src.shape[0] < 3:
        return None
    rows = np.column_stack(
        (
            normals[:, 0],
            normals[:, 1],
            normals[:, 1] * src[:, 0] - normals[:, 0] * src[:, 1],
        )
    )
    rhs = np.einsum("ij,ij->i", normals, dst - src)
    try:
        sol, _, rank, _ = np.linalg.lstsq(rows, rhs, rcond=None)
    except np.linalg.LinAlgError:
        return None
    if rank < 3:
        return None
    return (float(sol[0]), float(sol[1]), float(sol[2]))


def icp_se2(source: np.ndarray, target: np.ndarray) -> Se2Match:
    """Point-to-line ICP aligning ``source`` into ``target``."""
    src = downsample_xy(source)
    dst = downsample_xy(target)
    if src.shape[0] < MIN_SCAN_POINTS or dst.shape[0] < MIN_SCAN_POINTS:
        return Se2Match(ok=False)
    dst_normals = _line_normals(dst)
    pose = (0.0, 0.0, 0.0)
    residual = float("inf")
    inliers = 0
    max_dist = ICP_MAX_DIST_M
    for iteration in range(ICP_ITERATIONS):
        if iteration < 4:
            max_dist = ICP_MAX_DIST_M * 1.6
        else:
            max_dist = ICP_MAX_DIST_M
        transformed = apply_se2(src, pose)
        idx, dist = _nearest_neighbours(transformed, dst)
        mask = dist < max_dist
        inliers = int(mask.sum())
        if inliers < ICP_MIN_INLIERS:
            return Se2Match(ok=False, pose=pose, residual=residual, inliers=inliers)
        src_in = transformed[mask]
        dst_in = dst[idx[mask]]
        n_in = dst_normals[idx[mask]]
        delta = _point_to_line_delta(src_in, dst_in, n_in)
        if delta is None:
            umeyama = _se2_from_correspondences(src_in, dst_in)
            if umeyama is None:
                return Se2Match(ok=False, pose=pose, residual=residual, inliers=inliers)
            delta = umeyama
        pose = compose_se2(delta, pose)
        residual = float(np.mean(np.abs(np.einsum("ij,ij->i", n_in, dst_in - src_in))))
        if math.hypot(delta[0], delta[1]) < ICP_CONVERGE_TRANS_M and abs(delta[2]) < ICP_CONVERGE_YAW_RAD:
            break
    if inliers < ICP_MIN_INLIERS or not math.isfinite(residual):
        return Se2Match(ok=False, pose=pose, residual=residual, inliers=inliers)
    return Se2Match(ok=True, pose=pose, residual=residual, inliers=inliers)


class ScanTracker:
    """Incremental lidar-frame odometry from consecutive scans."""

    def __init__(self) -> None:
        self.poses: list[ScanPose] = []
        self.travel_m = 0.0
        self.turn_deg = 0.0
        self.failed = False
        self.detail = ""
        self._last_xy: Optional[np.ndarray] = None
        self._fail_streak = 0
        self._skipped = 0
        self._seen = 0
        self._residuals: list[float] = []

    def push(self, xy: np.ndarray, stamp: float) -> bool:
        """Incorporate one scan. Returns False once matching has failed."""
        if self.failed:
            return False
        pts = downsample_xy(xy)
        self._seen += 1
        if pts.shape[0] < MIN_SCAN_POINTS:
            self._skipped += 1
            return True
        if self._last_xy is None:
            self.poses.append(ScanPose(stamp=stamp, x=0.0, y=0.0, yaw=0.0))
            self._last_xy = pts
            return True
        match = icp_se2(pts, self._last_xy)
        if not match.ok:
            self._fail_streak += 1
            self._skipped += 1
            if self._fail_streak >= MATCH_FAIL_STREAK:
                self.failed = True
                self.detail = DETAIL_SCENE_CHANGED if len(self.poses) > 3 else DETAIL_MATCH_FAILED
                return False
            return True
        self._fail_streak = 0
        prev = self.poses[-1]
        composed = compose_se2((prev.x, prev.y, prev.yaw), match.pose)
        dx = composed[0] - prev.x
        dy = composed[1] - prev.y
        self.travel_m += math.hypot(dx, dy)
        dyaw = wrap_pi(composed[2] - prev.yaw)
        self.turn_deg += abs(math.degrees(dyaw))
        self.poses.append(ScanPose(stamp=stamp, x=composed[0], y=composed[1], yaw=composed[2]))
        self._residuals.append(match.residual)
        self._last_xy = pts
        return True

    def finish(self) -> ScanTrajectory:
        if self.failed:
            return ScanTrajectory(ok=False, detail=self.detail)
        if self._seen > 0 and self._skipped / float(self._seen) > SKIP_FRACTION_MAX:
            return ScanTrajectory(ok=False, detail=DETAIL_MATCH_FAILED)
        if len(self.poses) < 2:
            return ScanTrajectory(ok=False, detail=DETAIL_TOO_FEW_SCANS)
        mean_residual = float(np.mean(self._residuals)) if self._residuals else 0.0
        return ScanTrajectory(
            ok=True,
            poses=tuple(self.poses),
            travel_m=self.travel_m,
            turn_deg=self.turn_deg,
            mean_residual=mean_residual,
        )


def build_scan_trajectory(scans: Sequence[Any]) -> ScanTrajectory:
    """Match consecutive scans into a lidar-frame trajectory."""
    tracker = ScanTracker()
    for index, scan in enumerate(scans):
        xy = scan_xy(scan)
        stamp = scan_stamp(scan, fallback=float(index) * 0.1)
        if not tracker.push(xy, stamp):
            return tracker.finish()
    return tracker.finish()
