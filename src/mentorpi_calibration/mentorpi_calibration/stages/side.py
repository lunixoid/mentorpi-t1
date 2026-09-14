"""Side stage: transverse sign from one object in scan, cloud, and RGB (SD022 D2)."""

from __future__ import annotations

import math
import os
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, Sequence, TextIO

import numpy as np

from mentorpi_calibration.calibration_file import Calibration, resolve_dir, transverse_mirror, write_draft
from mentorpi_calibration.pointcloud2 import CloudLayoutError, clouds_to_xyz, count_cloud_points
from mentorpi_calibration.stages.planes import finite_nonzero_points
from mentorpi_calibration.stages.scan_lines import count_scan_rays, scans_to_xy
from mentorpi_calibration.stages.transforms import camera_base_link_from_optical, lidar_base_link_from_frame

DEFAULT_TIMEOUT = 5.0

STAGE_NAME = "side"
HINT = "place an object clearly to one side"

TOPIC_POINTS2 = "/aurora/points2"
TOPIC_SCAN = "/scan"
TOPIC_RGB = "/aurora/rgb/image_raw"
TOPIC_CAMERA_INFO = "/aurora/rgb/camera_info"

DETAIL_NO_CAMERA_DEPTH = "no camera depth"
DETAIL_NO_SCAN = "no /scan"
DETAIL_NO_CAMERA_RGB = "no camera rgb"
DETAIL_NO_CAMERA_INFO = "no camera info"
DETAIL_OBJECT_NOT_FOUND_SCAN = "object not found in scan"
DETAIL_OBJECT_NOT_FOUND_CLOUD = "object not found in cloud"
DETAIL_LIDAR_SIDE_DISAGREES = "lidar side disagrees with operator"
DETAIL_CANNOT_PROJECT = "cannot project to frame"

SIDE_LEFT = "left"
SIDE_RIGHT = "right"

LAYER_CLOUD_GEOMETRY = "cloud_geometry"
LAYER_PIXEL_MAPPING = "pixel_mapping"
LAYER_NONE = "none"
LAYER_LIDAR_SUSPECT = "lidar_suspect"

SIDE_FRAME_FILENAME = "side_frame.png"
_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_MARK_RGB = (255, 255, 0)
_MARK_OUTLINE = (0, 0, 0)

FIELD_CAMERA_TRANSVERSE_MIRROR = "camera_transverse_mirror"

# Horizontal slab around the lidar plane (stand floor is leveled by corner).
# Live Aurora histogram 2026-09-05: floor z<0.10, table/ceiling z>0.55, lidar 0.18 m.
# Sector ±40° is applied in _workspace_mask. Cloud is then binned in azimuth like /scan.
FRONTAL_HALF_FOV_DEG = 40.0
RANGE_MIN_M = 0.30
RANGE_MAX_M = 3.0
AZIMUTH_GAP_DEG = 6.0
RANGE_JUMP_M = 0.20
MAX_CLUSTER_WIDTH_DEG = 40.0
MIN_SCAN_CLUSTER_POINTS = 4
MIN_CLOUD_CLUSTER_POINTS = 8
CLOUD_Z_MIN_M = 0.12
CLOUD_Z_MAX_M = 0.40
CLOUD_POLAR_BIN_DEG = 0.5
# Pillow/box sitting against a wall is only a few cm closer than the wall
# (stand 2026-09-05: shaggy pillow on the left of a corner).
FOREGROUND_MARGIN_M = 0.04
MIN_OBJECT_ABS_BEARING_DEG = 8.0

_PROGRESS_INTERVAL_SEC = 0.1
_FRONTAL_HALF_FOV_RAD = math.radians(FRONTAL_HALF_FOV_DEG)
_AZIMUTH_GAP_RAD = math.radians(AZIMUTH_GAP_DEG)
_MAX_CLUSTER_WIDTH_RAD = math.radians(MAX_CLUSTER_WIDTH_DEG)


class RosUnavailableError(RuntimeError):
    """ROS Python bindings are not importable in this environment."""


@dataclass(frozen=True)
class SideSamples:
    """Raw samples gathered for the side calibration stage."""

    clouds: tuple[Any, ...]
    scans: tuple[Any, ...]
    images: tuple[Any, ...]
    frames: int
    cloud_points: Optional[int]
    scan_rays: Optional[int]
    rgb_frames: int
    ok: bool
    details: tuple[str, ...]
    camera_infos: tuple[Any, ...] = ()


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics of the RGB frame, same meaning as person_geometry.hpp.

    /aurora/points2 is not organized (SD022 D6.1), so a cloud point is matched to
    a pixel by projection, never by a slot index.
    """

    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    width: int = 0
    height: int = 0

    @property
    def valid(self) -> bool:
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(v) for v in values):
            return False
        return self.fx > 0.0 and self.fy > 0.0


@dataclass(frozen=True)
class SideEvaluation:
    """Result of side-stage cluster → bearings → D1.2 layer (D2.2–D2.4)."""

    ok: bool
    stage: str
    details: tuple[str, ...]
    operator_side: Optional[str] = None
    scan_side: Optional[str] = None
    cloud_side: Optional[str] = None
    scan_bearing_deg: Optional[float] = None
    cloud_bearing_deg: Optional[float] = None
    layer: Optional[str] = None
    propose_transverse_mirror: Optional[bool] = None
    cloud_cluster_bl: Optional[np.ndarray] = None
    scan_cluster_bl: Optional[np.ndarray] = None
    frame_path: Optional[str] = None
    frame_u_px: Optional[float] = None
    frame_side: Optional[str] = None


@dataclass(frozen=True)
class SideOperatorField:
    """Operator-facing before/after value for CLI ``field:`` lines."""

    name: str
    before: object
    after: object
    source: str = "computed"


@dataclass(frozen=True)
class _ObjectObservation:
    found: bool
    side: Optional[str] = None
    bearing_deg: Optional[float] = None
    points_bl: Optional[np.ndarray] = None
    points_opt: Optional[np.ndarray] = None


@dataclass(frozen=True)
class _FrameProjection:
    path: str
    u_px: float
    v_px: float
    frame_side: str


class SideCollector(Protocol):
    """Injectable backend for side sample collection."""

    def topics_available(self) -> tuple[bool, bool, bool, bool]:
        """Return whether points2, /scan, RGB, and camera_info are in the graph."""

    def run(
        self,
        timeout: float,
        on_progress: Callable[[int, Optional[int], Optional[int], int], None],
    ) -> tuple[list[Any], list[Any], list[Any], list[Any]]:
        """Collect clouds, scans, RGB images, and camera_info."""

    def close(self) -> None:
        """Release collector resources."""


def print_collect_progress(
    stdout: TextIO,
    frames: int,
    cloud_points: Optional[int],
    scan_rays: Optional[int],
    rgb_frames: int,
    *,
    include_header: bool = False,
) -> None:
    """Emit the side collection stdout contract."""
    if include_header:
        print("stage: {}".format(STAGE_NAME), file=stdout)
        print("hint: {}".format(HINT), file=stdout)
    print("frames: {}".format(frames), file=stdout)
    print(
        "cloud_points: {}".format(0 if cloud_points is None else cloud_points),
        file=stdout,
    )
    print("scan_rays: {}".format(0 if scan_rays is None else scan_rays), file=stdout)
    print("rgb_frames: {}".format(rgb_frames), file=stdout)
    stdout.flush()


def _failure_details(
    has_points2: bool,
    has_scan: bool,
    has_rgb: bool,
    has_info: bool,
    clouds: list[Any],
    scans: list[Any],
    images: list[Any],
    infos: list[Any],
) -> list[str]:
    details: list[str] = []
    if not has_points2 or not clouds:
        details.append(DETAIL_NO_CAMERA_DEPTH)
    if not has_scan or not scans:
        details.append(DETAIL_NO_SCAN)
    if not has_rgb or not images:
        details.append(DETAIL_NO_CAMERA_RGB)
    # D2.3: without intrinsics the cluster cannot be put on the frame, so the
    # pixel_mapping verdict would rest on nothing.
    if not has_info or not infos:
        details.append(DETAIL_NO_CAMERA_INFO)
    return details


def collect_side_samples(
    timeout: float = DEFAULT_TIMEOUT,
    *,
    collector: Optional[SideCollector] = None,
    stdout: Optional[TextIO] = None,
) -> SideSamples:
    """Collect side-stage sensor samples."""
    out = stdout if stdout is not None else sys.stdout
    own_collector = collector is None
    if own_collector:
        collector = _create_ros_collector()

    assert collector is not None
    try:
        has_points2, has_scan, has_rgb, has_info = collector.topics_available()
        graph_clouds = [object()] if has_points2 else []
        graph_scans = [object()] if has_scan else []
        graph_images = [object()] if has_rgb else []
        graph_infos = [object()] if has_info else []
        graph_details = _failure_details(
            has_points2,
            has_scan,
            has_rgb,
            has_info,
            graph_clouds,
            graph_scans,
            graph_images,
            graph_infos,
        )
        if graph_details:
            print_collect_progress(out, 0, None, None, 0, include_header=True)
            return SideSamples(
                clouds=(),
                scans=(),
                images=(),
                frames=0,
                cloud_points=None,
                scan_rays=None,
                rgb_frames=0,
                ok=False,
                details=tuple(graph_details),
            )

        print_collect_progress(out, 0, None, None, 0, include_header=True)

        def on_progress(
            frames: int,
            cloud_points: Optional[int],
            scan_rays: Optional[int],
            rgb_frames: int,
        ) -> None:
            print_collect_progress(out, frames, cloud_points, scan_rays, rgb_frames)

        clouds, scans, images, infos = collector.run(timeout, on_progress)
        details = _failure_details(True, True, True, True, clouds, scans, images, infos)
        cloud_points = count_cloud_points(clouds) if not details else None
        scan_rays = count_scan_rays(scans) if not details else None
        return SideSamples(
            clouds=tuple(clouds),
            scans=tuple(scans),
            images=tuple(images),
            frames=len(clouds),
            cloud_points=cloud_points,
            scan_rays=scan_rays,
            rgb_frames=len(images),
            ok=not details,
            details=tuple(details),
            camera_infos=tuple(infos),
        )
    finally:
        if own_collector:
            collector.close()


def collect_ros(
    timeout: float = DEFAULT_TIMEOUT,
    *,
    stdout: Optional[TextIO] = None,
) -> SideSamples:
    """Collect side samples through an ephemeral ROS node."""
    return collect_side_samples(timeout, stdout=stdout)


def _as_xyz(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)
    if pts.shape[1] == 2:
        return np.column_stack((pts, np.zeros(pts.shape[0], dtype=np.float64)))
    return pts[:, :3]


def _workspace_mask(
    points_bl: np.ndarray,
    *,
    z_min: Optional[float] = None,
    z_max: Optional[float] = None,
) -> np.ndarray:
    pts = _as_xyz(points_bl)
    if pts.shape[0] == 0:
        return np.zeros((0,), dtype=bool)
    finite = np.isfinite(pts).all(axis=1)
    azimuth = np.arctan2(pts[:, 1], pts[:, 0])
    ranges = np.hypot(pts[:, 0], pts[:, 1])
    mask = (
        finite
        & (np.abs(azimuth) <= _FRONTAL_HALF_FOV_RAD)
        & (ranges >= RANGE_MIN_M)
        & (ranges <= RANGE_MAX_M)
        & (pts[:, 0] > 0.0)
    )
    if z_min is not None:
        mask &= pts[:, 2] >= z_min
    if z_max is not None:
        mask &= pts[:, 2] <= z_max
    return mask


def _workspace_points(
    points_bl: np.ndarray,
    *,
    z_min: Optional[float] = None,
    z_max: Optional[float] = None,
) -> np.ndarray:
    pts = _as_xyz(points_bl)
    return pts[_workspace_mask(pts, z_min=z_min, z_max=z_max)]


def _cluster_index_groups(points_bl: np.ndarray) -> list[np.ndarray]:
    pts = _as_xyz(points_bl)
    if pts.shape[0] == 0:
        return []
    azimuth = np.arctan2(pts[:, 1], pts[:, 0])
    ranges = np.hypot(pts[:, 0], pts[:, 1])
    order = np.argsort(azimuth)
    azimuth = azimuth[order]
    ranges = ranges[order]
    groups: list[np.ndarray] = []
    start = 0
    for index in range(1, pts.shape[0]):
        azimuth_gap = azimuth[index] - azimuth[index - 1]
        range_jump = abs(ranges[index] - ranges[index - 1])
        if azimuth_gap > _AZIMUTH_GAP_RAD or range_jump > RANGE_JUMP_M:
            groups.append(order[start:index])
            start = index
    groups.append(order[start:])
    return groups


def _polar_median_points(points_bl: np.ndarray, bin_deg: float) -> np.ndarray:
    """Collapse a height slab to one median range per azimuth bin (virtual scan)."""
    pts = _as_xyz(points_bl)
    if pts.shape[0] == 0 or bin_deg <= 0.0:
        return np.empty((0, 3), dtype=np.float64)
    azimuth = np.arctan2(pts[:, 1], pts[:, 0])
    ranges = np.hypot(pts[:, 0], pts[:, 1])
    bin_rad = math.radians(bin_deg)
    bin_id = np.floor(azimuth / bin_rad).astype(np.int64)
    rows: list[tuple[float, float, float]] = []
    for bid in np.unique(bin_id):
        sel = bin_id == bid
        r_med = float(np.median(ranges[sel]))
        z_med = float(np.median(pts[sel, 2]))
        angle = (float(bid) + 0.5) * bin_rad
        rows.append((r_med * math.cos(angle), r_med * math.sin(angle), z_med))
    return np.asarray(rows, dtype=np.float64)


def _split_clusters(points_bl: np.ndarray) -> list[np.ndarray]:
    pts = _as_xyz(points_bl)
    return [pts[group] for group in _cluster_index_groups(pts)]


def _work_indices_for_polar_cluster(
    work: np.ndarray,
    polar_cluster: np.ndarray,
    *,
    min_points: int,
) -> Optional[np.ndarray]:
    """Map a polar-bin cluster back onto the original slab points."""
    if polar_cluster.shape[0] == 0:
        return None
    azimuth = np.arctan2(polar_cluster[:, 1], polar_cluster[:, 0])
    ranges = np.hypot(polar_cluster[:, 0], polar_cluster[:, 1])
    az_min = float(azimuth.min()) - math.radians(1.0)
    az_max = float(azimuth.max()) + math.radians(1.0)
    r_med = float(np.median(ranges))
    r_tol = max(0.12, float(ranges.max() - ranges.min()) + 0.08)
    work_az = np.arctan2(work[:, 1], work[:, 0])
    work_r = np.hypot(work[:, 0], work[:, 1])
    sel = (work_az >= az_min) & (work_az <= az_max) & (np.abs(work_r - r_med) <= r_tol)
    if int(sel.sum()) < min_points:
        return None
    return np.flatnonzero(sel)


def _nearest_compact_cluster_indices(
    points_bl: np.ndarray,
    groups: Sequence[np.ndarray],
    *,
    min_points: int,
) -> Optional[np.ndarray]:
    pts = _as_xyz(points_bl)
    candidates: list[tuple[float, np.ndarray]] = []
    for group in groups:
        if group.shape[0] < min_points:
            continue
        cluster = pts[group]
        azimuth = np.arctan2(cluster[:, 1], cluster[:, 0])
        width = float(azimuth.max() - azimuth.min())
        if width > _MAX_CLUSTER_WIDTH_RAD:
            continue
        ranges = np.hypot(cluster[:, 0], cluster[:, 1])
        candidates.append((float(np.median(ranges)), group))
    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def _nearest_compact_cluster(
    clusters: Sequence[np.ndarray],
    *,
    min_points: int,
) -> Optional[np.ndarray]:
    candidates: list[tuple[float, np.ndarray]] = []
    for cluster in clusters:
        if cluster.shape[0] < min_points:
            continue
        azimuth = np.arctan2(cluster[:, 1], cluster[:, 0])
        width = float(azimuth.max() - azimuth.min())
        if width > _MAX_CLUSTER_WIDTH_RAD:
            continue
        ranges = np.hypot(cluster[:, 0], cluster[:, 1])
        candidates.append((float(np.median(ranges)), cluster))
    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def _closer_than_background(points_bl: np.ndarray, margin: float) -> np.ndarray:
    """True where range is at least ``margin`` closer than the sector median."""
    pts = _as_xyz(points_bl)
    if pts.shape[0] == 0:
        return np.zeros((0,), dtype=bool)
    ranges = np.hypot(pts[:, 0], pts[:, 1])
    return ranges <= (float(np.median(ranges)) - margin)


def _side_from_bearing(bearing_rad: float) -> str:
    if bearing_rad > 0.0:
        return SIDE_LEFT
    return SIDE_RIGHT


def _observe_object(
    points_bl: np.ndarray,
    *,
    min_points: int,
    z_min: Optional[float] = None,
    z_max: Optional[float] = None,
    points_opt: Optional[np.ndarray] = None,
    polar_bin_deg: Optional[float] = None,
) -> _ObjectObservation:
    pts = _as_xyz(points_bl)
    mask = _workspace_mask(pts, z_min=z_min, z_max=z_max)
    work = pts[mask]
    work_idx = np.flatnonzero(mask)
    if work.shape[0] == 0:
        return _ObjectObservation(found=False)
    near = _closer_than_background(work, FOREGROUND_MARGIN_M)
    if int(near.sum()) >= min_points:
        work = work[near]
        work_idx = work_idx[near]
    if polar_bin_deg is not None:
        polar = _polar_median_points(work, polar_bin_deg)
        polar_cluster = _nearest_compact_cluster(
            _split_clusters(polar),
            min_points=max(3, min_points // 2),
        )
        if polar_cluster is None:
            return _ObjectObservation(found=False)
        local = _work_indices_for_polar_cluster(work, polar_cluster, min_points=min_points)
        if local is None:
            return _ObjectObservation(found=False)
    else:
        local = _nearest_compact_cluster_indices(
            work,
            _cluster_index_groups(work),
            min_points=min_points,
        )
        if local is None:
            return _ObjectObservation(found=False)
    source_idx = work_idx[local]
    cluster = pts[source_idx]
    mean_xy = cluster[:, :2].mean(axis=0)
    bearing_rad = math.atan2(float(mean_xy[1]), float(mean_xy[0]))
    if abs(math.degrees(bearing_rad)) < MIN_OBJECT_ABS_BEARING_DEG:
        return _ObjectObservation(found=False)
    cluster_opt = None
    if points_opt is not None and np.asarray(points_opt).shape[0] == pts.shape[0]:
        cluster_opt = np.asarray(points_opt, dtype=np.float64)[source_idx]
    return _ObjectObservation(
        found=True,
        side=_side_from_bearing(bearing_rad),
        bearing_deg=math.degrees(bearing_rad),
        points_bl=cluster,
        points_opt=cluster_opt,
    )


def _failure(details: tuple[str, ...], **kwargs: Any) -> SideEvaluation:
    return SideEvaluation(ok=False, stage=STAGE_NAME, details=details, **kwargs)


def _operator_fields(
    evaluation: SideEvaluation,
    calib: Calibration,
) -> tuple[SideOperatorField, ...]:
    if evaluation.propose_transverse_mirror is None:
        return ()
    return (
        SideOperatorField(
            name=FIELD_CAMERA_TRANSVERSE_MIRROR,
            before=1 if transverse_mirror(calib) else 0,
            after=1 if evaluation.propose_transverse_mirror else 0,
        ),
    )


def commit_side_pending(
    evaluation: SideEvaluation,
    calib: Calibration,
    dir: Optional[str] = None,
) -> tuple[SideOperatorField, ...]:
    """Write side pending convention when evaluation proposes a transverse sign."""
    if not evaluation.ok or evaluation.propose_transverse_mirror is None:
        return ()
    operator_fields = _operator_fields(evaluation, calib)
    write_draft(
        stage=STAGE_NAME,
        sensor="depth_cam",
        values={"depth_cam": {"transverse_mirror": evaluation.propose_transverse_mirror}},
        residual_before=0.0,
        residual_after=0.0,
        imu_residual_before=0.0,
        imu_residual_after=0.0,
        cause=evaluation.layer or "",
        dir=dir,
    )
    return operator_fields


side_operator_fields = _operator_fields


def _lround(value: float) -> int:
    """C++ std::lround: nearest integer, halfway away from zero."""
    if value >= 0.0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


def intrinsics_from_camera_info(msg: Any) -> Optional[CameraIntrinsics]:
    """CameraInfo → intrinsics (k = [fx, 0, cx, 0, fy, cy, 0, 0, 1])."""
    if msg is None:
        return None
    k = getattr(msg, "k", None)
    if k is None:
        k = getattr(msg, "K", None)
    if k is None:
        return None
    try:
        values = [float(v) for v in k]
    except (TypeError, ValueError):
        return None
    if len(values) < 6:
        return None
    cam = CameraIntrinsics(
        fx=values[0],
        fy=values[4],
        cx=values[2],
        cy=values[5],
        width=int(getattr(msg, "width", 0) or 0),
        height=int(getattr(msg, "height", 0) or 0),
    )
    return cam if cam.valid else None


def project_point(cam: CameraIntrinsics, point_opt: Sequence[float]) -> Optional[tuple[float, float]]:
    """Optical xyz → RGB pixel, same law as person_geometry.hpp project_point."""
    if cam is None or not cam.valid:
        return None
    if len(point_opt) < 3:
        return None
    x, y, z = float(point_opt[0]), float(point_opt[1]), float(point_opt[2])
    if not all(math.isfinite(v) for v in (x, y, z)) or z <= 0.0:
        return None
    u_px = cam.fx * (x / z) + cam.cx
    v_px = cam.fy * (y / z) + cam.cy
    if not math.isfinite(u_px) or not math.isfinite(v_px):
        return None
    return u_px, v_px


def side_frame_path(dir: Optional[str] = None) -> str:
    """PNG path next to the calibration file (I1 ``frame:``)."""
    return os.path.join(resolve_dir(dir), SIDE_FRAME_FILENAME)


def _png_chunk(tag: bytes, payload: bytes) -> bytes:
    crc = zlib.crc32(tag)
    crc = zlib.crc32(payload, crc) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + tag + payload + struct.pack(">I", crc)


def write_rgb_png(path: str, width: int, height: int, rgb: bytes | bytearray) -> None:
    """Write an 8-bit RGB PNG with stdlib only (zlib + struct)."""
    if width <= 0 or height <= 0:
        raise ValueError("invalid png size")
    expected = width * height * 3
    if len(rgb) < expected:
        raise ValueError("rgb buffer too small")
    raw = bytearray()
    stride = width * 3
    for row in range(height):
        raw.append(0)
        raw.extend(rgb[row * stride : (row + 1) * stride])
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    blob = (
        _PNG_SIGNATURE
        + _png_chunk(b"IHDR", ihdr)
        + _png_chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        + _png_chunk(b"IEND", b"")
    )
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(blob)


def _set_px(
    rgb: bytearray,
    width: int,
    height: int,
    x: int,
    y: int,
    color: tuple[int, int, int],
) -> None:
    if x < 0 or y < 0 or x >= width or y >= height:
        return
    index = (y * width + x) * 3
    rgb[index] = color[0]
    rgb[index + 1] = color[1]
    rgb[index + 2] = color[2]


def _draw_mark(rgb: bytearray, width: int, height: int, u_px: float, v_px: float) -> None:
    cx = max(0, min(width - 1, _lround(u_px)))
    cy = max(0, min(height - 1, _lround(v_px)))
    for y in range(height):
        for dx in range(-2, 3):
            _set_px(rgb, width, height, cx + dx, y, _MARK_OUTLINE)
    for x in range(max(0, cx - 16), min(width, cx + 17)):
        for dy in range(-2, 3):
            _set_px(rgb, width, height, x, cy + dy, _MARK_OUTLINE)
    for y in range(height):
        for dx in range(-1, 2):
            _set_px(rgb, width, height, cx + dx, y, _MARK_RGB)
    for x in range(max(0, cx - 15), min(width, cx + 16)):
        for dy in range(-1, 2):
            _set_px(rgb, width, height, x, cy + dy, _MARK_RGB)


def _decode_rgb_image(msg: Any) -> Optional[tuple[int, int, bytearray]]:
    if msg is None:
        return None
    try:
        width = int(msg.width)
        height = int(msg.height)
    except (AttributeError, TypeError, ValueError):
        return None
    if width <= 0 or height <= 0:
        return None
    encoding = str(getattr(msg, "encoding", "rgb8") or "rgb8").lower()
    try:
        payload = bytes(msg.data)
    except (AttributeError, TypeError):
        return None
    if encoding in ("rgb8", "bgr8"):
        bpp = 3
        swap_bgr = encoding == "bgr8"
    elif encoding in ("rgba8", "bgra8"):
        bpp = 4
        swap_bgr = encoding == "bgra8"
    elif encoding in ("mono8", "8uc1"):
        bpp = 1
        swap_bgr = False
    else:
        return None
    step = int(getattr(msg, "step", 0) or 0)
    if step < width * bpp:
        step = width * bpp
    if len(payload) < step * height:
        return None
    rows = np.frombuffer(payload, dtype=np.uint8, count=step * height).reshape(height, step)
    pixels = rows[:, : width * bpp].reshape(height, width, bpp)
    if bpp == 1:
        rgb = np.repeat(pixels, 3, axis=2)
    elif swap_bgr:
        rgb = pixels[:, :, [2, 1, 0]]
    else:
        rgb = pixels[:, :, :3]
    return width, height, bytearray(np.ascontiguousarray(rgb, dtype=np.uint8).reshape(-1))


def _cloud_optical_points(clouds: Sequence[Any]) -> np.ndarray:
    """Valid optical xyz of the clouds.

    No layout is assumed: /aurora/points2 is a compacted list of valid points
    with the holes dropped (SD022 D6.1), so a slot index carries no pixel.
    """
    return finite_nonzero_points(clouds_to_xyz(clouds))


def _write_cluster_frame(
    rgb_decoded: Optional[tuple[int, int, bytearray]],
    cloud_obs: _ObjectObservation,
    cam: Optional[CameraIntrinsics],
    dir: Optional[str] = None,
) -> Optional[_FrameProjection]:
    """Put the cloud cluster on the frame by projection (D2.3, same law as perception)."""
    if rgb_decoded is None or cam is None or not cam.valid:
        return None
    points_opt = cloud_obs.points_opt
    if points_opt is None or np.asarray(points_opt).shape[0] == 0:
        return None
    rgb_w, rgb_h, pixels = rgb_decoded
    centre = np.median(np.asarray(points_opt, dtype=np.float64), axis=0)
    mapped = project_point(cam, centre)
    if mapped is None:
        return None
    u_px, v_px = mapped
    marked = bytearray(pixels)
    _draw_mark(marked, rgb_w, rgb_h, u_px, v_px)
    path = side_frame_path(dir)
    try:
        write_rgb_png(path, rgb_w, rgb_h, marked)
    except OSError:
        return None
    frame_side = SIDE_LEFT if u_px < (float(rgb_w) * 0.5) else SIDE_RIGHT
    return _FrameProjection(path=path, u_px=u_px, v_px=v_px, frame_side=frame_side)


def evaluate_side_samples(
    samples: SideSamples,
    calib: Calibration,
    operator_side: str,
    dir: Optional[str] = None,
) -> SideEvaluation:
    """Find the side object in scan and cloud, then name the D1.2 layer."""
    if not samples.ok:
        return _failure(samples.details)

    rgb_decoded = _decode_rgb_image(samples.images[-1]) if samples.images else None
    cam = intrinsics_from_camera_info(samples.camera_infos[-1]) if samples.camera_infos else None
    # Cloud: last frame only. Concatenating full Aurora frames glues the floor
    # into one wide cluster (stand: 4 frames → object-not-found in cloud).
    # Scan: all revolutions. One LD19 sweep is too sparse when lidar is degraded
    # (stand after cloud fix: cloud found, scan still object-not-found).
    clouds = samples.clouds[-1:] if samples.clouds else samples.clouds
    scans = samples.scans

    try:
        cloud_opt = _cloud_optical_points(clouds)
    except CloudLayoutError as exc:
        return _failure((exc.detail,))

    scan_xy = scans_to_xy(scans)
    lidar = calib.sensors["lidar"]
    depth_cam = calib.sensors["depth_cam"]
    scan_bl = lidar_base_link_from_frame(scan_xy, lidar.xyz, lidar.rpy)
    cloud_bl = camera_base_link_from_optical(cloud_opt, depth_cam.xyz, depth_cam.rpy)

    scan_obs = _observe_object(scan_bl, min_points=MIN_SCAN_CLUSTER_POINTS)
    cloud_obs = _observe_object(
        cloud_bl,
        min_points=MIN_CLOUD_CLUSTER_POINTS,
        z_min=CLOUD_Z_MIN_M,
        z_max=CLOUD_Z_MAX_M,
        points_opt=cloud_opt,
        polar_bin_deg=CLOUD_POLAR_BIN_DEG,
    )
    missing: list[str] = []
    if not scan_obs.found:
        missing.append(DETAIL_OBJECT_NOT_FOUND_SCAN)
    if not cloud_obs.found:
        missing.append(DETAIL_OBJECT_NOT_FOUND_CLOUD)
    if missing:
        return _failure(tuple(missing))

    if scan_obs.side != operator_side:
        return _failure(
            (DETAIL_LIDAR_SIDE_DISAGREES,),
            operator_side=operator_side,
            scan_side=scan_obs.side,
            cloud_side=cloud_obs.side,
            scan_bearing_deg=scan_obs.bearing_deg,
            cloud_bearing_deg=cloud_obs.bearing_deg,
            layer=LAYER_LIDAR_SUSPECT,
            cloud_cluster_bl=cloud_obs.points_bl,
            scan_cluster_bl=scan_obs.points_bl,
        )

    if cloud_obs.side != scan_obs.side:
        layer = LAYER_CLOUD_GEOMETRY
        propose = True
    else:
        layer = LAYER_NONE
        propose = False

    projected = _write_cluster_frame(rgb_decoded, cloud_obs, cam, dir=dir)
    if projected is None:
        return _failure(
            (DETAIL_CANNOT_PROJECT,),
            operator_side=operator_side,
            scan_side=scan_obs.side,
            cloud_side=cloud_obs.side,
            scan_bearing_deg=scan_obs.bearing_deg,
            cloud_bearing_deg=cloud_obs.bearing_deg,
            cloud_cluster_bl=cloud_obs.points_bl,
            scan_cluster_bl=scan_obs.points_bl,
        )

    if layer == LAYER_NONE and projected.frame_side != operator_side:
        layer = LAYER_PIXEL_MAPPING
        propose = False

    evaluation = SideEvaluation(
        ok=True,
        stage=STAGE_NAME,
        details=(),
        operator_side=operator_side,
        scan_side=scan_obs.side,
        cloud_side=cloud_obs.side,
        scan_bearing_deg=scan_obs.bearing_deg,
        cloud_bearing_deg=cloud_obs.bearing_deg,
        layer=layer,
        propose_transverse_mirror=propose,
        cloud_cluster_bl=cloud_obs.points_bl,
        scan_cluster_bl=scan_obs.points_bl,
        frame_path=projected.path,
        frame_u_px=projected.u_px,
        frame_side=projected.frame_side,
    )
    commit_side_pending(evaluation, calib, dir=dir)
    return evaluation


run_side = evaluate_side_samples


def _create_ros_collector() -> SideCollector:
    try:
        import rclpy  # noqa: F401
        from sensor_msgs.msg import CameraInfo  # noqa: F401
        from sensor_msgs.msg import Image  # noqa: F401
        from sensor_msgs.msg import LaserScan  # noqa: F401
        from sensor_msgs.msg import PointCloud2  # noqa: F401
    except ImportError as exc:
        raise RosUnavailableError(
            "rclpy or sensor_msgs is not available",
        ) from exc
    return _RosSideCollector()


class _RosSideCollector:
    def __init__(self) -> None:
        import logging

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2

        logging.getLogger("rclpy").setLevel(logging.FATAL)
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node = Node("mentorpi_calib_side_collect")
        self._node.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
        self._clouds: list[Any] = []
        self._scans: list[Any] = []
        self._images: list[Any] = []
        self._infos: list[Any] = []
        self._node.create_subscription(
            PointCloud2,
            TOPIC_POINTS2,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self._node.create_subscription(
            LaserScan,
            TOPIC_SCAN,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._node.create_subscription(
            Image,
            TOPIC_RGB,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._node.create_subscription(
            CameraInfo,
            TOPIC_CAMERA_INFO,
            self._on_camera_info,
            qos_profile_sensor_data,
        )

    def _on_cloud(self, msg: Any) -> None:
        self._clouds.append(msg)

    def _on_scan(self, msg: Any) -> None:
        self._scans.append(msg)

    def _on_image(self, msg: Any) -> None:
        self._images.append(msg)

    def _on_camera_info(self, msg: Any) -> None:
        self._infos.append(msg)

    def topics_available(self) -> tuple[bool, bool, bool, bool]:
        names = {name for name, _ in self._node.get_topic_names_and_types()}
        return (
            TOPIC_POINTS2 in names,
            TOPIC_SCAN in names,
            TOPIC_RGB in names,
            TOPIC_CAMERA_INFO in names,
        )

    def run(
        self,
        timeout: float,
        on_progress: Callable[[int, Optional[int], Optional[int], int], None],
    ) -> tuple[list[Any], list[Any], list[Any], list[Any]]:
        deadline = time.monotonic() + timeout
        last_progress = 0.0
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_progress >= _PROGRESS_INTERVAL_SEC:
                on_progress(
                    len(self._clouds),
                    count_cloud_points(self._clouds),
                    count_scan_rays(self._scans),
                    len(self._images),
                )
                last_progress = now
        on_progress(
            len(self._clouds),
            count_cloud_points(self._clouds),
            count_scan_rays(self._scans),
            len(self._images),
        )
        return self._clouds, self._scans, self._images, self._infos

    def close(self) -> None:
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
