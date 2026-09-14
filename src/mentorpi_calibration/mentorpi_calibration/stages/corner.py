"""Corner stage: orthogonal walls + scan lines + IMU (SD012 D3.2)."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, TextIO

import numpy as np

from mentorpi_calibration.calibration_file import (
    Calibration,
    SensorPose,
    depth_cam_optical_rpy,
    operator_display,
    write_draft,
)
from mentorpi_calibration.pointcloud2 import CloudLayoutError, clouds_to_xyz, count_cloud_points
from mentorpi_calibration.stages.imu_mount import DETAIL_IMU_NOT_STILL, imu_gravity_residual_deg, imu_mount_from_gravity
from mentorpi_calibration.stages.planes import (
    DETAIL_CAMERA_HEIGHT_NOT_OBSERVABLE,
    DETAIL_ONE_WALL,
    DETAIL_TOO_FEW_WALL_POINTS,
    camera_tilt_from_vertical,
    camera_z_from_height,
    estimate_floor_strip,
    extract_orthogonal_walls,
    finite_nonzero_points,
    wall_mask_union,
)
from mentorpi_calibration.stages.scan_lines import (
    DETAIL_TOO_FEW_SCAN_POINTS,
    count_scan_rays,
    match_camera_yaw_xy,
    scans_to_xy,
)
from mentorpi_calibration.stages.tilt import rpy_from_up_keep_yaw
from mentorpi_calibration.stages.transforms import np_rotation, world_up_in_optical

DEFAULT_TIMEOUT = 5.0

STAGE_NAME = "corner"
HINT = "stand in front of a right-angle corner"

TOPIC_POINTS2 = "/aurora/points2"
TOPIC_SCAN = "/scan"
TOPIC_IMU = "/imu"

DETAIL_NO_CAMERA_DEPTH = "no camera depth"
DETAIL_NO_SCAN = "no /scan"
DETAIL_NO_IMU = "no /imu"

_PROGRESS_INTERVAL_SEC = 0.1
_POSE_COMPARE_EPS = 1e-6

_CORNER_SENSORS = ("depth_cam", "lidar", "imu")
_CORNER_SENSOR_LABEL = "depth_cam,lidar,imu"
_OPERATOR_PREFIX = {
    "depth_cam": "camera",
    "lidar": "lidar",
    "imu": "imu",
}


class RosUnavailableError(RuntimeError):
    """ROS Python bindings are not importable in this environment."""


@dataclass(frozen=True)
class CornerSamples:
    """Raw samples gathered for the corner calibration stage."""

    clouds: tuple[Any, ...]
    scans: tuple[Any, ...]
    imu_accels: tuple[tuple[float, float, float], ...]
    frames: int
    cloud_points: Optional[int]
    scan_rays: Optional[int]
    imu_samples: int
    ok: bool
    details: tuple[str, ...]


@dataclass(frozen=True)
class CornerEvaluation:
    """Result of corner-stage parse → walls → SE(2) → IMU (D3.2)."""

    ok: bool
    stage: str
    details: tuple[str, ...]
    depth_cam_xyz: Optional[tuple[float, float, float]] = None
    depth_cam_rpy: Optional[tuple[float, float, float]] = None
    lidar_xyz: Optional[tuple[float, float, float]] = None
    lidar_rpy: Optional[tuple[float, float, float]] = None
    imu_xyz: Optional[tuple[float, float, float]] = None
    imu_rpy: Optional[tuple[float, float, float]] = None
    residual_before: Optional[float] = None
    residual_after: Optional[float] = None
    imu_residual_before: Optional[float] = None
    imu_residual_after: Optional[float] = None
    cause: str = ""
    include_camera_z: bool = True


@dataclass(frozen=True)
class CornerOperatorField:
    """Operator-facing before/after value for CLI ``field:`` lines."""

    name: str
    before: float
    after: float
    source: str = "computed"


class CornerCollector(Protocol):
    """Injectable backend for corner sample collection."""

    def topics_available(self) -> tuple[bool, bool, bool]:
        """Return whether `/aurora/points2`, `/scan`, and `/imu` are in the graph."""

    def run(
        self,
        timeout: float,
        on_progress: Callable[[int, Optional[int], Optional[int], int], None],
    ) -> tuple[list[Any], list[Any], list[tuple[float, float, float]]]:
        """Collect clouds, scans, and linear_acceleration samples."""

    def close(self) -> None:
        """Release collector resources."""


def print_collect_progress(
    stdout: TextIO,
    frames: int,
    cloud_points: Optional[int],
    scan_rays: Optional[int],
    imu_samples: int,
    *,
    include_header: bool = False,
) -> None:
    """Emit the corner collection stdout contract."""
    if include_header:
        print("stage: {}".format(STAGE_NAME), file=stdout)
        print("hint: {}".format(HINT), file=stdout)
    print("frames: {}".format(frames), file=stdout)
    print(
        "cloud_points: {}".format(0 if cloud_points is None else cloud_points),
        file=stdout,
    )
    print("scan_rays: {}".format(0 if scan_rays is None else scan_rays), file=stdout)
    print("imu_samples: {}".format(imu_samples), file=stdout)
    stdout.flush()


def _failure_details(
    has_points2: bool,
    has_scan: bool,
    has_imu: bool,
    clouds: list[Any],
    scans: list[Any],
    imu_accels: list[Any],
) -> list[str]:
    details: list[str] = []
    if not has_points2 or not clouds:
        details.append(DETAIL_NO_CAMERA_DEPTH)
    if not has_scan or not scans:
        details.append(DETAIL_NO_SCAN)
    if not has_imu or not imu_accels:
        details.append(DETAIL_NO_IMU)
    return details


def collect_corner_samples(
    timeout: float = DEFAULT_TIMEOUT,
    *,
    collector: Optional[CornerCollector] = None,
    stdout: Optional[TextIO] = None,
) -> CornerSamples:
    """Collect corner-stage sensor samples."""
    out = stdout if stdout is not None else sys.stdout
    own_collector = collector is None
    if own_collector:
        collector = _create_ros_collector()

    assert collector is not None
    try:
        has_points2, has_scan, has_imu = collector.topics_available()
        graph_clouds = [object()] if has_points2 else []
        graph_scans = [object()] if has_scan else []
        graph_imu = [object()] if has_imu else []
        graph_details = _failure_details(
            has_points2,
            has_scan,
            has_imu,
            graph_clouds,
            graph_scans,
            graph_imu,
        )
        if graph_details:
            print_collect_progress(out, 0, None, None, 0, include_header=True)
            return CornerSamples(
                clouds=(),
                scans=(),
                imu_accels=(),
                frames=0,
                cloud_points=None,
                scan_rays=None,
                imu_samples=0,
                ok=False,
                details=tuple(graph_details),
            )

        print_collect_progress(out, 0, None, None, 0, include_header=True)

        def on_progress(
            frames: int,
            cloud_points: Optional[int],
            scan_rays: Optional[int],
            imu_samples: int,
        ) -> None:
            print_collect_progress(out, frames, cloud_points, scan_rays, imu_samples)

        clouds, scans, imu_accels = collector.run(timeout, on_progress)
        details = _failure_details(True, True, True, clouds, scans, imu_accels)
        cloud_points = count_cloud_points(clouds) if not details else None
        scan_rays = count_scan_rays(scans) if not details else None
        return CornerSamples(
            clouds=tuple(clouds),
            scans=tuple(scans),
            imu_accels=tuple(imu_accels),
            frames=len(clouds),
            cloud_points=cloud_points,
            scan_rays=scan_rays,
            imu_samples=len(imu_accels),
            ok=not details,
            details=tuple(details),
        )
    finally:
        if own_collector:
            collector.close()


def collect_ros(
    timeout: float = DEFAULT_TIMEOUT,
    *,
    stdout: Optional[TextIO] = None,
) -> CornerSamples:
    """Collect corner samples through an ephemeral ROS node."""
    return collect_corner_samples(timeout, stdout=stdout)


def _failure(details: tuple[str, ...]) -> CornerEvaluation:
    return CornerEvaluation(ok=False, stage=STAGE_NAME, details=details)


def _mean_accel(
    imu_accels: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float]:
    mean = np.mean(np.asarray(imu_accels, dtype=np.float64), axis=0)
    return (float(mean[0]), float(mean[1]), float(mean[2]))


def _axis_changed(before: float, after: float) -> bool:
    return abs(before - after) > _POSE_COMPARE_EPS


def _proposed_pose(
    evaluation: CornerEvaluation,
    sensor: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if sensor == "depth_cam":
        assert evaluation.depth_cam_xyz is not None
        assert evaluation.depth_cam_rpy is not None
        return evaluation.depth_cam_xyz, evaluation.depth_cam_rpy
    if sensor == "lidar":
        assert evaluation.lidar_xyz is not None
        assert evaluation.lidar_rpy is not None
        return evaluation.lidar_xyz, evaluation.lidar_rpy
    assert evaluation.imu_xyz is not None
    assert evaluation.imu_rpy is not None
    return evaluation.imu_xyz, evaluation.imu_rpy


def _calib_with_sensor_pose(
    calib: Calibration,
    sensor: str,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> Calibration:
    sensors = dict(calib.sensors)
    current = sensors[sensor]
    sensors[sensor] = SensorPose(
        parent=current.parent,
        xyz=xyz,
        rpy=rpy,
        source=current.source,
    )
    return Calibration(sensors=sensors, unused=calib.unused, reason=calib.reason)


def _pending_values(
    evaluation: CornerEvaluation,
    calib: Calibration,
) -> dict[str, dict[str, float]]:
    values: dict[str, dict[str, float]] = {}
    for sensor in _CORNER_SENSORS:
        proposed_xyz, proposed_rpy = _proposed_pose(evaluation, sensor)
        current = calib.sensors[sensor]
        partial: dict[str, float] = {}
        for index, field in enumerate(("x", "y", "z")):
            if sensor == "depth_cam" and field == "z" and not evaluation.include_camera_z:
                continue
            if _axis_changed(current.xyz[index], proposed_xyz[index]):
                partial[field] = proposed_xyz[index]
        for index, field in enumerate(("roll", "pitch", "yaw")):
            if _axis_changed(current.rpy[index], proposed_rpy[index]):
                partial[field] = proposed_rpy[index]
        if partial:
            values[sensor] = partial
    return values


def _operator_fields(
    evaluation: CornerEvaluation,
    calib: Calibration,
) -> tuple[CornerOperatorField, ...]:
    fields: list[CornerOperatorField] = []
    for sensor in _CORNER_SENSORS:
        proposed_xyz, proposed_rpy = _proposed_pose(evaluation, sensor)
        current = calib.sensors[sensor]
        proposed_calib = _calib_with_sensor_pose(
            calib,
            sensor,
            proposed_xyz,
            proposed_rpy,
        )
        before_xyz, before_rpy, _, _ = operator_display(calib, sensor)
        after_xyz, after_rpy, _, _ = operator_display(proposed_calib, sensor)
        prefix = _OPERATOR_PREFIX[sensor]
        for index, field in enumerate(("x", "y", "z")):
            if sensor == "depth_cam" and field == "z" and not evaluation.include_camera_z:
                continue
            if _axis_changed(current.xyz[index], proposed_xyz[index]):
                fields.append(
                    CornerOperatorField(
                        name="{}_{}".format(prefix, field),
                        before=before_xyz[index],
                        after=after_xyz[index],
                    )
                )
        for index, field in enumerate(("roll", "pitch", "yaw")):
            if _axis_changed(current.rpy[index], proposed_rpy[index]):
                fields.append(
                    CornerOperatorField(
                        name="{}_{}".format(prefix, field),
                        before=before_rpy[index],
                        after=after_rpy[index],
                    )
                )
    return tuple(fields)


def commit_corner_pending(
    evaluation: CornerEvaluation,
    calib: Calibration,
    dir: Optional[str] = None,
) -> tuple[CornerOperatorField, ...]:
    """Write corner pending proposal to the draft file when evaluation succeeded."""
    if not evaluation.ok:
        return ()
    assert evaluation.residual_before is not None
    assert evaluation.residual_after is not None
    assert evaluation.imu_residual_before is not None
    assert evaluation.imu_residual_after is not None
    values = _pending_values(evaluation, calib)
    operator_fields = _operator_fields(evaluation, calib)
    write_draft(
        stage=STAGE_NAME,
        sensor=_CORNER_SENSOR_LABEL,
        values=values,
        residual_before=evaluation.residual_before,
        residual_after=evaluation.residual_after,
        imu_residual_before=evaluation.imu_residual_before,
        imu_residual_after=evaluation.imu_residual_after,
        cause=evaluation.cause,
        dir=dir,
    )
    return operator_fields


corner_operator_fields = _operator_fields


def evaluate_corner_samples(
    samples: CornerSamples,
    calib: Calibration,
) -> CornerEvaluation:
    """Run corner gates and pose proposal; write pending on success."""
    if not samples.ok:
        return _failure(samples.details)

    try:
        points = finite_nonzero_points(clouds_to_xyz(samples.clouds))
    except CloudLayoutError as exc:
        return _failure((exc.detail,))

    if points.shape[0] < 3:
        return _failure((DETAIL_TOO_FEW_WALL_POINTS,))

    depth_cam = calib.sensors["depth_cam"]
    imu = calib.sensors["imu"]
    lidar = calib.sensors["lidar"]
    current_depth_cam_pose = (depth_cam.xyz, depth_cam.rpy)
    current_imu_pose = (imu.xyz, imu.rpy)
    lidar_pose = (lidar.xyz, lidar.rpy)

    up_prior = world_up_in_optical(depth_cam.rpy)
    walls = extract_orthogonal_walls(points, up_prior=up_prior)
    if not walls.ok or walls.up is None or walls.first is None or walls.second is None:
        return _failure((walls.detail or DETAIL_ONE_WALL,))

    tilt_xyz, tilt_rpy = camera_tilt_from_vertical(walls.up, current_depth_cam_pose)
    wall_mask = wall_mask_union(walls, points.shape[0])
    strip = estimate_floor_strip(points, walls.up, wall_mask=wall_mask)
    include_z = strip.observable and strip.height_h is not None
    if include_z:
        tilt_xyz = camera_z_from_height(strip.height_h, tilt_xyz)

    scan_xy = scans_to_xy(samples.scans)
    if scan_xy.shape[0] == 0:
        return _failure((DETAIL_TOO_FEW_SCAN_POINTS,))

    match = match_camera_yaw_xy(
        (walls.first, walls.second),
        scan_xy,
        current_depth_cam_pose,
        lidar_pose,
        tilt_xyz,
        tilt_rpy,
    )
    if not match.ok or match.xyz is None or match.rpy is None or match.lidar_xyz is None or match.lidar_rpy is None:
        return _failure((match.detail or DETAIL_ONE_WALL,))

    r_opt = np_rotation(depth_cam_optical_rpy())
    n_mount = r_opt @ walls.up
    roll, pitch, yaw = rpy_from_up_keep_yaw(n_mount, match.rpy)
    proposed_depth_cam_pose = (match.xyz, (roll, pitch, yaw))
    proposed_lidar_pose = (match.lidar_xyz, match.lidar_rpy)

    accel_mean = _mean_accel(samples.imu_accels)
    imu_result = imu_mount_from_gravity(accel_mean, current_imu_pose)
    if not imu_result.ok:
        return _failure((imu_result.detail or DETAIL_IMU_NOT_STILL,))
    assert imu_result.xyz is not None
    assert imu_result.rpy is not None
    proposed_imu_pose = (imu_result.xyz, imu_result.rpy)

    details: tuple[str, ...] = ()
    if not include_z:
        details = (DETAIL_CAMERA_HEIGHT_NOT_OBSERVABLE,)

    evaluation = CornerEvaluation(
        ok=True,
        stage=STAGE_NAME,
        details=details,
        depth_cam_xyz=proposed_depth_cam_pose[0],
        depth_cam_rpy=proposed_depth_cam_pose[1],
        lidar_xyz=proposed_lidar_pose[0],
        lidar_rpy=proposed_lidar_pose[1],
        imu_xyz=proposed_imu_pose[0],
        imu_rpy=proposed_imu_pose[1],
        residual_before=match.residual_before,
        residual_after=match.residual_after,
        imu_residual_before=imu_gravity_residual_deg(accel_mean, current_imu_pose[1]),
        imu_residual_after=imu_gravity_residual_deg(accel_mean, proposed_imu_pose[1]),
        cause=match.cause,
        include_camera_z=include_z,
    )
    commit_corner_pending(evaluation, calib)
    return evaluation


run_corner = evaluate_corner_samples


def _create_ros_collector() -> CornerCollector:
    try:
        import rclpy  # noqa: F401
        from sensor_msgs.msg import Imu  # noqa: F401
        from sensor_msgs.msg import LaserScan  # noqa: F401
        from sensor_msgs.msg import PointCloud2  # noqa: F401
    except ImportError as exc:
        raise RosUnavailableError(
            "rclpy or sensor_msgs is not available",
        ) from exc
    return _RosCornerCollector()


class _RosCornerCollector:
    def __init__(self) -> None:
        import logging

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, LaserScan, PointCloud2

        logging.getLogger("rclpy").setLevel(logging.FATAL)
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node = Node("mentorpi_calib_corner_collect")
        self._node.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
        self._clouds: list[Any] = []
        self._scans: list[Any] = []
        self._imu_accels: list[tuple[float, float, float]] = []
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
            Imu,
            TOPIC_IMU,
            self._on_imu,
            qos_profile_sensor_data,
        )

    def _on_cloud(self, msg: Any) -> None:
        self._clouds.append(msg)

    def _on_scan(self, msg: Any) -> None:
        self._scans.append(msg)

    def _on_imu(self, msg: Any) -> None:
        acc = msg.linear_acceleration
        self._imu_accels.append((acc.x, acc.y, acc.z))

    def topics_available(self) -> tuple[bool, bool, bool]:
        names = {name for name, _ in self._node.get_topic_names_and_types()}
        return TOPIC_POINTS2 in names, TOPIC_SCAN in names, TOPIC_IMU in names

    def run(
        self,
        timeout: float,
        on_progress: Callable[[int, Optional[int], Optional[int], int], None],
    ) -> tuple[list[Any], list[Any], list[tuple[float, float, float]]]:
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
                    len(self._imu_accels),
                )
                last_progress = now
        on_progress(
            len(self._clouds),
            count_cloud_points(self._clouds),
            count_scan_rays(self._scans),
            len(self._imu_accels),
        )
        return self._clouds, self._scans, self._imu_accels

    def close(self) -> None:
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
