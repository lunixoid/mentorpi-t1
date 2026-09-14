"""Drive stage: lidar yaw/x/y from scan-match motion (SD012 D3.4)."""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, Sequence, TextIO

import numpy as np

from mentorpi_calibration.calibration_file import (
    Calibration,
    SensorPose,
    operator_display,
    rotation_matrix_from_rpy,
    write_draft,
)
from mentorpi_calibration.stages.scan_match import (
    ScanPose,
    ScanTracker,
    ScanTrajectory,
    build_scan_trajectory,
    scan_stamp,
    scan_xy,
)
from mentorpi_calibration.stages.tilt import unwrap_angle_near, wrap_pi
from mentorpi_calibration.stages.transforms import LIDAR_OPTICAL_RPY

DEFAULT_TIMEOUT = 60.0

STAGE_NAME = "drive"
HINT_PAD = "you drive the pad; this command does not move the robot"
HINTS_FORWARD = (
    "drive forward",
    HINT_PAD,
)
HINTS_TURN = (
    "now turn in place",
    HINT_PAD,
)
HINTS = HINTS_FORWARD

TOPIC_SCAN = "/scan"
TOPIC_IMU = "/imu"

DETAIL_NO_SCAN = "no /scan"
DETAIL_NO_IMU = "no /imu"
DETAIL_TRAVEL_TOO_SHORT = "travel too short"
DETAIL_NO_TRANSLATION = "no translation segment"
DETAIL_NO_ROTATION = "no rotation segment"
IMU_SIGN_OK = "ok"
IMU_SIGN_MISMATCH = "mismatch"
IMU_SIGN_TAG = "drive"

# Implementation tuning constants, not an accuracy SLA.
MIN_TRAVEL_M = 0.30
MIN_TURN_DEG = 35.0
TRANS_STEP_MIN_M = 0.004
TRANS_YAW_MAX_RAD = math.radians(1.8)
ROT_YAW_MIN_RAD = math.radians(1.2)
MIN_TRANSLATION_LENGTH_M = 0.15
MIN_ROTATION_SPAN_RAD = math.radians(25.0)
IMU_GYRO_MIN_RAD_S = 0.08
_PROGRESS_INTERVAL_SEC = 0.1
_POSE_COMPARE_EPS = 1e-6
_OPTICAL_YAW = LIDAR_OPTICAL_RPY[2]


class RosUnavailableError(RuntimeError):
    """ROS Python bindings are not importable in this environment."""


@dataclass(frozen=True)
class DriveScan:
    """Laser returns already parsed to the scan plane."""

    xy: np.ndarray
    stamp: float


@dataclass(frozen=True)
class DriveImuSample:
    """Angular velocity in imu_link plus timestamp."""

    stamp: float
    gyro: tuple[float, float, float]


@dataclass(frozen=True)
class DriveSamples:
    """Raw samples gathered for the drive calibration stage."""

    scans: tuple[Any, ...]
    imu: tuple[DriveImuSample, ...]
    travel_m: float
    turn_deg: float
    ok: bool
    details: tuple[str, ...]


@dataclass(frozen=True)
class DriveEvaluation:
    """Result of scan-match → segments → lidar x/y/yaw + IMU sign check."""

    ok: bool
    stage: str
    details: tuple[str, ...]
    lidar_xyz: Optional[tuple[float, float, float]] = None
    lidar_rpy: Optional[tuple[float, float, float]] = None
    residual_before: Optional[float] = None
    residual_after: Optional[float] = None
    imu_yaw_sign: str = IMU_SIGN_MISMATCH


@dataclass(frozen=True)
class DriveOperatorField:
    """Operator-facing before/after value for CLI ``field:`` lines."""

    name: str
    before: object
    after: object
    source: str = "computed"


@dataclass(frozen=True)
class MotionStep:
    start: ScanPose
    end: ScanPose
    dx: float
    dy: float
    dyaw: float
    dt: float


@dataclass(frozen=True)
class MotionSegment:
    kind: str
    steps: tuple[MotionStep, ...]

    @property
    def length_m(self) -> float:
        return float(sum(math.hypot(step.dx, step.dy) for step in self.steps))

    @property
    def yaw_span_rad(self) -> float:
        return float(sum(abs(step.dyaw) for step in self.steps))

    @property
    def duration_s(self) -> float:
        return float(sum(max(step.dt, 1e-3) for step in self.steps))


@dataclass(frozen=True)
class MotionSegments:
    translation: tuple[MotionSegment, ...]
    rotation: tuple[MotionSegment, ...]


class DriveCollector(Protocol):
    """Injectable backend for drive sample collection."""

    def topics_available(self) -> tuple[bool, bool]:
        """Return whether `/scan` and `/imu` are in the graph."""

    def run(
        self,
        timeout: float,
        on_progress: Callable[[float, float, int, int], None],
        should_stop: Callable[[float, float], bool],
    ) -> tuple[list[Any], list[DriveImuSample]]:
        """Collect scans and IMU gyro samples. Progress is travel m / turn deg."""

    def close(self) -> None:
        """Release collector resources."""


def print_collect_progress(
    stdout: TextIO,
    travel_m: float,
    turn_deg: float,
    *,
    include_header: bool = False,
    hints: tuple[str, ...] = HINTS_FORWARD,
    clear_hints: bool = False,
) -> None:
    """Emit the drive collection stdout contract."""
    if include_header:
        print("stage: {}".format(STAGE_NAME), file=stdout)
        if clear_hints:
            print("hint:", file=stdout)
        for hint in hints:
            print("hint: {}".format(hint), file=stdout)
    print("travel: {:.3f}".format(travel_m), file=stdout)
    print("turn: {:.1f}".format(turn_deg), file=stdout)
    stdout.flush()


def _as_imu_sample(item: Any) -> DriveImuSample:
    if isinstance(item, DriveImuSample):
        return item
    if isinstance(item, tuple) and len(item) == 2:
        stamp, gyro = item
        gx, gy, gz = gyro
        return DriveImuSample(float(stamp), (float(gx), float(gy), float(gz)))
    if isinstance(item, tuple) and len(item) == 4:
        stamp, gx, gy, gz = item
        return DriveImuSample(float(stamp), (float(gx), float(gy), float(gz)))
    stamp = scan_stamp(item, fallback=0.0)
    gyro_msg = getattr(item, "angular_velocity", None)
    if gyro_msg is not None:
        return DriveImuSample(
            stamp,
            (float(gyro_msg.x), float(gyro_msg.y), float(gyro_msg.z)),
        )
    return DriveImuSample(0.0, (0.0, 0.0, 0.0))


def _failure_details(has_scan: bool, has_imu: bool, scans: list[Any], imu: list[Any]) -> list[str]:
    details: list[str] = []
    if not has_scan or not scans:
        details.append(DETAIL_NO_SCAN)
    if not has_imu or not imu:
        details.append(DETAIL_NO_IMU)
    return details


def collect_drive_samples(
    timeout: float = DEFAULT_TIMEOUT,
    *,
    collector: Optional[DriveCollector] = None,
    stdout: Optional[TextIO] = None,
) -> DriveSamples:
    """Collect drive-stage `/scan` and `/imu` samples."""
    out = stdout if stdout is not None else sys.stdout
    own_collector = collector is None
    if own_collector:
        collector = _create_ros_collector()

    assert collector is not None
    try:
        has_scan, has_imu = collector.topics_available()
        graph_details = _failure_details(
            has_scan,
            has_imu,
            [object()] if has_scan else [],
            [object()] if has_imu else [],
        )
        print_collect_progress(out, 0.0, 0.0, include_header=True, hints=HINTS_FORWARD)
        if graph_details:
            return DriveSamples(
                scans=(),
                imu=(),
                travel_m=0.0,
                turn_deg=0.0,
                ok=False,
                details=tuple(graph_details),
            )

        last = {"travel": 0.0, "turn": 0.0}
        phase = {"n": 1, "turn0": 0.0}

        def _display_turn(turn_deg: float) -> float:
            if phase["n"] == 1:
                return 0.0
            return max(0.0, turn_deg - phase["turn0"])

        def on_progress(travel_m: float, turn_deg: float, _scans: int, _imu: int) -> None:
            last["travel"] = travel_m
            last["turn"] = turn_deg
            if phase["n"] == 1 and travel_m >= MIN_TRAVEL_M:
                phase["n"] = 2
                phase["turn0"] = turn_deg
                print_collect_progress(
                    out,
                    travel_m,
                    0.0,
                    include_header=True,
                    hints=HINTS_TURN,
                    clear_hints=True,
                )
                return
            print_collect_progress(out, travel_m, _display_turn(turn_deg))

        def should_stop(travel_m: float, turn_deg: float) -> bool:
            del travel_m
            if phase["n"] < 2:
                return False
            return (turn_deg - phase["turn0"]) >= MIN_TURN_DEG

        scans, imu_raw = collector.run(timeout, on_progress, should_stop)
        imu = tuple(_as_imu_sample(item) for item in imu_raw)
        details = _failure_details(True, True, scans, list(imu))
        if not details:
            if phase["n"] < 2:
                details.append(DETAIL_TRAVEL_TOO_SHORT)
            elif (last["turn"] - phase["turn0"]) < MIN_TURN_DEG:
                details.append(DETAIL_NO_ROTATION)
        return DriveSamples(
            scans=tuple(scans),
            imu=imu,
            travel_m=last["travel"],
            turn_deg=last["turn"],
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
) -> DriveSamples:
    """Collect drive samples through an ephemeral ROS node."""
    return collect_drive_samples(timeout, stdout=stdout)


def _motion_steps(poses: Sequence[ScanPose]) -> list[MotionStep]:
    steps: list[MotionStep] = []
    for prev, curr in zip(poses[:-1], poses[1:]):
        dx = curr.x - prev.x
        dy = curr.y - prev.y
        dyaw = wrap_pi(curr.yaw - prev.yaw)
        dt = max(curr.stamp - prev.stamp, 1e-3)
        steps.append(MotionStep(start=prev, end=curr, dx=dx, dy=dy, dyaw=dyaw, dt=dt))
    return steps


def _label_step(step: MotionStep) -> str:
    if abs(step.dyaw) < TRANS_YAW_MAX_RAD and math.hypot(step.dx, step.dy) >= TRANS_STEP_MIN_M:
        return "translation"
    if abs(step.dyaw) >= ROT_YAW_MIN_RAD:
        return "rotation"
    return "other"


def segment_motion(poses: Sequence[ScanPose]) -> MotionSegments:
    """Split a lidar-frame trajectory into pure translation and rotation."""
    steps = _motion_steps(poses)
    translation: list[MotionSegment] = []
    rotation: list[MotionSegment] = []
    run: list[MotionStep] = []
    run_kind = ""
    for step in steps:
        kind = _label_step(step)
        if kind == run_kind:
            run.append(step)
            continue
        if run_kind == "translation":
            segment = MotionSegment(kind=run_kind, steps=tuple(run))
            if segment.length_m >= MIN_TRANSLATION_LENGTH_M:
                translation.append(segment)
        elif run_kind == "rotation":
            segment = MotionSegment(kind=run_kind, steps=tuple(run))
            if segment.yaw_span_rad >= MIN_ROTATION_SPAN_RAD:
                rotation.append(segment)
        run = [step]
        run_kind = kind
    if run_kind == "translation":
        segment = MotionSegment(kind=run_kind, steps=tuple(run))
        if segment.length_m >= MIN_TRANSLATION_LENGTH_M:
            translation.append(segment)
    elif run_kind == "rotation":
        segment = MotionSegment(kind=run_kind, steps=tuple(run))
        if segment.yaw_span_rad >= MIN_ROTATION_SPAN_RAD:
            rotation.append(segment)
    return MotionSegments(translation=tuple(translation), rotation=tuple(rotation))


def _circular_mean(angles: Sequence[float], weights: Sequence[float]) -> float:
    sine = 0.0
    cosine = 0.0
    for angle, weight in zip(angles, weights):
        sine += weight * math.sin(angle)
        cosine += weight * math.cos(angle)
    if abs(sine) < 1e-18 and abs(cosine) < 1e-18:
        return 0.0
    return math.atan2(sine, cosine)


def _frame_yaw_from_mount(mount_yaw: float) -> float:
    return wrap_pi(mount_yaw + _OPTICAL_YAW)


def _mount_yaw_from_frame(frame_yaw: float) -> float:
    return wrap_pi(frame_yaw - _OPTICAL_YAW)


def estimate_lidar_yaw(segments: Sequence[MotionSegment]) -> Optional[float]:
    """Lidar-frame yaw in base_link from nonholonomic translation.

    On a translation segment the lidar origin moves along base +x. In the
    first-scan world that direction is ``φ``, and lidar-frame yaw ``θ`` is
    constant, so ``ψ_frame = θ - φ``.
    """
    angles: list[float] = []
    weights: list[float] = []
    for segment in segments:
        dx = sum(step.dx for step in segment.steps)
        dy = sum(step.dy for step in segment.steps)
        length = math.hypot(dx, dy)
        if length < MIN_TRANSLATION_LENGTH_M:
            continue
        phi = math.atan2(dy, dx)
        theta = _circular_mean([step.start.yaw for step in segment.steps], [1.0] * len(segment.steps))
        angles.append(wrap_pi(theta - phi))
        weights.append(length)
    if not angles:
        return None
    return _circular_mean(angles, weights)


def _fit_circle(points: np.ndarray) -> Optional[tuple[float, float, float]]:
    if points.shape[0] < 3:
        return None
    x = points[:, 0]
    y = points[:, 1]
    matrix = np.column_stack((x, y, np.ones(points.shape[0])))
    rhs = -(x * x + y * y)
    try:
        delta, epsilon, f_term = np.linalg.lstsq(matrix, rhs, rcond=None)[0]
    except np.linalg.LinAlgError:
        return None
    cx = -0.5 * float(delta)
    cy = -0.5 * float(epsilon)
    radius2 = cx * cx + cy * cy - float(f_term)
    if radius2 <= 0.0 or not math.isfinite(radius2):
        return None
    return cx, cy, math.sqrt(radius2)


def estimate_lidar_xy(
    segments: Sequence[MotionSegment],
    psi_frame: float,
) -> Optional[tuple[float, float]]:
    """Lidar origin in base_link from in-place rotation about the base."""
    offsets: list[np.ndarray] = []
    for segment in segments:
        points = np.array(
            [[step.start.x, step.start.y] for step in segment.steps]
            + [[segment.steps[-1].end.x, segment.steps[-1].end.y]],
            dtype=np.float64,
        )
        fitted = _fit_circle(points)
        if fitted is None:
            continue
        cx, cy, _radius = fitted
        thetas = [step.start.yaw for step in segment.steps] + [segment.steps[-1].end.yaw]
        pts = list(points)
        local: list[np.ndarray] = []
        for pose_xy, theta in zip(pts, thetas):
            delta = pose_xy - np.array([cx, cy], dtype=np.float64)
            cosine = math.cos(-theta)
            sine = math.sin(-theta)
            local.append(
                np.array(
                    [cosine * delta[0] - sine * delta[1], sine * delta[0] + cosine * delta[1]],
                    dtype=np.float64,
                )
            )
        if not local:
            continue
        offsets.append(np.mean(local, axis=0))
    if not offsets:
        return None
    offset = np.mean(offsets, axis=0)
    cosine = math.cos(psi_frame)
    sine = math.sin(psi_frame)
    lx = cosine * float(offset[0]) - sine * float(offset[1])
    ly = sine * float(offset[0]) + cosine * float(offset[1])
    return (lx, ly)


def translation_lateral_rms(
    segments: Sequence[MotionSegment],
    psi_frame: float,
) -> float:
    """RMS metres of sideways motion relative to base +x at ``psi_frame``."""
    laterals: list[float] = []
    for segment in segments:
        for step in segment.steps:
            length = math.hypot(step.dx, step.dy)
            if length < TRANS_STEP_MIN_M:
                continue
            phi = math.atan2(step.dy, step.dx)
            expected = wrap_pi(step.start.yaw - psi_frame)
            laterals.append(abs(length * math.sin(wrap_pi(phi - expected))))
    if not laterals:
        return float("inf")
    stacked = np.asarray(laterals, dtype=np.float64)
    return float(np.sqrt(np.mean(stacked * stacked)))


def check_imu_yaw_sign(
    segments: Sequence[MotionSegment],
    imu: Sequence[DriveImuSample],
    imu_rpy: tuple[float, float, float],
) -> str:
    """Verify IMU rotation axis and sign against scan-match yaw rate.

    Fine IMU yaw is not recovered. Returns ``ok`` or ``mismatch``.
    """
    if not segments or not imu:
        return IMU_SIGN_MISMATCH
    rotation = np.asarray(rotation_matrix_from_rpy(imu_rpy), dtype=np.float64)
    scan_rates: list[float] = []
    gyro_body: list[np.ndarray] = []
    gyro_raw: list[np.ndarray] = []
    for segment in segments:
        t0 = segment.steps[0].start.stamp
        t1 = segment.steps[-1].end.stamp
        if t1 <= t0:
            continue
        signed_yaw = sum(step.dyaw for step in segment.steps)
        scan_rates.append(signed_yaw / (t1 - t0))
        window = [sample for sample in imu if t0 - 0.05 <= sample.stamp <= t1 + 0.05]
        if not window:
            window = list(imu)
        raw = np.array([sample.gyro for sample in window], dtype=np.float64)
        if raw.size == 0:
            continue
        gyro_raw.append(np.mean(np.abs(raw), axis=0))
        mapped = (rotation @ raw.T).T
        gyro_body.append(np.mean(mapped, axis=0))
    if not scan_rates or not gyro_body:
        return IMU_SIGN_MISMATCH
    omega_scan = float(np.mean(scan_rates))
    omega_body = np.mean(np.stack(gyro_body, axis=0), axis=0)
    if abs(float(omega_body[2])) < IMU_GYRO_MIN_RAD_S and abs(omega_scan) < IMU_GYRO_MIN_RAD_S:
        return IMU_SIGN_MISMATCH
    sign_ok = omega_scan * float(omega_body[2]) > 0.0
    mean_abs = np.mean(np.stack(gyro_raw, axis=0), axis=0)
    dominant = int(np.argmax(mean_abs))
    mapped_axis = rotation[:, dominant]
    axis_ok = abs(float(mapped_axis[2])) >= abs(float(mapped_axis[0])) and abs(float(mapped_axis[2])) >= abs(
        float(mapped_axis[1])
    )
    if sign_ok and axis_ok:
        return IMU_SIGN_OK
    return IMU_SIGN_MISMATCH


def _failure(details: tuple[str, ...]) -> DriveEvaluation:
    return DriveEvaluation(ok=False, stage=STAGE_NAME, details=details)


def _axis_changed(before: float, after: float) -> bool:
    return abs(before - after) > _POSE_COMPARE_EPS


def _proposed_xyz_rpy(
    evaluation: DriveEvaluation,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    assert evaluation.lidar_xyz is not None
    assert evaluation.lidar_rpy is not None
    return evaluation.lidar_xyz, evaluation.lidar_rpy


def _calib_with_lidar(
    calib: Calibration,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> Calibration:
    current = calib.sensors["lidar"]
    sensors = dict(calib.sensors)
    sensors["lidar"] = SensorPose(
        parent=current.parent,
        xyz=xyz,
        rpy=rpy,
        source=current.source,
    )
    return Calibration(sensors=sensors, unused=calib.unused, reason=calib.reason)


def drive_operator_fields(
    evaluation: DriveEvaluation,
    calib: Calibration,
) -> tuple[DriveOperatorField, ...]:
    """Operator-facing lidar x/y/yaw plus IMU sign check."""
    if not evaluation.ok:
        return ()
    xyz, rpy = _proposed_xyz_rpy(evaluation)
    proposed = _calib_with_lidar(calib, xyz, rpy)
    before_xyz, before_rpy, _, _ = operator_display(calib, "lidar")
    after_xyz, after_rpy, _, _ = operator_display(proposed, "lidar")
    fields = [
        DriveOperatorField(name="lidar_x", before=before_xyz[0], after=after_xyz[0]),
        DriveOperatorField(name="lidar_y", before=before_xyz[1], after=after_xyz[1]),
        DriveOperatorField(
            name="lidar_yaw",
            before=before_rpy[2],
            after=after_rpy[2],
        ),
        DriveOperatorField(
            name="imu_yaw_sign",
            before=evaluation.imu_yaw_sign,
            after=evaluation.imu_yaw_sign,
            source=IMU_SIGN_TAG,
        ),
    ]
    return tuple(fields)


def commit_drive_pending(
    evaluation: DriveEvaluation,
    calib: Calibration,
    dir: Optional[str] = None,
) -> tuple[DriveOperatorField, ...]:
    """Write drive pending proposal when evaluation succeeded."""
    if not evaluation.ok:
        return ()
    assert evaluation.residual_before is not None
    assert evaluation.residual_after is not None
    xyz, rpy = _proposed_xyz_rpy(evaluation)
    current = calib.sensors["lidar"]
    partial: dict[str, float] = {}
    if _axis_changed(current.xyz[0], xyz[0]):
        partial["x"] = xyz[0]
    if _axis_changed(current.xyz[1], xyz[1]):
        partial["y"] = xyz[1]
    if _axis_changed(current.rpy[2], rpy[2]):
        partial["yaw"] = rpy[2]
    write_draft(
        stage=STAGE_NAME,
        sensor="lidar",
        values={"lidar": partial} if partial else {},
        residual_before=evaluation.residual_before,
        residual_after=evaluation.residual_after,
        imu_residual_before=0.0,
        imu_residual_after=0.0,
        cause="",
        dir=dir,
    )
    return drive_operator_fields(evaluation, calib)


def evaluate_drive_trajectory(
    trajectory: ScanTrajectory,
    imu: Sequence[DriveImuSample],
    calib: Calibration,
) -> DriveEvaluation:
    """Estimate lidar mount from an already-matched trajectory."""
    if not trajectory.ok:
        detail = trajectory.detail or DETAIL_TRAVEL_TOO_SHORT
        return _failure((detail,))
    if trajectory.travel_m < MIN_TRAVEL_M:
        return _failure((DETAIL_TRAVEL_TOO_SHORT,))
    segments = segment_motion(trajectory.poses)
    if not segments.translation:
        return _failure((DETAIL_NO_TRANSLATION,))
    if not segments.rotation:
        return _failure((DETAIL_NO_ROTATION,))

    psi_frame = estimate_lidar_yaw(segments.translation)
    if psi_frame is None:
        return _failure((DETAIL_NO_TRANSLATION,))
    xy = estimate_lidar_xy(segments.rotation, psi_frame)
    if xy is None:
        return _failure((DETAIL_NO_ROTATION,))

    lidar = calib.sensors["lidar"]
    imu_pose = calib.sensors["imu"]
    current_frame = _frame_yaw_from_mount(lidar.rpy[2])
    residual_before = translation_lateral_rms(segments.translation, current_frame)
    residual_after = translation_lateral_rms(segments.translation, psi_frame)
    mount_yaw = unwrap_angle_near(lidar.rpy[2], _mount_yaw_from_frame(psi_frame))
    imu_sign = check_imu_yaw_sign(segments.rotation, imu, imu_pose.rpy)
    return DriveEvaluation(
        ok=True,
        stage=STAGE_NAME,
        details=(),
        lidar_xyz=(xy[0], xy[1], lidar.xyz[2]),
        lidar_rpy=(lidar.rpy[0], lidar.rpy[1], mount_yaw),
        residual_before=residual_before,
        residual_after=residual_after,
        imu_yaw_sign=imu_sign,
    )


def evaluate_drive_samples(
    samples: DriveSamples,
    calib: Calibration,
) -> DriveEvaluation:
    """Run scan matching, segment, estimate; write pending on success."""
    if not samples.ok:
        return _failure(samples.details)
    trajectory = build_scan_trajectory(samples.scans)
    evaluation = evaluate_drive_trajectory(trajectory, samples.imu, calib)
    if evaluation.ok:
        commit_drive_pending(evaluation, calib)
    return evaluation


run_drive = evaluate_drive_samples


def _create_ros_collector() -> DriveCollector:
    try:
        import rclpy  # noqa: F401
        from sensor_msgs.msg import Imu  # noqa: F401
        from sensor_msgs.msg import LaserScan  # noqa: F401
    except ImportError as exc:
        raise RosUnavailableError(
            "rclpy or sensor_msgs is not available",
        ) from exc
    return _RosDriveCollector()


class _RosDriveCollector:
    def __init__(self) -> None:
        import logging

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, LaserScan

        logging.getLogger("rclpy").setLevel(logging.FATAL)
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node = Node("mentorpi_calib_drive_collect")
        self._node.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
        self._scans: list[Any] = []
        self._imu: list[DriveImuSample] = []
        self._tracker = ScanTracker()
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

    def _on_scan(self, msg: Any) -> None:
        self._scans.append(msg)
        self._tracker.push(scan_xy(msg), scan_stamp(msg, fallback=time.monotonic()))

    def _on_imu(self, msg: Any) -> None:
        gyro = msg.angular_velocity
        self._imu.append(
            DriveImuSample(
                stamp=scan_stamp(msg, fallback=time.monotonic()),
                gyro=(float(gyro.x), float(gyro.y), float(gyro.z)),
            )
        )

    def topics_available(self) -> tuple[bool, bool]:
        names = {name for name, _ in self._node.get_topic_names_and_types()}
        return TOPIC_SCAN in names, TOPIC_IMU in names

    def run(
        self,
        timeout: float,
        on_progress: Callable[[float, float, int, int], None],
        should_stop: Callable[[float, float], bool],
    ) -> tuple[list[Any], list[DriveImuSample]]:
        deadline = time.monotonic() + timeout
        last_progress = 0.0
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_progress >= _PROGRESS_INTERVAL_SEC:
                on_progress(
                    self._tracker.travel_m,
                    self._tracker.turn_deg,
                    len(self._scans),
                    len(self._imu),
                )
                last_progress = now
                if should_stop(self._tracker.travel_m, self._tracker.turn_deg):
                    break
            if self._tracker.failed:
                break
        on_progress(
            self._tracker.travel_m,
            self._tracker.turn_deg,
            len(self._scans),
            len(self._imu),
        )
        return self._scans, self._imu

    def close(self) -> None:
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
