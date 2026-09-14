"""Manual lidar height/tilt and camera height (SD012 D3.5)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Optional

from mentorpi_calibration.calibration_file import (
    MAX_ROTATION_RAD,
    MAX_TRANSLATION_M,
    REASON_OUT_OF_RANGE,
    Calibration,
    SensorPose,
    operator_display,
    operator_height_to_mount_z,
    operator_rpy_deg_to_mount,
    values_in_range,
    write_draft,
)

STAGE_LIDAR = "lidar"
STAGE_CAMERA = "camera"
SOURCE_MEASURED = "measured"

DETAIL_INVALID_VALUE = "invalid value"
DETAIL_OUT_OF_RANGE = REASON_OUT_OF_RANGE
DETAIL_INCOMPLETE = "incomplete options"

_LIDAR_SENSOR = "lidar"
_CAMERA_SENSOR = "depth_cam"


@dataclass(frozen=True)
class ManualOperatorField:
    """Operator-facing before/after value for CLI ``field:`` lines."""

    name: str
    before: float
    after: float
    source: str = SOURCE_MEASURED


@dataclass(frozen=True)
class ManualEvaluation:
    """Result of parsing operator-entered lidar or camera values."""

    ok: bool
    stage: str
    details: tuple[str, ...]
    sensor: str = ""
    values: Mapping[str, Mapping[str, float]] = field(default_factory=dict)
    fields: tuple[ManualOperatorField, ...] = ()


def _finite_numbers(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


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


def _height_in_operator_range(height_m: float) -> bool:
    if height_m <= 0.0:
        return False
    mount_z = operator_height_to_mount_z(height_m)
    return abs(mount_z) <= MAX_TRANSLATION_M


def _angle_in_range(degrees: float) -> bool:
    return abs(math.radians(degrees)) <= MAX_ROTATION_RAD


def evaluate_lidar(
    height_m: float,
    pitch_deg: float,
    roll_deg: float,
    calib: Calibration,
) -> ManualEvaluation:
    """Build a lidar pending proposal from operator height (m) and tilt (deg)."""
    if not _finite_numbers(height_m, pitch_deg, roll_deg):
        return ManualEvaluation(ok=False, stage=STAGE_LIDAR, details=(DETAIL_INVALID_VALUE,))
    if not _height_in_operator_range(height_m) or not _angle_in_range(pitch_deg) or not _angle_in_range(roll_deg):
        return ManualEvaluation(ok=False, stage=STAGE_LIDAR, details=(DETAIL_OUT_OF_RANGE,))

    current = calib.sensors[_LIDAR_SENSOR]
    before_xyz, before_rpy, _, _ = operator_display(calib, _LIDAR_SENSOR)
    mount_z = operator_height_to_mount_z(height_m)
    mount_rpy = operator_rpy_deg_to_mount(_LIDAR_SENSOR, (roll_deg, pitch_deg, before_rpy[2]))
    proposed_xyz = (current.xyz[0], current.xyz[1], mount_z)
    proposed_rpy = (mount_rpy[0], mount_rpy[1], current.rpy[2])
    if not values_in_range(proposed_xyz, proposed_rpy):
        return ManualEvaluation(ok=False, stage=STAGE_LIDAR, details=(DETAIL_OUT_OF_RANGE,))

    proposed = _calib_with_sensor_pose(calib, _LIDAR_SENSOR, proposed_xyz, proposed_rpy)
    after_xyz, after_rpy, _, _ = operator_display(proposed, _LIDAR_SENSOR)
    values = {
        _LIDAR_SENSOR: {
            "z": proposed_xyz[2],
            "roll": proposed_rpy[0],
            "pitch": proposed_rpy[1],
        }
    }
    fields = (
        ManualOperatorField(name="lidar_z", before=before_xyz[2], after=after_xyz[2]),
        ManualOperatorField(name="lidar_pitch", before=before_rpy[1], after=after_rpy[1]),
        ManualOperatorField(name="lidar_roll", before=before_rpy[0], after=after_rpy[0]),
    )
    return ManualEvaluation(
        ok=True,
        stage=STAGE_LIDAR,
        details=(),
        sensor=_LIDAR_SENSOR,
        values=values,
        fields=fields,
    )


def evaluate_camera(height_m: float, calib: Calibration) -> ManualEvaluation:
    """Build a camera pending proposal from operator height in metres."""
    if not _finite_numbers(height_m):
        return ManualEvaluation(ok=False, stage=STAGE_CAMERA, details=(DETAIL_INVALID_VALUE,))
    if not _height_in_operator_range(height_m):
        return ManualEvaluation(ok=False, stage=STAGE_CAMERA, details=(DETAIL_OUT_OF_RANGE,))

    current = calib.sensors[_CAMERA_SENSOR]
    mount_z = operator_height_to_mount_z(height_m)
    proposed_xyz = (current.xyz[0], current.xyz[1], mount_z)
    if not values_in_range(proposed_xyz, current.rpy):
        return ManualEvaluation(ok=False, stage=STAGE_CAMERA, details=(DETAIL_OUT_OF_RANGE,))

    proposed = _calib_with_sensor_pose(calib, _CAMERA_SENSOR, proposed_xyz, current.rpy)
    before_xyz, _, _, _ = operator_display(calib, _CAMERA_SENSOR)
    after_xyz, _, _, _ = operator_display(proposed, _CAMERA_SENSOR)
    values = {_CAMERA_SENSOR: {"z": proposed_xyz[2]}}
    fields = (ManualOperatorField(name="camera_z", before=before_xyz[2], after=after_xyz[2]),)
    return ManualEvaluation(
        ok=True,
        stage=STAGE_CAMERA,
        details=(),
        sensor=_CAMERA_SENSOR,
        values=values,
        fields=fields,
    )


def commit_manual_pending(
    evaluation: ManualEvaluation,
    dir: Optional[str] = None,
) -> None:
    """Write the manual proposal to the draft file when evaluation succeeded."""
    if not evaluation.ok:
        return
    write_draft(
        stage=evaluation.stage,
        sensor=evaluation.sensor,
        values=evaluation.values,
        residual_before=0.0,
        residual_after=0.0,
        imu_residual_before=0.0,
        imu_residual_after=0.0,
        cause="",
        dir=dir,
    )
