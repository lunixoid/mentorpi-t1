"""Read and write sensor_calibration.yaml and apply poses (SD012 T1–T4).

Factory mount poses mirror origin in mentorpi_description xacro. Missing file,
unknown version, unreadable YAML, invalid schema, or values outside chassis-
scale limits return those factory poses with unused=True and a reason — never
an exception. xacro_mappings and fallback_tf_args compose those mounts with
the base_footprint offset and optical joints. operator_display is the read-side
pose for t1ctl. Draft files hold accepted stage poses plus an optional pending
proposal until promote_draft writes accepted sensors to the main file.
A top-level conventions section stores depth_cam.transverse_mirror; it is not a
pose (rpy cannot express a reflection). Absence of the section is false/factory
and does not mark the file unused.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any, Mapping, Optional

import yaml

CALIBRATION_FILENAME = "sensor_calibration.yaml"
DRAFT_FILENAME = "sensor_calibration.draft.yaml"
DEFAULT_CALIBRATION_DIR = "/home/ubuntu/mentorpi_t1_ws/config/platform/t1"
ENV_CALIBRATION_DIR = "T1_CALIBRATION_DIR"
SCHEMA_VERSION = 1

SENSOR_NAMES = ("lidar", "imu", "depth_cam")
FIELD_NAMES = ("x", "y", "z", "roll", "pitch", "yaw")
PARENT_FRAME = "base_link"
FILE_SOURCES = frozenset({"computed", "measured", "factory"})
CONVENTION_SOURCES = frozenset({"factory", "computed"})
MEASURED_STAGES = frozenset({"lidar", "camera"})
_CONVENTION_FIELD = "transverse_mirror"
_DEFAULT_MIRROR = False
_DEFAULT_MIRROR_SOURCE = "factory"

REASON_NO_FILE = "no file"
REASON_NO_DRAFT = "no draft"
REASON_UNREADABLE_YAML = "unreadable yaml"
REASON_UNKNOWN_VERSION = "unknown version"
REASON_INVALID_SCHEMA = "invalid schema"
REASON_OUT_OF_RANGE = "out of range"
REASON_NO_CHANGES = "no changes"

_POSE_COMPARE_EPS = 1e-6

# Chassis-scale sanity bounds, not an accuracy SLA. A value outside this is
# treated as a broken file.
MAX_TRANSLATION_M = 2.0
MAX_ROTATION_RAD = 2.0 * math.pi

# lidar_joint origin in lidar.urdf.xacro
# imu_joint origin in imu.urdf.xacro (M_PI as written there)
# depth_cam_* defaults in mentorpi_t1.urdf.xacro
_XACRO_M_PI = 3.1415926535897931

FACTORY_POSES = {
    "lidar": {
        "xyz": (0.0900034859353204, 0.0, 0.0405195774179554),
        "rpy": (0.0, 0.0, 0.0),
    },
    "imu": {
        "xyz": (0.0048416, 0.011168, -0.0057398),
        "rpy": (_XACRO_M_PI, 0.0, -_XACRO_M_PI / 2.0),
    },
    "depth_cam": {
        "xyz": (0.10, 0.0, 0.05),
        "rpy": (0.0, 0.0, 0.0),
    },
}

# base_joint origin z in car_tank.urdf.xacro (base_footprint → base_link).
BASE_LINK_OFFSET_Z = 0.127
FALLBACK_PARENT_FRAME = "base_footprint"
FALLBACK_CHILD_FRAMES = {
    "lidar": "lidar_frame",
    "imu": "imu_link",
    "depth_cam": "depth_camera_link",
}

# Optical joints stay in xacro and are not calibrated (D2.3).
# lidar_frame_joint rpy="0 0 π"; depth_cam_optical_joint rpy="-π/2 0 -π/2".
_OPTICAL_RPY = {
    "lidar": (0.0, 0.0, _XACRO_M_PI),
    "imu": (0.0, 0.0, 0.0),
    "depth_cam": (-_XACRO_M_PI / 2.0, 0.0, -_XACRO_M_PI / 2.0),
}

_XACRO_PREFIX = {
    "lidar": "lidar",
    "imu": "imu",
    "depth_cam": "depth_cam",
}


def _factory_sources() -> dict[str, str]:
    return {name: "factory" for name in FIELD_NAMES}


@dataclass(frozen=True)
class SensorPose:
    parent: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    source: Mapping[str, str]


@dataclass(frozen=True)
class Calibration:
    sensors: Mapping[str, SensorPose]
    unused: bool
    reason: str
    depth_cam_transverse_mirror: bool = False
    depth_cam_transverse_mirror_source: str = "factory"


@dataclass(frozen=True)
class PendingProposal:
    stage: str
    sensor: str
    values: Mapping[str, Mapping[str, float | bool]]
    residual_before: float
    residual_after: float
    imu_residual_before: float
    imu_residual_after: float
    cause: str


@dataclass(frozen=True)
class DraftCalibration:
    """Draft file state. absent=True only when the draft file is missing."""

    sensors: Mapping[str, SensorPose]
    pending: Optional[PendingProposal]
    absent: bool
    unused: bool
    reason: str
    depth_cam_transverse_mirror: bool = False
    depth_cam_transverse_mirror_source: str = "factory"


def resolve_dir(dir: Optional[str] = None) -> str:
    if dir is not None:
        return dir
    env = os.environ.get(ENV_CALIBRATION_DIR)
    if env:
        return env
    return DEFAULT_CALIBRATION_DIR


def calibration_path(dir: Optional[str] = None) -> str:
    return os.path.join(resolve_dir(dir), CALIBRATION_FILENAME)


def draft_path(dir: Optional[str] = None) -> str:
    return os.path.join(resolve_dir(dir), DRAFT_FILENAME)


def factory_calibration(*, unused: bool = False, reason: str = "") -> Calibration:
    sensors = {}
    for name in SENSOR_NAMES:
        pose = FACTORY_POSES[name]
        sensors[name] = SensorPose(
            parent=PARENT_FRAME,
            xyz=pose["xyz"],
            rpy=pose["rpy"],
            source=_factory_sources(),
        )
    return Calibration(sensors=sensors, unused=unused, reason=reason)


def load(dir: Optional[str] = None) -> Calibration:
    path = calibration_path(dir)
    if not os.path.isfile(path):
        return factory_calibration(unused=True, reason=REASON_NO_FILE)
    try:
        with open(path, encoding="utf-8") as handle:
            raw = handle.read()
    except OSError:
        return factory_calibration(unused=True, reason=REASON_UNREADABLE_YAML)
    try:
        data = yaml.safe_load(raw)
    except yaml.YAMLError:
        return factory_calibration(unused=True, reason=REASON_UNREADABLE_YAML)
    return _parse(data)


def save(calib: Calibration, dir: Optional[str] = None) -> None:
    _atomic_write(calibration_path(dir), _dump(calib))


def transverse_mirror(calib: Calibration) -> bool:
    """depth_cam cloud Y reflection in the optical frame. Default false."""
    return calib.depth_cam_transverse_mirror


def transverse_mirror_source(calib: Calibration) -> str:
    """Origin of transverse_mirror: factory | computed."""
    return calib.depth_cam_transverse_mirror_source


def load_draft(dir: Optional[str] = None) -> DraftCalibration:
    path = draft_path(dir)
    if not os.path.isfile(path):
        return _absent_draft()
    try:
        with open(path, encoding="utf-8") as handle:
            raw = handle.read()
    except OSError:
        return _broken_draft(REASON_UNREADABLE_YAML)
    try:
        data = yaml.safe_load(raw)
    except yaml.YAMLError:
        return _broken_draft(REASON_UNREADABLE_YAML)
    return _parse_draft(data)


def write_draft(
    stage: str,
    sensor: str,
    values: Mapping[str, Mapping[str, float | bool]],
    residual_before: float,
    residual_after: float,
    imu_residual_before: float,
    imu_residual_after: float,
    cause: str = "",
    dir: Optional[str] = None,
) -> None:
    pending = PendingProposal(
        stage=stage,
        sensor=sensor,
        values=values,
        residual_before=residual_before,
        residual_after=residual_after,
        imu_residual_before=imu_residual_before,
        imu_residual_after=imu_residual_after,
        cause=cause,
    )
    existing = load_draft(dir)
    if existing.absent or existing.unused:
        baseline = _baseline_calibration(dir)
        sensors = baseline.sensors
        mirror = baseline.depth_cam_transverse_mirror
        mirror_source = baseline.depth_cam_transverse_mirror_source
    else:
        sensors = existing.sensors
        mirror = existing.depth_cam_transverse_mirror
        mirror_source = existing.depth_cam_transverse_mirror_source
    draft = DraftCalibration(
        sensors=sensors,
        pending=pending,
        absent=False,
        unused=False,
        reason="",
        depth_cam_transverse_mirror=mirror,
        depth_cam_transverse_mirror_source=mirror_source,
    )
    _atomic_write(draft_path(dir), _dump_draft(draft))


def drop_pending(dir: Optional[str] = None) -> None:
    existing = load_draft(dir)
    if existing.absent or existing.unused or existing.pending is None:
        return
    draft = DraftCalibration(
        sensors=existing.sensors,
        pending=None,
        absent=False,
        unused=False,
        reason="",
        depth_cam_transverse_mirror=existing.depth_cam_transverse_mirror,
        depth_cam_transverse_mirror_source=existing.depth_cam_transverse_mirror_source,
    )
    _atomic_write(draft_path(dir), _dump_draft(draft))


def accept_pending(dir: Optional[str] = None) -> None:
    existing = load_draft(dir)
    if existing.absent or existing.unused:
        raise ValueError("no draft")
    if existing.pending is None:
        raise ValueError("no pending")
    sensors = dict(existing.sensors)
    field_source = pending_field_source(existing.pending.stage)
    mirror = existing.depth_cam_transverse_mirror
    mirror_source = existing.depth_cam_transverse_mirror_source
    for name, partial in existing.pending.values.items():
        if name not in SENSOR_NAMES:
            raise ValueError("unknown sensor in pending: {}".format(name))
        pose_partial = {field: float(value) for field, value in partial.items() if field in FIELD_NAMES}
        if pose_partial:
            sensors[name] = _merge_partial_pose(sensors[name], pose_partial, field_source)
        if name == "depth_cam" and _CONVENTION_FIELD in partial:
            mirror = bool(partial[_CONVENTION_FIELD])
            mirror_source = "computed"
    draft = DraftCalibration(
        sensors=sensors,
        pending=None,
        absent=False,
        unused=False,
        reason="",
        depth_cam_transverse_mirror=mirror,
        depth_cam_transverse_mirror_source=mirror_source,
    )
    _atomic_write(draft_path(dir), _dump_draft(draft))


def abort_draft(dir: Optional[str] = None) -> None:
    path = draft_path(dir)
    if os.path.isfile(path):
        os.remove(path)


def promote_draft(dir: Optional[str] = None) -> None:
    existing = load_draft(dir)
    if existing.absent or existing.unused:
        raise ValueError("no draft")
    proposed = Calibration(
        sensors=existing.sensors,
        unused=False,
        reason="",
        depth_cam_transverse_mirror=existing.depth_cam_transverse_mirror,
        depth_cam_transverse_mirror_source=existing.depth_cam_transverse_mirror_source,
    )
    baseline = _baseline_calibration(dir)
    if _poses_equal(proposed, baseline) and _conventions_equal(proposed, baseline):
        raise ValueError(REASON_NO_CHANGES)
    save(proposed, dir)
    abort_draft(dir)


def xacro_mappings(calib: Calibration) -> dict[str, str]:
    """Flat lidar_x … depth_cam_yaw strings for xacro.process_file mappings."""
    mappings: dict[str, str] = {}
    for name in SENSOR_NAMES:
        pose = calib.sensors[name]
        prefix = _XACRO_PREFIX[name]
        values = pose.xyz + pose.rpy
        for field, value in zip(FIELD_NAMES, values):
            mappings["{}_{}".format(prefix, field)] = _fmt(value)
    return mappings


def fallback_tf_args(calib: Calibration, sensor: str) -> list[str]:
    """Eight static_transform_publisher args: x y z yaw pitch roll parent child.

    Composes base_footprint → base_link (z offset), the calibrated mount on
    base_link, and the optical joint. Optical rpy is not taken from the file.
    """
    if sensor not in SENSOR_NAMES:
        raise ValueError("unknown sensor: {}".format(sensor))
    pose = calib.sensors[sensor]
    roll, pitch, yaw = _compose_rpy(pose.rpy, _OPTICAL_RPY[sensor])
    return [
        _fmt(pose.xyz[0]),
        _fmt(pose.xyz[1]),
        _fmt(pose.xyz[2] + BASE_LINK_OFFSET_Z),
        _fmt(yaw),
        _fmt(pitch),
        _fmt(roll),
        FALLBACK_PARENT_FRAME,
        FALLBACK_CHILD_FRAMES[sensor],
    ]


def operator_source(calib: Calibration) -> str:
    """Aggregate origin for t1ctl: factory | file | unused.

    Missing file is factory (poses from the platform model). unused is only
    when a file is present and was not applied.
    """
    if not calib.unused:
        return "file"
    if calib.reason == REASON_NO_FILE:
        return "factory"
    return "unused"


def pending_field_source(stage: str) -> str:
    """Origin written on accept: measured for lidar/camera, computed otherwise."""
    if stage in MEASURED_STAGES:
        return "measured"
    return "computed"


def operator_height_to_mount_z(height_m: float) -> float:
    """Convert floor-relative height (operator xyz z) to base_link mount z."""
    return height_m - BASE_LINK_OFFSET_Z


def operator_rpy_deg_to_mount(sensor: str, rpy_deg: tuple[float, float, float]) -> tuple[float, float, float]:
    """Inverse of the rpy half of operator_display: degrees → mount radians."""
    if sensor not in SENSOR_NAMES:
        raise ValueError("unknown sensor: {}".format(sensor))
    op_rad = (
        math.radians(rpy_deg[0]),
        math.radians(rpy_deg[1]),
        math.radians(rpy_deg[2]),
    )
    if sensor == "lidar":
        return _compose_rpy(op_rad, _inverse_rpy(_OPTICAL_RPY["lidar"]))
    if sensor == "imu":
        return _compose_rpy(FACTORY_POSES["imu"]["rpy"], op_rad)
    return op_rad


def values_in_range(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> bool:
    return _in_range(xyz, rpy)


def operator_display(calib: Calibration, sensor: str) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    str,
    str,
]:
    """Operator pose relative to base_footprint: xyz m, rpy deg, row sources.

    Camera omits optical rpy. Lidar includes the lidar_frame yaw of π so the
    factory facing shows as 180°. IMU rpy is the extra rotation beyond the
    factory chip mount, so factory tilt prints as zeros.
    """
    if sensor not in SENSOR_NAMES:
        raise ValueError("unknown sensor: {}".format(sensor))
    pose = calib.sensors[sensor]
    xyz = (pose.xyz[0], pose.xyz[1], pose.xyz[2] + BASE_LINK_OFFSET_Z)
    rpy_deg = tuple(_deg(value) for value in _operator_rpy(sensor, pose.rpy))
    return (
        xyz,
        rpy_deg,
        _row_source(pose.source, ("x", "y", "z")),
        _row_source(pose.source, ("roll", "pitch", "yaw")),
    )


def rotation_matrix_from_rpy(
    rpy: tuple[float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    """URDF / tf2 setRPY rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    return _rpy_matrix(rpy)


def mat_vec3(
    matrix: tuple[tuple[float, float, float], ...],
    vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Apply a 3×3 rotation to a 3-vector."""
    return tuple(matrix[i][0] * vec[0] + matrix[i][1] * vec[1] + matrix[i][2] * vec[2] for i in range(3))


def depth_cam_optical_rpy() -> tuple[float, float, float]:
    """Fixed depth_cam_link → depth_camera_link optical joint rpy (not in file)."""
    return _OPTICAL_RPY["depth_cam"]


def depth_cam_optical_to_mount_matrix() -> tuple[tuple[float, float, float], ...]:
    """Rotation from depth_camera_link (optical) to depth_cam_link (mount)."""
    return _transpose(rotation_matrix_from_rpy(_OPTICAL_RPY["depth_cam"]))


def _rpy_matrix(rpy: tuple[float, float, float]) -> tuple[tuple[float, float, float], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF / tf2 setRPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _matmul3(
    left: tuple[tuple[float, float, float], ...],
    right: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(left[i][0] * right[0][j] + left[i][1] * right[1][j] + left[i][2] * right[2][j] for j in range(3))
        for i in range(3)
    )


def _matrix_rpy(
    matrix: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float]:
    pitch = math.atan2(
        -matrix[2][0],
        math.hypot(matrix[2][1], matrix[2][2]),
    )
    if abs(matrix[2][0]) < 0.999999:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        # Gimbal lock: yaw and roll share an axis; put the remainder in yaw.
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    return (roll, pitch, yaw)


def _compose_rpy(
    mount: tuple[float, float, float],
    optical: tuple[float, float, float],
) -> tuple[float, float, float]:
    return _matrix_rpy(_matmul3(_rpy_matrix(mount), _rpy_matrix(optical)))


def _transpose(
    matrix: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(matrix[j][i] for j in range(3)) for i in range(3))


def _inverse_rpy(rpy: tuple[float, float, float]) -> tuple[float, float, float]:
    return _matrix_rpy(_transpose(_rpy_matrix(rpy)))


def _operator_rpy(sensor: str, mount_rpy: tuple[float, float, float]) -> tuple[float, float, float]:
    if sensor == "lidar":
        return _compose_rpy(mount_rpy, _OPTICAL_RPY["lidar"])
    if sensor == "imu":
        return _compose_rpy(_inverse_rpy(FACTORY_POSES["imu"]["rpy"]), mount_rpy)
    return mount_rpy


def _deg(rad: float) -> float:
    degrees = math.degrees(rad)
    if abs(degrees) < 1e-9:
        return 0.0
    return degrees


def _row_source(source: Mapping[str, str], fields: tuple[str, ...]) -> str:
    values = [source[field] for field in fields]
    if all(value == values[0] for value in values):
        return values[0]
    if "measured" in values:
        return "measured"
    if "computed" in values:
        return "computed"
    return "factory"


def _parse(data: Any) -> Calibration:
    if not isinstance(data, dict):
        return factory_calibration(unused=True, reason=REASON_INVALID_SCHEMA)
    if "version" not in data:
        return factory_calibration(unused=True, reason=REASON_INVALID_SCHEMA)
    try:
        version = int(data["version"])
    except (TypeError, ValueError):
        return factory_calibration(unused=True, reason=REASON_UNKNOWN_VERSION)
    if version != SCHEMA_VERSION:
        return factory_calibration(unused=True, reason=REASON_UNKNOWN_VERSION)

    sensors_raw = data.get("sensors")
    if not isinstance(sensors_raw, dict):
        return factory_calibration(unused=True, reason=REASON_INVALID_SCHEMA)

    sensors: dict[str, SensorPose] = {}
    for name in SENSOR_NAMES:
        parsed = _parse_sensor(sensors_raw.get(name))
        if parsed is None:
            return factory_calibration(unused=True, reason=REASON_INVALID_SCHEMA)
        if parsed == REASON_OUT_OF_RANGE:
            return factory_calibration(unused=True, reason=REASON_OUT_OF_RANGE)
        sensors[name] = parsed
    parsed_conventions = _conventions_from_data(data)
    if parsed_conventions is None:
        return factory_calibration(unused=True, reason=REASON_INVALID_SCHEMA)
    mirror, mirror_source = parsed_conventions
    return Calibration(
        sensors=sensors,
        unused=False,
        reason="",
        depth_cam_transverse_mirror=mirror,
        depth_cam_transverse_mirror_source=mirror_source,
    )


def _parse_sensor(raw: Any) -> SensorPose | str | None:
    if not isinstance(raw, dict):
        return None
    parent = raw.get("parent", PARENT_FRAME)
    if parent != PARENT_FRAME:
        return None
    xyz = _parse_vec3(raw.get("xyz"))
    rpy = _parse_vec3(raw.get("rpy"))
    source = _parse_source(raw.get("source"))
    if xyz is None or rpy is None or source is None:
        return None
    if not _in_range(xyz, rpy):
        return REASON_OUT_OF_RANGE
    return SensorPose(parent=parent, xyz=xyz, rpy=rpy, source=source)


def _parse_vec3(raw: Any) -> tuple[float, float, float] | None:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        return None
    values = []
    for item in raw:
        try:
            number = float(item)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        values.append(number)
    return (values[0], values[1], values[2])


def _parse_source(raw: Any) -> dict[str, str] | None:
    if not isinstance(raw, dict):
        return None
    parsed = {}
    for field in FIELD_NAMES:
        value = raw.get(field)
        if value not in FILE_SOURCES:
            return None
        parsed[field] = value
    return parsed


def _in_range(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> bool:
    for value in xyz:
        if abs(value) > MAX_TRANSLATION_M:
            return False
    for value in rpy:
        if abs(value) > MAX_ROTATION_RAD:
            return False
    return True


def _fmt(value: float) -> str:
    return format(value, ".16g")


def _yaml_bool(value: bool) -> str:
    return "true" if value else "false"


def _dump_conventions_lines(mirror: bool, source: str) -> list[str]:
    return [
        "conventions:",
        "  depth_cam:",
        "    transverse_mirror: {}".format(_yaml_bool(mirror)),
        "    source: {}".format(source),
    ]


def _conventions_from_data(data: dict) -> tuple[bool, str] | None:
    if "conventions" not in data:
        return _DEFAULT_MIRROR, _DEFAULT_MIRROR_SOURCE
    raw = data["conventions"]
    if raw is None:
        return _DEFAULT_MIRROR, _DEFAULT_MIRROR_SOURCE
    return _parse_conventions(raw)


def _parse_conventions(raw: Any) -> tuple[bool, str] | None:
    if not isinstance(raw, dict):
        return None
    if not raw:
        return _DEFAULT_MIRROR, _DEFAULT_MIRROR_SOURCE
    depth = raw.get("depth_cam")
    if depth is None:
        return _DEFAULT_MIRROR, _DEFAULT_MIRROR_SOURCE
    if not isinstance(depth, dict):
        return None
    if _CONVENTION_FIELD not in depth:
        if "source" not in depth:
            return _DEFAULT_MIRROR, _DEFAULT_MIRROR_SOURCE
        return None
    mirror = depth[_CONVENTION_FIELD]
    if not isinstance(mirror, bool):
        return None
    source = depth.get("source", _DEFAULT_MIRROR_SOURCE)
    if source not in CONVENTION_SOURCES:
        return None
    return mirror, source


def _dump(calib: Calibration) -> str:
    lines = ["version: {}".format(SCHEMA_VERSION), "sensors:"]
    for name in SENSOR_NAMES:
        pose = calib.sensors[name]
        xyz = ", ".join(_fmt(v) for v in pose.xyz)
        rpy = ", ".join(_fmt(v) for v in pose.rpy)
        lines.append("  {}:".format(name))
        lines.append("    parent: {}".format(pose.parent))
        lines.append("    xyz: [{}]".format(xyz))
        lines.append("    rpy: [{}]".format(rpy))
        lines.append("    source:")
        for field in FIELD_NAMES:
            lines.append("      {}: {}".format(field, pose.source[field]))
    lines.extend(
        _dump_conventions_lines(
            calib.depth_cam_transverse_mirror,
            calib.depth_cam_transverse_mirror_source,
        )
    )
    lines.append("")
    return "\n".join(lines)


def _atomic_write(path: str, text: str) -> None:
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        handle.write(text)
    os.replace(tmp_path, path)


def _baseline_calibration(dir: Optional[str] = None) -> Calibration:
    current = load(dir)
    if current.unused:
        return factory_calibration()
    return current


def _absent_draft() -> DraftCalibration:
    return DraftCalibration(
        sensors=factory_calibration().sensors,
        pending=None,
        absent=True,
        unused=True,
        reason=REASON_NO_DRAFT,
    )


def _broken_draft(reason: str) -> DraftCalibration:
    return DraftCalibration(
        sensors=factory_calibration().sensors,
        pending=None,
        absent=False,
        unused=True,
        reason=reason,
    )


def _parse_draft(data: Any) -> DraftCalibration:
    if not isinstance(data, dict):
        return _broken_draft(REASON_INVALID_SCHEMA)
    if "version" not in data:
        return _broken_draft(REASON_INVALID_SCHEMA)
    try:
        version = int(data["version"])
    except (TypeError, ValueError):
        return _broken_draft(REASON_UNKNOWN_VERSION)
    if version != SCHEMA_VERSION:
        return _broken_draft(REASON_UNKNOWN_VERSION)

    sensors_raw = data.get("sensors")
    if not isinstance(sensors_raw, dict):
        return _broken_draft(REASON_INVALID_SCHEMA)

    sensors: dict[str, SensorPose] = {}
    for name in SENSOR_NAMES:
        parsed = _parse_sensor(sensors_raw.get(name))
        if parsed is None:
            return _broken_draft(REASON_INVALID_SCHEMA)
        if parsed == REASON_OUT_OF_RANGE:
            return _broken_draft(REASON_OUT_OF_RANGE)
        sensors[name] = parsed

    parsed_conventions = _conventions_from_data(data)
    if parsed_conventions is None:
        return _broken_draft(REASON_INVALID_SCHEMA)
    mirror, mirror_source = parsed_conventions

    pending_raw = data.get("pending")
    pending: Optional[PendingProposal] = None
    if pending_raw is not None:
        parsed_pending = _parse_pending(pending_raw)
        if parsed_pending is None:
            return _broken_draft(REASON_INVALID_SCHEMA)
        pending = parsed_pending

    return DraftCalibration(
        sensors=sensors,
        pending=pending,
        absent=False,
        unused=False,
        reason="",
        depth_cam_transverse_mirror=mirror,
        depth_cam_transverse_mirror_source=mirror_source,
    )


def _parse_pending(raw: Any) -> PendingProposal | None:
    if not isinstance(raw, dict):
        return None
    stage = raw.get("stage")
    sensor = raw.get("sensor")
    if not isinstance(stage, str) or not stage or not isinstance(sensor, str) or not sensor:
        return None
    values_raw = raw.get("values")
    if not isinstance(values_raw, dict):
        return None
    values: dict[str, dict[str, float | bool]] = {}
    for sensor_name, partial_raw in values_raw.items():
        if sensor_name not in SENSOR_NAMES or not isinstance(partial_raw, dict):
            return None
        partial: dict[str, float | bool] = {}
        for field, value in partial_raw.items():
            if field == _CONVENTION_FIELD:
                if sensor_name != "depth_cam" or not isinstance(value, bool):
                    return None
                partial[field] = value
                continue
            if field not in FIELD_NAMES:
                return None
            try:
                number = float(value)
            except (TypeError, ValueError):
                return None
            if not math.isfinite(number):
                return None
            partial[field] = number
        if not partial:
            return None
        values[sensor_name] = partial

    residual_before = _parse_float(raw.get("residual_before"))
    residual_after = _parse_float(raw.get("residual_after"))
    imu_residual_before = _parse_float(raw.get("imu_residual_before"))
    imu_residual_after = _parse_float(raw.get("imu_residual_after"))
    if residual_before is None or residual_after is None or imu_residual_before is None or imu_residual_after is None:
        return None
    cause = raw.get("cause", "")
    if cause is None:
        cause = ""
    if not isinstance(cause, str):
        return None
    return PendingProposal(
        stage=stage,
        sensor=sensor,
        values=values,
        residual_before=residual_before,
        residual_after=residual_after,
        imu_residual_before=imu_residual_before,
        imu_residual_after=imu_residual_after,
        cause=cause,
    )


def _parse_float(raw: Any) -> float | None:
    try:
        number = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _dump_draft(draft: DraftCalibration) -> str:
    lines = ["version: {}".format(SCHEMA_VERSION), "sensors:"]
    for name in SENSOR_NAMES:
        pose = draft.sensors[name]
        xyz = ", ".join(_fmt(v) for v in pose.xyz)
        rpy = ", ".join(_fmt(v) for v in pose.rpy)
        lines.append("  {}:".format(name))
        lines.append("    parent: {}".format(pose.parent))
        lines.append("    xyz: [{}]".format(xyz))
        lines.append("    rpy: [{}]".format(rpy))
        lines.append("    source:")
        for field in FIELD_NAMES:
            lines.append("      {}: {}".format(field, pose.source[field]))
    lines.extend(
        _dump_conventions_lines(
            draft.depth_cam_transverse_mirror,
            draft.depth_cam_transverse_mirror_source,
        )
    )
    if draft.pending is not None:
        pending = draft.pending
        lines.append("pending:")
        lines.append("  stage: {}".format(pending.stage))
        lines.append("  sensor: {}".format(pending.sensor))
        lines.append("  values:")
        for sensor_name in sorted(pending.values.keys()):
            partial = pending.values[sensor_name]
            lines.append("    {}:".format(sensor_name))
            for field in FIELD_NAMES:
                if field in partial:
                    lines.append("      {}: {}".format(field, _fmt(float(partial[field]))))
            if _CONVENTION_FIELD in partial:
                lines.append(
                    "      {}: {}".format(
                        _CONVENTION_FIELD,
                        _yaml_bool(bool(partial[_CONVENTION_FIELD])),
                    )
                )
        lines.append("  residual_before: {}".format(_fmt(pending.residual_before)))
        lines.append("  residual_after: {}".format(_fmt(pending.residual_after)))
        lines.append("  imu_residual_before: {}".format(_fmt(pending.imu_residual_before)))
        lines.append("  imu_residual_after: {}".format(_fmt(pending.imu_residual_after)))
        lines.append("  cause: {}".format(repr(pending.cause)))
    lines.append("")
    return "\n".join(lines)


def _merge_partial_pose(
    pose: SensorPose,
    partial: Mapping[str, float],
    field_source: str = "computed",
) -> SensorPose:
    xyz = list(pose.xyz)
    rpy = list(pose.rpy)
    source = dict(pose.source)
    for field, value in partial.items():
        if field in ("x", "y", "z"):
            xyz[("x", "y", "z").index(field)] = value
        elif field in ("roll", "pitch", "yaw"):
            rpy[("roll", "pitch", "yaw").index(field)] = value
        source[field] = field_source
    merged_xyz = tuple(xyz)
    merged_rpy = tuple(rpy)
    if not _in_range(merged_xyz, merged_rpy):
        raise ValueError(REASON_OUT_OF_RANGE)
    return SensorPose(
        parent=pose.parent,
        xyz=merged_xyz,
        rpy=merged_rpy,
        source=source,
    )


def _poses_equal(left: Calibration, right: Calibration) -> bool:
    for name in SENSOR_NAMES:
        left_pose = left.sensors[name]
        right_pose = right.sensors[name]
        for left_value, right_value in zip(left_pose.xyz, right_pose.xyz):
            if abs(left_value - right_value) > _POSE_COMPARE_EPS:
                return False
        for left_value, right_value in zip(left_pose.rpy, right_pose.rpy):
            if abs(left_value - right_value) > _POSE_COMPARE_EPS:
                return False
        if dict(left_pose.source) != dict(right_pose.source):
            return False
    return True


def _conventions_equal(left: Calibration, right: Calibration) -> bool:
    return (
        left.depth_cam_transverse_mirror == right.depth_cam_transverse_mirror
        and left.depth_cam_transverse_mirror_source == right.depth_cam_transverse_mirror_source
    )
