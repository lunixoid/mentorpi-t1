"""In-container calibration CLI (SD012 I3.1 / I3.2)."""

from __future__ import annotations

import argparse
import sys
from typing import Optional

from mentorpi_calibration.calibration_file import (
    Calibration,
    abort_draft,
    accept_pending,
    drop_pending,
    factory_calibration,
    load,
    load_draft,
    operator_display,
    operator_source,
    promote_draft,
)
from mentorpi_calibration.stages.corner import DEFAULT_TIMEOUT
from mentorpi_calibration.stages.corner import STAGE_NAME as CORNER_STAGE
from mentorpi_calibration.stages.corner import CornerCollector
from mentorpi_calibration.stages.corner import RosUnavailableError as CornerRosUnavailableError
from mentorpi_calibration.stages.corner import collect_corner_samples, corner_operator_fields, evaluate_corner_samples
from mentorpi_calibration.stages.drive import DEFAULT_TIMEOUT as DRIVE_TIMEOUT
from mentorpi_calibration.stages.drive import STAGE_NAME as DRIVE_STAGE
from mentorpi_calibration.stages.drive import DriveCollector
from mentorpi_calibration.stages.drive import RosUnavailableError as DriveRosUnavailableError
from mentorpi_calibration.stages.drive import collect_drive_samples, drive_operator_fields, evaluate_drive_samples
from mentorpi_calibration.stages.manual import (
    STAGE_CAMERA,
    STAGE_LIDAR,
    commit_manual_pending,
    evaluate_camera,
    evaluate_lidar,
)
from mentorpi_calibration.stages.side import DEFAULT_TIMEOUT as SIDE_TIMEOUT
from mentorpi_calibration.stages.side import STAGE_NAME as SIDE_STAGE
from mentorpi_calibration.stages.side import RosUnavailableError as SideRosUnavailableError
from mentorpi_calibration.stages.side import (
    SideCollector,
    collect_side_samples,
    evaluate_side_samples,
    side_operator_fields,
)

_CLI_SENSORS = (
    ("camera", "depth_cam"),
    ("lidar", "lidar"),
    ("imu", "imu"),
)

_STAGE_ORDER = ("side", "corner", "drive", "lidar", "camera")
_POSE_COMPARE_EPS = 1e-6
_CONVENTION_FIELD = "transverse_mirror"

_KNOWN_COMMANDS = frozenset(
    {
        "show",
        "accept",
        "reject",
        "save",
        "abort",
        "side",
        "corner",
        "drive",
        "lidar",
        "camera",
    }
)

_XYZ_FIELDS = ("x", "y", "z")
_RPY_FIELDS = ("roll", "pitch", "yaw")

_STAGE_FIELD_PATTERNS: dict[str, tuple[tuple[str, tuple[str, ...]], ...]] = {
    "side": (("depth_cam", (_CONVENTION_FIELD,)),),
    "corner": (
        ("depth_cam", ("x", "y", "z", "roll", "pitch", "yaw")),
        ("lidar", ("yaw",)),
        ("imu", ("roll", "pitch")),
    ),
    "drive": (("lidar", ("x", "y", "yaw")),),
    "lidar": (("lidar", ("z", "roll", "pitch")),),
    "camera": (("depth_cam", ("z",)),),
}

_STAGE_SOURCES: dict[str, str] = {
    "side": "computed",
    "corner": "computed",
    "drive": "computed",
    "lidar": "measured",
    "camera": "measured",
}


def _baseline_calibration(dir: Optional[str] = None):
    current = load(dir)
    if current.unused:
        return factory_calibration()
    return current


def _pose_field_value(pose, field: str) -> float:
    if field in _XYZ_FIELDS:
        return pose.xyz[_XYZ_FIELDS.index(field)]
    return pose.rpy[_RPY_FIELDS.index(field)]


def _field_changed(left, right, field: str) -> bool:
    return abs(_pose_field_value(left, field) - _pose_field_value(right, field)) > _POSE_COMPARE_EPS


def _convention_accepted(draft, baseline, expected_source: str) -> bool:
    if draft.depth_cam_transverse_mirror_source != expected_source:
        return False
    return (
        draft.depth_cam_transverse_mirror != baseline.depth_cam_transverse_mirror
        or draft.depth_cam_transverse_mirror_source != baseline.depth_cam_transverse_mirror_source
    )


def _stage_accepted(stage: str, draft, baseline) -> bool:
    expected_source = _STAGE_SOURCES[stage]
    for sensor, fields in _STAGE_FIELD_PATTERNS[stage]:
        for field in fields:
            if field == _CONVENTION_FIELD:
                if sensor == "depth_cam" and _convention_accepted(draft, baseline, expected_source):
                    return True
                continue
            draft_pose = draft.sensors[sensor]
            baseline_pose = baseline.sensors[sensor]
            if _field_changed(draft_pose, baseline_pose, field) and draft_pose.source[field] == expected_source:
                return True
    return False


def _draft_accepted_stages(dir: Optional[str] = None) -> tuple[str, ...]:
    draft = load_draft(dir)
    if draft.absent or draft.unused:
        return ()
    baseline = _baseline_calibration(dir)
    return tuple(stage for stage in _STAGE_ORDER if _stage_accepted(stage, draft, baseline))


def _print_ok(ok: bool) -> None:
    print("T1CTL_CALIB_OK={}".format(1 if ok else 0))


def _print_detail(message: str) -> None:
    print("detail: {}".format(message))


def _print_stage_failure(stage: str, details: tuple[str, ...] | list[str]) -> int:
    _print_ok(False)
    print("stage: {}".format(stage))
    for detail in details:
        _print_detail(detail)
    return 1


def cmd_show(dir: Optional[str] = None) -> int:
    calib = load(dir)
    source = operator_source(calib)
    _print_ok(True)
    print("source: {}".format(source))
    if source == "unused" and calib.reason:
        print("reason: {}".format(calib.reason))
    accepted = _draft_accepted_stages(dir)
    if accepted:
        print("draft: {}".format(" ".join(accepted)))
    for cli_name, sensor in _CLI_SENSORS:
        xyz, rpy_deg, xyz_src, rpy_src = operator_display(calib, sensor)
        print("pose: {} xyz {:.16g} {:.16g} {:.16g} {}".format(cli_name, xyz[0], xyz[1], xyz[2], xyz_src))
        print("pose: {} rpy {:.16g} {:.16g} {:.16g} {}".format(cli_name, rpy_deg[0], rpy_deg[1], rpy_deg[2], rpy_src))
    return 0


def cmd_corner(
    timeout: float = DEFAULT_TIMEOUT,
    *,
    collector: Optional[CornerCollector] = None,
    dir: Optional[str] = None,
) -> int:
    calib = load(dir)
    try:
        samples = collect_corner_samples(timeout, collector=collector)
    except CornerRosUnavailableError as exc:
        return _print_stage_failure(CORNER_STAGE, (str(exc),))

    if not samples.ok:
        return _print_stage_failure(CORNER_STAGE, samples.details)

    evaluation = evaluate_corner_samples(samples, calib)
    if not evaluation.ok:
        return _print_stage_failure(CORNER_STAGE, evaluation.details)

    _print_ok(True)
    print("stage: {}".format(CORNER_STAGE))
    if evaluation.cause:
        print("cause: {}".format(evaluation.cause))
    print("residual_before: {}".format(evaluation.residual_before))
    print("residual_after: {}".format(evaluation.residual_after))
    print("imu_residual_before: {}".format(evaluation.imu_residual_before))
    print("imu_residual_after: {}".format(evaluation.imu_residual_after))
    for field in corner_operator_fields(evaluation, calib):
        print(
            "field: {} {} {} {}".format(
                field.name,
                field.before,
                field.after,
                field.source,
            )
        )
    for detail in evaluation.details:
        _print_detail(detail)
    return 0


def cmd_side(
    operator_side: str,
    timeout: float = SIDE_TIMEOUT,
    *,
    collector: Optional[SideCollector] = None,
    dir: Optional[str] = None,
) -> int:
    calib = load(dir)
    try:
        samples = collect_side_samples(timeout, collector=collector)
    except SideRosUnavailableError as exc:
        return _print_stage_failure(SIDE_STAGE, (str(exc),))

    if not samples.ok:
        return _print_stage_failure(SIDE_STAGE, samples.details)

    evaluation = evaluate_side_samples(samples, calib, operator_side, dir=dir)
    if not evaluation.ok:
        return _print_stage_failure(SIDE_STAGE, evaluation.details)

    _print_ok(True)
    print("stage: {}".format(SIDE_STAGE))
    print("observed: operator {}".format(evaluation.operator_side))
    print(
        "observed: scan {} {:.1f}".format(
            evaluation.scan_side,
            evaluation.scan_bearing_deg,
        )
    )
    print(
        "observed: cloud {} {:.1f}".format(
            evaluation.cloud_side,
            evaluation.cloud_bearing_deg,
        )
    )
    print("layer: {}".format(evaluation.layer))
    print("frame: {}".format(evaluation.frame_path))
    for field in side_operator_fields(evaluation, calib):
        print(
            "field: {} {} {} {}".format(
                field.name,
                field.before,
                field.after,
                field.source,
            )
        )
    return 0


def cmd_accept(dir: Optional[str] = None) -> int:
    draft = load_draft(dir)
    if draft.absent or draft.unused or draft.pending is None:
        _print_ok(False)
        if draft.absent or draft.unused:
            _print_detail("no draft")
        else:
            _print_detail("no pending")
        return 1
    stage = draft.pending.stage
    try:
        accept_pending(dir)
    except ValueError as exc:
        _print_ok(False)
        _print_detail(str(exc))
        return 1
    _print_ok(True)
    print("stage: {}".format(stage))
    return 0


def cmd_reject(dir: Optional[str] = None) -> int:
    draft = load_draft(dir)
    if draft.absent or draft.unused or draft.pending is None:
        _print_ok(False)
        if draft.absent or draft.unused:
            _print_detail("no draft")
        else:
            _print_detail("no pending")
        return 1
    stage = draft.pending.stage
    drop_pending(dir)
    _print_ok(True)
    print("stage: {}".format(stage))
    return 0


def _save_field_lines(draft, baseline) -> list[str]:
    lines: list[str] = []
    for cli_name, sensor in _CLI_SENSORS:
        draft_pose = draft.sensors[sensor]
        baseline_pose = baseline.sensors[sensor]
        before_xyz, before_rpy, _, _ = operator_display(baseline, sensor)
        merged = Calibration(
            sensors={**baseline.sensors, sensor: draft_pose},
            unused=False,
            reason="",
        )
        after_xyz, after_rpy, _, _ = operator_display(merged, sensor)
        for index, field in enumerate(_XYZ_FIELDS):
            if _field_changed(draft_pose, baseline_pose, field):
                lines.append(
                    "field: {}_{} {} {} {}".format(
                        cli_name,
                        field,
                        before_xyz[index],
                        after_xyz[index],
                        draft_pose.source[field],
                    )
                )
        for index, field in enumerate(_RPY_FIELDS):
            if _field_changed(draft_pose, baseline_pose, field):
                lines.append(
                    "field: {}_{} {} {} {}".format(
                        cli_name,
                        field,
                        before_rpy[index],
                        after_rpy[index],
                        draft_pose.source[field],
                    )
                )
    return lines


def cmd_save(dir: Optional[str] = None) -> int:
    draft = load_draft(dir)
    if draft.absent or draft.unused:
        _print_ok(False)
        _print_detail("no draft")
        return 1
    baseline = _baseline_calibration(dir)
    field_lines = _save_field_lines(draft, baseline)
    try:
        promote_draft(dir)
    except ValueError as exc:
        _print_ok(False)
        _print_detail(str(exc))
        return 1
    _print_ok(True)
    for line in field_lines:
        print(line)
    return 0


def cmd_abort(dir: Optional[str] = None) -> int:
    abort_draft(dir)
    _print_ok(True)
    return 0


def cmd_drive(
    *,
    timeout: float = DRIVE_TIMEOUT,
    collector: Optional[DriveCollector] = None,
    dir: Optional[str] = None,
) -> int:
    calib = load(dir)
    try:
        samples = collect_drive_samples(timeout, collector=collector)
    except DriveRosUnavailableError as exc:
        return _print_stage_failure(DRIVE_STAGE, (str(exc),))

    if not samples.ok:
        return _print_stage_failure(DRIVE_STAGE, samples.details)

    evaluation = evaluate_drive_samples(samples, calib)
    if not evaluation.ok:
        return _print_stage_failure(DRIVE_STAGE, evaluation.details)

    _print_ok(True)
    print("stage: {}".format(DRIVE_STAGE))
    print("residual_before: {}".format(evaluation.residual_before))
    print("residual_after: {}".format(evaluation.residual_after))
    for field in drive_operator_fields(evaluation, calib):
        print(
            "field: {} {} {} {}".format(
                field.name,
                field.before,
                field.after,
                field.source,
            )
        )
    return 0


def _emit_manual_proposal(evaluation, dir: Optional[str] = None) -> int:
    commit_manual_pending(evaluation, dir=dir)
    _print_ok(True)
    print("stage: {}".format(evaluation.stage))
    for field in evaluation.fields:
        print(
            "field: {} {} {} {}".format(
                field.name,
                field.before,
                field.after,
                field.source,
            )
        )
    return 0


def cmd_lidar(
    *,
    height: float,
    pitch: float,
    roll: float,
    dir: Optional[str] = None,
) -> int:
    evaluation = evaluate_lidar(height, pitch, roll, load(dir))
    if not evaluation.ok:
        return _print_stage_failure(STAGE_LIDAR, evaluation.details)
    return _emit_manual_proposal(evaluation, dir=dir)


def cmd_camera(*, height: float, dir: Optional[str] = None) -> int:
    evaluation = evaluate_camera(height, load(dir))
    if not evaluation.ok:
        return _print_stage_failure(STAGE_CAMERA, evaluation.details)
    return _emit_manual_proposal(evaluation, dir=dir)


def _parse_error_detail(message: str) -> str:
    lower = message.lower()
    if "required" in lower or "expected one argument" in lower or "the following arguments" in lower:
        return "incomplete options"
    if "invalid" in lower:
        return "invalid value"
    return message


class _ContractParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise ValueError(message)


def _build_parser() -> argparse.ArgumentParser:
    parser = _ContractParser(prog="calib")
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("show", help="show calibration state")

    sub.add_parser("accept", help="accept pending stage proposal")
    sub.add_parser("reject", help="reject pending stage proposal")
    sub.add_parser("save", help="promote draft to main calibration file")
    sub.add_parser("abort", help="delete draft file")

    side = sub.add_parser("side", help="side calibration stage")
    side.add_argument("--side", choices=("left", "right"), required=True)
    side.add_argument("--timeout", type=float, default=SIDE_TIMEOUT)

    corner = sub.add_parser("corner", help="corner calibration stage")
    corner.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)

    drive = sub.add_parser("drive", help="drive calibration stage")
    drive.add_argument("--timeout", type=float, default=DRIVE_TIMEOUT)

    lidar = sub.add_parser("lidar", help="manual lidar mount values")
    lidar.add_argument("--height", type=float, required=True)
    lidar.add_argument("--pitch", type=float, required=True)
    lidar.add_argument("--roll", type=float, required=True)

    camera = sub.add_parser("camera", help="manual camera height")
    camera.add_argument("--height", type=float, required=True)

    return parser


def main(argv: list[str] | None = None) -> int:
    args_list = sys.argv[1:] if argv is None else list(argv)
    if not args_list:
        return cmd_show()

    if args_list[0] not in _KNOWN_COMMANDS:
        _print_ok(False)
        _print_detail("unknown command")
        return 1

    parser = _build_parser()
    try:
        ns = parser.parse_args(args_list)
    except ValueError as exc:
        return _print_stage_failure(args_list[0], (_parse_error_detail(str(exc)),))

    if ns.command == "show":
        return cmd_show()
    if ns.command == "accept":
        return cmd_accept()
    if ns.command == "reject":
        return cmd_reject()
    if ns.command == "save":
        return cmd_save()
    if ns.command == "abort":
        return cmd_abort()
    if ns.command == "side":
        return cmd_side(operator_side=ns.side, timeout=ns.timeout)
    if ns.command == "corner":
        return cmd_corner(timeout=ns.timeout)
    if ns.command == "drive":
        return cmd_drive(timeout=ns.timeout)
    if ns.command == "lidar":
        return cmd_lidar(height=ns.height, pitch=ns.pitch, roll=ns.roll)
    if ns.command == "camera":
        return cmd_camera(height=ns.height)

    _print_ok(False)
    _print_detail("unknown command")
    return 1


if __name__ == "__main__":
    sys.exit(main())
