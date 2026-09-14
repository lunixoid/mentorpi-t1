#!/usr/bin/env python3
"""SD006 T1 + SD007 T3: lidar sensor layer for demo contour (F03).

Demo contract (validated on mentorpi-t1 stand):
- A1: mentorpi_bringup local launch (`lidar_a1.launch.py` on /dev/lidar,
  `lidar_ld19.launch.py` on /dev/ldlidar — vendor sllidar_a1 hardcodes
  /dev/lidar only)
- LD19/G4: hiwonder_peripherals/launch/lidar.launch.py
- LaserScan topic: /scan (vendor scan_topic default)
- Frame id: lidar_frame (LaserScan.header.frame_id)
- LIDAR_TYPE from /home/ubuntu/ros2_ws/.hiwonderrc via .typerc (A1 on T1)

TF contract (SD007 T3 / SD012 T2, coordinated with robot_model_layer via stage1):
- enable_robot_model:=true and URDF model layer is launch-ready → lidar TF
  comes only from robot_state_publisher (base_footprint -> ... -> lidar_frame),
  with mount pose from calibration mappings.
- enable_robot_model:=false or model layer unavailable at launch → publish
  lidar_frame_tf_fallback from fallback_tf_args (static base_footprint ->
  lidar_frame). Broken calibration logs degraded and stays on factory pose.
- Never publish fallback when URDF model TF is active (no duplicate child frame).

Degraded: missing hiwonder_peripherals, unsupported LIDAR_TYPE, or absent
hardware must not abort stage1. Readiness is reported by t1ctl status (T2).
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Demo-contour contract for downstream features (F12) and viewer (SD005/SD007).
LIDAR_SCAN_TOPIC = "scan"
LIDAR_FRAME_ID = "lidar_frame"
BASE_FRAME_ID = "base_footprint"
MODEL_XACRO = "urdf/mentorpi_t1.urdf.xacro"

_VENDOR_LIDAR_TYPES = frozenset({"A1", "LD19", "G4"})


def _resolve_lidar_serial_port() -> str:
    for candidate in ("/dev/lidar", "/dev/ldlidar"):
        if os.path.exists(candidate):
            return candidate
    return "/dev/lidar"


def _truthy(value: str) -> bool:
    return value.lower() in ("true", "1", "yes")


def _lidar_calibration():
    """Return (mappings, tf_args, unused_reason). tf_args is None on import error."""
    try:
        from mentorpi_calibration.calibration_file import REASON_NO_FILE, fallback_tf_args, load, xacro_mappings

        calib = load()
        unused_reason = ""
        if calib.unused and calib.reason != REASON_NO_FILE:
            unused_reason = calib.reason
        return xacro_mappings(calib), fallback_tf_args(calib, "lidar"), unused_reason
    except Exception as exc:  # noqa: BLE001 — launch must not abort
        return {}, None, str(exc)


def _robot_model_tf_available(mappings) -> bool:
    """Mirror robot_model_layer launch-time readiness (no rsp node here)."""
    try:
        description_share = get_package_share_directory("mentorpi_description")
    except PackageNotFoundError:
        return False

    xacro_path = os.path.join(description_share, MODEL_XACRO)
    if not os.path.isfile(xacro_path):
        return False

    try:
        import xacro

        xacro.process_file(xacro_path, mappings=mappings).toxml()
    except Exception:
        return False
    return True


def _use_lidar_tf_fallback(context, mappings) -> bool:
    enable_robot_model = _truthy(
        LaunchConfiguration("enable_robot_model").perform(context),
    )
    if not enable_robot_model:
        return True
    return not _robot_model_tf_available(mappings)


def _vendor_lidar_ready() -> tuple[bool, str]:
    lidar_type = os.environ.get("LIDAR_TYPE", "").strip()
    if not lidar_type:
        return False, ("LIDAR_TYPE is unset (source /home/ubuntu/ros2_ws/.hiwonderrc)")
    if lidar_type not in _VENDOR_LIDAR_TYPES:
        return False, f"unsupported LIDAR_TYPE={lidar_type!r}"
    if "need_compile" not in os.environ:
        return False, ("need_compile is unset (source /home/ubuntu/ros2_ws/.hiwonderrc)")
    try:
        get_package_share_directory("hiwonder_peripherals")
    except PackageNotFoundError:
        return False, "hiwonder_peripherals package is not installed"
    return True, ""


def _launch_setup(context):
    actions = []

    if _truthy(LaunchConfiguration("enable_lidar").perform(context)):
        ready, reason = _vendor_lidar_ready()
        if ready:
            lidar_type = os.environ.get("LIDAR_TYPE", "").strip()
            serial_port = _resolve_lidar_serial_port()
            if lidar_type == "A1":
                bringup_share = get_package_share_directory(
                    "mentorpi_bringup",
                )
                if serial_port == "/dev/ldlidar":
                    local_launch = os.path.join(
                        bringup_share,
                        "launch",
                        "lidar_ld19.launch.py",
                    )
                    launch_label = "local lidar_ld19.launch.py"
                else:
                    local_launch = os.path.join(
                        bringup_share,
                        "launch",
                        "lidar_a1.launch.py",
                    )
                    launch_label = "local lidar_a1.launch.py"
                actions.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(local_launch),
                        launch_arguments={
                            "lidar_frame": LIDAR_FRAME_ID,
                            "scan_topic": LIDAR_SCAN_TOPIC,
                            "scan_raw": "scan_raw",
                            "serial_port": serial_port,
                        }.items(),
                    )
                )
                actions.append(
                    LogInfo(
                        msg=(
                            f"[lidar_layer] {launch_label}"
                            f" (LIDAR_TYPE=A1, serial_port={serial_port},"
                            f" topic=/{LIDAR_SCAN_TOPIC},"
                            f" frame={LIDAR_FRAME_ID})"
                        ),
                    )
                )
            elif lidar_type == "LD19" and serial_port == "/dev/ldlidar":
                bringup_share = get_package_share_directory(
                    "mentorpi_bringup",
                )
                ld19_launch = os.path.join(
                    bringup_share,
                    "launch",
                    "lidar_ld19.launch.py",
                )
                actions.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(ld19_launch),
                        launch_arguments={
                            "lidar_frame": LIDAR_FRAME_ID,
                            "scan_topic": LIDAR_SCAN_TOPIC,
                            "scan_raw": "scan_raw",
                            "serial_port": serial_port,
                        }.items(),
                    )
                )
                actions.append(
                    LogInfo(
                        msg=(
                            "[lidar_layer] local lidar_ld19.launch.py"
                            f" (LIDAR_TYPE=LD19, serial_port={serial_port},"
                            f" topic=/{LIDAR_SCAN_TOPIC},"
                            f" frame={LIDAR_FRAME_ID})"
                        ),
                    )
                )
            else:
                peripherals_share = get_package_share_directory(
                    "hiwonder_peripherals",
                )
                lidar_launch = os.path.join(
                    peripherals_share,
                    "launch",
                    "lidar.launch.py",
                )
                actions.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(lidar_launch),
                        launch_arguments={
                            "lidar_frame": LIDAR_FRAME_ID,
                            "scan_topic": LIDAR_SCAN_TOPIC,
                            "scan_raw": "scan_raw",
                        }.items(),
                    )
                )
                actions.append(
                    LogInfo(
                        msg=(
                            "[lidar_layer] vendor"
                            " hiwonder_peripherals/lidar.launch.py"
                            f" (LIDAR_TYPE={lidar_type},"
                            f" topic=/{LIDAR_SCAN_TOPIC},"
                            f" frame={LIDAR_FRAME_ID})"
                        ),
                    )
                )
        else:
            actions.append(
                LogInfo(
                    msg=("[lidar_layer] degraded — vendor lidar skipped:" f" {reason}"),
                )
            )
    else:
        actions.append(
            LogInfo(
                msg="[lidar_layer] enable_lidar:=false — vendor driver skipped",
            )
        )

    mappings, tf_args, unused_reason = _lidar_calibration()
    if unused_reason:
        actions.append(
            LogInfo(
                msg=("[lidar_layer] degraded — sensor calibration unused:" f" {unused_reason}"),
            )
        )

    if _use_lidar_tf_fallback(context, mappings):
        if tf_args is None:
            actions.append(
                LogInfo(
                    msg=("[lidar_layer] degraded — lidar TF skipped:" " calibration helper is not available"),
                )
            )
        else:
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="lidar_frame_tf_fallback",
                    output="screen",
                    arguments=tf_args,
                )
            )
            actions.append(
                LogInfo(
                    msg=(
                        "[lidar_layer] lidar_frame_tf_fallback active"
                        " (base_footprint -> lidar_frame; URDF model TF off)"
                    ),
                )
            )
    else:
        actions.append(
            LogInfo(
                msg=("[lidar_layer] lidar TF from URDF model layer" " (robot_state_publisher); fallback suppressed"),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
                description="try vendor hiwonder_peripherals/lidar.launch.py",
            ),
            DeclareLaunchArgument(
                "enable_robot_model",
                default_value="true",
                description=(
                    "SD007: when true and mentorpi_description is launch-ready,"
                    " lidar TF is published by robot_state_publisher only;"
                    " when false or model unavailable, lidar_frame_tf_fallback"
                    " is used (must match stage1 / robot_model_layer)"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
