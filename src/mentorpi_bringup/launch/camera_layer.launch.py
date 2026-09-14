#!/usr/bin/env python3
"""SD008 T1: depth-camera layer for demo contour (F05).

Stand T1 (MentorPi_Tank + USB 3251:1930 Aurora 930):
- Driver: deptrum-ros-driver-aurora930 / aurora930_node
- Stock launch on image: aurora930_launch.py (not overlay
  hiwonder_peripherals/depth_camera.launch.py — that file is Dabai-only)
- Topics (driver namespace aurora, no Hiwonder ascamera remaps):
  /aurora/rgb/image_raw, /aurora/depth/image_raw, /aurora/points2
  IR off (SD014 T6). rgb_fps=10. Image/cloud publishers BEST_EFFORT KeepLast(1).
- Frames: mount depth_cam_link (URDF); optical depth_camera_link (depth/cloud);
  rgb_camera_link from the driver (static depth->rgb). Do not duplicate those.

Overlay .typerc may still say DEPTH_CAMERA_TYPE=Dabai. Hardware USB and
stage1 DEPTH_CAMERA_TYPE=aurora win. Missing package/USB/launch must not
abort stage1 — log [camera_layer] degraded.

TF: robot_state_publisher when the model layer is launch-ready (mount pose
from calibration mappings); otherwise depth_cam_frame_tf_fallback from
fallback_tf_args (base_footprint -> depth_camera_link). Never both. Broken
calibration logs degraded and stays on factory pose.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERA_PACKAGE = "deptrum-ros-driver-aurora930"
CAMERA_EXECUTABLE = "aurora930_node"
CAMERA_LAUNCH = "aurora930_launch.py"
COLOR_TOPIC = "/aurora/rgb/image_raw"
DEPTH_TOPIC = "/aurora/depth/image_raw"
CLOUD_TOPIC = "/aurora/points2"
MOUNT_FRAME_ID = "depth_cam_link"
DEPTH_FRAME_ID = "depth_camera_link"
RGB_FRAME_ID = "rgb_camera_link"
BASE_FRAME_ID = "base_footprint"
MODEL_XACRO = "urdf/mentorpi_t1.urdf.xacro"
AURORA_USB = ("3251", "1930")
DEPTRUM_PREFIX = "/home/ubuntu/third_party_ros2/third_party_ws/install/" "deptrum-ros-driver-aurora930"

# Overlay Node params: vendor aurora930_launch.py does not accept qos_overrides.
# ir_fps must be >= rgb_fps or vendor ignores rgb_fps; valid ir_fps: 5, 10, 12, 15.
_AURORA_PUBLISHER_QOS = {
    "reliability": "best_effort",
    "history": "keep_last",
    "depth": 1,
}


def _aurora_node_parameters():
    return {
        "rgb_enable": True,
        "ir_enable": False,
        "depth_enable": True,
        "rgbd_enable": True,
        "point_cloud_enable": True,
        "boot_order": 1,
        "ir_fps": 10,
        "rgb_fps": 10,
        "exposure_enable": True,
        "exposure_time": 10,
        "gain_enable": True,
        "gain_value": 10,
        "usb_port_number": "",
        "threshold_size": 30,
        "depth_correction": True,
        "align_mode": True,
        "laser_power": 3.0,
        "minimum_filter_depth_value": 150,
        "maximum_filter_depth_value": 4000,
        "resolution_mode_index": 2,
        "log_dir": "/tmp/",
        "stream_sdk_log_enable": False,
        "heart_enable": False,
        "update_file_path": "",
        "qos_overrides": {
            "/aurora/rgb/image_raw": {"publisher": dict(_AURORA_PUBLISHER_QOS)},
            "/aurora/depth/image_raw": {"publisher": dict(_AURORA_PUBLISHER_QOS)},
            "/aurora/points2": {"publisher": dict(_AURORA_PUBLISHER_QOS)},
        },
    }


def _truthy(value: str) -> bool:
    return value.lower() in ("true", "1", "yes")


def _read_sysfs(path: str) -> str:
    try:
        with open(path, encoding="ascii") as handle:
            return handle.read().strip()
    except OSError:
        return ""


def _aurora_usb_present() -> bool:
    sysfs = "/sys/bus/usb/devices"
    if not os.path.isdir(sysfs):
        return False
    for entry in os.listdir(sysfs):
        base = os.path.join(sysfs, entry)
        vendor = _read_sysfs(os.path.join(base, "idVendor")).lower()
        product = _read_sysfs(os.path.join(base, "idProduct")).lower()
        if (vendor, product) == AURORA_USB:
            return True
    return False


def _ensure_deptrum_env() -> None:
    """Isolated colcon prefix is not on overlay AMENT_PREFIX_PATH by default."""
    if not os.path.isdir(DEPTRUM_PREFIX):
        return
    prefixes = [item for item in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if item]
    if DEPTRUM_PREFIX not in prefixes:
        os.environ["AMENT_PREFIX_PATH"] = ":".join(
            [DEPTRUM_PREFIX] + prefixes,
        )
    libdir = os.path.join(DEPTRUM_PREFIX, "lib")
    if os.path.isdir(libdir):
        libs = [item for item in os.environ.get("LD_LIBRARY_PATH", "").split(":") if item]
        if libdir not in libs:
            os.environ["LD_LIBRARY_PATH"] = ":".join([libdir] + libs)


def _camera_calibration():
    """Return (mappings, tf_args, unused_reason). tf_args is None on import error."""
    try:
        from mentorpi_calibration.calibration_file import REASON_NO_FILE, fallback_tf_args, load, xacro_mappings

        calib = load()
        unused_reason = ""
        if calib.unused and calib.reason != REASON_NO_FILE:
            unused_reason = calib.reason
        return (
            xacro_mappings(calib),
            fallback_tf_args(calib, "depth_cam"),
            unused_reason,
        )
    except Exception as exc:  # noqa: BLE001 — launch must not abort
        return {}, None, str(exc)


def _robot_model_tf_available(mappings) -> bool:
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


def _use_camera_tf_fallback(context, mappings) -> bool:
    enable_robot_model = _truthy(
        LaunchConfiguration("enable_robot_model").perform(context),
    )
    if not enable_robot_model:
        return True
    return not _robot_model_tf_available(mappings)


def _vendor_camera_ready() -> tuple[bool, str]:
    _ensure_deptrum_env()
    camera_type = os.environ.get("DEPTH_CAMERA_TYPE", "").strip()
    usb_ok = _aurora_usb_present()
    if camera_type and camera_type != "aurora" and not usb_ok:
        return False, (f"unsupported DEPTH_CAMERA_TYPE={camera_type!r} " "(T1 expects aurora / USB 3251:1930)")
    if not usb_ok:
        return False, "Aurora 930 USB 3251:1930 not present"
    try:
        share = get_package_share_directory(CAMERA_PACKAGE)
    except PackageNotFoundError:
        return False, f"{CAMERA_PACKAGE} package is not installed"
    launch_path = os.path.join(share, "launch", CAMERA_LAUNCH)
    if not os.path.isfile(launch_path):
        return False, f"{CAMERA_LAUNCH} is missing under {share}"
    return True, ""


def _launch_setup(context):
    actions = []
    stale_type = os.environ.get("DEPTH_CAMERA_TYPE", "").strip()

    if _truthy(LaunchConfiguration("enable_depth_camera").perform(context)):
        ready, reason = _vendor_camera_ready()
        if ready:
            actions.append(
                Node(
                    package=CAMERA_PACKAGE,
                    executable=CAMERA_EXECUTABLE,
                    namespace="aurora",
                    output="screen",
                    emulate_tty=True,
                    parameters=[_aurora_node_parameters()],
                )
            )
            stale_note = ""
            if stale_type and stale_type != "aurora":
                stale_note = f" (DEPTH_CAMERA_TYPE={stale_type!r} overridden by USB)"
            actions.append(
                LogInfo(
                    msg=(
                        f"[camera_layer] vendor {CAMERA_PACKAGE}/{CAMERA_EXECUTABLE}"
                        f"{stale_note} ir_enable=false rgb_fps=10 ir_fps=10"
                        f" color={COLOR_TOPIC} depth={DEPTH_TOPIC}"
                        f" cloud={CLOUD_TOPIC} frame={DEPTH_FRAME_ID}"
                        f" rgb_frame={RGB_FRAME_ID} mount={MOUNT_FRAME_ID}"
                    ),
                )
            )
        else:
            actions.append(
                LogInfo(
                    msg=("[camera_layer] degraded — vendor camera skipped:" f" {reason}"),
                )
            )
    else:
        actions.append(
            LogInfo(
                msg=("[camera_layer] degraded — enable_depth_camera:=false" " — vendor driver skipped"),
            )
        )

    mappings, tf_args, unused_reason = _camera_calibration()
    if unused_reason:
        actions.append(
            LogInfo(
                msg=("[camera_layer] degraded — sensor calibration unused:" f" {unused_reason}"),
            )
        )

    if _use_camera_tf_fallback(context, mappings):
        if tf_args is None:
            actions.append(
                LogInfo(
                    msg=("[camera_layer] degraded — camera TF skipped:" " calibration helper is not available"),
                )
            )
        else:
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="depth_cam_frame_tf_fallback",
                    output="screen",
                    arguments=tf_args,
                )
            )
            actions.append(
                LogInfo(
                    msg=(
                        "[camera_layer] depth_cam_frame_tf_fallback active"
                        f" ({BASE_FRAME_ID} -> {DEPTH_FRAME_ID}; URDF model TF off)"
                    ),
                )
            )
    else:
        actions.append(
            LogInfo(
                msg=("[camera_layer] camera TF from URDF model layer" " (robot_state_publisher); fallback suppressed"),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_depth_camera",
                default_value="true",
                description=("try vendor deptrum-ros-driver-aurora930/aurora930_launch.py"),
            ),
            DeclareLaunchArgument(
                "enable_robot_model",
                default_value="true",
                description=(
                    "when true and mentorpi_description is launch-ready, camera TF"
                    " is published by robot_state_publisher only; when false or"
                    " model unavailable, depth_cam_frame_tf_fallback is used"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
