#!/usr/bin/env python3
"""SD010 T1: IMU sensor layer for demo contour (F04).

Stand T1 (vendor hiwonder_peripherals/launch/imu_filter.launch.py):
- Input: /ros_robot_controller/imu_raw (sensor_msgs/Imu, frame_id=imu_link)
  from ros_robot_controller (stage1); RRC params unchanged.
- Chain: imu_calib/apply_calib (raw -> imu_corrected) +
  imu_complementary_filter/complementary_filter_node (imu_corrected -> /imu).
  Overlay image installs ros-humble-imu-complementary-filter (apt); vendor
  MentorPi image does not ship that package.
- Vendor output topic before overlay remap: /imu (launch remap imu/data -> imu).
- Filter node name: imu_filter; calib node name: imu_calib.
- Vendor launch remaps /tf -> tf (relative); no publish_tf override in vendor file.

TF: robot_state_publisher when model layer is launch-ready (mount pose from
calibration mappings); otherwise imu_link_tf_fallback from fallback_tf_args
(base_footprint -> imu_link). Never both. Broken calibration logs degraded
and stays on factory pose.

Degraded: missing hiwonder_peripherals, imu_calib, imu_complementary_filter,
need_compile, or enable_imu:=false must not abort stage1.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

IMU_PACKAGE = "hiwonder_peripherals"
IMU_LAUNCH = "imu_filter.launch.py"
IMU_INPUT_TOPIC = "/ros_robot_controller/imu_raw"
IMU_OUTPUT_TOPIC = "/imu"
IMU_FRAME_ID = "imu_link"
BASE_FRAME_ID = "base_footprint"
MODEL_XACRO = "urdf/mentorpi_t1.urdf.xacro"
FILTER_NODE_NAME = "imu_filter"
CALIB_NODE_NAME = "imu_calib"

_VENDOR_IMU_PACKAGES = (
    "hiwonder_peripherals",
    "imu_calib",
    "imu_complementary_filter",
    "hiwonder_calibration",
)


def _truthy(value: str) -> bool:
    return value.lower() in ("true", "1", "yes")


def _imu_calibration():
    """Return (mappings, tf_args, unused_reason). tf_args is None on import error."""
    try:
        from mentorpi_calibration.calibration_file import REASON_NO_FILE, fallback_tf_args, load, xacro_mappings

        calib = load()
        unused_reason = ""
        if calib.unused and calib.reason != REASON_NO_FILE:
            unused_reason = calib.reason
        return xacro_mappings(calib), fallback_tf_args(calib, "imu"), unused_reason
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


def _use_imu_tf_fallback(context, mappings) -> bool:
    enable_robot_model = _truthy(
        LaunchConfiguration("enable_robot_model").perform(context),
    )
    if not enable_robot_model:
        return True
    return not _robot_model_tf_available(mappings)


def _vendor_imu_ready() -> tuple[bool, str]:
    if "need_compile" not in os.environ:
        return False, ("need_compile is unset (source /home/ubuntu/ros2_ws/.hiwonderrc)")
    for package in _VENDOR_IMU_PACKAGES:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            return False, f"{package} package is not installed"
    launch_path = os.path.join(
        get_package_share_directory(IMU_PACKAGE),
        "launch",
        IMU_LAUNCH,
    )
    if not os.path.isfile(launch_path):
        return False, f"{IMU_LAUNCH} is missing under {IMU_PACKAGE}"
    return True, ""


def _launch_setup(context):
    actions = []

    if _truthy(LaunchConfiguration("enable_imu").perform(context)):
        ready, reason = _vendor_imu_ready()
        if ready:
            peripherals_share = get_package_share_directory(IMU_PACKAGE)
            launch_path = os.path.join(
                peripherals_share,
                "launch",
                IMU_LAUNCH,
            )
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_path),
                )
            )
            actions.append(
                LogInfo(
                    msg=(
                        f"[imu_layer] vendor {IMU_PACKAGE}/{IMU_LAUNCH}"
                        f" input={IMU_INPUT_TOPIC}"
                        f" output={IMU_OUTPUT_TOPIC}"
                        f" frame={IMU_FRAME_ID}"
                        f" nodes={CALIB_NODE_NAME},{FILTER_NODE_NAME}"
                    ),
                )
            )
        else:
            actions.append(
                LogInfo(
                    msg=("[imu_layer] degraded — vendor IMU skipped:" f" {reason}"),
                )
            )
    else:
        actions.append(
            LogInfo(
                msg=("[imu_layer] degraded — enable_imu:=false" " — vendor filter skipped"),
            )
        )

    mappings, tf_args, unused_reason = _imu_calibration()
    if unused_reason:
        actions.append(
            LogInfo(
                msg=("[imu_layer] degraded — sensor calibration unused:" f" {unused_reason}"),
            )
        )

    if _use_imu_tf_fallback(context, mappings):
        if tf_args is None:
            actions.append(
                LogInfo(
                    msg=("[imu_layer] degraded — IMU TF skipped:" " calibration helper is not available"),
                )
            )
        else:
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="imu_link_tf_fallback",
                    output="screen",
                    arguments=tf_args,
                )
            )
            actions.append(
                LogInfo(
                    msg=(
                        "[imu_layer] imu_link_tf_fallback active"
                        f" ({BASE_FRAME_ID} -> {IMU_FRAME_ID}; URDF model TF off)"
                    ),
                )
            )
    else:
        actions.append(
            LogInfo(
                msg=("[imu_layer] imu TF from URDF model layer" " (robot_state_publisher); fallback suppressed"),
            )
        )

    actions.append(
        Node(
            package="mentorpi_localization",
            executable="imu_odometry",
            name="imu_odometry",
            output="screen",
            parameters=[
                {
                    "imu_topic": IMU_OUTPUT_TOPIC,
                    "odom_topic": "/imu_odom",
                    "odom_frame_id": "imu_odom",
                    "base_frame_id": BASE_FRAME_ID,
                }
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description=("try vendor hiwonder_peripherals/imu_filter.launch.py"),
            ),
            DeclareLaunchArgument(
                "enable_robot_model",
                default_value="true",
                description=(
                    "when true and mentorpi_description is launch-ready, IMU TF"
                    " is published by robot_state_publisher only; when false or"
                    " model unavailable, imu_link_tf_fallback is used"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
