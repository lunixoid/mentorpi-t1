#!/usr/bin/env python3
"""SD007 T2: robot model layer for demo contour (F07).

Publishes /robot_description and static TF from mentorpi_description Xacro.
Mount poses come from mentorpi_calibration (SD012): xacro.process_file gets
mappings from the calibration file. Missing package, xacro errors, or a
broken calibration file must not abort stage1 — factory mounts stay in the
xacro defaults / loader fallback.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODEL_XACRO = "urdf/mentorpi_t1.urdf.xacro"


def _truthy(value: str) -> bool:
    return value.lower() in ("true", "1", "yes")


def _calibration_mappings():
    """Return (mappings, unused_reason). unused_reason is empty if applied."""
    try:
        from mentorpi_calibration.calibration_file import REASON_NO_FILE, load, xacro_mappings

        calib = load()
        mappings = xacro_mappings(calib)
        if calib.unused and calib.reason != REASON_NO_FILE:
            return mappings, calib.reason
        return mappings, ""
    except Exception as exc:  # noqa: BLE001 — launch must not abort
        return {}, str(exc)


def _launch_setup(context):
    actions = []

    if not _truthy(LaunchConfiguration("enable_robot_model").perform(context)):
        actions.append(
            LogInfo(
                msg=("[robot_model_layer] enable_robot_model:=false" " — model layer skipped"),
            )
        )
        return actions

    try:
        description_share = get_package_share_directory("mentorpi_description")
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg=(
                    "[robot_model_layer] degraded — robot model skipped:"
                    " mentorpi_description package is not installed"
                ),
            )
        )
        return actions

    xacro_path = os.path.join(description_share, MODEL_XACRO)
    if not os.path.isfile(xacro_path):
        actions.append(
            LogInfo(
                msg=("[robot_model_layer] degraded — robot model skipped:" f" xacro file not found: {xacro_path}"),
            )
        )
        return actions

    try:
        import xacro
    except ImportError:
        actions.append(
            LogInfo(
                msg=("[robot_model_layer] degraded — robot model skipped:" " xacro Python module is not available"),
            )
        )
        return actions

    mappings, unused_reason = _calibration_mappings()
    if unused_reason:
        actions.append(
            LogInfo(
                msg=("[robot_model_layer] degraded — sensor calibration unused:" f" {unused_reason}"),
            )
        )

    try:
        robot_description_xml = xacro.process_file(
            xacro_path,
            mappings=mappings,
        ).toxml()
    except Exception as exc:  # launch must not abort on URDF/xacro errors
        actions.append(
            LogInfo(
                msg=("[robot_model_layer] degraded — robot model skipped:" f" xacro processing failed: {exc}"),
            )
        )
        return actions

    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_xml}],
        )
    )
    actions.append(
        LogInfo(
            msg=("[robot_model_layer] robot_state_publisher started" f" (xacro={xacro_path})"),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_robot_model",
                default_value="true",
                description=("start SD007 robot model layer (robot_state_publisher)"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
