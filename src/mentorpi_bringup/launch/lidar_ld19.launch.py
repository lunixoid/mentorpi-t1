#!/usr/bin/env python3
"""SD006: local LD19 lidar equivalent for stands without /dev/lidar.

Vendor hiwonder_peripherals/include/ldlidar_LD19.launch.py hardcodes
port_name=/dev/lidar. On mentorpi-t1 the udev symlink is /dev/ldlidar.
This launch mirrors vendor LD19 + filter chain with a resolved serial port.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_serial_port() -> str:
    for candidate in ("/dev/lidar", "/dev/ldlidar"):
        if os.path.exists(candidate):
            return candidate
    return "/dev/lidar"


def generate_launch_description():
    lidar_frame = LaunchConfiguration("lidar_frame", default="lidar_frame")
    scan_raw = LaunchConfiguration("scan_raw", default="scan_raw")
    scan_topic = LaunchConfiguration("scan_topic", default="scan")
    serial_port = LaunchConfiguration("serial_port")

    lidar_frame_arg = DeclareLaunchArgument(
        "lidar_frame",
        default_value="lidar_frame",
    )
    scan_raw_arg = DeclareLaunchArgument(
        "scan_raw",
        default_value="scan_raw",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="scan",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value=_default_serial_port(),
        description="LD19 serial device (/dev/lidar or /dev/ldlidar on stand)",
    )

    peripherals_share = get_package_share_directory("hiwonder_peripherals")
    laser_filters_config = os.path.join(
        peripherals_share,
        "config",
        "lidar_filters_config_ld19.yaml",
    )

    ld19_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD19",
        output="screen",
        parameters=[
            {
                "topic_name": "scan",
                "product_name": "LDLiDAR_LD19",
                "port_baudrate": 230400,
                "port_name": ParameterValue(serial_port, value_type=str),
                "frame_id": lidar_frame,
                "laser_scan_dir": True,
                "enable_angle_crop_func": False,
                "angle_crop_min": 135.0,
                "angle_crop_max": 225.0,
            },
        ],
        remappings=[("scan", scan_raw)],
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        output="screen",
        parameters=[laser_filters_config],
        remappings=[
            ("scan", scan_raw),
            ("scan_filtered", scan_topic),
        ],
    )

    return LaunchDescription(
        [
            lidar_frame_arg,
            scan_raw_arg,
            scan_topic_arg,
            serial_port_arg,
            ld19_node,
            laser_filter_node,
        ]
    )
