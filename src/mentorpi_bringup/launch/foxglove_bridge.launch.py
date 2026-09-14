#!/usr/bin/env python3
"""SD005 T2: read-only Foxglove bridge for demo viewer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    config = os.path.join(
        get_package_share_directory("mentorpi_bringup"),
        "config",
        "foxglove_bridge.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="8765",
                description="Foxglove WebSocket TCP port (ws://<host>:<port>)",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    config,
                    {
                        "port": ParameterValue(port, value_type=int),
                    },
                ],
            ),
        ]
    )
