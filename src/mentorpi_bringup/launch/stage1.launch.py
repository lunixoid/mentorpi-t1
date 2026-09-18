#!/usr/bin/env python3
"""Stage 1 bringup: chassis SIT (SD003), lidar (SD006), IMU (SD010 T1),
camera (SD008 T1), control mux (SD011), person perception (SD013),
FollowPerson status (SD019), motion control (SD020), obstacle avoidance and guard
(SD036 F12-F14), voice control (SD031 F21).
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_robot_model = LaunchConfiguration("enable_robot_model")
    enable_depth_camera = LaunchConfiguration("enable_depth_camera")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_voice = LaunchConfiguration("enable_voice")

    # F02 adapter + vendor chassis SIT (ros_robot_controller, odom_publisher)
    # live in this file. Do NOT IncludeLaunchDescription of Hiwonder
    # start_app / lidar_controller / joystick_control /
    # hiwonder_controller.launch.py / odom_publisher.launch.py (also starts
    # robot_description — F07) / bringup.launch.py / controller.launch.py /
    # lidar.launch.py / depth_camera.launch.py, rosbridge, web_video_server,
    # Nav2, MQTT, or voice. Vendor pkgs are on the Pi image; T5 sources
    # /home/ubuntu/ros2_ws/.hiwonderrc then overlay setup.bash (bash, not zsh)
    # so they are on AMENT_PREFIX_PATH.
    # Remaining:
    #   F03 lidar — lidar_layer.launch.py (A1 local / LD19/G4 vendor)
    #   F05 camera — camera_layer.launch.py (aurora / deptrum, not
    #       hiwonder_peripherals/depth_camera.launch.py which is Dabai-only)
    #   F06 control — control_state + control_mux (SD011)
    #   F07 urdf — robot_model_layer.launch.py (SD007 T2)
    #   F08 perception — person_perception + person_detect_pi after camera_layer
    #       (SD013, SD026, SD034). Onboard person_detect_pi enabled by default.
    #   F09 follow (track)
    #   F10 follow (behavior) — mission_control (SD019)
    #   F11 motion — motion_control (SD020, SD025)
    #   F12 obstacle avoidance — virtual bumper inside motion_control (SD036)
    #   F13 safety, F14 invariant — obstacle_guard publishes /control/motion_restriction (SD036)
    #   F21 voice — command_dispatcher + voice_command (SD031 overlay, not vendor)
    control_state = Node(
        package="mentorpi_control",
        executable="control_state",
        name="control_state",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "remote_timeout_ms": 1000,
                "rate_hz": 10.0,
            }
        ],
    )

    pad_teleop = Node(
        package="mentorpi_control",
        executable="pad_teleop",
        name="pad_teleop",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "mode_button": 10,
                "mode_button_linux": 8,
                "linear_axis": 1,
                "angular_axis": 2,
                "max_linear": 0.5,
                "max_angular": 2.0,
                # Stand calibration (not SLA). enter=existing stick deadzone.
                # min = 20% of max: vendor has no min-RPS; after renormalize a
                # stick just above 0.1 mapped to ~0.006 m/s and buzzed in place.
                "linear_enter": 0.10,
                "linear_release": 0.06,
                "linear_min": 0.10,
                "angular_enter": 0.10,
                "angular_release": 0.06,
                "angular_min": 0.40,
                "joy_timeout_ms": 1000,
                # 2 * (1000 / linux_joy.rate_hz); must stay < joy_timeout_ms.
                "cmd_freshness_ms": 100,
                "rate_hz": 20.0,
            }
        ],
    )

    # T8 — USB dongle on the Pi is a Linux joystick (ShanWan 2563:0575,
    # by-id …WirelessGamepad-joystick, often js0). Stock pygame waits on
    # js0; ROS 2 joy_node is SDL and ignores `dev`. We publish /joy from
    # linux_joy (retry, by-id then js*). Not vendor joystick_control.
    linux_joy = Node(
        package="mentorpi_control",
        executable="linux_joy",
        name="linux_joy",
        output="screen",
        parameters=[
            {
                "device": "",
                "name_substr": "WirelessGamepad",
                "rate_hz": 20.0,
                "retry_ms": 200,
            }
        ],
    )

    # SD011: sole publisher of /vehicle/cmd_vel. pad_teleop writes
    # /control/manual_cmd_vel; platform_adapter stays the last gate to
    # /hiwonder_controller/cmd_vel (watchdog + FORBIDDEN fail-safe).
    control_mux = Node(
        package="mentorpi_control",
        executable="control_mux",
        name="control_mux",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "rate_hz": 20.0,
            }
        ],
    )

    ros_robot_controller = Node(
        package="ros_robot_controller",
        executable="ros_robot_controller",
        output="screen",
        # Overlay T1 RRC: UART init (JGB37 x2, battery, full zero) then speed.
        # t1-stop must kill this node before opening /dev/rrc.
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            {
                "imu_frame": "imu_link",
            }
        ],
    )

    # SD005 T1: viewer contract — odom_publisher is the sole odometry + TF
    # source in demo contour. Topic /odom_raw, frames odom -> base_footprint;
    # no EKF/URDF.
    odom_parameters = [
        {
            "base_frame_id": "base_footprint",
            "odom_frame_id": "odom",
            "odom_topic": "odom_raw",
            "pub_odom_topic": True,
            "publish_tf": True,
        }
    ]
    try:
        hiwonder_share = get_package_share_directory("hiwonder_controller")
        calibrate_yaml = os.path.join(hiwonder_share, "config", "calibrate_params.yaml")
        if os.path.isfile(calibrate_yaml):
            odom_parameters.insert(0, calibrate_yaml)
    except PackageNotFoundError:
        pass

    odom_publisher = Node(
        package="hiwonder_controller",
        executable="odom_publisher",
        name="odom_publisher",
        output="screen",
        parameters=odom_parameters,
    )

    platform_adapter = Node(
        package="mentorpi_platform",
        executable="platform_adapter",
        name="platform_adapter",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "cmd_timeout_ms": 100,
                "rate_hz": 20.0,
            }
        ],
    )

    # SD007 T2: platform URDF via robot_state_publisher — no motion path.
    robot_model_layer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mentorpi_bringup"),
                        "launch",
                        "robot_model_layer.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "enable_robot_model": enable_robot_model,
        }.items(),
    )

    # SD008 T1: Deptrum Aurora 930; TF from URDF or fallback, not both.
    camera_layer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mentorpi_bringup"),
                        "launch",
                        "camera_layer.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "enable_depth_camera": enable_depth_camera,
            "enable_robot_model": enable_robot_model,
        }.items(),
    )

    # SD013 T5 / D9, SD026 D6, SD034: F08 on the robot. YAML from package share.
    # Do not IncludeLaunchDescription vendor YOLO, HailoRT, or vendor bringup.launch.py.
    person_perception = Node(
        package="mentorpi_perception",
        executable="person_perception",
        name="person_perception",
        output="screen",
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("mentorpi_perception"),
                    "config",
                    "person_perception.yaml",
                ]
            ),
        ],
    )

    # SD026 T4 / SD034: onboard YOLO11n NCNN + ByteTrack; enabled by default (YAML).
    person_detect_pi = Node(
        package="mentorpi_person_detect",
        executable="person_detect_pi",
        name="person_detect_pi",
        output="screen",
        emulate_tty=True,
        additional_env={
            "OMP_NUM_THREADS": "2",
            "NCNN_NUM_THREADS": "2",
        },
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("mentorpi_person_detect"),
                    "config",
                    "person_detect_pi.yaml",
                ]
            ),
        ],
    )

    mission_control = Node(
        package="mission_control",
        executable="mission_control",
        name="mission_control",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "nearest_timeout_ms": 1000,
                "rate_hz": 10.0,
            }
        ],
    )

    command_dispatcher = Node(
        package="mentorpi_commands",
        executable="command_dispatcher",
        name="command_dispatcher",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "set_mode_timeout_ms": 1000,
            }
        ],
    )

    voice_command = Node(
        package="mentorpi_voice",
        executable="voice_command",
        name="voice_command",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(enable_voice),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("mentorpi_voice"),
                    "config",
                    "voice_command.yaml",
                ]
            ),
        ],
    )

    # SD036 I10: follow law + virtual bumper; footprint.yaml is shared with obstacle_guard.
    motion_control_share = FindPackageShare("motion_control")
    footprint_yaml = PathJoinSubstitution([motion_control_share, "config", "footprint.yaml"])
    motion_control = Node(
        package="motion_control",
        executable="motion_control",
        name="motion_control",
        output="screen",
        emulate_tty=True,
        parameters=[
            footprint_yaml,
            PathJoinSubstitution([motion_control_share, "config", "motion_control.yaml"]),
        ],
    )

    # SD036 D6: F13/F14, the only publisher of /control/motion_restriction (was stub_graph).
    obstacle_guard = Node(
        package="motion_control",
        executable="obstacle_guard",
        name="obstacle_guard",
        output="screen",
        emulate_tty=True,
        parameters=[
            footprint_yaml,
            PathJoinSubstitution([motion_control_share, "config", "obstacle_guard.yaml"]),
        ],
    )

    # SD010 T1: vendor imu_filter; TF from URDF or fallback, not both.
    imu_layer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mentorpi_bringup"),
                        "launch",
                        "imu_layer.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "enable_imu": enable_imu,
            "enable_robot_model": enable_robot_model,
        }.items(),
    )

    # SD006 T1 + SD007 T3: lidar driver; TF from URDF or fallback, not both.
    lidar_layer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mentorpi_bringup"),
                        "launch",
                        "lidar_layer.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "enable_lidar": enable_lidar,
            "enable_robot_model": enable_robot_model,
        }.items(),
    )

    # SD029: UDPv4 8 MB socket buffer for camera frames (all stage1 nodes inherit via env).
    bringup_share = get_package_share_directory("mentorpi_bringup")
    fastdds_camera_frames = os.path.join(bringup_share, "config", "fastdds_camera_frames.xml")
    if os.path.isfile(fastdds_camera_frames):
        fastdds_profile_action = SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_camera_frames)
    else:
        fastdds_profile_action = LogInfo(msg=f"[stage1] fastdds profile missing: {fastdds_camera_frames}")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
                description="start SD006 lidar layer (vendor hiwonder_peripherals)",
            ),
            DeclareLaunchArgument(
                "enable_robot_model",
                default_value="true",
                description="start SD007 robot model layer (robot_state_publisher)",
            ),
            DeclareLaunchArgument(
                "enable_depth_camera",
                default_value="true",
                description="start SD008 camera layer (vendor deptrum Aurora 930)",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="start SD010 IMU layer (vendor hiwonder_peripherals)",
            ),
            DeclareLaunchArgument(
                "enable_voice",
                default_value="true",
                description="start SD031 voice_command (command_dispatcher always runs)",
            ),
            # After .hiwonderrc (JetRover_Mecanum / stale Dabai). Overlay is T1 tank.
            SetEnvironmentVariable("MACHINE_TYPE", "MentorPi_Tank"),
            SetEnvironmentVariable("DEPTH_CAMERA_TYPE", "aurora"),
            fastdds_profile_action,
            control_state,
            pad_teleop,
            linux_joy,
            control_mux,
            ros_robot_controller,
            odom_publisher,
            platform_adapter,
            robot_model_layer_launch,
            imu_layer_launch,
            lidar_layer_launch,
            camera_layer_launch,
            person_perception,
            person_detect_pi,
            mission_control,
            command_dispatcher,
            voice_command,
            motion_control,
            obstacle_guard,
        ]
    )
