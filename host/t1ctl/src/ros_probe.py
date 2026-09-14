#!/usr/bin/env python3
"""In-container ROS state collector for t1ctl status (SD009)."""

import sys
import time

# imu_calib holds /imu for ~2s (gyro bias). Apt complementary_filter_node
# publishes RELIABLE; sensor_data is BEST_EFFORT. Dual subscribe + this
# window so t1ctl status right after restart is not a false "no /imu".
PROBE_SEC = 5.0
BASE_FRAME = "base_footprint"
LIDAR_FRAME = "lidar_frame"
MODEL_BASE_FRAME = "base_link"
MODEL_IMU_FRAME = "imu_link"
MODEL_DEPTH_FRAME = "depth_cam_frame"
CAMERA_FRAME = "depth_camera_link"
ODOM_FRAME = "odom"
IMU_TOPIC = "/imu"
IMU_ODOM_TOPIC = "/imu_odom"
ODOM_RAW_TOPIC = "/odom_raw"


def _calibration_token():
    try:
        from mentorpi_calibration.calibration_file import load, operator_source

        return operator_source(load())
    except Exception:
        return None


def _print_results(
    chassis,
    lidar_scan,
    lidar_tf,
    camera_color,
    camera_depth,
    camera_tf,
    imu_msg,
    imu_odom,
    imu_tf,
    odom_msg,
    odom_tf,
    model_description,
    model_tf_base,
    model_tf_lidar,
    model_tf_imu,
    model_tf_depth,
    control_state=None,
    control_remote=None,
    control_reason=None,
    calibration=None,
):
    lidar = 1 if lidar_scan and lidar_tf else 0
    camera = 1 if camera_color and camera_depth and camera_tf else 0
    imu = 1 if imu_msg and imu_odom and imu_tf else 0
    odom = 1 if odom_msg and odom_tf else 0
    model = 1 if model_description and model_tf_base and model_tf_lidar and model_tf_imu and model_tf_depth else 0
    print(f"T1CTL_CHASSIS={1 if chassis else 0}")
    print(f"T1CTL_LIDAR={lidar}")
    print(f"T1CTL_LIDAR_SCAN={1 if lidar_scan else 0}")
    print(f"T1CTL_LIDAR_TF={1 if lidar_tf else 0}")
    print(f"T1CTL_CAMERA={camera}")
    print(f"T1CTL_CAMERA_COLOR={1 if camera_color else 0}")
    print(f"T1CTL_CAMERA_DEPTH={1 if camera_depth else 0}")
    print(f"T1CTL_CAMERA_TF={1 if camera_tf else 0}")
    print(f"T1CTL_IMU={imu}")
    print(f"T1CTL_IMU_MSG={1 if imu_msg else 0}")
    print(f"T1CTL_IMU_ODOM={1 if imu_odom else 0}")
    print(f"T1CTL_IMU_TF={1 if imu_tf else 0}")
    print(f"T1CTL_ODOM={odom}")
    print(f"T1CTL_ODOM_MSG={1 if odom_msg else 0}")
    print(f"T1CTL_ODOM_TF={1 if odom_tf else 0}")
    print(f"T1CTL_MODEL={model}")
    print(f"T1CTL_MODEL_DESCRIPTION={1 if model_description else 0}")
    print(f"T1CTL_MODEL_TF_BASE={1 if model_tf_base else 0}")
    print(f"T1CTL_MODEL_TF_LIDAR={1 if model_tf_lidar else 0}")
    print(f"T1CTL_MODEL_TF_IMU={1 if model_tf_imu else 0}")
    print(f"T1CTL_MODEL_TF_DEPTH={1 if model_tf_depth else 0}")
    if calibration in ("factory", "file", "unused"):
        print(f"T1CTL_CALIB={calibration}")
    if control_state is not None and control_remote is not None:
        print(f"state: {control_state}")
        print(f"remote_controller: {'true' if control_remote else 'false'}")
        print(f"reason: {control_reason or ''}")


def _print_all_inactive():
    _print_results(
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        calibration=_calibration_token(),
    )


def main():
    try:
        import logging

        import rclpy
        import tf2_ros
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener

        from mentorpi_msgs.msg import ChassisStatus, ControlStatus
    except ImportError:
        _print_all_inactive()
        return

    logging.getLogger("rclpy").setLevel(logging.FATAL)

    chassis = False
    lidar_scan = False
    camera_color = False
    camera_depth = False
    imu_msg = False
    imu_odom = False
    odom_msg = False
    model_description = False
    control_state = None
    control_remote = None
    control_reason = None

    tf_ok = {
        LIDAR_FRAME: False,
        MODEL_BASE_FRAME: False,
        MODEL_IMU_FRAME: False,
        MODEL_DEPTH_FRAME: False,
        CAMERA_FRAME: False,
    }
    odom_tf = False

    rclpy.init(args=None)

    class Probe(Node):
        def __init__(self):
            super().__init__("t1ctl_ros_probe")
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)

            chassis_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
            latched_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )

            self.create_subscription(ChassisStatus, "/vehicle/status", self._on_chassis, chassis_qos)
            self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
            self.create_subscription(
                Image,
                "/aurora/rgb/image_raw",
                self._on_color,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                "/aurora/rgb/image_raw/compressed",
                self._on_color,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                "/aurora/depth/image_raw",
                self._on_depth,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PointCloud2,
                "/aurora/points2",
                self._on_depth,
                qos_profile_sensor_data,
            )
            imu_reliable_qos = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.create_subscription(Imu, IMU_TOPIC, self._on_imu, qos_profile_sensor_data)
            self.create_subscription(Imu, IMU_TOPIC, self._on_imu, imu_reliable_qos)
            self.create_subscription(
                Odometry,
                IMU_ODOM_TOPIC,
                self._on_imu_odom,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Odometry,
                ODOM_RAW_TOPIC,
                self._on_odom_raw,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                "/robot_description",
                self._on_robot_description,
                latched_qos,
            )
            self.create_subscription(ControlStatus, "/control/status", self._on_control, latched_qos)

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        def _on_chassis(self, _msg):
            nonlocal chassis
            chassis = True

        def _on_scan(self, _msg):
            nonlocal lidar_scan
            lidar_scan = True

        def _on_color(self, _msg):
            nonlocal camera_color
            camera_color = True

        def _on_depth(self, _msg):
            nonlocal camera_depth
            camera_depth = True

        def _on_imu(self, _msg):
            nonlocal imu_msg
            imu_msg = True

        def _on_imu_odom(self, _msg):
            nonlocal imu_odom
            imu_odom = True

        def _on_odom_raw(self, _msg):
            nonlocal odom_msg
            odom_msg = True

        def _on_robot_description(self, msg):
            nonlocal model_description
            if "<robot" in msg.data:
                model_description = True

        def _on_control(self, msg):
            nonlocal control_state, control_remote, control_reason
            control_state = msg.state
            control_remote = msg.remote_controller
            control_reason = msg.reason

        def lookup_tf(self, frame):
            if tf_ok[frame]:
                return True
            try:
                self.tf_buffer.lookup_transform(BASE_FRAME, frame, rclpy.time.Time())
                tf_ok[frame] = True
                return True
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                return False

        def lookup_odom_tf(self):
            nonlocal odom_tf
            if odom_tf:
                return True
            try:
                self.tf_buffer.lookup_transform(ODOM_FRAME, BASE_FRAME, rclpy.time.Time())
                odom_tf = True
                return True
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                return False

    node = Probe()
    deadline = time.monotonic() + PROBE_SEC
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        for frame in tf_ok:
            node.lookup_tf(frame)
        node.lookup_odom_tf()
        if (
            chassis
            and camera_color
            and camera_depth
            and tf_ok[CAMERA_FRAME]
            and imu_msg
            and imu_odom
            and tf_ok[MODEL_IMU_FRAME]
            and odom_msg
            and odom_tf
            and control_state is not None
        ):
            break

    node.destroy_node()
    rclpy.shutdown()

    _print_results(
        chassis,
        lidar_scan,
        tf_ok[LIDAR_FRAME],
        camera_color,
        camera_depth,
        tf_ok[CAMERA_FRAME],
        imu_msg,
        imu_odom,
        tf_ok[MODEL_IMU_FRAME],
        odom_msg,
        odom_tf,
        model_description,
        tf_ok[MODEL_BASE_FRAME],
        tf_ok[LIDAR_FRAME],
        tf_ok[MODEL_IMU_FRAME],
        tf_ok[MODEL_DEPTH_FRAME],
        control_state,
        control_remote,
        control_reason,
        _calibration_token(),
    )


if __name__ == "__main__":
    try:
        main()
    except Exception:
        _print_all_inactive()
        sys.exit(0)
