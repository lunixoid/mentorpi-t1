#!/usr/bin/env python3
"""In-container SetControlMode helper for t1ctl mode (SD011)."""

import os
import sys
import time

WAIT_SEC = 5.0
SERVICE_NAME = "/control/set_mode"
STATUS_TOPIC = "/control/status"


def _print_ok(frm, to, reason):
    print("T1CTL_MODE_OK=1")
    if frm is not None:
        print(f"from: {frm}")
    print(f"to: {to}")
    print(f"reason: {reason or ''}")


def _print_fail(*details):
    print("T1CTL_MODE_OK=0")
    for detail in details:
        if detail:
            print(f"detail: {detail}")


def _parse_target():
    raw = os.environ.get("T1CTL_MODE_TARGET", "")
    try:
        target = int(raw)
    except ValueError:
        return None
    if target not in (0, 1, 2):
        return None
    return target


def main():
    target = _parse_target()
    if target is None:
        _print_fail("invalid target_state")
        return 1

    try:
        import logging

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        from mentorpi_msgs.msg import ControlStatus
        from mentorpi_msgs.srv import SetControlMode
    except ImportError:
        _print_fail("rclpy or mentorpi_msgs is not available")
        return 1

    logging.getLogger("rclpy").setLevel(logging.FATAL)
    rclpy.init(args=None)

    current_state = {"value": None}

    class ModeHelper(Node):
        def __init__(self):
            super().__init__("t1ctl_ros_mode")
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
            status_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.create_subscription(ControlStatus, STATUS_TOPIC, self._on_status, status_qos)
            self.client = self.create_client(SetControlMode, SERVICE_NAME)

        def _on_status(self, msg):
            current_state["value"] = msg.state

    node = ModeHelper()
    deadline = time.monotonic() + WAIT_SEC
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.client.service_is_ready() and current_state["value"] is not None:
            break

    if not node.client.service_is_ready():
        _print_fail("service /control/set_mode is not available")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    request = SetControlMode.Request()
    request.target_state = target
    future = node.client.call_async(request)
    call_deadline = time.monotonic() + WAIT_SEC
    while time.monotonic() < call_deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)

    if not future.done():
        _print_fail("service /control/set_mode timed out")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    try:
        response = future.result()
    except Exception as exc:
        _print_fail(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.destroy_node()
    rclpy.shutdown()

    if response is None or not response.success:
        _print_fail("SetControlMode rejected")
        return 1

    _print_ok(current_state["value"], response.active_state, response.reason)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        _print_fail(str(exc))
        sys.exit(1)
