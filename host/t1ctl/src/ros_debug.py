#!/usr/bin/env python3
"""In-container publish_overlay helper for t1ctl debug (SD014)."""

import os
import sys
import time

WAIT_SEC = 5.0
NODE_NAME = "person_perception"
PARAM_NAME = "publish_overlay"
GET_SRV = f"/{NODE_NAME}/get_parameters"
SET_SRV = f"/{NODE_NAME}/set_parameters"


def _print_ok(overlay_on):
    print("T1CTL_DEBUG_OK=1")
    print(f"overlay: {'on' if overlay_on else 'off'}")


def _print_fail(*details):
    print("T1CTL_DEBUG_OK=0")
    for detail in details:
        if detail:
            print(f"detail: {detail}")


def _parse_action():
    raw = os.environ.get("T1CTL_DEBUG_ACTION", "").strip().lower()
    if raw in ("status", "on", "off"):
        return raw
    return None


def main():
    action = _parse_action()
    if action is None:
        _print_fail("invalid debug action")
        return 1

    try:
        import logging

        import rclpy
        from rcl_interfaces.msg import Parameter as ParamMsg
        from rcl_interfaces.msg import ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters
        from rclpy.node import Node
    except ImportError:
        _print_fail("rclpy is not available")
        return 1

    logging.getLogger("rclpy").setLevel(logging.FATAL)
    rclpy.init(args=None)

    class DebugHelper(Node):
        def __init__(self):
            super().__init__("t1ctl_ros_debug")
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
            self.get_client = self.create_client(GetParameters, GET_SRV)
            self.set_client = self.create_client(SetParameters, SET_SRV)

    node = DebugHelper()
    deadline = time.monotonic() + WAIT_SEC
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.get_client.service_is_ready():
            break

    if not node.get_client.service_is_ready():
        _print_fail("node /person_perception is not available")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    def read_overlay():
        request = GetParameters.Request()
        request.names = [PARAM_NAME]
        future = node.get_client.call_async(request)
        call_deadline = time.monotonic() + WAIT_SEC
        while time.monotonic() < call_deadline and not future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            return None, "get_parameters timed out"
        try:
            response = future.result()
        except Exception as exc:
            return None, str(exc)
        if response is None or not response.values:
            return None, "publish_overlay is not set"
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_BOOL:
            return None, "publish_overlay is not bool"
        return bool(value.bool_value), None

    if action == "status":
        overlay_on, err = read_overlay()
        node.destroy_node()
        rclpy.shutdown()
        if err is not None:
            _print_fail(err)
            return 1
        _print_ok(overlay_on)
        return 0

    if not node.set_client.service_is_ready():
        set_deadline = time.monotonic() + WAIT_SEC
        while time.monotonic() < set_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.set_client.service_is_ready():
                break
    if not node.set_client.service_is_ready():
        _print_fail("node /person_perception is not available")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    overlay_on = action == "on"
    param = ParamMsg()
    param.name = PARAM_NAME
    param.value = ParameterValue()
    param.value.type = ParameterType.PARAMETER_BOOL
    param.value.bool_value = overlay_on
    request = SetParameters.Request()
    request.parameters = [param]
    future = node.set_client.call_async(request)
    call_deadline = time.monotonic() + WAIT_SEC
    while time.monotonic() < call_deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)

    if not future.done():
        _print_fail("set_parameters timed out")
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

    if response is None or not response.results:
        _print_fail("set_parameters rejected")
        return 1
    result = response.results[0]
    if not result.successful:
        _print_fail(result.reason or "set_parameters rejected")
        return 1

    _print_ok(overlay_on)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        _print_fail(str(exc))
        sys.exit(1)
