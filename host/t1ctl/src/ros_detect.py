#!/usr/bin/env python3
"""In-container detect-source helper for t1ctl detect (SD026)."""

import os
import sys
import time

WAIT_SEC = 5.0
PERCEPTION_NODE = "person_perception"
DETECT_NODE = "person_detect_pi"
MOTION_NODE = "motion_control"
SOURCE_PARAM = "detections_source"
ENABLED_PARAM = "enabled"


def _node_services(node_name):
    return (
        f"/{node_name}/get_parameters",
        f"/{node_name}/set_parameters",
    )


def _print_ok(source):
    print("T1CTL_DETECT_OK=1")
    print(f"source: {source}")


def _print_fail(*details):
    print("T1CTL_DETECT_OK=0")
    for detail in details:
        if detail:
            print(f"detail: {detail}")


def _parse_action():
    raw = os.environ.get("T1CTL_DETECT_ACTION", "").strip().lower()
    if raw in ("status", "offline", "mac"):
        return raw
    return None


def _source_from_perception(value):
    if value == "offline":
        return "offline"
    if value == "mac":
        return "mac"
    return None


def main():
    action = _parse_action()
    if action is None:
        _print_fail("invalid detect action")
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

    class DetectHelper(Node):
        def __init__(self):
            super().__init__("t1ctl_ros_detect")
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
            self.perception_get_srv, self.perception_set_srv = _node_services(PERCEPTION_NODE)
            self.detect_get_srv, self.detect_set_srv = _node_services(DETECT_NODE)
            self.motion_set_srv = f"/{MOTION_NODE}/set_parameters"
            self.perception_get = self.create_client(GetParameters, self.perception_get_srv)
            self.perception_set = self.create_client(SetParameters, self.perception_set_srv)
            self.detect_get = self.create_client(GetParameters, self.detect_get_srv)
            self.detect_set = self.create_client(SetParameters, self.detect_set_srv)
            self.motion_set = self.create_client(SetParameters, self.motion_set_srv)

    node = DetectHelper()
    clients = (
        node.perception_get,
        node.perception_set,
        node.detect_get,
        node.detect_set,
        node.motion_set,
    )
    deadline = time.monotonic() + WAIT_SEC
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if all(client.service_is_ready() for client in clients):
            break

    missing = []
    if not node.perception_get.service_is_ready() or not node.perception_set.service_is_ready():
        missing.append("node /person_perception is not available")
    if not node.detect_get.service_is_ready() or not node.detect_set.service_is_ready():
        missing.append("node /person_detect_pi is not available")
    if not node.motion_set.service_is_ready():
        missing.append("node /motion_control is not available")
    if missing:
        _print_fail(*missing)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    def call_get(client, names):
        request = GetParameters.Request()
        request.names = list(names)
        future = client.call_async(request)
        call_deadline = time.monotonic() + WAIT_SEC
        while time.monotonic() < call_deadline and not future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            return None, "get_parameters timed out"
        try:
            response = future.result()
        except Exception as exc:
            return None, str(exc)
        if response is None or len(response.values) != len(names):
            return None, "get_parameters returned incomplete values"
        return response.values, None

    def read_source():
        values, err = call_get(node.perception_get, [SOURCE_PARAM])
        if err is not None:
            return None, err
        value = values[0]
        if value.type != ParameterType.PARAMETER_STRING:
            return None, f"{SOURCE_PARAM} is not string"
        source = _source_from_perception(value.string_value)
        if source is None:
            return None, f"{SOURCE_PARAM} has invalid value"
        return source, None

    def read_enabled():
        values, err = call_get(node.detect_get, [ENABLED_PARAM])
        if err is not None:
            return None, err
        value = values[0]
        if value.type != ParameterType.PARAMETER_BOOL:
            return None, f"{ENABLED_PARAM} is not bool"
        return bool(value.bool_value), None

    def set_param(client, name, param_type, **fields):
        param = ParamMsg()
        param.name = name
        param.value = ParameterValue()
        param.value.type = param_type
        if param_type == ParameterType.PARAMETER_BOOL:
            param.value.bool_value = fields["bool_value"]
        elif param_type == ParameterType.PARAMETER_STRING:
            param.value.string_value = fields["string_value"]
        else:
            return False, f"unsupported parameter type for {name}"
        request = SetParameters.Request()
        request.parameters = [param]
        future = client.call_async(request)
        call_deadline = time.monotonic() + WAIT_SEC
        while time.monotonic() < call_deadline and not future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            return False, "set_parameters timed out"
        try:
            response = future.result()
        except Exception as exc:
            return False, str(exc)
        if response is None or not response.results:
            return False, "set_parameters rejected"
        result = response.results[0]
        if not result.successful:
            return False, result.reason or "set_parameters rejected"
        return True, None

    if action == "status":
        source, err = read_source()
        if err is not None:
            _print_fail(err)
            node.destroy_node()
            rclpy.shutdown()
            return 1
        _print_ok(source)
        node.destroy_node()
        rclpy.shutdown()
        return 0

    if action == "offline":
        ok, err = set_param(node.detect_set, ENABLED_PARAM, ParameterType.PARAMETER_BOOL, bool_value=True)
        if not ok:
            _print_fail(err)
            node.destroy_node()
            rclpy.shutdown()
            return 1
        ok, err = set_param(
            node.perception_set,
            SOURCE_PARAM,
            ParameterType.PARAMETER_STRING,
            string_value="offline",
        )
        if not ok:
            rollback_ok, rollback_err = set_param(
                node.detect_set, ENABLED_PARAM, ParameterType.PARAMETER_BOOL, bool_value=False
            )
            detail = err
            if not rollback_ok:
                detail = f"{err}\nrollback enabled=false failed: {rollback_err}"
            else:
                detail = f"{err}\nrolled back enabled=false"
            _print_fail(detail)
            node.destroy_node()
            rclpy.shutdown()
            return 1
        ok, err = set_param(
            node.motion_set,
            SOURCE_PARAM,
            ParameterType.PARAMETER_STRING,
            string_value="offline",
        )
        if not ok:
            rollback_src_ok, rollback_src_err = set_param(
                node.perception_set,
                SOURCE_PARAM,
                ParameterType.PARAMETER_STRING,
                string_value="mac",
            )
            rollback_en_ok, rollback_en_err = set_param(
                node.detect_set, ENABLED_PARAM, ParameterType.PARAMETER_BOOL, bool_value=False
            )
            detail = err
            if not rollback_src_ok:
                detail = f"{err}\nrollback detections_source=mac failed: {rollback_src_err}"
            if not rollback_en_ok:
                detail = f"{detail}\nrollback enabled=false failed: {rollback_en_err}"
            _print_fail(detail)
            node.destroy_node()
            rclpy.shutdown()
            return 1
        _print_ok("offline")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    ok, err = set_param(
        node.perception_set,
        SOURCE_PARAM,
        ParameterType.PARAMETER_STRING,
        string_value="mac",
    )
    if not ok:
        _print_fail(err)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    ok, err = set_param(
        node.motion_set,
        SOURCE_PARAM,
        ParameterType.PARAMETER_STRING,
        string_value="mac",
    )
    if not ok:
        _print_fail(err)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    ok, err = set_param(node.detect_set, ENABLED_PARAM, ParameterType.PARAMETER_BOOL, bool_value=False)
    if not ok:
        detail = err
        _print_fail(detail)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    _print_ok("mac")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        _print_fail(str(exc))
        sys.exit(1)
