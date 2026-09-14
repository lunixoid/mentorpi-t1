#!/usr/bin/env python3
"""Echo /perception/detections_2d with the same DDS/QoS as person_detect (no ros2 daemon)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2DArray

TOPIC = "/perception/detections_2d"


class DetectionsEcho(Node):
    def __init__(self) -> None:
        super().__init__("detections_echo")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Detection2DArray, TOPIC, self._on_msg, qos)
        self.get_logger().info(f"waiting for {TOPIC}")

    def _on_msg(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            print("detections: []")
            return
        for i, det in enumerate(msg.detections):
            center = det.bbox.center
            position = getattr(center, "position", None)
            if position is not None:
                x, y = position.x, position.y
            else:
                x, y = center.x, center.y
            score = det.results[0].hypothesis.score if det.results else 0.0
            class_id = det.results[0].hypothesis.class_id if det.results else ""
            print(
                f"[{i}] id={det.id!r} {class_id} score={score:.3f} "
                f"center=({x:.1f},{y:.1f}) "
                f"size=({det.bbox.size_x:.1f},{det.bbox.size_y:.1f})"
            )
        print("---")


def main() -> None:
    rclpy.init()
    node = DetectionsEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
