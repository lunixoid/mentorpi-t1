#!/usr/bin/env python3
"""Native macOS person detector (SD013 T2, SD017 T1).

Subscribes to Aurora RGB, runs official Ultralytics YOLO11n track (ByteTrack,
persist) on MPS (with NMS fallback), publishes vision_msgs/Detection2DArray
with ByteTrack id in Detection2D.id. Does not publish PersonArray, overlay,
Twist, or keypoints.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

# torchvision::nms is not implemented on MPS; rest of YOLO11n detect stays on Metal.
os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from person_class import PERSON_CLASS, resolve_person_class_ids
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

DETECTIONS_TOPIC = "/perception/detections_2d"
DDS_PEER_TOPIC = "/perception/dds_peer"
DEFAULT_IMAGE_TOPIC = "/aurora/rgb/image_raw"
DEFAULT_WEIGHTS = "yolo11n.pt"
DEFAULT_CONFIDENCE = 0.25
TRACKER_YAML = os.path.join(os.path.dirname(__file__), "bytetrack.yaml")


def _require_native_mps() -> str:
    if sys.platform != "darwin":
        raise SystemExit("person_detect is native macOS only (not Docker linux/arm64)")
    if not torch.backends.mps.is_available():
        raise SystemExit("MPS is unavailable; YOLO11n detect must run on Apple Silicon")
    return "mps"


def _image_to_bgr(msg: Image, bridge: CvBridge) -> np.ndarray:
    if msg.encoding in ("bgr8", "rgb8"):
        channels = 3
        row_bytes = msg.width * channels
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.step == row_bytes:
            img = buf.reshape((msg.height, msg.width, channels))
        else:
            img = buf.reshape((msg.height, msg.step))[:, :row_bytes]
            img = img.reshape((msg.height, msg.width, channels))
        if msg.encoding == "rgb8":
            return img[:, :, ::-1].copy()
        return np.ascontiguousarray(img)
    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def _fill_bbox_center(bbox, cx: float, cy: float) -> None:
    center = bbox.center
    position = getattr(center, "position", None)
    if position is not None:
        position.x = cx
        position.y = cy
    else:
        center.x = cx
        center.y = cy
    center.theta = 0.0


def _is_person(model: YOLO, cls_id: int) -> bool:
    names = getattr(model, "names", {}) or {}
    name = names.get(int(cls_id), names.get(str(cls_id), ""))
    return name == PERSON_CLASS


class PersonDetectNode(Node):
    def __init__(
        self,
        image_topic: str,
        weights: str,
        confidence_threshold: float,
        device: str,
    ) -> None:
        super().__init__("person_detect")
        self._bridge = CvBridge()
        self._confidence = confidence_threshold
        self._device = device
        self._model = YOLO(weights)
        model_names = dict(getattr(self._model, "names", {}) or {})
        person_ids = resolve_person_class_ids(model_names)
        if not person_ids:
            raise SystemExit(f"weights {weights!r}: no class {PERSON_CLASS!r} in " f"model.names={model_names}")
        self._person_class_ids = person_ids
        self._logged_first_rgb = False
        self._dds_peer: Optional[str] = None

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        peer_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(Detection2DArray, DETECTIONS_TOPIC, pub_qos)
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(String, DDS_PEER_TOPIC, self._on_dds_peer, peer_qos)
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        self.get_logger().info(
            f"person_detect domain={domain} image={image_topic} "
            f"out={DETECTIONS_TOPIC} weights={weights} "
            f"person_ids={person_ids} names={model_names} "
            f"conf={confidence_threshold} device={device}"
        )
        self._wait_timer = self.create_timer(5.0, self._warn_if_no_rgb)

    def _on_dds_peer(self, msg: String) -> None:
        addr = msg.data
        if addr == self._dds_peer:
            return
        self._dds_peer = addr
        self.get_logger().info(f"{DDS_PEER_TOPIC} {addr}")

    def _warn_if_no_rgb(self) -> None:
        if self._logged_first_rgb:
            self._wait_timer.cancel()
            return
        self.get_logger().warn("no RGB yet: no publisher on this DDS graph (check Pi LOCALHOST=0)")

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = _image_to_bgr(msg, self._bridge)
        except Exception as exc:
            self.get_logger().error(f"RGB convert failed: {exc}")
            return
        if not self._logged_first_rgb:
            self._logged_first_rgb = True
            self.get_logger().info(f"first RGB {msg.width}x{msg.height} encoding={msg.encoding}")
        out = Detection2DArray()
        out.header = msg.header
        try:
            results = self._model.track(
                source=bgr,
                conf=self._confidence,
                device=self._device,
                classes=self._person_class_ids,
                persist=True,
                tracker=TRACKER_YAML,
                verbose=False,
                save=False,
            )
        except Exception as exc:
            self.get_logger().error(f"YOLO11n track failed: {exc}")
            self._pub.publish(out)
            return
        if results:
            boxes = results[0].boxes
            if boxes is not None:
                for box in boxes:
                    if box.id is None:
                        continue
                    cls_id = int(box.cls[0].detach().cpu())
                    if not _is_person(self._model, cls_id):
                        continue
                    x1, y1, x2, y2 = box.xyxy[0].detach().cpu().numpy().tolist()
                    det = Detection2D()
                    det.header = msg.header
                    det.id = str(int(box.id[0].detach().cpu()))
                    _fill_bbox_center(det.bbox, (x1 + x2) / 2.0, (y1 + y2) / 2.0)
                    det.bbox.size_x = float(max(0.0, x2 - x1))
                    det.bbox.size_y = float(max(0.0, y2 - y1))
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = PERSON_CLASS
                    hyp.hypothesis.score = float(box.conf[0].detach().cpu())
                    det.results.append(hyp)
                    out.detections.append(det)
        self._pub.publish(out)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO11n detect person_detect (macOS MPS, not Docker)")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--weights", default=DEFAULT_WEIGHTS)
    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=DEFAULT_CONFIDENCE,
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    device = _require_native_mps()
    rclpy.init(args=None)
    node = PersonDetectNode(
        image_topic=args.image_topic,
        weights=args.weights,
        confidence_threshold=args.confidence_threshold,
        device=device,
    )
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
