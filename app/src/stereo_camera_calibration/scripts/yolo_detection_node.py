#!/usr/bin/env python3

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YoloDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detection_node")

        model_path = self.declare_parameter("model", "yolo11n.pt").value
        self.confidence = float(self.declare_parameter("confidence", 0.4).value)
        self.device = str(self.declare_parameter("device", "0").value)
        self.classes = list(self.declare_parameter("classes", [32]).value)
        self.minimum_depth = float(self.declare_parameter("minimum_depth", 0.15).value)
        self.maximum_depth = float(self.declare_parameter("maximum_depth", 2.0).value)

        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_subscriber = message_filters.Subscriber(
            self, Image, "/stereo/left/image_rect", qos_profile=sensor_qos
        )
        depth_subscriber = message_filters.Subscriber(
            self, Image, "/stereo/depth", qos_profile=sensor_qos
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [image_subscriber, depth_subscriber], queue_size=2, slop=0.02
        )
        self.synchronizer.registerCallback(self.process_frame)

        cv2.namedWindow("YOLO detections", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLO detections", 1280, 720)
        self.get_logger().info(
            f"Loaded {model_path} on CUDA device {self.device}; "
            f"class filter: {self.classes or 'all'}; press 'q' or Escape to stop"
        )

    def process_frame(self, image_message: Image, depth_message: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="32FC1")

        result = self.model.predict(
            source=image,
            conf=self.confidence,
            classes=self.classes or None,
            device=self.device,
            verbose=False,
        )[0]
        annotated = result.plot()

        image_height, image_width = image.shape[:2]
        depth_height, depth_width = depth.shape[:2]
        scale_x = depth_width / image_width
        scale_y = depth_height / image_height

        if result.boxes is not None:
            for box in result.boxes.xyxy.cpu().numpy():
                x1, y1, x2, y2 = box

                # Ignore the outer 20% of the detection to reduce background contamination.
                margin_x = 0.2 * (x2 - x1)
                margin_y = 0.2 * (y2 - y1)
                depth_x1 = int(np.clip((x1 + margin_x) * scale_x, 0, depth_width - 1))
                depth_y1 = int(np.clip((y1 + margin_y) * scale_y, 0, depth_height - 1))
                depth_x2 = int(np.clip((x2 - margin_x) * scale_x, depth_x1 + 1, depth_width))
                depth_y2 = int(np.clip((y2 - margin_y) * scale_y, depth_y1 + 1, depth_height))

                region = depth[depth_y1:depth_y2, depth_x1:depth_x2]
                valid = region[
                    np.isfinite(region)
                    & (region >= self.minimum_depth)
                    & (region <= self.maximum_depth)
                ]

                label = "Depth: unavailable"
                if valid.size:
                    label = f"Depth: {float(np.median(valid)):.3f} m"

                text_origin = (int(x1), max(25, int(y2) - 8))
                cv2.putText(
                    annotated,
                    label,
                    text_origin,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

        cv2.imshow("YOLO detections", annotated)
        key = cv2.waitKey(1)
        if key in (ord("q"), 27):
            rclpy.shutdown()

    def destroy_node(self) -> None:
        cv2.destroyAllWindows()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
