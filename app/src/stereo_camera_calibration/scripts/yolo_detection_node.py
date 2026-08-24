#!/usr/bin/env python3

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray


class YoloDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detection_node")

        model_path = self.declare_parameter("model", "yolo11n.pt").value
        self.confidence = float(self.declare_parameter("confidence", 0.4).value)
        self.device = str(self.declare_parameter("device", "0").value)
        self.classes = list(self.declare_parameter("classes", [32]).value)
        self.minimum_depth = float(self.declare_parameter("minimum_depth", 0.15).value)
        self.maximum_depth = float(self.declare_parameter("maximum_depth", 2.0).value)
        self.show_window = bool(self.declare_parameter("show_window", True).value)

        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.window_initialized = False
        self.depth_intrinsics = None

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
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "/stereo/depth/camera_info",
            self.store_camera_info,
            sensor_qos,
        )
        self.annotated_image_publisher = self.create_publisher(
            Image, "/yolo/image_annotated", sensor_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/yolo/markers", sensor_qos
        )

        self.get_logger().info(
            f"Loaded {model_path} on CUDA device {self.device}; "
            f"class filter: {self.classes or 'all'}; waiting for stereo topics"
        )

    def store_camera_info(self, message: CameraInfo) -> None:
        self.depth_intrinsics = (message.k[0], message.k[4], message.k[2], message.k[5])

    def process_frame(self, image_message: Image, depth_message: Image) -> None:
        if self.show_window and not self.window_initialized:
            cv2.namedWindow("YOLO detections", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("YOLO detections", 1280, 720)
            self.window_initialized = True
            self.get_logger().info("Receiving stereo frames; press 'q' or Escape to stop")

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

        markers = MarkerArray()
        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            confidences = result.boxes.conf.cpu().numpy()
            for detection_index, (box, class_id, confidence) in enumerate(
                zip(boxes, class_ids, confidences)
            ):
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
                    median_depth = float(np.median(valid))
                    label = f"Depth: {median_depth:.3f} m"

                    if self.depth_intrinsics is not None:
                        fx, fy, cx, cy = self.depth_intrinsics
                        center_x = 0.5 * (depth_x1 + depth_x2)
                        center_y = 0.5 * (depth_y1 + depth_y2)
                        point_x = (center_x - cx) * median_depth / fx
                        point_y = (center_y - cy) * median_depth / fy

                        sphere = Marker()
                        sphere.header = depth_message.header
                        sphere.ns = "detected_objects"
                        sphere.id = detection_index * 2
                        sphere.type = Marker.SPHERE
                        sphere.action = Marker.ADD
                        sphere.pose.position.x = float(point_x)
                        sphere.pose.position.y = float(point_y)
                        sphere.pose.position.z = median_depth
                        sphere.pose.orientation.w = 1.0
                        sphere.scale.x = 0.067
                        sphere.scale.y = 0.067
                        sphere.scale.z = 0.067
                        sphere.color.r = 0.1
                        sphere.color.g = 1.0
                        sphere.color.b = 0.1
                        sphere.color.a = 0.9
                        sphere.lifetime = Duration(seconds=0.25).to_msg()
                        markers.markers.append(sphere)

                        text = Marker()
                        text.header = depth_message.header
                        text.ns = "detected_object_labels"
                        text.id = detection_index * 2 + 1
                        text.type = Marker.TEXT_VIEW_FACING
                        text.action = Marker.ADD
                        text.pose.position.x = float(point_x)
                        text.pose.position.y = float(point_y - 0.06)
                        text.pose.position.z = median_depth
                        text.pose.orientation.w = 1.0
                        text.scale.z = 0.04
                        text.color.r = 1.0
                        text.color.g = 1.0
                        text.color.b = 1.0
                        text.color.a = 1.0
                        text.text = (
                            f"{self.model.names[class_id]} "
                            f"{confidence:.2f} | {median_depth:.3f} m"
                        )
                        text.lifetime = Duration(seconds=0.25).to_msg()
                        markers.markers.append(text)

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

        annotated_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_message.header = image_message.header
        self.annotated_image_publisher.publish(annotated_message)
        self.marker_publisher.publish(markers)

        if self.show_window:
            cv2.imshow("YOLO detections", annotated)
            key = cv2.waitKey(1)
            if key in (ord("q"), 27):
                rclpy.shutdown()

    def destroy_node(self) -> None:
        if self.window_initialized:
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
