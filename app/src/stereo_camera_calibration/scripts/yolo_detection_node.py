#!/usr/bin/env python3

import os
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO, YOLOE
from visualization_msgs.msg import Marker, MarkerArray


class YoloDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detection_node")

        model_path = self.declare_parameter("model", "yolo11n.pt").value
        self.confidence = float(self.declare_parameter("confidence", 0.4).value)
        self.image_size = int(self.declare_parameter("imgsz", 640).value)
        self.device = str(self.declare_parameter("device", "0").value)
        self.classes = list(self.declare_parameter("classes", [32]).value)
        self.prompt = str(self.declare_parameter("prompt", "").value)
        self.minimum_depth = float(self.declare_parameter("minimum_depth", 0.15).value)
        self.maximum_depth = float(self.declare_parameter("maximum_depth", 2.0).value)
        # Manual base_link vertical adjustment for stationary calibration tests.
        self.base_vertical_offset = float(
            self.declare_parameter("base_vertical_offset", 0.0).value
        )
        if not np.isfinite(self.base_vertical_offset):
            raise ValueError("base_vertical_offset must be finite")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ball_diameter = float(self.declare_parameter("ball_diameter", 0.04).value)
        self.show_window = bool(self.declare_parameter("show_window", True).value)
        self.publish_annotated_image = bool(
            self.declare_parameter("publish_annotated_image", True).value
        )
        self.publish_metrics = bool(
            self.declare_parameter("publish_metrics", False).value
        )

        self.bridge = CvBridge()
        previous_directory = os.getcwd()
        resolved_model_path = os.path.abspath(os.path.expanduser(model_path))
        try:
            if os.path.isfile(resolved_model_path):
                os.chdir(os.path.dirname(resolved_model_path))
                model_path = os.path.basename(resolved_model_path)
            if self.prompt:
                self.model = YOLOE(model_path)
                self.model.set_classes([self.prompt])
            else:
                self.model = YOLO(model_path)
        finally:
            os.chdir(previous_directory)
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
        self.metrics_publisher = self.create_publisher(
            DiagnosticArray, "/pipeline/metrics", sensor_qos
        )

        self.get_logger().info(
            f"Loaded {model_path} on CUDA device {self.device}; "
            f"target: {self.prompt or self.classes or 'all'}; "
            f"imgsz: {self.image_size}; waiting for stereo topics"
        )

    def store_camera_info(self, message: CameraInfo) -> None:
        self.depth_intrinsics = (message.k[0], message.k[4], message.k[2], message.k[5])

    def process_frame(self, image_message: Image, depth_message: Image) -> None:
        callback_start = time.perf_counter()
        callback_ros_time = self.get_clock().now()
        if self.show_window and not self.window_initialized:
            cv2.namedWindow("YOLO detections", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("YOLO detections", 1280, 720)
            self.window_initialized = True
            self.get_logger().info("Receiving stereo frames; press 'q' or Escape to stop")

        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="32FC1")
        conversion_end = time.perf_counter()

        predict_options = {
            "source": image,
            "conf": self.confidence,
            "imgsz": self.image_size,
            "device": self.device,
            "verbose": False,
        }
        if not self.prompt:
            predict_options["classes"] = self.classes or None
        inference_start = time.perf_counter()
        result = self.model.predict(**predict_options)[0]
        inference_end = time.perf_counter()
        publish_annotated = (
            self.publish_annotated_image
            and self.annotated_image_publisher.get_subscription_count() > 0
        )
        needs_annotated = self.show_window or publish_annotated
        annotated = result.plot() if needs_annotated else None

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

                # YOLOE provides a segmentation mask. It avoids mixing the ball
                # depth with the background, which is common with a small object.
                depth_mask = np.zeros((depth_height, depth_width), dtype=np.uint8)
                if result.masks is not None and detection_index < len(result.masks.xy):
                    polygon = result.masks.xy[detection_index].copy()
                    if polygon.size:
                        polygon[:, 0] *= scale_x
                        polygon[:, 1] *= scale_y
                        polygon = np.rint(polygon).astype(np.int32)
                        cv2.fillPoly(depth_mask, [polygon], 1)
                        eroded = cv2.erode(depth_mask, np.ones((3, 3), np.uint8))
                        if np.count_nonzero(eroded) >= 8:
                            depth_mask = eroded

                # Detection-only models do not expose a mask. Keep the inner
                # part of their bounding box as a fallback.
                if not np.any(depth_mask):
                    margin_x = 0.2 * (x2 - x1)
                    margin_y = 0.2 * (y2 - y1)
                    depth_x1 = int(
                        np.clip((x1 + margin_x) * scale_x, 0, depth_width - 1)
                    )
                    depth_y1 = int(
                        np.clip((y1 + margin_y) * scale_y, 0, depth_height - 1)
                    )
                    depth_x2 = int(
                        np.clip(
                            (x2 - margin_x) * scale_x,
                            depth_x1 + 1,
                            depth_width,
                        )
                    )
                    depth_y2 = int(
                        np.clip(
                            (y2 - margin_y) * scale_y,
                            depth_y1 + 1,
                            depth_height,
                        )
                    )
                    depth_mask[depth_y1:depth_y2, depth_x1:depth_x2] = 1

                valid_mask = (
                    depth_mask.astype(bool)
                    & np.isfinite(depth)
                    & (depth >= self.minimum_depth)
                    & (depth <= self.maximum_depth)
                )
                valid = depth[valid_mask]

                label = "Depth: unavailable"
                if valid.size:
                    median_depth = float(np.median(valid))
                    label = f"Depth: {median_depth:.3f} m"

                    if self.depth_intrinsics is not None:
                        fx, fy, cx, cy = self.depth_intrinsics
                        mask_y, mask_x = np.nonzero(depth_mask)
                        center_x = float(np.median(mask_x))
                        center_y = float(np.median(mask_y))
                        point_x = (center_x - cx) * median_depth / fx
                        point_y = (center_y - cy) * median_depth / fy
                        point_z = median_depth
                        if self.base_vertical_offset != 0.0:
                            try:
                                transform = self.tf_buffer.lookup_transform(
                                    depth_message.header.frame_id,
                                    "base_link",
                                    rclpy.time.Time.from_msg(depth_message.header.stamp),
                                    timeout=Duration(seconds=0.1),
                                )
                            except TransformException as error:
                                self.get_logger().warning(
                                    f"Cannot apply vertical detection offset: {error}",
                                    throttle_duration_sec=2.0,
                                )
                                continue
                            q = transform.transform.rotation
                            # Rotate a vector, not a point: camera translation
                            # must not be added to the requested base-Z offset.
                            delta = self.base_vertical_offset * np.array([
                                2.0 * (q.x * q.z + q.w * q.y),
                                2.0 * (q.y * q.z - q.w * q.x),
                                1.0 - 2.0 * (q.x * q.x + q.y * q.y),
                            ])
                            point_x += float(delta[0])
                            point_y += float(delta[1])
                            point_z += float(delta[2])

                        sphere = Marker()
                        sphere.header = depth_message.header
                        sphere.ns = "detected_objects"
                        sphere.id = detection_index * 2
                        sphere.type = Marker.SPHERE
                        sphere.action = Marker.ADD
                        sphere.pose.position.x = float(point_x)
                        sphere.pose.position.y = float(point_y)
                        sphere.pose.position.z = point_z
                        sphere.pose.orientation.w = 1.0
                        sphere.scale.x = self.ball_diameter
                        sphere.scale.y = self.ball_diameter
                        sphere.scale.z = self.ball_diameter
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
                        text.pose.position.z = point_z
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
                if annotated is not None:
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

        if publish_annotated:
            annotated_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_message.header = image_message.header
            self.annotated_image_publisher.publish(annotated_message)
        self.marker_publisher.publish(markers)
        publish_end = time.perf_counter()

        if self.publish_metrics:
            capture_time = rclpy.time.Time.from_msg(image_message.header.stamp)
            metrics = {
                "queue_age_ms": max(
                    0.0,
                    (callback_ros_time - capture_time).nanoseconds / 1_000_000.0,
                ),
                "conversion_ms": (conversion_end - callback_start) * 1000.0,
                "inference_wall_ms": (inference_end - inference_start) * 1000.0,
                "annotate_and_3d_ms": (publish_end - inference_end) * 1000.0,
                "total_callback_ms": (publish_end - callback_start) * 1000.0,
                "capture_to_detection_ms": max(
                    0.0,
                    (self.get_clock().now() - capture_time).nanoseconds / 1_000_000.0,
                ),
                "detection_count": float(
                    sum(marker.ns == "detected_objects" for marker in markers.markers)
                ),
            }
            for name, value in result.speed.items():
                metrics[f"ultralytics_{name}_ms"] = float(value)
            self.publish_timing(image_message.header, metrics)

        if self.show_window:
            cv2.imshow("YOLO detections", annotated)
            key = cv2.waitKey(1)
            if key in (ord("q"), 27):
                rclpy.shutdown()

    def publish_timing(self, header, metrics: dict[str, float]) -> None:
        message = DiagnosticArray()
        message.header = header
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "yolo"
        status.message = "frame timings"
        status.hardware_id = "cuda"
        status.values = [
            KeyValue(key=name, value=f"{value:.6f}")
            for name, value in metrics.items()
        ]
        message.status = [status]
        self.metrics_publisher.publish(message)

    def destroy_node(self) -> None:
        if self.window_initialized:
            cv2.destroyAllWindows()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = YoloDetectionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
