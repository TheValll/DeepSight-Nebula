#!/usr/bin/env python3

from collections import deque
from copy import deepcopy
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class BallGoalPreviewNode(Node):
    """Preview a ball goal in RViz without planning or commanding the robot."""

    def __init__(self) -> None:
        super().__init__("ball_goal_preview_node")

        self.target_frame = str(self.declare_parameter("target_frame", "base_link").value)
        # Optional empirical base-frame offset; disabled until calibrated.
        # This is not a general sphere/depth fit.
        self.center_offset_z = float(
            self.declare_parameter("center_offset_z", 0.0).value
        )
        if not np.isfinite(self.center_offset_z):
            raise ValueError("center_offset_z must be finite")
        if self.center_offset_z != 0.0 and self.target_frame != "base_link":
            raise ValueError("Nonzero center_offset_z requires target_frame=base_link")
        self.allow_latest_transform = bool(
            self.declare_parameter("allow_latest_transform", False).value
        )
        self.publish_metrics = bool(
            self.declare_parameter("publish_metrics", False).value
        )
        sample_count = int(self.declare_parameter("sample_count", 10).value)
        self.positions = deque(maxlen=max(1, sample_count))
        self.position_times = deque(maxlen=max(1, sample_count))
        self.last_log_time = self.get_clock().now() - Duration(seconds=10.0)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.marker_subscription = self.create_subscription(
            MarkerArray, "/yolo/markers", self.process_markers, sensor_qos
        )
        self.pose_publisher = self.create_publisher(PoseStamped, "/ball_goal/pose", 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/ball_goal/markers", 10
        )
        self.metrics_publisher = self.create_publisher(
            DiagnosticArray, "/pipeline/metrics", sensor_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            "Passive preview enabled: publishes the detected ball goal without "
            "planning or sending robot commands"
        )

    def process_markers(self, message: MarkerArray) -> None:
        callback_start = time.perf_counter()
        balls = [
            marker
            for marker in message.markers
            if marker.ns == "detected_objects" and marker.type == Marker.SPHERE
        ]
        if not balls:
            return

        # If several balls are visible, preview the closest one to the camera.
        ball = min(balls, key=lambda marker: marker.pose.position.z)
        point = PointStamped()
        point.header = ball.header
        point.point = ball.pose.position

        transform_time = (
            rclpy.time.Time()
            if self.allow_latest_transform
            else rclpy.time.Time.from_msg(point.header.stamp)
        )
        try:
            # Latest TF is allowed only by the static mock-hardware launch. Real
            # motion must use the transform matching the image timestamp.
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point.header.frame_id,
                transform_time,
                timeout=Duration(seconds=0.1),
            )
            transformed = do_transform_point(point, transform)
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot transform ball from {point.header.frame_id} to "
                f"{self.target_frame}: {error}",
                throttle_duration_sec=2.0,
            )
            return

        position = np.array(
            [transformed.point.x, transformed.point.y, transformed.point.z],
            dtype=float,
        )
        position[2] += self.center_offset_z
        if self.positions and np.linalg.norm(position - self.positions[-1]) > 0.10:
            self.positions.clear()
            self.position_times.clear()
        self.positions.append(position)
        self.position_times.append(self.get_clock().now())
        filtered = np.median(np.stack(self.positions), axis=0)

        stamp = self.get_clock().now().to_msg()
        goal = PoseStamped()
        goal.header.stamp = stamp
        goal.header.frame_id = self.target_frame
        goal.pose.position.x = float(filtered[0])
        goal.pose.position.y = float(filtered[1])
        goal.pose.position.z = float(filtered[2])
        goal.pose.orientation.w = 1.0
        self.pose_publisher.publish(goal)

        self.publish_goal_transform(goal)
        self.publish_preview_markers(goal)

        if self.publish_metrics:
            source_time = rclpy.time.Time.from_msg(ball.header.stamp)
            now = self.get_clock().now()
            filter_window_ms = 0.0
            if self.position_times:
                filter_window_ms = (
                    now - self.position_times[0]
                ).nanoseconds / 1_000_000.0
            self.publish_timing(
                ball.header,
                {
                    "tf_filter_publish_ms": (time.perf_counter() - callback_start)
                    * 1000.0,
                    "filter_window_ms": filter_window_ms,
                    "capture_to_goal_ms": max(
                        0.0, (now - source_time).nanoseconds / 1_000_000.0
                    ),
                    "filter_sample_count": float(len(self.positions)),
                },
            )

        now = self.get_clock().now()
        if now - self.last_log_time >= Duration(seconds=2.0):
            self.get_logger().info(
                f"Ball goal in {self.target_frame}: "
                f"x={filtered[0]:.3f} m, y={filtered[1]:.3f} m, "
                f"z={filtered[2]:.3f} m"
            )
            self.last_log_time = now

    def publish_timing(self, header, metrics: dict[str, float]) -> None:
        message = DiagnosticArray()
        message.header = header
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "goal"
        status.message = "goal timings"
        status.hardware_id = "tf"
        status.values = [
            KeyValue(key=name, value=f"{value:.6f}")
            for name, value in metrics.items()
        ]
        message.status = [status]
        self.metrics_publisher.publish(message)

    def publish_goal_transform(self, goal: PoseStamped) -> None:
        transform = TransformStamped()
        transform.header = goal.header
        transform.child_frame_id = "ball_goal"
        transform.transform.translation.x = goal.pose.position.x
        transform.transform.translation.y = goal.pose.position.y
        transform.transform.translation.z = goal.pose.position.z
        transform.transform.rotation = goal.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_preview_markers(self, goal: PoseStamped) -> None:
        lifetime = Duration(seconds=0.5).to_msg()
        markers = MarkerArray()

        target = Marker()
        target.header = goal.header
        target.ns = "ball_goal"
        target.id = 0
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose = deepcopy(goal.pose)
        target.scale.x = 0.04
        target.scale.y = 0.04
        target.scale.z = 0.04
        target.color.r = 1.0
        target.color.g = 0.1
        target.color.b = 0.1
        target.color.a = 0.9
        target.lifetime = lifetime
        markers.markers.append(target)

        label = Marker()
        label.header = goal.header
        label.ns = "ball_goal"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = deepcopy(target.pose)
        label.pose.position.z += 0.025
        label.scale.z = 0.025
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = (
            f"goal x={goal.pose.position.x:.3f} "
            f"y={goal.pose.position.y:.3f} z={goal.pose.position.z:.3f}"
        )
        label.lifetime = lifetime
        markers.markers.append(label)

        self.marker_publisher.publish(markers)


def main() -> None:
    rclpy.init()
    node = BallGoalPreviewNode()
    # TF must keep arriving while a marker callback waits for its image timestamp.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
