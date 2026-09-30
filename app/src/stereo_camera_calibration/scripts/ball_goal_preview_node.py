#!/usr/bin/env python3

from collections import deque
from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.duration import Duration
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
        self.pregrasp_height = float(
            self.declare_parameter("pregrasp_height", 0.05).value
        )
        self.allow_latest_transform = bool(
            self.declare_parameter("allow_latest_transform", False).value
        )
        sample_count = int(self.declare_parameter("sample_count", 10).value)
        self.positions = deque(maxlen=max(1, sample_count))
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            "Passive preview enabled: publishes the detected ball goal without "
            "planning or sending robot commands"
        )

    def process_markers(self, message: MarkerArray) -> None:
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
        if self.positions and np.linalg.norm(position - self.positions[-1]) > 0.10:
            self.positions.clear()
        self.positions.append(position)
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

        now = self.get_clock().now()
        if now - self.last_log_time >= Duration(seconds=2.0):
            self.get_logger().info(
                f"Ball goal in {self.target_frame}: "
                f"x={filtered[0]:.3f} m, y={filtered[1]:.3f} m, "
                f"z={filtered[2]:.3f} m"
            )
            self.last_log_time = now

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
        target.scale.x = 0.045
        target.scale.y = 0.045
        target.scale.z = 0.045
        target.color.r = 1.0
        target.color.g = 0.1
        target.color.b = 0.1
        target.color.a = 0.9
        target.lifetime = lifetime
        markers.markers.append(target)

        pregrasp = Marker()
        pregrasp.header = goal.header
        pregrasp.ns = "ball_goal"
        pregrasp.id = 1
        pregrasp.type = Marker.SPHERE
        pregrasp.action = Marker.ADD
        pregrasp.pose = deepcopy(goal.pose)
        pregrasp.pose.position.z += self.pregrasp_height
        pregrasp.scale.x = 0.025
        pregrasp.scale.y = 0.025
        pregrasp.scale.z = 0.025
        pregrasp.color.r = 0.1
        pregrasp.color.g = 0.4
        pregrasp.color.b = 1.0
        pregrasp.color.a = 0.9
        pregrasp.lifetime = lifetime
        markers.markers.append(pregrasp)

        approach = Marker()
        approach.header = goal.header
        approach.ns = "ball_goal"
        approach.id = 2
        approach.type = Marker.ARROW
        approach.action = Marker.ADD
        approach.points = [
            deepcopy(pregrasp.pose.position),
            deepcopy(target.pose.position),
        ]
        approach.scale.x = 0.008
        approach.scale.y = 0.016
        approach.scale.z = 0.02
        approach.color.r = 0.2
        approach.color.g = 0.6
        approach.color.b = 1.0
        approach.color.a = 0.9
        approach.lifetime = lifetime
        markers.markers.append(approach)

        label = Marker()
        label.header = goal.header
        label.ns = "ball_goal"
        label.id = 3
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = deepcopy(pregrasp.pose)
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
