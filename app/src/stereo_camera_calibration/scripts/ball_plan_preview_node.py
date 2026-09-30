#!/usr/bin/env python3

"""Ask MoveIt for a ball pre-grasp trajectory without ever executing it."""

from collections import deque
from copy import deepcopy
from math import dist
from statistics import median

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    DisplayTrajectory,
    MoveItErrorCodes,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger


class BallPlanPreviewNode(Node):
    """Plan-only bridge between a stable detected ball and MoveIt."""

    def __init__(self) -> None:
        super().__init__("ball_plan_preview_node")

        self.group_name = str(self.declare_parameter("group_name", "arm").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.tip_link = str(
            self.declare_parameter("tip_link", "gripper_tcp_link").value
        )
        self.pregrasp_height = float(
            self.declare_parameter("pregrasp_height", 0.05).value
        )
        self.position_tolerance = float(
            self.declare_parameter("position_tolerance", 0.01).value
        )
        self.goal_max_age = float(self.declare_parameter("goal_max_age", 1.0).value)
        self.auto_plan = bool(self.declare_parameter("auto_plan", False).value)
        self.stable_sample_count = int(
            self.declare_parameter("stable_sample_count", 6).value
        )
        self.stable_tolerance = float(
            self.declare_parameter("stable_tolerance", 0.025).value
        )
        self.replan_distance = float(
            self.declare_parameter("replan_distance", 0.025).value
        )
        self.min_replan_interval = float(
            self.declare_parameter("min_replan_interval", 1.5).value
        )

        # Optional correction from the selected tip. The normal configuration uses
        # gripper_tcp_link, whose measured 8 cm offset is already in the URDF.
        self.tcp_offset = Vector3(
            x=float(self.declare_parameter("tcp_offset_x", 0.0).value),
            y=float(self.declare_parameter("tcp_offset_y", 0.0).value),
            z=float(self.declare_parameter("tcp_offset_z", 0.0).value),
        )

        self.latest_ball: PoseStamped | None = None
        self.latest_ball_received = None
        self.plan_in_progress = False
        self.ball_samples = deque(maxlen=max(1, self.stable_sample_count))
        self.last_requested_target = None
        self.last_request_time = self.get_clock().now() - Duration(seconds=10.0)

        self.create_subscription(PoseStamped, "/ball_goal/pose", self.store_ball, 10)
        self.create_service(Trigger, "/plan_ball_goal", self.request_plan)
        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.display_publisher = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )
        self.create_timer(0.2, self.auto_plan_if_needed)

        self.get_logger().info(
            "Plan-only ball preview ready"
            + (" with automatic stable-target replanning" if self.auto_plan else "")
            + ". This node has no trajectory execution client and cannot command "
            "the robot."
        )

    def store_ball(self, message: PoseStamped) -> None:
        self.latest_ball = deepcopy(message)
        self.latest_ball_received = self.get_clock().now()
        point = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        if self.ball_samples and dist(point, self.ball_samples[-1]) > 0.10:
            self.ball_samples.clear()
        self.ball_samples.append(point)

    def auto_plan_if_needed(self) -> None:
        if not self.auto_plan or self.plan_in_progress:
            return
        if self.latest_ball is None or self.latest_ball_received is None:
            return
        if self.get_clock().now() - self.latest_ball_received > Duration(
            seconds=self.goal_max_age
        ):
            return
        if len(self.ball_samples) < self.stable_sample_count:
            return
        centre = tuple(median(axis) for axis in zip(*self.ball_samples))
        if max(dist(sample, centre) for sample in self.ball_samples) > self.stable_tolerance:
            return

        target = deepcopy(self.latest_ball)
        target.pose.position.x, target.pose.position.y, target.pose.position.z = centre
        target = self.make_pregrasp_target(target)
        target_xyz = self.pose_xyz(target)
        if (
            self.last_requested_target is not None
            and dist(target_xyz, self.last_requested_target) < self.replan_distance
        ):
            return
        if self.get_clock().now() - self.last_request_time < Duration(
            seconds=self.min_replan_interval
        ):
            return
        if not self.move_group.server_is_ready():
            return
        self.start_plan(target, automatic=True)

    def request_plan(self, request: Trigger.Request, response: Trigger.Response):
        del request
        if self.plan_in_progress:
            response.success = False
            response.message = "Une planification est deja en cours."
            return response
        if self.latest_ball is None or self.latest_ball_received is None:
            response.success = False
            response.message = "Aucune balle detectee."
            return response
        age = self.get_clock().now() - self.latest_ball_received
        if age > Duration(seconds=self.goal_max_age):
            response.success = False
            response.message = "La derniere detection est trop ancienne."
            return response
        if not self.move_group.server_is_ready():
            response.success = False
            response.message = "MoveIt /move_action n'est pas disponible."
            return response

        target = self.make_pregrasp_target(self.latest_ball)
        self.start_plan(target, automatic=False)
        response.success = True
        response.message = (
            "Planification seule demandee pour le pre-grasp "
            f"x={target.pose.position.x:.3f}, y={target.pose.position.y:.3f}, "
            f"z={target.pose.position.z:.3f} m."
        )
        return response

    def make_pregrasp_target(self, ball: PoseStamped) -> PoseStamped:
        target = deepcopy(ball)
        target.header.frame_id = self.base_frame
        target.pose.position.z += self.pregrasp_height
        return target

    @staticmethod
    def pose_xyz(target: PoseStamped) -> tuple[float, float, float]:
        return (
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        )

    def start_plan(self, target: PoseStamped, automatic: bool) -> None:
        self.clear_display()

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [self.make_position_constraint(target)]

        # The central safety property of this node: MoveIt computes a trajectory,
        # but must never forward it to a controller.
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.plan_in_progress = True
        self.last_requested_target = self.pose_xyz(target)
        self.last_request_time = self.get_clock().now()
        future = self.move_group.send_goal_async(goal)
        future.add_done_callback(self.goal_response)
        mode = "automatique" if automatic else "manuelle"
        self.get_logger().info(
            f"Replanification {mode}: x={target.pose.position.x:.3f}, "
            f"y={target.pose.position.y:.3f}, z={target.pose.position.z:.3f} m"
        )

    def clear_display(self) -> None:
        empty = DisplayTrajectory()
        empty.model_id = "hiwonder_esp32"
        self.display_publisher.publish(empty)

    def make_position_constraint(self, target: PoseStamped) -> Constraints:
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]

        sphere_pose = Pose()
        sphere_pose.position = deepcopy(target.pose.position)
        sphere_pose.orientation.w = 1.0

        position = PositionConstraint()
        position.header = target.header
        position.link_name = self.tip_link
        position.target_point_offset = self.tcp_offset
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [sphere_pose]
        position.weight = 1.0

        constraints = Constraints()
        constraints.name = "ball_pregrasp"
        constraints.position_constraints = [position]
        return constraints

    def goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future transports exceptions
            self.plan_in_progress = False
            self.clear_display()
            self.get_logger().error(f"Impossible d'envoyer le plan: {error}")
            return
        if not goal_handle.accepted:
            self.plan_in_progress = False
            self.clear_display()
            self.get_logger().error("MoveIt a refuse la demande de planification.")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.plan_result)

    def plan_result(self, future) -> None:
        self.plan_in_progress = False
        try:
            result = future.result().result
        except Exception as error:  # noqa: BLE001 - ROS future transports exceptions
            self.clear_display()
            self.get_logger().error(f"La planification a echoue: {error}")
            return

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.clear_display()
            self.get_logger().error(
                "Aucun plan valide vers la balle (code MoveIt "
                f"{result.error_code.val}). Aucun mouvement n'a ete execute."
            )
            return

        display = DisplayTrajectory()
        display.model_id = "hiwonder_esp32"
        display.trajectory_start = result.trajectory_start
        display.trajectory = [result.planned_trajectory]
        self.display_publisher.publish(display)
        points = result.planned_trajectory.joint_trajectory.points
        duration = 0.0
        if points:
            last = points[-1].time_from_start
            duration = last.sec + last.nanosec * 1e-9
        self.get_logger().info(
            f"Plan trouve: {len(points)} points, {duration:.2f} s. "
            "Affichage RViz uniquement; aucun mouvement n'a ete execute."
        )


def main() -> None:
    rclpy.init()
    node = BallPlanPreviewNode()
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
