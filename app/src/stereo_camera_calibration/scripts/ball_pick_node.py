#!/usr/bin/env python3

"""Execute one explicit ball-pick sequence on the physical robot."""

from copy import deepcopy
from math import isfinite, sqrt
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, DisplayTrajectory, JointConstraint, MoveItErrorCodes, PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def load_joint_state(srdf_path: Path, group_name: str, state_name: str):
    """Read the named arm pose from the same SRDF used by MoveIt."""
    root = ET.parse(srdf_path).getroot()
    for state in root.findall('group_state'):
        if state.get('name') == state_name and state.get('group') == group_name:
            joints = [(joint.attrib['name'], float(joint.attrib['value']))
                      for joint in state.findall('joint')]
            if (not joints or len({name for name, _ in joints}) != len(joints)
                    or not all(isfinite(value) for _, value in joints)):
                raise ValueError(f'Invalid joint state {group_name}/{state_name}')
            return joints
    raise ValueError(f'Missing joint state {group_name}/{state_name} in {srdf_path}')


class BallPickNode(Node):
    """Open, reach the ball, close, then return the arm to its default pose."""

    def __init__(self) -> None:
        super().__init__('ball_pick_node')

        self.group_name = str(self.declare_parameter('group_name', 'arm').value)
        self.return_to_default = bool(
            self.declare_parameter('return_to_default', True).value
        )
        self.default_joint_positions = []
        if self.return_to_default:
            config_path = Path(get_package_share_directory(
                'hiwonder_xarm_esp32_moveit_config'))
            self.default_joint_positions = load_joint_state(
                config_path / 'config' / 'hiwonder_esp32.srdf', self.group_name, 'default'
            )
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.tip_link = str(
            self.declare_parameter('tip_link', 'gripper_tcp_link').value
        )
        self.position_tolerance = float(
            self.declare_parameter('position_tolerance', 0.01).value
        )
        self.goal_max_age = float(self.declare_parameter('goal_max_age', 1.0).value)
        self.max_target_distance = float(
            self.declare_parameter('max_target_distance', 0.60).value
        )
        self.planning_attempts = int(
            self.declare_parameter('planning_attempts', 2).value
        )
        self.allowed_planning_time = float(
            self.declare_parameter('allowed_planning_time', 1.0).value
        )
        self.velocity_scaling = float(
            self.declare_parameter('velocity_scaling', 0.35).value
        )
        self.acceleration_scaling = float(
            self.declare_parameter('acceleration_scaling', 0.35).value
        )
        self.gripper_open_position = float(
            self.declare_parameter('gripper_open_position', 0.013651).value
        )
        self.gripper_closed_position = float(
            self.declare_parameter('gripper_closed_position', -0.013651).value
        )
        self.gripper_max_effort = float(
            self.declare_parameter('gripper_max_effort', 10.0).value
        )
        self.tcp_offset = Vector3(
            x=float(self.declare_parameter('tcp_offset_x', 0.0).value),
            y=float(self.declare_parameter('tcp_offset_y', 0.0).value),
            z=float(self.declare_parameter('tcp_offset_z', 0.0).value),
        )

        self.latest_ball: PoseStamped | None = None
        self.latest_ball_received = None
        self.active_target: PoseStamped | None = None
        self.frozen_target: PoseStamped | None = None
        self.busy = False
        self.pick_outcome = ''

        self.create_subscription(PoseStamped, '/ball_goal/pose', self.store_ball, 10)
        self.create_service(Trigger, '/pick_ball', self.request_pick)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.gripper = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        self.display_publisher = self.create_publisher(
            DisplayTrajectory, '/display_planned_path', 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/ball_goal/markers', 10
        )
        self.create_timer(0.2, self.publish_frozen_target)

        self.get_logger().warning(
            'Pick pipeline ready. No automatic movement: call /pick_ball '
            'explicitly to open, move to the ball, close the gripper'
            + (', then return the arm to default.' if self.return_to_default else '.')
        )

    def store_ball(self, message: PoseStamped) -> None:
        self.latest_ball = deepcopy(message)
        self.latest_ball_received = self.get_clock().now()

    def request_pick(self, request: Trigger.Request, response: Trigger.Response):
        del request
        if self.busy:
            response.success = False
            response.message = 'Une prise est deja en cours.'
            return response
        if self.latest_ball is None or self.latest_ball_received is None:
            response.success = False
            response.message = 'Aucune balle detectee.'
            return response
        if self.get_clock().now() - self.latest_ball_received > Duration(
            seconds=self.goal_max_age
        ):
            response.success = False
            response.message = 'La derniere detection est trop ancienne.'
            return response
        if not self.move_group.server_is_ready():
            response.success = False
            response.message = 'MoveIt /move_action n est pas disponible.'
            return response
        if not self.gripper.server_is_ready():
            response.success = False
            response.message = 'Le controleur de pince n est pas disponible.'
            return response

        target = deepcopy(self.latest_ball)
        target.header.frame_id = self.base_frame
        coordinates = (
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        )
        distance = sqrt(sum(value * value for value in coordinates))
        if not all(isfinite(value) for value in coordinates):
            response.success = False
            response.message = 'Le goal contient une position invalide.'
            return response
        if distance > self.max_target_distance:
            response.success = False
            response.message = (
                f'Goal refuse a {distance:.3f} m: limite de securite '
                f'{self.max_target_distance:.3f} m.'
            )
            return response

        self.busy = True
        self.pick_outcome = ''
        self.active_target = target
        self.frozen_target = deepcopy(target)
        self.publish_frozen_target()
        self.send_gripper_goal(self.gripper_open_position, self.gripper_opened)
        response.success = True
        response.message = (
            'Prise acceptee: ouverture de la pince puis mouvement vers '
            f'x={coordinates[0]:.3f}, y={coordinates[1]:.3f}, z={coordinates[2]:.3f} m.'
        )
        return response

    def send_gripper_goal(self, position: float, result_callback) -> None:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.gripper_max_effort
        future = self.gripper.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self.gripper_goal_response(completed, result_callback)
        )

    def gripper_goal_response(self, future, result_callback) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.abort(f'Commande de pince impossible: {error}')
            return
        if not goal_handle.accepted:
            self.abort('Commande de pince refusee.')
            return
        goal_handle.get_result_async().add_done_callback(result_callback)

    def gripper_opened(self, future) -> None:
        try:
            result = future.result().result
        except Exception as error:  # noqa: BLE001
            self.abort(f'Ouverture de la pince echouee: {error}')
            return
        if not result.reached_goal:
            self.abort(
                'La pince ne confirme pas son ouverture '
                f'(position={result.position:.4f}, stalled={result.stalled}).'
            )
            return
        self.get_logger().info('Pince ouverte. Envoi du goal exact de la balle.')
        self.send_arm_goal()

    def send_arm_goal(self) -> None:
        if self.active_target is None:
            self.abort('Goal de balle perdu avant le mouvement.')
            return

        goal = self.make_arm_goal(self.make_position_constraint(self.active_target))
        self.move_group.send_goal_async(goal).add_done_callback(self.arm_goal_response)

    def make_arm_goal(self, constraints: Constraints) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.pipeline_id = 'ompl'
        goal.request.num_planning_attempts = max(1, self.planning_attempts)
        goal.request.allowed_planning_time = max(0.1, self.allowed_planning_time)
        goal.request.max_velocity_scaling_factor = min(
            1.0, max(0.01, self.velocity_scaling)
        )
        goal.request.max_acceleration_scaling_factor = min(
            1.0, max(0.01, self.acceleration_scaling)
        )
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def arm_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.abort(f'Goal MoveIt impossible: {error}')
            return
        if not goal_handle.accepted:
            self.abort('Goal MoveIt refuse.')
            return
        goal_handle.get_result_async().add_done_callback(self.arm_finished)

    def arm_finished(self, future) -> None:
        try:
            result = future.result().result
        except Exception as error:  # noqa: BLE001
            self.abort(f'Mouvement du bras echoue: {error}')
            return
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.abort(f'Mouvement MoveIt echoue, code {result.error_code.val}.')
            return

        display = DisplayTrajectory()
        display.model_id = 'hiwonder_esp32'
        display.trajectory_start = result.trajectory_start
        display.trajectory = [result.planned_trajectory]
        self.display_publisher.publish(display)
        self.get_logger().info('Goal balle atteint. Fermeture de la pince.')
        self.send_gripper_goal(self.gripper_closed_position, self.gripper_closed)

    def gripper_closed(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as error:  # noqa: BLE001
            self.abort(f'Fermeture de la pince echouee: {error}')
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.abort(f'Fermeture interrompue, statut {wrapped.status}.')
            return
        if result.reached_goal:
            self.return_after_pick('Pince fermee sur sa position cible.')
        elif result.stalled:
            self.return_after_pick('Pince bloquee sur un objet.')
        else:
            self.abort(
                'La pince ne confirme pas la fermeture '
                f'(position={result.position:.4f}).'
            )

    def return_after_pick(self, message: str) -> None:
        if not self.return_to_default:
            self.finish(f'Prise terminee: {message}')
            return
        self.pick_outcome = message
        constraints = Constraints()
        constraints.name = 'return_to_default'
        for name, value in self.default_joint_positions:
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=value, tolerance_above=0.005,
                tolerance_below=0.005, weight=1.0,
            ))
        self.get_logger().info(f'{message} Retour du bras en default, pince fermee.')
        try:
            future = self.move_group.send_goal_async(self.make_arm_goal(constraints))
            future.add_done_callback(self.default_goal_response)
        except Exception as error:  # noqa: BLE001
            self.abort(f'Retour en default impossible: {error}')

    def default_goal_response(self, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self.abort('Retour en default refuse.')
                return
            handle.get_result_async().add_done_callback(self.default_finished)
        except Exception as error:  # noqa: BLE001
            self.abort(f'Retour en default impossible: {error}')

    def default_finished(self, future) -> None:
        try:
            wrapped = future.result()
            code = wrapped.result.error_code.val
            if (wrapped.status != GoalStatus.STATUS_SUCCEEDED
                    or code != MoveItErrorCodes.SUCCESS):
                self.abort(f'Retour en default echoue, code MoveIt {code}.')
                return
        except Exception as error:  # noqa: BLE001
            self.abort(f'Retour en default echoue: {error}')
            return
        self.finish(f'Prise terminee: {self.pick_outcome} Bras revenu en default.')

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
        constraints.name = 'ball_pick'
        constraints.position_constraints = [position]
        return constraints

    def publish_frozen_target(self) -> None:
        if self.frozen_target is None:
            return

        markers = MarkerArray()

        target = Marker()
        target.header.frame_id = self.frozen_target.header.frame_id
        target.header.stamp = self.get_clock().now().to_msg()
        target.ns = 'pick_target'
        target.id = 10
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose = deepcopy(self.frozen_target.pose)
        target.scale.x = 0.045
        target.scale.y = 0.045
        target.scale.z = 0.045
        target.color.r = 0.0
        target.color.g = 0.9
        target.color.b = 1.0
        target.color.a = 0.9
        target.lifetime = Duration(seconds=0.5).to_msg()
        markers.markers.append(target)

        label = Marker()
        label.header = target.header
        label.ns = 'pick_target'
        label.id = 11
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = deepcopy(target.pose)
        label.pose.position.z += 0.035
        label.scale.z = 0.025
        label.color.r = 0.0
        label.color.g = 0.9
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = 'cible figee MoveIt'
        label.lifetime = Duration(seconds=0.5).to_msg()
        markers.markers.append(label)

        self.marker_publisher.publish(markers)

    def clear_frozen_target(self) -> None:
        self.frozen_target = None
        markers = MarkerArray()
        for marker_id in (10, 11):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.ns = 'pick_target'
            marker.id = marker_id
            marker.action = Marker.DELETE
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)

    def finish(self, message: str) -> None:
        self.get_logger().info(message)
        self.busy = False
        self.active_target = None
        self.clear_frozen_target()

    def abort(self, message: str) -> None:
        self.get_logger().error(message)
        self.busy = False
        self.active_target = None
        self.clear_frozen_target()


def main() -> None:
    rclpy.init()
    node = BallPickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
