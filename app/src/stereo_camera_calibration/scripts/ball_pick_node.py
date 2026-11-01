#!/usr/bin/env python3

"""Execute one explicit ball-pick sequence on the physical robot."""

from copy import deepcopy
import json
from math import isfinite, sqrt
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, DisplayTrajectory, JointConstraint, MoveItErrorCodes, PositionConstraint,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
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
        # Tuning parameters have no code default: they come from
        # config/pipeline.yaml so that every launch uses the same values.
        self.base_frame = self.required('base_frame', Parameter.Type.STRING)
        self.tip_link = self.required('tip_link', Parameter.Type.STRING)
        self.position_tolerance = self.required('position_tolerance', Parameter.Type.DOUBLE)
        self.goal_max_age = self.required('goal_max_age', Parameter.Type.DOUBLE)
        self.max_target_distance = self.required('max_target_distance', Parameter.Type.DOUBLE)
        self.planning_attempts = self.required('planning_attempts', Parameter.Type.INTEGER)
        self.allowed_planning_time = self.required('allowed_planning_time', Parameter.Type.DOUBLE)
        self.velocity_scaling = self.required('velocity_scaling', Parameter.Type.DOUBLE)
        self.acceleration_scaling = self.required('acceleration_scaling', Parameter.Type.DOUBLE)
        self.gripper_open_position = self.required('gripper_open_position', Parameter.Type.DOUBLE)
        self.gripper_closed_position = self.required(
            'gripper_closed_position', Parameter.Type.DOUBLE)
        self.gripper_max_effort = self.required('gripper_max_effort', Parameter.Type.DOUBLE)
        # Horizontal grasp offsets in base_link, relative to the base -> ball
        # direction: radial moves away from the base, lateral to the arm's left.
        self.grasp_radial_offset = self.required('grasp_radial_offset', Parameter.Type.DOUBLE)
        self.grasp_lateral_offset = self.required('grasp_lateral_offset', Parameter.Type.DOUBLE)

        self.grasp_joints: dict[str, float] = {}
        self.latest_joints: dict[str, float] = {}

        self.latest_ball: PoseStamped | None = None
        self.latest_ball_received = None
        self.active_target: PoseStamped | None = None
        self.frozen_target: PoseStamped | None = None
        self.busy = False
        self.approach_only = False
        self.pick_outcome = ''
        self.operation_id = 0
        self.operation_state = 'idle'
        self.operation_message = ''
        self.operation_target = None

        self.create_subscription(PoseStamped, '/ball_goal/pose', self.store_ball, 10)
        self.create_subscription(JointState, '/joint_states', self.store_joints, 10)
        self.create_service(Trigger, '/pick_ball', self.request_pick)
        self.create_service(Trigger, '/approach_ball', self.request_approach)
        self.create_service(Trigger, '/prepare_pick_trial', self.prepare_trial)
        self.create_service(Trigger, '/pick_status', self.pick_status)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.gripper = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
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

    def required(self, name: str, kind: Parameter.Type):
        value = self.declare_parameter(name, kind).value
        if value is None:
            raise ValueError(f'Missing ROS parameter {name!r} (see config/pipeline.yaml)')
        if isinstance(value, float) and not isfinite(value):
            raise ValueError(f'ROS parameter {name!r} must be finite')
        return value

    def store_ball(self, message: PoseStamped) -> None:
        self.latest_ball = deepcopy(message)
        self.latest_ball_received = self.get_clock().now()

    def store_joints(self, message: JointState) -> None:
        self.latest_joints.update(zip(message.name, message.position))

    def pick_status(self, request, response):
        del request
        response.success = True
        response.message = json.dumps({
            'operation_id': self.operation_id,
            'state': self.operation_state,
            'busy': self.busy,
            'message': self.operation_message,
            'target': self.operation_target,
            'joints': self.latest_joints,
            'return_to_default': self.return_to_default,
        })
        return response

    def prepare_trial(self, request, response):
        del request
        if self.busy or not self.return_to_default or not self.move_group.server_is_ready():
            response.success = False
            response.message = 'Preparation refusee: robot occupe, retour default desactive ou MoveIt indisponible.'
            return response
        self.busy = True
        self.operation_id += 1
        self.operation_state = 'running'
        self.operation_message = 'Positionnement initial en default.'
        self.operation_target = None
        self.return_after_pick('Preparation de l essai.')
        response.success = True
        response.message = str(self.operation_id)
        return response

    def request_approach(self, request: Trigger.Request, response: Trigger.Response):
        response = self.request_pick(request, response)
        if response.success:
            self.approach_only = True
            response.message = response.message.replace(
                'Prise acceptee', 'Approche seule acceptee'
            ) + ' La pince restera ouverte, sans retour automatique.'
        return response

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
        if not self.ik_client.service_is_ready():
            response.success = False
            response.message = 'Le service MoveIt /compute_ik n est pas disponible.'
            return response

        target = deepcopy(self.latest_ball)
        target.header.frame_id = self.base_frame
        self.apply_grasp_offsets(target.pose.position)
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
        self.operation_id += 1
        self.operation_state = 'running'
        self.operation_message = 'Prise en cours.'
        self.operation_target = dict(zip(('x', 'y', 'z'), coordinates))
        self.approach_only = False
        self.pick_outcome = ''
        self.active_target = target
        self.frozen_target = deepcopy(target)
        self.publish_frozen_target()
        self.grasp_joints = {}
        self.find_grasp_ik(0)
        response.success = True
        response.message = (
            'Prise acceptee: verification de la portee, ouverture de la pince puis mouvement vers '
            f'x={coordinates[0]:.3f}, y={coordinates[1]:.3f}, z={coordinates[2]:.3f} m.'
        )
        return response

    def apply_grasp_offsets(self, position) -> None:
        radial = sqrt(position.x ** 2 + position.y ** 2)
        if radial <= 1e-6:
            return
        outward_x = position.x / radial
        outward_y = position.y / radial
        position.x += (self.grasp_radial_offset * outward_x
                       - self.grasp_lateral_offset * outward_y)
        position.y += (self.grasp_radial_offset * outward_y
                       + self.grasp_lateral_offset * outward_x)

    IK_SEED_OFFSETS = ([0.0] * 5, [0.0, -0.4, 0.3, 0.3, 0.0], [0.0, -0.8, -0.3, 0.6, 0.4])

    def ik_request(self, position, seed: dict[str, float], timeout: float) -> GetPositionIK.Request:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.tip_link
        request.ik_request.robot_state.is_diff = True
        request.ik_request.robot_state.joint_state.name = list(seed)
        request.ik_request.robot_state.joint_state.position = list(seed.values())
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=timeout).to_msg()
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.pose.position = deepcopy(position)
        request.ik_request.pose_stamped.pose.orientation.w = 1.0
        return request

    def find_grasp_ik(self, attempt: int) -> None:
        # Check the grasp itself before any movement: far balls force the
        # servo 1 housing into the desk and have no collision-free solution.
        names = [name for name, _ in self.default_joint_positions] or list(self.latest_joints)
        values = [value for _, value in self.default_joint_positions] or list(self.latest_joints.values())
        offsets = self.IK_SEED_OFFSETS[attempt]
        seed = {name: value + (offsets[i] if i < len(offsets) else 0.0)
                for i, (name, value) in enumerate(zip(names, values))}
        request = self.ik_request(self.active_target.pose.position, seed, 0.2)
        self.ik_client.call_async(request).add_done_callback(
            lambda future: self.grasp_ik_done(future, attempt))

    def grasp_ik_done(self, future, attempt: int) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.abort(f'Verification de la prise impossible: {error}')
            return
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            if attempt + 1 < len(self.IK_SEED_OFFSETS):
                self.find_grasp_ik(attempt + 1)
                return
            position = self.active_target.pose.position
            self.abort(
                'Balle hors de portee: aucune prise sans collision a '
                f'{sqrt(position.x ** 2 + position.y ** 2):.3f} m de la base. '
                'Trop loin, le boitier du servo 1 toucherait le bureau; trop pres, '
                'le coude arrive en butee. Deplacez la balle.'
            )
            return
        names = [name for name, _ in self.default_joint_positions] or list(self.latest_joints)
        solution = dict(zip(response.solution.joint_state.name,
                            response.solution.joint_state.position))
        self.grasp_joints = {name: solution[name] for name in names if name in solution}
        self.send_gripper_goal(self.gripper_open_position, self.gripper_opened)

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

        planned = getattr(result, 'planned_trajectory', None)
        if planned is not None:
            display = DisplayTrajectory()
            display.model_id = 'hiwonder_esp32'
            display.trajectory_start = result.trajectory_start
            display.trajectory = [planned]
            self.display_publisher.publish(display)

        self.arm_reached()

    def arm_reached(self) -> None:
        if self.approach_only:
            self.finish('Approche terminee. Pince ouverte pour verifier le centrage; aucun retour automatique.')
            return
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
        self.default_replan_count = 0
        self.default_replan_timer = None
        self.send_default_goal()

    def send_default_goal(self) -> None:
        constraints = Constraints()
        constraints.name = 'return_to_default'
        for name, value in self.default_joint_positions:
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=value, tolerance_above=0.005,
                tolerance_below=0.005, weight=1.0,
            ))
        self.get_logger().info(f'{self.pick_outcome} Retour du bras en default, pince fermee.')
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
                if (wrapped.status == GoalStatus.STATUS_ABORTED
                        and code == MoveItErrorCodes.CONTROL_FAILED
                        and self.default_replan_count < 2):
                    self.default_replan_count += 1
                    self.get_logger().warning(
                        f'Retour default refuse ou interrompu: nouvelle planification '
                        f'{self.default_replan_count}/2 apres stabilisation.')
                    self.default_replan_timer = self.create_timer(
                        0.4, self.retry_default_return)
                    return
                self.abort(f'Retour en default echoue, code MoveIt {code}.')
                return
        except Exception as error:  # noqa: BLE001
            self.abort(f'Retour en default echoue: {error}')
            return
        self.finish(f'Prise terminee: {self.pick_outcome} Bras revenu en default.')

    def retry_default_return(self) -> None:
        self.default_replan_timer.cancel()
        self.default_replan_timer = None
        self.send_default_goal()

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
        self.operation_state = 'succeeded'
        self.operation_message = message
        self.busy = False
        self.active_target = None
        self.clear_frozen_target()

    def abort(self, message: str) -> None:
        self.get_logger().error(message)
        self.operation_state = 'failed'
        self.operation_message = message
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
