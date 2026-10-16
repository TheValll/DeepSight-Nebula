#!/usr/bin/env python3

"""Execute one explicit ball-pick sequence on the physical robot."""

from copy import deepcopy
from math import ceil, isfinite, sqrt
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, DisplayTrajectory, JointConstraint, MoveItErrorCodes, PositionConstraint,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
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
            self.declare_parameter('acceleration_scaling', 0.05).value
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
        # The camera measures the visible ball surface, which faces the robot.
        # Move the target horizontally away from the base toward the ball centre.
        self.grasp_radial_offset = float(
            self.declare_parameter('grasp_radial_offset', 0.0).value
        )
        # Horizontal shift to the arm's left, seen from the base toward the ball.
        self.grasp_lateral_offset = float(
            self.declare_parameter('grasp_lateral_offset', 0.0).value
        )
        self.tcp_offset = Vector3(
            x=float(self.declare_parameter('tcp_offset_x', 0.0).value),
            y=float(self.declare_parameter('tcp_offset_y', 0.0).value),
            z=float(self.declare_parameter('tcp_offset_z', 0.0).value),
        )

        # Under load, the servos stop short of their command although their
        # encoders see the real angle. After MoveIt, command the remaining
        # joint error again before closing the gripper.
        self.sag_iterations = int(
            self.declare_parameter('sag_compensation_iterations', 2).value
        )
        self.sag_tolerance = float(
            self.declare_parameter('sag_tolerance', 0.005).value
        )
        self.sag_max_correction = float(
            self.declare_parameter('sag_max_correction', 0.08).value
        )
        self.sag_settle_time = float(
            self.declare_parameter('sag_settle_time', 0.4).value
        )
        self.sag_move_duration = float(
            self.declare_parameter('sag_move_duration', 0.6).value
        )
        # Stop above the ball, measure the sag there, then descend vertically with
        # that joint correction already added so the arm does not dip to the desk.
        self.pregrasp_height = float(
            self.declare_parameter('pregrasp_height', 0.03).value
        )
        self.descent_duration = float(
            self.declare_parameter('descent_duration', 2.0).value
        )
        self.descent_step = float(self.declare_parameter('descent_step', 0.002).value)
        self.descent_max_joint_step = float(
            self.declare_parameter('descent_max_joint_step', 0.05).value
        )
        self.stage = 'final'
        self.grasp_joints: dict[str, float] = {}
        self.pregrasp_joints: dict[str, float] = {}
        self.descent_delta: dict[str, float] = {}
        self.latest_joints: dict[str, float] = {}
        self.sag_target: dict[str, float] = {}
        self.sag_command: dict[str, float] = {}
        self.sag_iteration = 0
        self.sag_timer = None

        self.latest_ball: PoseStamped | None = None
        self.latest_ball_received = None
        self.active_target: PoseStamped | None = None
        self.frozen_target: PoseStamped | None = None
        self.busy = False
        self.approach_only = False
        self.pick_outcome = ''

        self.create_subscription(PoseStamped, '/ball_goal/pose', self.store_ball, 10)
        self.create_subscription(JointState, '/joint_states', self.store_joints, 10)
        self.create_service(Trigger, '/pick_ball', self.request_pick)
        self.create_service(Trigger, '/approach_ball', self.request_approach)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.gripper = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        self.arm_trajectory = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
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

    def store_ball(self, message: PoseStamped) -> None:
        self.latest_ball = deepcopy(message)
        self.latest_ball_received = self.get_clock().now()

    def store_joints(self, message: JointState) -> None:
        self.latest_joints.update(zip(message.name, message.position))

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
        self.approach_only = False
        self.pick_outcome = ''
        self.stage = 'pregrasp' if self.pregrasp_height > 0.0 else 'final'
        self.active_target = target
        self.frozen_target = deepcopy(target)
        self.publish_frozen_target()
        self.grasp_joints = {}
        self.pregrasp_joints = {}
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
        if self.stage != 'pregrasp':
            self.send_gripper_goal(self.gripper_open_position, self.gripper_opened)
            return
        above = deepcopy(self.active_target.pose.position)
        above.z += self.pregrasp_height
        request = self.ik_request(above, self.grasp_joints, 0.1)
        self.ik_client.call_async(request).add_done_callback(self.pregrasp_ik_done)

    def pregrasp_ik_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.abort(f'Position au-dessus de la balle impossible: {error}')
            return
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.abort('Aucune position sure 3 cm au-dessus de la balle.')
            return
        solution = dict(zip(response.solution.joint_state.name,
                            response.solution.joint_state.position))
        self.pregrasp_joints = {name: solution[name] for name in self.grasp_joints}
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
        self.get_logger().info(
            'Pince ouverte. Mouvement au-dessus de la balle.' if self.stage == 'pregrasp'
            else 'Pince ouverte. Envoi du goal exact de la balle.')
        self.send_arm_goal()

    def send_arm_goal(self) -> None:
        if self.active_target is None:
            self.abort('Goal de balle perdu avant le mouvement.')
            return

        if self.stage == 'pregrasp' and getattr(self, 'pregrasp_joints', None):
            # Joint goal derived from a checked grasp configuration, so the
            # vertical descent continues on the same IK branch.
            constraints = Constraints()
            constraints.name = 'above_ball'
            for name, value in self.pregrasp_joints.items():
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=name, position=value, tolerance_above=0.005,
                    tolerance_below=0.005, weight=1.0,
                ))
            goal = self.make_arm_goal(constraints)
        else:
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

        target = self.planned_final_positions(planned)
        if target and self.sag_iterations > 0:
            self.sag_target = target
            self.sag_command = dict(target)
            self.sag_iteration = 0
            self.schedule_sag_check()
            return
        self.arm_reached()

    @staticmethod
    def planned_final_positions(planned) -> dict[str, float]:
        if planned is None:
            return {}
        trajectory = planned.joint_trajectory
        if not trajectory.points:
            return {}
        return dict(zip(trajectory.joint_names, trajectory.points[-1].positions))

    def schedule_sag_check(self) -> None:
        if self.sag_timer is not None:
            self.sag_timer.cancel()
        self.sag_timer = self.create_timer(self.sag_settle_time, self.check_sag)

    def check_sag(self) -> None:
        self.sag_timer.cancel()
        self.sag_timer = None
        if not all(name in self.latest_joints for name in self.sag_target):
            self.get_logger().warning('Angles reels indisponibles: aucune compensation.')
            self.arm_reached()
            return
        error = {name: target - self.latest_joints[name]
                 for name, target in self.sag_target.items()}
        worst = max(abs(value) for value in error.values())
        self.get_logger().info('Ecart par articulation (rad): ' + ', '.join(
            f'{name.split("_to_")[0]}={value:+.4f}' for name, value in error.items()))
        if worst <= self.sag_tolerance:
            self.get_logger().info(f'Ecart articulaire {worst:.4f} rad: position atteinte.')
            self.arm_reached()
            return
        if self.sag_iteration >= self.sag_iterations:
            self.get_logger().warning(
                f'Ecart articulaire restant {worst:.4f} rad apres '
                f'{self.sag_iteration} correction(s).'
            )
            self.arm_reached()
            return
        command = {name: self.sag_command[name] + value for name, value in error.items()}
        if any(abs(command[name] - target) > self.sag_max_correction
               for name, target in self.sag_target.items()):
            self.get_logger().warning(
                f'Ecart articulaire {worst:.4f} rad trop grand pour une correction sure.'
            )
            self.arm_reached()
            return
        self.sag_iteration += 1
        self.sag_command = command
        self.get_logger().info(
            f'Ecart articulaire {worst:.4f} rad: correction {self.sag_iteration}.'
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(command)
        point = JointTrajectoryPoint()
        point.positions = list(command.values())
        point.time_from_start = Duration(seconds=self.sag_move_duration).to_msg()
        goal.trajectory.points = [point]
        future = self.arm_trajectory.send_goal_async(goal)
        future.add_done_callback(self.sag_goal_response)

    def sag_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.abort(f'Correction du bras impossible: {error}')
            return
        if not handle.accepted:
            self.abort('Correction du bras refusee.')
            return
        handle.get_result_async().add_done_callback(lambda _: self.schedule_sag_check())

    def arm_reached(self) -> None:
        if getattr(self, 'stage', 'final') == 'pregrasp':
            self.start_descent()
            return
        if self.approach_only:
            self.finish('Approche terminee. Pince ouverte pour verifier le centrage; aucun retour automatique.')
            return
        self.get_logger().info('Goal balle atteint. Fermeture de la pince.')
        self.send_gripper_goal(self.gripper_closed_position, self.gripper_closed)

    def start_descent(self) -> None:
        if not self.sag_target:
            self.abort('Angles de la position au-dessus de la balle inconnus: descente annulee.')
            return
        self.descent_delta = {name: self.sag_command[name] - target
                              for name, target in self.sag_target.items()}
        request = GetPositionFK.Request()
        request.header.frame_id = self.base_frame
        request.fk_link_names = [self.tip_link]
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(self.sag_target)
        request.robot_state.joint_state.position = list(self.sag_target.values())
        self.fk_client.call_async(request).add_done_callback(self.descent_fk_done)

    def descent_fk_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.abort(f'Calcul de la descente impossible: {error}')
            return
        if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
            self.abort(f'Pose de la pince inconnue, code {response.error_code.val}.')
            return
        self.descent_pose = deepcopy(response.pose_stamped[0].pose)
        self.descent_names = list(self.sag_target)
        self.descent_points = []
        self.descent_steps = max(1, ceil(self.pregrasp_height / self.descent_step))
        self.request_descent_ik(list(self.sag_target.values()))

    def request_descent_ik(self, seed) -> None:
        # MoveIt's Cartesian path stops after a few millimetres on this 5-axis arm:
        # descend with position-only IK, each step seeded from the previous one.
        step = len(self.descent_points) + 1
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.tip_link
        request.ik_request.robot_state.is_diff = True
        request.ik_request.robot_state.joint_state.name = self.descent_names
        request.ik_request.robot_state.joint_state.position = list(seed)
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=0.05).to_msg()
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.pose = deepcopy(self.descent_pose)
        request.ik_request.pose_stamped.pose.position.z -= (
            self.pregrasp_height * step / self.descent_steps)
        self.ik_client.call_async(request).add_done_callback(self.descent_ik_done)

    def descent_ik_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.abort(f'Descente verticale impossible: {error}')
            return
        step = len(self.descent_points) + 1
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.abort(
                f'Descente verticale bloquee au pas {step}/{self.descent_steps} '
                f'(code {response.error_code.val}).'
            )
            return
        solution = dict(zip(response.solution.joint_state.name,
                            response.solution.joint_state.position))
        point = [solution[name] for name in self.descent_names]
        previous = self.descent_points[-1] if self.descent_points else list(self.sag_target.values())
        if max(abs(a - b) for a, b in zip(point, previous)) > self.descent_max_joint_step:
            self.abort(f'Saut articulaire pendant la descente au pas {step}.')
            return
        self.descent_points.append(point)
        if step < self.descent_steps:
            self.request_descent_ik(point)
            return
        self.execute_descent()

    def execute_descent(self) -> None:
        goal = make_descent_goal(
            self.descent_names, self.descent_points, self.descent_delta, self.descent_duration,
        )
        self.stage = 'final'
        self.sag_target = dict(zip(self.descent_names, self.descent_points[-1]))
        self.sag_command = {name: value + self.descent_delta.get(name, 0.0)
                            for name, value in self.sag_target.items()}
        self.sag_iteration = 0
        self.get_logger().info(
            f'Descente verticale de {self.pregrasp_height * 100:.1f} cm avec la correction '
            'mesuree au-dessus de la balle.'
        )
        self.arm_trajectory.send_goal_async(goal).add_done_callback(self.sag_goal_response)

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


def make_descent_goal(names, positions, delta, duration) -> FollowJointTrajectory.Goal:
    """Joint path with the measured sag correction, evenly timed over duration."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(names)
    count = len(positions)
    for index, values in enumerate(positions):
        point = JointTrajectoryPoint()
        point.positions = [value + delta.get(name, 0.0) for name, value in zip(names, values)]
        point.time_from_start = Duration(
            seconds=max(0.1, duration) * (index + 1) / count).to_msg()
        goal.trajectory.points.append(point)
    return goal


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
