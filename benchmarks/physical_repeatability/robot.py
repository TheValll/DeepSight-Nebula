"""ROS 2 / MoveIt arm interface for the benchmark. Imported only by `run`/live preflight.

* Arm joint goals only. No gripper command is ever sent; the gripper controller
  is neither activated nor deactivated here.
* rclpy is initialised without its own signal handlers, so SIGINT/SIGTERM set a
  stop event while the ROS context is still alive; the active MoveIt goal and any
  arm_controller trajectory goals are then cancelled. No automatic homing.
"""
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from errors import Interrupted, MotionFailure, SafetyRejection
from settings import ARM_JOINTS, GRIPPER_JOINT, ROLL_JOINT, ROOT, utc, wait_for_joint_feedback

STATUS_SUCCEEDED = 4


def reference_pose(name):
    srdf = ET.parse(ROOT / 'app/src/hiwonder_xarm_esp32_moveit_config/config/hiwonder_esp32.srdf').getroot()
    state = next(s for s in srdf.findall('group_state') if s.get('name') == name and s.get('group') == 'arm')
    return {j.get('name'): float(j.get('value')) for j in state.findall('joint')}


class Robot:
    def __init__(self, config, stop_event):
        self.config = config
        self.stop_event = stop_event
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('physical_repeatability_benchmark')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.lock = threading.Lock()
        self._joints = {}
        self._joint_stamp = None
        self.node.create_subscription(JointState, '/joint_states', self._on_joints, 20)
        self.move = ActionClient(self.node, MoveGroup, '/move_action')
        self.execute = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')
        self.scene = self.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.cancel_arm = self.node.create_client(
            CancelGoal, config.get('arm_controller_cancel_service',
                                   '/arm_controller/follow_joint_trajectory/_action/cancel_goal'))
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.home = reference_pose(config['reference_name'])
        self.active_handles = []

    # ------------------------------------------------------------------ feedback
    def _on_joints(self, msg):
        with self.lock:
            self._joints.update(zip(msg.name, msg.position))
            self._joint_stamp = time.monotonic()

    def joints(self):
        """(joint dict copy, feedback age in s)."""
        with self.lock:
            age = float('inf') if self._joint_stamp is None else time.monotonic() - self._joint_stamp
            return dict(self._joints), age

    def wait_ready(self, timeout=10):
        end = time.monotonic() + timeout
        needed = set(ARM_JOINTS) | {GRIPPER_JOINT}
        while time.monotonic() < end:
            joints, age = self.joints()
            if needed <= set(joints) and age < self.config['joint_state_max_age_s']:
                break
            time.sleep(0.05)
        else:
            raise MotionFailure('fresh /joint_states with all arm and gripper joints unavailable')
        if not self.move.wait_for_server(timeout_sec=10):
            raise MotionFailure('MoveIt /move_action unavailable (start it with run.sh stack)')
        if self.config['motion_execution'] == 'plan_then_execute' and not self.execute.wait_for_server(timeout_sec=10):
            raise MotionFailure('MoveIt /execute_trajectory unavailable')

    def controller_states(self):
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise MotionFailure('controller_manager list_controllers unavailable')
            response = self._wait(client.call_async(ListControllers.Request()), 5)
            return {c.name: c.state for c in response.controller}
        finally:
            self.node.destroy_client(client)

    # ------------------------------------------------------------------ helpers
    def _wait(self, future, timeout, interruptible=True):
        end = time.monotonic() + timeout
        while not future.done():
            if interruptible and self.stop_event.is_set():
                raise Interrupted('stop requested')
            if time.monotonic() > end:
                raise TimeoutError('ROS request timed out')
            time.sleep(0.01)
        return future.result()

    def _constraints(self, joints):
        constraints = Constraints()
        tolerance = self.config['goal_joint_tolerance_rad']
        for name in ARM_JOINTS:
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(joints[name]), tolerance_above=tolerance,
                tolerance_below=tolerance, weight=1.0))
        return constraints

    def _goal(self, joints, plan_only, start=None):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'arm'
        request.pipeline_id = 'ompl'
        request.allowed_planning_time = float(self.config['planning_time_s'])
        request.num_planning_attempts = 1
        request.max_velocity_scaling_factor = float(self.config['velocity_scaling'])
        request.max_acceleration_scaling_factor = float(self.config['acceleration_scaling'])
        request.goal_constraints = [self._constraints(joints)]
        request.start_state.is_diff = True
        if start is not None:
            state, _ = self.joints()
            state.update(start)
            request.start_state.joint_state.name = list(state)
            request.start_state.joint_state.position = [float(v) for v in state.values()]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def add_obstacles(self):
        if not self.scene.wait_for_service(timeout_sec=5):
            raise SafetyRejection('planning scene service unavailable')
        scene = PlanningScene(is_diff=True)
        for item in self.config['obstacles']:
            obj = CollisionObject(id=item['id'])
            obj.header.frame_id = self.config['base_frame']
            obj.operation = CollisionObject.ADD
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, item['center_m'])
            q = item.get('rotation_xyzw', [0, 0, 0, 1])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, item['size_m'])))]
            obj.primitive_poses = [pose]
            scene.world.collision_objects.append(obj)
        if not self._wait(self.scene.call_async(ApplyPlanningScene.Request(scene=scene)), 10).success:
            raise SafetyRejection('obstacles not accepted by the planning scene')

    def _trajectory_summary(self, trajectory, goal):
        names = list(trajectory.joint_trajectory.joint_names)
        if GRIPPER_JOINT in names or not set(names) <= set(ARM_JOINTS):
            raise SafetyRejection(f'planned trajectory contains non-arm joints {names}')
        points = trajectory.joint_trajectory.points
        excursion = 0.0
        if ROLL_JOINT in names and points:
            index = names.index(ROLL_JOINT)
            excursion = max(abs(p.positions[index] - goal[ROLL_JOINT]) for p in points)
        duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9 if points else 0.0
        return {'points': len(points), 'planned_duration_s': duration, 'transit_roll_excursion_rad': excursion}

    def _check_excursion(self, summary):
        limit = self.config.get('max_transit_roll_excursion_rad')
        if limit is not None and summary['transit_roll_excursion_rad'] > limit:
            raise SafetyRejection(f'planned wrist roll excursion {summary["transit_roll_excursion_rad"]:.4f} rad > {limit}')

    def plan(self, joints, start=None):
        """Plan only (no motion); returns a trajectory summary."""
        if not self.move.wait_for_server(timeout_sec=5):
            raise MotionFailure('MoveIt unavailable')
        handle = self._wait(self.move.send_goal_async(self._goal(joints, True, start)), 10)
        if not handle or not handle.accepted:
            raise SafetyRejection('planning request rejected')
        result = self._wait(handle.get_result_async(), 30)
        code = result.result.error_code.val
        if result.status != STATUS_SUCCEEDED or code != MoveItErrorCodes.SUCCESS:
            raise SafetyRejection(f'planning failed: MoveIt {code}, action status {result.status}')
        summary = self._trajectory_summary(result.result.planned_trajectory, joints)
        self._check_excursion(summary)
        return summary, result.result.planned_trajectory

    # ------------------------------------------------------------------ motion
    def cancel_active(self):
        """Cancel our MoveIt goals and every arm_controller trajectory goal."""
        report = {'utc': utc(), 'moveit': [], 'arm_controller': None}
        for handle in list(self.active_handles):
            try:
                response = self._wait(handle.cancel_goal_async(), 5, interruptible=False)
                report['moveit'].append(len(response.goals_canceling))
            except Exception as error:  # report, continue cancelling
                report['moveit'].append(repr(error))
        try:
            if self.cancel_arm.wait_for_service(timeout_sec=2):
                # Empty goal id + zero stamp = cancel all goals of the controller action.
                response = self._wait(self.cancel_arm.call_async(CancelGoal.Request()), 5, interruptible=False)
                report['arm_controller'] = {'return_code': response.return_code,
                                            'goals_canceling': len(response.goals_canceling)}
            else:
                report['arm_controller'] = 'cancel service unavailable'
        except Exception as error:
            report['arm_controller'] = repr(error)
        return report

    def _run_action(self, client, goal, timeout, label):
        # Not interruptible: a goal accepted after we stopped waiting would move untracked.
        try:
            handle = self._wait(client.send_goal_async(goal), 10, interruptible=False)
        except TimeoutError:
            self.last_cancel = self.cancel_active()
            raise MotionFailure(f'{label}: goal acceptance timed out; arm goals cancelled') from None
        if not handle or not handle.accepted:
            raise MotionFailure(f'{label}: goal rejected')
        self.active_handles.append(handle)
        try:
            future = handle.get_result_async()
            end = time.monotonic() + timeout
            while not future.done():
                if self.stop_event.is_set():
                    self.last_cancel = self.cancel_active()
                    raise Interrupted(f'{label}: stop requested; motion goals cancelled, no homing')
                if time.monotonic() > end:
                    self.last_cancel = self.cancel_active()
                    raise MotionFailure(f'{label}: timeout after {timeout:.0f} s; goals cancelled')
                time.sleep(0.01)
            return future.result()
        finally:
            self.active_handles.remove(handle)

    def go(self, joints, label='motion'):
        """Execute an arm joint goal; success needs action success AND measured agreement."""
        before, _, feedback_recovery = wait_for_joint_feedback(
            self.joints, self.config, self.stop_event, f'{label} before motion')
        started = time.monotonic()
        if self.config['motion_execution'] == 'plan_then_execute':
            summary, trajectory = self.plan(joints)
            goal = ExecuteTrajectory.Goal(trajectory=trajectory)
            result = self._run_action(self.execute, goal, self.config['motion_timeout_s'], label)
            code = result.result.error_code.val
        else:
            result = self._run_action(self.move, self._goal(joints, False), self.config['motion_timeout_s'], label)
            code = result.result.error_code.val
            summary = self._trajectory_summary(result.result.planned_trajectory, joints)
        if result.status != STATUS_SUCCEEDED or code != MoveItErrorCodes.SUCCESS:
            raise MotionFailure(f'{label}: MoveIt {code}, action status {result.status}')
        action_s = time.monotonic() - started
        tolerance = self.config['max_joint_position_error_rad']
        deadline = time.monotonic() + self.config['joint_agreement_timeout_s']
        feedback_recoveries_after_motion = []
        while True:
            after, age, recovery = wait_for_joint_feedback(
                self.joints, self.config, self.stop_event, f'{label} after motion')
            if recovery is not None:
                feedback_recoveries_after_motion.append(recovery)
                deadline += recovery['wait_s']
            errors = {name: after.get(name, float('inf')) - joints[name] for name in ARM_JOINTS}
            agree = age < self.config['joint_state_max_age_s'] and max(abs(e) for e in errors.values()) <= tolerance
            if agree or time.monotonic() > deadline:
                break
            time.sleep(0.02)
        log = {'label': label, 'utc': utc(), 'commanded_joints': joints, 'feedback_before': before,
               'feedback_wait_before_motion': feedback_recovery,
               'feedback_waits_after_motion': feedback_recoveries_after_motion,
               'feedback_after': after, 'feedback_age_s': age, 'errors_rad': errors,
               'max_abs_error_rad': max(abs(e) for e in errors.values()), 'action_s': action_s,
               'agreement_s': time.monotonic() - started - action_s, **summary}
        if not agree:
            error = MotionFailure(f'{label}: measured joints differ from command: {errors}')
            error.log = log
            raise error
        return log

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=3)
            self.thread.join(timeout=3)
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
