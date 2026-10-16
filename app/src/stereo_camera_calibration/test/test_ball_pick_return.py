import importlib.util
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes

SOURCE = Path(__file__).resolve().parents[1] / 'scripts' / 'ball_pick_node.py'
spec = importlib.util.spec_from_file_location('ball_pick_under_test', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
SRDF = (SOURCE.parents[2] / 'hiwonder_xarm_esp32_moveit_config'
        / 'config' / 'hiwonder_esp32.srdf')


def completed(result):
    future = Future()
    future.set_result(result)
    return future


class ReturnAfterPickTest(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(module.BallPickNode)
        self.node.return_to_default = True
        self.node.approach_only = False
        self.node.default_joint_positions = module.load_joint_state(SRDF, 'arm', 'default')
        self.node.group_name = 'arm'
        self.node.planning_attempts = 2
        self.node.allowed_planning_time = 3.0
        self.node.velocity_scaling = 0.1
        self.node.acceleration_scaling = 0.1
        self.node.busy = True
        self.node.active_target = object()
        self.node.base_frame = 'base_link'
        self.node.frozen_target = object()
        self.node.marker_publisher = Mock()
        self.node.move_group = Mock()
        self.node.move_group.send_goal_async.return_value = Future()
        self.node.gripper = Mock()
        self.logger = Mock()
        self.log_patch = patch.object(module.BallPickNode, 'get_logger', return_value=self.logger)
        self.log_patch.start()
        self.addCleanup(self.log_patch.stop)

    def close(self, reached=True, stalled=False, status=GoalStatus.STATUS_SUCCEEDED):
        self.node.gripper_closed(completed(SimpleNamespace(
            status=status,
            result=SimpleNamespace(reached_goal=reached, stalled=stalled, position=-0.01),
        )))

    def test_close_returns_only_arm_and_stays_busy_until_default_is_reached(self):
        self.close()
        self.assertTrue(self.node.busy)
        self.node.move_group.send_goal_async.assert_called_once()
        goal = self.node.move_group.send_goal_async.call_args.args[0]
        self.assertFalse(goal.planning_options.plan_only)
        self.assertEqual(goal.request.group_name, 'arm')
        self.assertEqual(goal.request.max_velocity_scaling_factor, 0.1)
        joints = goal.request.goal_constraints[0].joint_constraints
        self.assertEqual([(j.joint_name, j.position) for j in joints],
                         self.node.default_joint_positions)
        self.assertTrue(all('gripper' not in j.joint_name for j in joints))
        self.node.gripper.send_goal_async.assert_not_called()
        result = Future()
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: result)
        self.node.move_group.send_goal_async.return_value.set_result(handle)
        self.assertTrue(self.node.busy)
        result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                          result=SimpleNamespace(error_code=SimpleNamespace(val=1))))
        self.assertFalse(self.node.busy)
        self.assertIsNone(self.node.active_target)

    def test_successful_object_stall_also_returns_to_default(self):
        self.close(reached=False, stalled=True)
        self.node.move_group.send_goal_async.assert_called_once()
        self.assertTrue(self.node.busy)

    def test_approach_stops_with_gripper_open_without_return(self):
        self.node.approach_only = True
        self.node.arm_finished(completed(SimpleNamespace(
            result=SimpleNamespace(error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS)),
        )))
        self.assertFalse(self.node.busy)
        self.node.gripper.send_goal_async.assert_not_called()
        self.node.move_group.send_goal_async.assert_not_called()

    def test_failed_approach_does_not_close_or_return(self):
        self.node.approach_only = True
        self.node.arm_finished(completed(SimpleNamespace(
            result=SimpleNamespace(error_code=SimpleNamespace(val=MoveItErrorCodes.CONTROL_FAILED)),
        )))
        self.assertFalse(self.node.busy)
        self.node.gripper.send_goal_async.assert_not_called()
        self.node.move_group.send_goal_async.assert_not_called()

    def start_sag(self, actual):
        self.node.gripper_closed_position = -0.013651
        self.node.gripper_max_effort = 10.0
        self.node.gripper.send_goal_async.return_value = Future()
        self.node.sag_iterations = 2
        self.node.sag_tolerance = 0.005
        self.node.sag_max_correction = 0.08
        self.node.sag_move_duration = 0.6
        self.node.sag_timer = Mock()
        self.node.arm_trajectory = Mock()
        self.node.arm_trajectory.send_goal_async.return_value = Future()
        self.node.sag_target = {'a': 1.0, 'b': -0.5}
        self.node.sag_command = dict(self.node.sag_target)
        self.node.sag_iteration = 0
        self.node.latest_joints = actual
        self.node.check_sag()

    def test_sag_commands_remaining_joint_error_before_closing(self):
        self.start_sag({'a': 0.97, 'b': -0.5})
        self.node.gripper.send_goal_async.assert_not_called()
        goal = self.node.arm_trajectory.send_goal_async.call_args.args[0]
        self.assertEqual(goal.trajectory.joint_names, ['a', 'b'])
        self.assertAlmostEqual(goal.trajectory.points[0].positions[0], 1.03)
        self.assertAlmostEqual(goal.trajectory.points[0].positions[1], -0.5)
        self.assertEqual(self.node.sag_iteration, 1)

    def test_small_sag_closes_without_correction(self):
        self.start_sag({'a': 0.998, 'b': -0.501})
        self.node.arm_trajectory.send_goal_async.assert_not_called()
        self.node.gripper.send_goal_async.assert_called_once()

    def test_unsafe_sag_correction_is_skipped(self):
        self.start_sag({'a': 0.8, 'b': -0.5})
        self.node.arm_trajectory.send_goal_async.assert_not_called()
        self.node.gripper.send_goal_async.assert_called_once()

    def test_grasp_offsets_move_outward_and_to_the_arm_left(self):
        self.node.grasp_radial_offset = 0.028
        self.node.grasp_lateral_offset = 0.005
        position = SimpleNamespace(x=0.0, y=-0.25, z=0.01)
        self.node.apply_grasp_offsets(position)
        self.assertAlmostEqual(position.y, -0.278)
        # Facing -y from the base, the arm's left is +x.
        self.assertAlmostEqual(position.x, 0.005)
        self.assertAlmostEqual(position.z, 0.01)

    def test_descent_goal_adds_measured_sag_and_spreads_time(self):
        goal = module.make_descent_goal(
            ['a', 'b'], [[1.0, 2.0], [1.1, 2.2]], {'a': 0.02}, 2.0)
        self.assertEqual(goal.trajectory.joint_names, ['a', 'b'])
        self.assertEqual([list(p.positions) for p in goal.trajectory.points],
                         [[1.02, 2.0], [1.12, 2.2]])
        self.assertEqual([p.time_from_start.sec for p in goal.trajectory.points], [1, 2])

    def test_pregrasp_starts_descent_instead_of_closing(self):
        self.node.stage = 'pregrasp'
        self.node.base_frame = 'base_link'
        self.node.tip_link = 'gripper_tcp_link'
        self.node.sag_target = {'a': 1.0}
        self.node.sag_command = {'a': 1.03}
        self.node.fk_client = Mock()
        self.node.fk_client.call_async.return_value = Future()
        self.node.arm_reached()
        self.node.gripper.send_goal_async.assert_not_called()
        self.node.fk_client.call_async.assert_called_once()
        self.assertAlmostEqual(self.node.descent_delta['a'], 0.03)

    def test_descent_without_measured_pose_aborts(self):
        self.node.stage = 'pregrasp'
        self.node.sag_target = {}
        self.node.arm_reached()
        self.assertFalse(self.node.busy)
        self.node.gripper.send_goal_async.assert_not_called()

    def descent_node(self):
        self.node.group_name = 'arm'
        self.node.tip_link = 'gripper_tcp_link'
        self.node.base_frame = 'base_link'
        self.node.pregrasp_height = 0.004
        self.node.descent_step = 0.002
        self.node.descent_max_joint_step = 0.05
        self.node.descent_duration = 1.0
        self.node.sag_target = {'a': 1.0}
        self.node.sag_command = {'a': 1.02}
        self.node.descent_delta = {'a': 0.02}
        self.node.ik_client = Mock()
        self.node.ik_client.call_async.return_value = Future()
        self.node.arm_trajectory = Mock()
        self.node.arm_trajectory.send_goal_async.return_value = Future()
        pose = SimpleNamespace(pose=module.Pose())
        pose.pose.position.z = 0.05
        self.node.descent_fk_done(completed(SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS), pose_stamped=[pose])))

    @staticmethod
    def ik_answer(value, code=MoveItErrorCodes.SUCCESS):
        return completed(SimpleNamespace(
            error_code=SimpleNamespace(val=code),
            solution=SimpleNamespace(joint_state=SimpleNamespace(name=['a'], position=[value]))))

    def test_stepwise_descent_executes_with_measured_sag(self):
        self.descent_node()
        first = self.node.ik_client.call_async.call_args.args[0]
        self.assertAlmostEqual(first.ik_request.pose_stamped.pose.position.z, 0.048)
        self.node.descent_ik_done(self.ik_answer(1.01))
        self.node.descent_ik_done(self.ik_answer(1.02))
        goal = self.node.arm_trajectory.send_goal_async.call_args.args[0]
        self.assertEqual([list(p.positions) for p in goal.trajectory.points], [[1.03], [1.04]])
        self.assertEqual(self.node.stage, 'final')
        self.assertEqual(self.node.sag_command, {'a': 1.04})

    def test_descent_stops_when_ik_fails_or_jumps(self):
        self.descent_node()
        self.node.descent_ik_done(self.ik_answer(0.0, code=MoveItErrorCodes.NO_IK_SOLUTION))
        self.assertFalse(self.node.busy)
        self.node.busy = True
        self.descent_node()
        self.node.descent_ik_done(self.ik_answer(1.2))
        self.assertFalse(self.node.busy)
        self.node.arm_trajectory.send_goal_async.assert_not_called()

    def reach_node(self):
        self.node.stage = 'pregrasp'
        self.node.pregrasp_height = 0.03
        self.node.tip_link = 'gripper_tcp_link'
        self.node.latest_joints = {}
        self.node.active_target = SimpleNamespace(pose=SimpleNamespace(
            position=module.PoseStamped().pose.position))
        self.node.active_target.pose.position.x = -0.126
        self.node.active_target.pose.position.y = -0.281
        self.node.ik_client = Mock()
        self.node.ik_client.call_async.return_value = Future()
        self.node.gripper_open_position = 0.0137
        self.node.gripper_max_effort = 10.0
        self.node.gripper.send_goal_async.return_value = Future()
        self.node.find_grasp_ik(0)

    def ik_reply(self, ok=True):
        names = [name for name, _ in self.node.default_joint_positions]
        return completed(SimpleNamespace(
            error_code=SimpleNamespace(
                val=MoveItErrorCodes.SUCCESS if ok else MoveItErrorCodes.NO_IK_SOLUTION),
            solution=SimpleNamespace(joint_state=SimpleNamespace(
                name=names, position=[0.1] * len(names)))))

    def test_unreachable_ball_is_refused_before_any_motion(self):
        self.reach_node()
        for attempt in range(len(module.BallPickNode.IK_SEED_OFFSETS)):
            self.node.grasp_ik_done(self.ik_reply(ok=False), attempt)
        self.assertFalse(self.node.busy)
        self.assertIn('hors de portee', self.logger.error.call_args.args[0])
        self.node.gripper.send_goal_async.assert_not_called()
        self.node.move_group.send_goal_async.assert_not_called()

    def test_reachable_ball_plans_above_from_grasp_then_opens(self):
        self.reach_node()
        self.node.grasp_ik_done(self.ik_reply(), 0)
        above = self.node.ik_client.call_async.call_args.args[0]
        self.assertAlmostEqual(above.ik_request.pose_stamped.pose.position.z, 0.03)
        self.assertEqual(list(above.ik_request.robot_state.joint_state.position),
                         [0.1] * 5)
        self.node.gripper.send_goal_async.assert_not_called()
        self.node.pregrasp_ik_done(self.ik_reply())
        self.node.gripper.send_goal_async.assert_called_once()
        self.assertEqual(len(self.node.pregrasp_joints), 5)

    def test_failed_or_canceled_closure_does_not_start_return(self):
        for reached, status in [(False, GoalStatus.STATUS_SUCCEEDED),
                                (True, GoalStatus.STATUS_CANCELED),
                                (True, GoalStatus.STATUS_ABORTED)]:
            self.node.busy = True
            self.close(reached=reached, status=status)
            self.node.move_group.send_goal_async.assert_not_called()
            self.assertFalse(self.node.busy)

    def test_failed_return_aborts_without_retry_or_gripper_command(self):
        self.close()
        self.node.default_finished(completed(SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED,
            result=SimpleNamespace(error_code=SimpleNamespace(val=MoveItErrorCodes.CONTROL_FAILED)),
        )))
        self.assertFalse(self.node.busy)
        self.node.move_group.send_goal_async.assert_called_once()
        self.node.gripper.send_goal_async.assert_not_called()
        self.logger.error.assert_called_once()

    def test_return_can_be_disabled(self):
        self.node.return_to_default = False
        self.close()
        self.assertFalse(self.node.busy)
        self.node.move_group.send_goal_async.assert_not_called()

    def test_invalid_named_pose_is_rejected(self):
        for body in ['', '<joint name="j" value="nan"/>',
                     '<joint name="j" value="0"/><joint name="j" value="1"/>']:
            tree = ET.ElementTree(ET.fromstring(
                f'<robot><group_state name="default" group="arm">{body}</group_state></robot>'
            ))
            with patch.object(module.ET, 'parse', return_value=tree):
                with self.assertRaises(ValueError):
                    module.load_joint_state(SRDF, 'arm', 'default')

    def test_finish_and_abort_delete_only_frozen_pick_markers(self):
        for method in (self.node.finish, self.node.abort):
            self.node.frozen_target = object()
            self.node.marker_publisher.reset_mock()
            method('done')
            self.assertIsNone(self.node.frozen_target)
            markers = self.node.marker_publisher.publish.call_args.args[0].markers
            self.assertEqual([(m.ns, m.id, m.action) for m in markers],
                             [('pick_target', 10, module.Marker.DELETE),
                              ('pick_target', 11, module.Marker.DELETE)])


if __name__ == '__main__':
    unittest.main()
