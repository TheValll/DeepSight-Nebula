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
