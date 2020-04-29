#!/usr/bin/env python

from threading import Lock
import unittest
try:
    import unittest.mock as mock
except ImportError:
    import mock

import rospy

from actionlib_msgs.msg import GoalStatus

from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds
from ros_bt_py_msgs.msg import NodeDataWiring, NodeDataLocation
from ros_bt_py_msgs.msg import FindBestExecutorResult

from ros_bt_py.nodes.constant import Constant
from ros_bt_py.nodes.passthrough_node import PassthroughNode
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.nodes.shovable import Shovable
from ros_bt_py.nodes.mock_nodes import MockLeaf

from ros_bt_py.exceptions import BehaviorTreeException

PKG = 'ros_bt_py'


def make_shovable(action_name):
    return Shovable(options={
        'find_best_executor_action': action_name,
        'wait_for_find_best_executor_seconds': 1.0,
        'find_best_executor_timeout_seconds': 1.0,
        'remote_tick_frequency_hz': 20,
        'run_tree_action_timeout_seconds': 1.0,
        'wait_for_run_tree_seconds': 1.0})


class TestShovable(unittest.TestCase):
    def setUp(self):
        self.immediate_success = MockLeaf(options={
            'output_type': int,
            'state_values': [NodeMsg.SUCCEEDED],
            'output_values': [1]})
        self.delayed_success = MockLeaf(options={
            'output_type': int,
            'state_values': [NodeMsg.RUNNING, NodeMsg.SUCCEEDED],
            'output_values': [0, 1]})
        self.immediate_failure = MockLeaf(options={
            'output_type': int,
            'state_values': [NodeMsg.FAILED],
            'output_values': [1]})
        self.infinite_running = MockLeaf(options={
            'output_type': int,
            'state_values': [NodeMsg.RUNNING],
            'output_values': [1]})
        self.constant = Constant(options={
            'constant_type': int,
            'constant_value': 42
        })
        self.inner_passthrough = PassthroughNode(
            name='inner',
            options={
                'passthrough_type': int
            })
        self.outer_passthrough = PassthroughNode(
            name='outer',
            options={
                'passthrough_type': int
            })

    def testWithoutChild(self):
        shovable = make_shovable('has_no_child')

        self.assertRaises(BehaviorTreeException, shovable.setup)

    def testMissingActionServer(self):
        shovable = make_shovable('missing_action_server')
        shovable.add_child(self.immediate_success)

        self.assertRaises(BehaviorTreeException, shovable.setup)

    def testWaitForUtilityResponseTakesTooLong(self):
        shovable = make_shovable('slow_evaluate_utility_local')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        self.assertEqual(shovable.tick(), NodeMsg.RUNNING)
        rospy.sleep(3.0)
        self.assertEqual(shovable.tick(), NodeMsg.FAILED)

    def testLocalExecution(self):
        shovable = make_shovable('evaluate_utility_local')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        self.assertEqual(shovable.tick(), NodeMsg.RUNNING)
        rospy.sleep(0.1)
        self.assertEqual(shovable.tick(), NodeMsg.SUCCEEDED)

        self.assertFalse(shovable.outputs['running_remotely'])

    def testRemoteExecution(self):
        shovable = make_shovable('evaluate_utility_remote')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        # Keep ticking for a maximum of 1 second, but stop if the node succeeds
        # before the limit
        ticks = 1
        new_state = shovable.tick()
        for _ in range(10):
            if new_state == NodeMsg.RUNNING:
                rospy.sleep(0.1)
                new_state = shovable.tick()
                ticks += 1
            else:
                break

        self.assertEqual(new_state, NodeMsg.SUCCEEDED)
        self.assertTrue(shovable.outputs['running_remotely'])

        ticks2 = 1
        new_state = shovable.tick()
        for _ in range(10):
            if new_state == NodeMsg.RUNNING:
                rospy.sleep(0.1)
                new_state = shovable.tick()
                ticks2 += 1
            else:
                break

        self.assertEqual(new_state, NodeMsg.SUCCEEDED)
        self.assertTrue(shovable.outputs['running_remotely'])

        # If anything, the second time around should take fewer ticks, since we
        # don't need to wait for the ActionServer to connect!
        self.assertGreaterEqual(ticks, ticks2)

        self.assertEqual(shovable.shutdown(), NodeMsg.SHUTDOWN)

    def testRemoteExecutionUntick(self):
        shovable = make_shovable('evaluate_utility_remote')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        shovable.tick()

        shovable.untick()

    def testRemoteExecutionFindExecutorError(self):
        shovable = make_shovable('evaluate_utility_remote')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        shovable._find_best_executor_ac.get_state = mock.MagicMock()
        shovable._find_best_executor_ac.get_state.return_value = GoalStatus.ABORTED

        self.assertEqual(shovable.tick(), NodeMsg.FAILED)

    def testUnableToExecuteAnywhere(self):
        shovable = make_shovable('evaluate_utility_remote')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        shovable._find_best_executor_start_time = rospy.Time.now()
        shovable._state = Shovable.WAIT_FOR_UTILITY_RESPONSE

        shovable._find_best_executor_ac.get_state = mock.MagicMock()
        shovable._find_best_executor_ac.get_state.return_value = GoalStatus.SUCCEEDED

        shovable._find_best_executor_ac.get_result = mock.MagicMock()
        shovable._find_best_executor_ac.get_result.return_value = FindBestExecutorResult(
            local_is_best=False,
            best_executor_namespace=False
        )

        self.assertEqual(shovable.tick(), NodeMsg.FAILED)

    def testRemoteNamespace(self):
        shovable = make_shovable('evaluate_utility_remote')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        shovable._find_best_executor_start_time = rospy.Time.now()
        shovable._state = Shovable.WAIT_FOR_UTILITY_RESPONSE

        shovable._find_best_executor_ac.get_state = mock.MagicMock()
        shovable._find_best_executor_ac.get_state.return_value = GoalStatus.SUCCEEDED

        shovable._find_best_executor_ac.get_result = mock.MagicMock()
        shovable._find_best_executor_ac.get_result.return_value = FindBestExecutorResult(
            local_is_best=False,
            best_executor_namespace='remote'
        )

        shovable._remote_namespace = 'remote'

        self.assertRaises(AttributeError, shovable.tick)

    def testRunTreeActionClientNotRunning(self):
        shovable = make_shovable('evaluate_utility_remote')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        shovable._state = Shovable.ACTION_CLIENT_INIT

        shovable._subtree_action_client = mock.MagicMock()
        shovable._subtree_action_client.wait_for_server = mock.MagicMock()
        shovable._subtree_action_client.wait_for_server.return_value = False

        shovable._subtree_action_client_creation_time = rospy.Time.now() - rospy.Duration(10.0)

        self.assertEqual(shovable.tick(), NodeMsg.FAILED)

    def testExecuteRemoteEarlyFail(self):
        shovable = make_shovable('evaluate_utility_remote')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        shovable._state = Shovable.EXECUTE_REMOTE

        shovable._subtree_action_start_time = rospy.Time.now() - rospy.Duration(10.0)

        shovable._subtree_action_client = mock.MagicMock()
        shovable._subtree_action_client.cancel_goal = mock.MagicMock()
        shovable._subtree_action_client.cancel_goal.return_value = False

        self.assertEqual(shovable.tick(), NodeMsg.FAILED)

    def testExecuteRemote(self):
        shovable = make_shovable('evaluate_utility_remote')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        shovable._state = Shovable.EXECUTE_REMOTE

        shovable._subtree_action_start_time = rospy.Time.now()

        shovable._subtree_action_client = mock.MagicMock()
        shovable._subtree_action_client.get_state = mock.MagicMock()
        shovable._subtree_action_client.get_state.return_value = GoalStatus.ABORTED

        shovable._subtree_action_client.cancel_goal = mock.MagicMock()
        shovable._subtree_action_client.cancel_goal.return_value = False

        self.assertEqual(shovable.tick(), NodeMsg.FAILED)

    def testRemoteExecutionWithIO(self):
        shovable = make_shovable('evaluate_utility_remote')
        root = Sequence()\
            .add_child(self.constant)\
            .add_child(shovable
                       .add_child(self.inner_passthrough))\
            .add_child(self.outer_passthrough)

        self.inner_passthrough.wire_data(
            NodeDataWiring(
                source=NodeDataLocation(
                    node_name=self.constant.name,
                    data_kind=NodeDataLocation.OUTPUT_DATA,
                    data_key='constant'),
                target=NodeDataLocation(
                    node_name=self.inner_passthrough.name,
                    data_kind=NodeDataLocation.INPUT_DATA,
                    data_key='in')))

        self.outer_passthrough.wire_data(
            NodeDataWiring(
                source=NodeDataLocation(
                    node_name=self.inner_passthrough.name,
                    data_kind=NodeDataLocation.OUTPUT_DATA,
                    data_key='out'),
                target=NodeDataLocation(
                    node_name=self.outer_passthrough.name,
                    data_kind=NodeDataLocation.INPUT_DATA,
                    data_key='in')))

        root.setup()

        # Do this twice, to ensure we can execute a tree repeatedly
        for _ in range(2):
            # Keep ticking for a maximum of 1 second, but stop if the node succeeds
            # before the limit
            new_state = root.tick()
            for _ in range(10):
                if new_state == NodeMsg.RUNNING:
                    rospy.sleep(0.1)
                    new_state = root.tick()
                else:
                    break

            self.assertEqual(new_state, NodeMsg.SUCCEEDED)
            self.assertTrue(shovable.outputs['running_remotely'])
            self.assertEqual(self.outer_passthrough.outputs['out'],
                             self.constant.outputs['constant'])

    def testCalculateUtility(self):
        shovable = make_shovable('evaluate_utility_local')
        shovable.add_child(self.immediate_success)

        shovable.setup()

        self.assertEqual(shovable.calculate_utility(),
                         UtilityBounds(can_execute=False,
                                       has_lower_bound_success=True,
                                       has_upper_bound_success=True,
                                       has_lower_bound_failure=True,
                                       has_upper_bound_failure=True))

        shovable = make_shovable('not_available')

        self.assertEqual(shovable.calculate_utility(),
                         UtilityBounds())


if __name__ == '__main__':
    rospy.init_node('test_shovable_decorator')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_shovable_decorator')
    rostest.rosrun(PKG, 'test_shovable_decorator', TestShovable,
                   sysargs=sys.argv + ['--cov'])
