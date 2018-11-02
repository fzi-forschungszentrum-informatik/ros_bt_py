#!/usr/bin/env python2.7

from threading import Lock
import unittest

import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.nodes.parallel_if_remote import ParallelIfRemote
from ros_bt_py.nodes.shovable import Shovable
from ros_bt_py.nodes.mock_nodes import MockLeaf

PKG = 'ros_bt_py'


def make_shovable(action_name):
    return Shovable(options={
        'find_best_executor_action': action_name,
        'wait_for_find_best_executor_seconds': 1.0,
        'find_best_executor_timeout_seconds': 1.0,
        'remote_tick_frequency_hz': 20,
        'run_tree_action_timeout_seconds': 1.0,
        'wait_for_run_tree_seconds': 1.0})


class TestParallelIfRemote(unittest.TestCase):
    def setUp(self):
        self.infinite_running = MockLeaf(options={
            'output_type': int,
            'state_values': [NodeMsg.RUNNING],
            'output_values': [1]})

        self.immediate_success = MockLeaf(options={
            'output_type': int,
            'state_values': [NodeMsg.SUCCEEDED],
            'output_values': [1]})

        self.parseq = ParallelIfRemote()

    def testLocalExecution(self):
        shovable = make_shovable('evaluate_utility_local').add_child(
            self.infinite_running)

        self.parseq\
            .add_child(shovable)\
            .add_child(self.immediate_success)

        self.parseq.setup()
        self.assertEqual(self.parseq.state, NodeMsg.IDLE)
        self.assertEqual(self.immediate_success.state, NodeMsg.IDLE)
        self.assertEqual(shovable.state, NodeMsg.IDLE)
        # Shovable should not set up its child yet
        self.assertEqual(self.infinite_running.state, NodeMsg.UNINITIALIZED)

        for _ in range(10):
            new_state = self.parseq.tick()
            self.assertEqual(new_state, NodeMsg.RUNNING)
            self.assertEqual(self.immediate_success.tick_count, 0)
            rospy.sleep(0.1)

        self.parseq.shutdown()
        self.assertEqual(self.parseq.state, NodeMsg.SHUTDOWN)

    def testRemoteExecution(self):
        shovable = make_shovable('evaluate_utility_remote').add_child(
            self.infinite_running)

        self.parseq\
            .add_child(shovable)\
            .add_child(self.immediate_success)

        self.parseq.setup()
        self.assertEqual(self.parseq.state, NodeMsg.IDLE)
        self.assertEqual(self.immediate_success.state, NodeMsg.IDLE)
        self.assertEqual(shovable.state, NodeMsg.IDLE)
        # Shovable should not set up its child yet
        self.assertEqual(self.infinite_running.state, NodeMsg.UNINITIALIZED)

        for _ in range(10):
            new_state = self.parseq.tick()
            self.assertEqual(new_state, NodeMsg.RUNNING)
            rospy.sleep(0.1)

        # Even though the shovable is still running, it's running
        # remotely, so the ParallelIfRemote's second child should have
        # been ticked
        self.assertTrue(shovable.outputs['running_remotely'])
        self.assertGreater(self.immediate_success.tick_count, 0)

        self.parseq.shutdown()
        self.assertEqual(self.parseq.state, NodeMsg.SHUTDOWN)


if __name__ == '__main__':
    rospy.init_node('test_parallel_if_remote')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_parallel_if_remote')
    rostest.rosrun(PKG, 'test_parallel_if_remote', TestParallelIfRemote,
                   sysargs=sys.argv + ['--cov'])
