#!/usr/bin/env python2.7
from threading import Lock
import unittest

import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.nodes.shovable import Shovable
from ros_bt_py.nodes.mock_nodes import MockLeaf

PKG = 'ros_bt_py'


def make_shovable(service_name):
    return Shovable(options={
        'utility_evaluator_service': service_name,
        'remote_tick_frequency_hz': 20.0,
        'wait_for_service_seconds': 1.0,
        'action_timeout_seconds': 1.0})


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

if __name__ == '__main__':
    rospy.init_node('test_shovable_decorator')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_shovable_decorator')
    rostest.rosrun(PKG, 'test_shovable_decorator', TestShovable,
                   sysargs=sys.argv + ['--cov'])
