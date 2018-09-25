#!/usr/bin/env python2.7

from threading import Lock
import unittest

import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeDataWiring, NodeDataLocation

from ros_bt_py.nodes.constant import Constant
from ros_bt_py.nodes.passthrough_node import PassthroughNode
from ros_bt_py.nodes.sequence import Sequence
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


if __name__ == '__main__':
    rospy.init_node('test_shovable_decorator')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_shovable_decorator')
    rostest.rosrun(PKG, 'test_shovable_decorator', TestShovable,
                   sysargs=sys.argv + ['--cov'])
