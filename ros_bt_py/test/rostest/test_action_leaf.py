#!/usr/bin/env python2.7
import unittest

import rospy

from actionlib_msgs.msg import GoalStatus
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal, FibonacciFeedback, FibonacciResult
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.action import Action

PKG = 'ros_bt_py'


class TestActionLeaf(unittest.TestCase):
    def setUp(self):
        self.action_leaf = Action(options={
            'action_type': FibonacciAction,
            'goal_type': FibonacciGoal,
            'feedback_type': FibonacciFeedback,
            'result_type': FibonacciResult,
            'action_name': '/fibonacci',
            'wait_for_action_server_seconds': 1.0,
            # time out slightly before the server sends a second piece of
            # feedback
            'timeout_seconds': 0.8
            })
        self.action_leaf.setup()

    def tearDown(self):
        self.action_leaf.shutdown()

    def testQuickReturn(self):
        # should get as immediate a response as possible (i.e. second tick)
        self.action_leaf.inputs['goal'] = FibonacciGoal(order=0)

        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.1)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.action_leaf.outputs['goal_status'], GoalStatus.SUCCEEDED)
        # Server doesn't send feedback for order=0
        self.assertIsNone(self.action_leaf.outputs['feedback'])
        self.assertSequenceEqual(self.action_leaf.outputs['result'].sequence, [0, 1])

        # Should re-send the goal with the next tick - order=1 also returns
        # immediately, but sends feedback
        self.action_leaf.inputs['goal'] = FibonacciGoal(order=1)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.4)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.RUNNING)
        self.assertEqual(self.action_leaf.outputs['goal_status'], GoalStatus.ACTIVE)
        self.assertSequenceEqual(self.action_leaf.outputs['feedback'].sequence, [0, 1, 1])

        self.action_leaf.untick()

    def testTimeout(self):
        # The Fibonacci server sleeps for a second after generating each
        # number, so this will time out.
        self.action_leaf.inputs['goal'] = FibonacciGoal(order=2)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.9)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.FAILED)
        self.assertEqual(self.action_leaf.outputs['goal_status'], GoalStatus.LOST)
        self.assertSequenceEqual(self.action_leaf.outputs['feedback'].sequence, [0, 1, 1])

        self.action_leaf.reset()
        self.assertIsNone(self.action_leaf.outputs['feedback'])
        self.assertEqual(self.action_leaf.outputs['goal_status'], GoalStatus.LOST)
        self.assertIsNone(self.action_leaf.outputs['result'])


if __name__ == '__main__':
    rospy.init_node('test_action_leaf')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_action_leaf')
    rostest.rosrun(PKG, 'test_action_leaf', TestActionLeaf,
                   sysargs=sys.argv + ['--cov'])
