#!/usr/bin/env python
#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
import unittest

import rospy

from actionlib_msgs.msg import GoalStatus
from actionlib_tutorials.msg import (
    FibonacciAction,
    FibonacciGoal,
    FibonacciFeedback,
    FibonacciResult,
)
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.action import Action

from ros_bt_py.exceptions import BehaviorTreeException

PKG = "ros_bt_py"


class TestActionLeaf(unittest.TestCase):
    def setUp(self):
        self.action_leaf = Action(
            options={
                "action_type": FibonacciAction,
                "goal_type": FibonacciGoal,
                "feedback_type": FibonacciFeedback,
                "result_type": FibonacciResult,
                "action_name": "/fibonacci",
                "wait_for_action_server_seconds": 2.0,
                # time out slightly before the server sends a second piece of
                # feedback
                "timeout_seconds": 0.8,
            }
        )
        self.action_leaf.setup()

    def tearDown(self):
        self.action_leaf.shutdown()

    def testQuickReturn(self):
        # should get as immediate a response as possible (i.e. second tick)
        self.action_leaf.inputs["goal"] = FibonacciGoal(order=0)

        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.1)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.action_leaf.outputs["goal_status"], GoalStatus.SUCCEEDED)
        # Server doesn't send feedback for order=0
        self.assertIsNone(self.action_leaf.outputs["feedback"])
        self.assertSequenceEqual(self.action_leaf.outputs["result"].sequence, [0, 1])

        # Should re-send the goal with the next tick - order=1 also returns
        # immediately, but sends feedback
        self.action_leaf.inputs["goal"] = FibonacciGoal(order=1)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.4)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.RUNNING)
        self.assertEqual(self.action_leaf.outputs["goal_status"], GoalStatus.ACTIVE)
        self.assertSequenceEqual(
            self.action_leaf.outputs["feedback"].sequence, [0, 1, 1]
        )

        self.action_leaf.inputs["goal"] = FibonacciGoal(order=0)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.RUNNING)

        self.action_leaf.untick()

    def testUtility(self):
        expected_bounds = UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )
        self.assertEqual(self.action_leaf.calculate_utility(), expected_bounds)

    def testTimeout(self):
        # The Fibonacci server sleeps for a second after generating each
        # number, so this will time out.
        self.action_leaf.inputs["goal"] = FibonacciGoal(order=2)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.9)
        self.action_leaf.tick()
        self.assertEqual(self.action_leaf.state, NodeMsg.FAILED)
        self.assertEqual(self.action_leaf.outputs["goal_status"], GoalStatus.LOST)
        self.assertSequenceEqual(
            self.action_leaf.outputs["feedback"].sequence, [0, 1, 1]
        )

        self.action_leaf.reset()
        self.assertIsNone(self.action_leaf.outputs["feedback"])
        self.assertEqual(self.action_leaf.outputs["goal_status"], GoalStatus.LOST)
        self.assertIsNone(self.action_leaf.outputs["result"])

    def testActionNotAvailable(self):
        action = Action(
            options={
                "action_type": FibonacciAction,
                "goal_type": FibonacciGoal,
                "feedback_type": FibonacciFeedback,
                "result_type": FibonacciResult,
                "action_name": "/this_action_does_not_exist",
                "wait_for_action_server_seconds": 0.5,
                "timeout_seconds": 0.8,
                "fail_if_not_available": False,
            }
        )

        self.assertRaises(BehaviorTreeException, action.setup)

        action = Action(
            options={
                "action_type": FibonacciAction,
                "goal_type": FibonacciGoal,
                "feedback_type": FibonacciFeedback,
                "result_type": FibonacciResult,
                "action_name": "/this_action_does_not_exist",
                "wait_for_action_server_seconds": 0.5,
                "timeout_seconds": 0.8,
                "fail_if_not_available": True,
            }
        )

        action.inputs["goal"] = FibonacciGoal(order=0)

        self.assertEqual(action.state, NodeMsg.UNINITIALIZED)
        action.setup()
        self.assertEqual(action.state, NodeMsg.IDLE)
        action.tick()
        self.assertEqual(action.state, NodeMsg.FAILED)

        expected_bounds = UtilityBounds()
        self.assertEqual(action.calculate_utility(), expected_bounds)

    def testFailingAction(self):
        action = Action(
            options={
                "action_type": FibonacciAction,
                "goal_type": FibonacciGoal,
                "feedback_type": FibonacciFeedback,
                "result_type": FibonacciResult,
                "action_name": "fibonacci_fail",
                "wait_for_action_server_seconds": 0.5,
                "timeout_seconds": 0.8,
                "fail_if_not_available": False,
            }
        )

        action.inputs["goal"] = FibonacciGoal(order=0)

        self.assertEqual(action.state, NodeMsg.UNINITIALIZED)
        action.setup()
        self.assertEqual(action.state, NodeMsg.IDLE)
        action.tick()
        self.assertEqual(action.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)
        action.tick()
        rospy.sleep(0.5)
        self.assertEqual(action.state, NodeMsg.FAILED)


if __name__ == "__main__":
    rospy.init_node("test_action_leaf", log_level=rospy.DEBUG)
    import rostest
    import sys
    import os

    os.environ["COVERAGE_FILE"] = "%s.%s.coverage" % (PKG, "test_action_leaf")
    rostest.rosrun(
        PKG, "test_action_leaf", TestActionLeaf, sysargs=sys.argv + ["--cov"]
    )
