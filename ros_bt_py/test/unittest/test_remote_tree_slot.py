# Copyright 2018-2023 FZI Forschungszentrum Informatik
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
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
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


import unittest

try:
    import unittest.mock as mock
except ImportError:
    import mock
import rospy

from ros_bt_py_msgs.msg import Node, Tree
from ros_bt_py_msgs.msg import RunTreeGoal
from ros_bt_py_msgs.srv import EvaluateUtilityRequest
from ros_bt_py_msgs.srv import ControlTreeExecutionRequest, ControlTreeExecutionResponse

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.remote_tree_slot import RemoteTreeSlot
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.nodes.mock_nodes import MockLeaf, MockUtilityLeaf


class MockGoalHandle(object):
    """Simple Facsimile of the GoalHandle used by actionlib.ActionServer"""

    INIT = 0
    RUNNING = 1
    REJECTED = 2
    CANCELED = 3
    SUCCEEDED = 4
    ACCEPTED = 5

    def __init__(self, goal, goal_id):
        self.state = MockGoalHandle.INIT
        self.result = None
        self.goal = goal
        self.goal_id = goal_id

    def get_goal(self):
        return self.goal

    def get_goal_id(self):
        return self.goal_id

    def set_accepted(self, result=None):
        self.state = MockGoalHandle.ACCEPTED
        self.result = result

    def set_rejected(self, result=None, text=""):
        self.state = MockGoalHandle.REJECTED
        self.result = result

    def set_succeeded(self, result=None):
        self.state = MockGoalHandle.SUCCEEDED
        self.result = result

    def set_failed(self, result=None):
        self.state = MockGoalHandle.FAILED
        self.result = result

    def set_canceled(self, result=None):
        self.state = MockGoalHandle.CANCELED
        self.result = result


class TestRemoteTreeSlot(unittest.TestCase):
    def setUp(self):
        self.cheap_fail = MockUtilityLeaf(
            name="cheap_fail",
            options={
                "can_execute": True,
                "utility_lower_bound_success": 5.0,
                "utility_upper_bound_success": 10.0,
                "utility_lower_bound_failure": 1.0,
                "utility_upper_bound_failure": 2.0,
            },
        )
        self.cheap_success = MockUtilityLeaf(
            name="cheap_success",
            options={
                "can_execute": True,
                "utility_lower_bound_success": 1.0,
                "utility_upper_bound_success": 2.0,
                "utility_lower_bound_failure": 5.0,
                "utility_upper_bound_failure": 10.0,
            },
        )

        self.utility_root = (
            Sequence().add_child(self.cheap_fail).add_child(self.cheap_success)
        )

        self.run_then_fail = MockLeaf(
            name="run_then_fail",
            options={
                "output_type": int,
                "state_values": [Node.RUNNING, Node.FAILED],
                "output_values": [1, 1],
            },
        )
        self.run_then_succeed = MockLeaf(
            name="run_then_succeed",
            options={
                "output_type": int,
                "state_values": [Node.RUNNING, Node.SUCCEEDED],
                "output_values": [1, 1],
            },
        )
        self.execute_root = (
            Sequence().add_child(self.run_then_succeed).add_child(self.run_then_fail)
        )

        self.slot_state = None
        self.remote_slot = RemoteTreeSlot(publish_slot_state=self.update_state)

    def update_state(self, msg):
        self.slot_state = msg

    def testEvalUtility(self):
        utility_tree, _, _ = self.utility_root.get_subtree_msg()
        res = self.remote_slot.evaluate_utility_handler(
            EvaluateUtilityRequest(utility_tree)
        )

        self.assertEqual(res.utility, self.utility_root.calculate_utility())

        # Try to evaluate utility of a tree that does not exist
        utility_tree = Tree(path="package://ros_bt_py/etc/trees/notareal.file")
        res = self.remote_slot.evaluate_utility_handler(
            EvaluateUtilityRequest(utility_tree)
        )

        self.assertEqual(res.utility.can_execute, False)

    def testLoadRemoteTree(self):
        execute_tree, _, _ = self.execute_root.get_subtree_msg()

        self.assertFalse(self.slot_state.tree_in_slot)
        self.assertFalse(self.slot_state.tree_running)

        gh = MockGoalHandle(RunTreeGoal(tree=execute_tree), goal_id=1)
        self.assertEqual(gh.state, MockGoalHandle.INIT)

        self.remote_slot.run_tree_handler(gh)

        self.assertEqual(gh.state, MockGoalHandle.ACCEPTED)

        # The RemoteTreeSlot should report a tree has been loaded (but
        # it's not running yet)
        self.assertTrue(self.slot_state.tree_in_slot)
        self.assertFalse(self.slot_state.tree_running)

        # If we give another goal before the first is done (or
        # canceled), we get rejected
        gh_2 = MockGoalHandle(RunTreeGoal(tree=execute_tree), goal_id=2)
        self.assertEqual(gh_2.state, MockGoalHandle.INIT)

        self.remote_slot.run_tree_handler(gh_2)
        self.assertEqual(gh_2.state, MockGoalHandle.REJECTED)

        # If we cancel the first goal...
        self.remote_slot.cancel_run_tree_handler(gh)
        self.assertEqual(gh.state, MockGoalHandle.CANCELED)

        # We should be notified that no tree is loaded
        self.assertFalse(self.slot_state.tree_in_slot)

        # If we then retry, we should be good to go!
        self.remote_slot.run_tree_handler(gh_2)
        self.assertEqual(gh_2.state, MockGoalHandle.ACCEPTED)
        self.assertTrue(self.slot_state.tree_in_slot)

        # Try to evaluate utility when another tree is already loaded
        utility_tree, _, _ = self.utility_root.get_subtree_msg()
        res = self.remote_slot.evaluate_utility_handler(
            EvaluateUtilityRequest(utility_tree)
        )

        self.assertEqual(res.utility.can_execute, False)

    def testExecuteRemoteTree(self):
        execute_tree, _, _ = self.execute_root.get_subtree_msg()

        # Nothing to do, succeeding
        res = self.remote_slot.control_tree_execution_handler(
            ControlTreeExecutionRequest(command=ControlTreeExecutionRequest.STOP)
        )
        self.assertTrue(get_success(res), get_error_message(res))

        gh = MockGoalHandle(RunTreeGoal(tree=execute_tree), goal_id=1)
        self.assertEqual(gh.state, MockGoalHandle.INIT)

        self.remote_slot.run_tree_handler(gh)

        self.assertEqual(gh.state, MockGoalHandle.ACCEPTED)
        self.assertTrue(self.slot_state.tree_in_slot)
        self.assertFalse(self.slot_state.tree_running)
        self.assertFalse(self.slot_state.tree_finished)

        res = self.remote_slot.control_tree_execution_handler(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_PERIODICALLY,
                tick_frequency_hz=10,
            )
        )
        self.assertFalse(get_success(res), get_error_message(res))

        rospy.sleep(0.2)

        res = self.remote_slot.control_tree_execution_handler(
            ControlTreeExecutionRequest(command=ControlTreeExecutionRequest.STOP)
        )
        self.assertTrue(get_success(res), get_error_message(res))

        res = self.remote_slot.control_tree_execution_handler(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10,
            )
        )

        self.assertTrue(get_success(res), get_error_message(res))
        # If the ControlTreeExecution call was successful, the tree
        # should now be running, but not finished yet
        self.assertTrue(self.slot_state.tree_running)
        self.assertFalse(self.slot_state.tree_finished)

        self.assertEqual(gh.state, MockGoalHandle.ACCEPTED)

        # Should need 4 ticks to get a result, so only 0.4 seconds,
        # but let's play it safe here
        rospy.sleep(1.0)

        self.assertEqual(gh.state, MockGoalHandle.SUCCEEDED)
        # Tree is done, so not running any more, but finished
        self.assertFalse(self.slot_state.tree_running)
        self.assertTrue(self.slot_state.tree_finished)

        # Try to run a tree that does not exist
        not_available_tree = Tree(path="package://ros_bt_py/etc/trees/notareal.file")
        gh = MockGoalHandle(RunTreeGoal(tree=not_available_tree), goal_id=1)
        self.assertEqual(gh.state, MockGoalHandle.INIT)

        self.remote_slot.run_tree_handler(gh)

        self.assertEqual(gh.state, MockGoalHandle.REJECTED)
        self.assertFalse(self.slot_state.tree_in_slot)
        self.assertFalse(self.slot_state.tree_running)

    def testCancelBeforeDone(self):
        execute_tree, _, _ = self.execute_root.get_subtree_msg()

        gh = MockGoalHandle(RunTreeGoal(tree=execute_tree), goal_id=1)
        self.assertEqual(gh.state, MockGoalHandle.INIT)

        self.remote_slot.run_tree_handler(gh)

        self.assertEqual(gh.state, MockGoalHandle.ACCEPTED)

        res = self.remote_slot.control_tree_execution_handler(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10,
            )
        )

        self.assertTrue(get_success(res), get_error_message(res))
        self.assertEqual(self.remote_slot.tree_manager.get_state(), Tree.TICKING)
        # If the ControlTreeExecution call was successful, the tree
        # should now be running, but not finished yet
        self.assertTrue(self.slot_state.tree_running)
        self.assertFalse(self.slot_state.tree_finished)

        self.remote_slot.cancel_run_tree_handler(gh)
        self.assertEqual(gh.state, MockGoalHandle.CANCELED)

        self.remote_slot.cancel_run_tree_handler(gh)
        self.assertEqual(gh.state, MockGoalHandle.CANCELED)

        # Ensure tree is actually stopped
        self.assertEqual(self.remote_slot.tree_manager.get_state(), Tree.EDITABLE)
        self.assertFalse(self.slot_state.tree_running)
        self.assertFalse(self.slot_state.tree_finished)

    def testRunTreeHandler(self):
        self.remote_slot.tree_manager.control_execution = mock.MagicMock()
        self.remote_slot.tree_manager.control_execution.return_value = (
            ControlTreeExecutionResponse(success=False)
        )

        execute_tree, _, _ = self.execute_root.get_subtree_msg()

        gh = MockGoalHandle(RunTreeGoal(tree=execute_tree), goal_id=1)
        self.assertEqual(gh.state, MockGoalHandle.INIT)

        self.remote_slot.run_tree_handler(gh)

    def testControlTreeExecutionHandler(self):
        execute_tree, _, _ = self.execute_root.get_subtree_msg()

        gh = MockGoalHandle(RunTreeGoal(tree=execute_tree), goal_id=1)
        self.assertEqual(gh.state, MockGoalHandle.INIT)

        self.remote_slot.run_tree_handler(gh)

        self.remote_slot.tree_manager.control_execution = mock.MagicMock()
        self.remote_slot.tree_manager.control_execution.return_value = (
            ControlTreeExecutionResponse(success=False)
        )

        res = self.remote_slot.control_tree_execution_handler(
            ControlTreeExecutionRequest(command=ControlTreeExecutionRequest.TICK_ONCE)
        )
        self.assertFalse(get_success(res))

    def testCancelRunTreeHandler(self):
        execute_tree, _, _ = self.execute_root.get_subtree_msg()

        gh = MockGoalHandle(RunTreeGoal(tree=execute_tree), goal_id=1)
        self.assertEqual(gh.state, MockGoalHandle.INIT)

        self.remote_slot.run_tree_handler(gh)

        self.remote_slot.tree_manager.control_execution = mock.MagicMock()
        self.remote_slot.tree_manager.control_execution.return_value = (
            ControlTreeExecutionResponse(success=False)
        )

        self.assertRaises(Exception, self.remote_slot.cancel_run_tree_handler, gh)

        self.remote_slot.update_tree_msg = mock.MagicMock()
        self.remote_slot.latest_tree = Tree()

        self.remote_slot.tree_manager.control_execution.return_value = (
            ControlTreeExecutionResponse(success=True)
        )

        self.remote_slot.cancel_run_tree_handler(gh)
        self.assertEqual(gh.state, MockGoalHandle.CANCELED)


def get_success(response):
    if isinstance(response, dict):
        return response["success"]

    return response.success


def get_error_message(response):
    if isinstance(response, dict):
        return response["error_message"]

    return response.error_message
