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

try:
    import unittest.mock as mock
except ImportError:
    import mock

import rospy
import rosservice

from std_msgs.msg import Bool

from ros_bt_py_msgs.srv import LoadTree, LoadTreeRequest
from ros_bt_py_msgs.srv import ControlTreeExecution, ControlTreeExecutionRequest
from ros_bt_py_msgs.srv import ControlTreeExecutionResponse
from ros_bt_py_msgs.msg import Tree, Node as NodeMsg, UtilityBounds

from ros_bt_py.nodes.remote_slot import RemoteSlot

from ros_bt_py.helpers import json_encode, json_decode
from ros_bt_py.ros_helpers import AsyncServiceProxy

PKG = "ros_bt_py"


@unittest.skip("Testcase fails at random times.")
class TestShoveTree(unittest.TestCase):
    def setUp(self):
        self.load_tree_proxy = rospy.ServiceProxy("/tree_node/load_tree", LoadTree)

        self.control_tree_proxy = rospy.ServiceProxy(
            "/tree_node/control_tree_execution", ControlTreeExecution
        )

        self.control_slot_proxy = rospy.ServiceProxy(
            "/remote_slot/remote_slot/control_slot_execution", ControlTreeExecution
        )

        # These throw an exception if they time out, failing the test
        self.load_tree_proxy.wait_for_service(1.0)
        self.control_tree_proxy.wait_for_service(1.0)
        self.control_slot_proxy.wait_for_service(1.0)

        self.tree_msg = None
        self.tree_sub = rospy.Subscriber(
            "/tree_node/tree", Tree, self.update_tree_msg, queue_size=1
        )

    def tearDown(self):
        stop_tree_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(command=ControlTreeExecutionRequest.SHUTDOWN)
        )
        self.assertTrue(stop_tree_res.success, stop_tree_res.error_message)
        stop_slot_res = self.control_slot_proxy(
            ControlTreeExecutionRequest(command=ControlTreeExecutionRequest.SHUTDOWN)
        )
        self.assertTrue(stop_slot_res.success, stop_slot_res.error_message)

        rospy.sleep(1.0)

    def update_tree_msg(self, msg):
        self.tree_msg = msg

    def testRemoteExecutionHangs(self):
        """The Shoved tree should not execute if we don't explicitly allow the slot to run"""
        # First, load the test tree
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Should be enough time for the tree to be shoved to the
        # remote_slot namespace and time out
        rospy.sleep(2.0)

        # Should be a given, since the tree message is updated even just at load
        self.assertIsNotNone(self.tree_msg)

        root_msg = get_root(self.tree_msg)
        self.assertIsNotNone(root_msg)

        shovable_node_msg = get_node_by_name(self.tree_msg, "Shovable")
        self.assertIsNotNone(shovable_node_msg)

        self.assertEqual(root_msg.state, NodeMsg.RUNNING)
        self.assertEqual(shovable_node_msg.state, NodeMsg.RUNNING)
        self.assertTrue(get_output(shovable_node_msg, "running_remotely"))

    def testRemoteExecution(self):
        """Tree is shoved to slot, exected, and reports results"""
        # First, load the test tree
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Enough for the tree to be shoved over
        rospy.sleep(2.0)

        # Signal the slot that it's allowed to start
        slot_exec_res = self.control_slot_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(slot_exec_res.success, slot_exec_res.error_message)

        # Should be enough time for the tree to be shoved to the
        # remote_slot namespace, execute there, and return
        rospy.sleep(4.0)

        # Should be a given, since the tree message is updated even just at load
        self.assertIsNotNone(self.tree_msg)

        root_msg = get_root(self.tree_msg)
        self.assertIsNotNone(root_msg)

        shovable_node_msg = get_node_by_name(self.tree_msg, "Shovable")
        self.assertIsNotNone(shovable_node_msg)

        subscriber_node_msg = get_node_by_name(self.tree_msg, "TopicSubscriber")
        self.assertIsNotNone(subscriber_node_msg)

        self.assertEqual(root_msg.state, NodeMsg.SUCCEEDED)
        self.assertTrue(get_output(shovable_node_msg, "running_remotely"))
        self.assertEqual(get_output(subscriber_node_msg, "message"), Bool(data=True))

    def testRemoteExecutionWithRemoteSlotNode(self):
        """Control the execution of the loaded tree with a RemoteSlot node"""

        # First, create and set up the RemoteSlot node that will control the slot
        remote_slot_node = RemoteSlot(
            options={
                "slot_namespace": "/remote_slot/remote_slot",
                "wait_for_service_seconds": 5.0,
            }
        )

        remote_slot_node.setup()
        self.assertEqual(remote_slot_node.state, NodeMsg.IDLE)

        # No tree in the slot yet, so the node should short-circuit to
        # succeeding
        self.assertEqual(remote_slot_node.tick(), NodeMsg.SUCCEEDED)

        # Now load the tree, as above:
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Enough for the tree to be shoved over
        rospy.sleep(2.0)

        # Signal the slot that it's allowed to start by ticking the
        # RemoteSlot node again (this should return RUNNING, first
        # because the service call takes time, and second because the
        # tree takes time to finish.
        self.assertEqual(remote_slot_node.tick(), NodeMsg.RUNNING)

        # Should be enough time for the tree to be shoved to the
        # remote_slot namespace, execute there, and return
        rospy.sleep(4.0)

        # If we tick again now, we should get a SUCCEEDED response
        self.assertEqual(remote_slot_node.tick(), NodeMsg.SUCCEEDED)

        # Check that result is the same as when we manually controlled the slot
        self.assertIsNotNone(self.tree_msg)

        root_msg = get_root(self.tree_msg)
        self.assertIsNotNone(root_msg)

        shovable_node_msg = get_node_by_name(self.tree_msg, "Shovable")
        self.assertIsNotNone(shovable_node_msg)

        subscriber_node_msg = get_node_by_name(self.tree_msg, "TopicSubscriber")
        self.assertIsNotNone(subscriber_node_msg)

        self.assertEqual(root_msg.state, NodeMsg.SUCCEEDED)
        self.assertTrue(get_output(shovable_node_msg, "running_remotely"))
        self.assertEqual(get_output(subscriber_node_msg, "message"), Bool(data=True))

        # shove the same tree again
        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Wait again
        rospy.sleep(0.5)

        is_running = False
        for _ in range(80):
            rospy.sleep(0.1)
            if remote_slot_node.tick() == NodeMsg.RUNNING:
                is_running = True
                break
        if not is_running and remote_slot_node.state:
            raise Exception(
                "Slot is not running after 16 seconds, but in state %s"
                % remote_slot_node.state
            )

        rospy.sleep(0.05)
        # Should still be running
        self.assertEqual(remote_slot_node.tick(), NodeMsg.RUNNING)
        # This should stop the tree from executing, so it won't finish
        self.assertEqual(remote_slot_node.untick(), NodeMsg.IDLE)

        # Should be enough time for the tree to be shoved to the
        # remote_slot namespace, execute there, and return
        rospy.sleep(2.0)

        # The tree root shold still be RUNNING, because the slot is stopped
        self.assertIsNotNone(self.tree_msg)

        root_msg = get_root(self.tree_msg)
        self.assertIsNotNone(root_msg)
        self.assertEqual(root_msg.state, NodeMsg.RUNNING)

        # Shut down the RemoteSlot for good measure
        self.assertEqual(remote_slot_node.shutdown(), NodeMsg.SHUTDOWN)

    def testRemoteExecutionWithRemoteSlotNodeEarlyShutdown(self):
        # First, create and set up the RemoteSlot node that will control the slot
        remote_slot_node = RemoteSlot(
            options={
                "slot_namespace": "/remote_slot/remote_slot",
                "wait_for_service_seconds": 5.0,
            }
        )

        remote_slot_node.setup()
        self.assertEqual(remote_slot_node.state, NodeMsg.IDLE)

        # No tree in the slot yet, so the node should short-circuit to
        # succeeding
        self.assertEqual(remote_slot_node.tick(), NodeMsg.SUCCEEDED)

        # Now load the tree, as above:
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Enough for the tree to be shoved over
        rospy.sleep(2.0)

        # Signal the slot that it's allowed to start by ticking the
        # RemoteSlot node again (this should return RUNNING, first
        # because the service call takes time, and second because the
        # tree takes time to finish.
        self.assertEqual(remote_slot_node.tick(), NodeMsg.RUNNING)

        # Shut down the RemoteSlot for good measure
        self.assertEqual(remote_slot_node.shutdown(), NodeMsg.SHUTDOWN)

    def testRemoteExecutionWithRemoteSlotNodeEarlyReset(self):
        # First, create and set up the RemoteSlot node that will control the slot
        remote_slot_node = RemoteSlot(
            options={
                "slot_namespace": "/remote_slot/remote_slot",
                "wait_for_service_seconds": 5.0,
            }
        )

        remote_slot_node.setup()
        self.assertEqual(remote_slot_node.state, NodeMsg.IDLE)

        # No tree in the slot yet, so the node should short-circuit to
        # succeeding
        self.assertEqual(remote_slot_node.tick(), NodeMsg.SUCCEEDED)

        # Now load the tree, as above:
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Enough for the tree to be shoved over
        rospy.sleep(2.0)

        # Signal the slot that it's allowed to start by ticking the
        # RemoteSlot node again (this should return RUNNING, first
        # because the service call takes time, and second because the
        # tree takes time to finish.
        self.assertEqual(remote_slot_node.tick(), NodeMsg.RUNNING)

        self.assertEqual(remote_slot_node.reset(), NodeMsg.IDLE)

    def testRemoteExecutionWithRemoteSlotNodeEarlyUntick(self):
        # First, create and set up the RemoteSlot node that will control the slot
        remote_slot_node = RemoteSlot(
            options={
                "slot_namespace": "/remote_slot/remote_slot",
                "wait_for_service_seconds": 5.0,
            }
        )

        remote_slot_node.setup()
        self.assertEqual(remote_slot_node.state, NodeMsg.IDLE)

        # No tree in the slot yet, so the node should short-circuit to
        # succeeding
        self.assertEqual(remote_slot_node.tick(), NodeMsg.SUCCEEDED)

        # Now load the tree, as above:
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Enough for the tree to be shoved over
        rospy.sleep(2.0)

        # Signal the slot that it's allowed to start by ticking the
        # RemoteSlot node again (this should return RUNNING, first
        # because the service call takes time, and second because the
        # tree takes time to finish.
        self.assertEqual(remote_slot_node.tick(), NodeMsg.RUNNING)

        # Shut down the RemoteSlot for good measure
        self.assertEqual(remote_slot_node.untick(), NodeMsg.IDLE)

    def testRemoteExecutionWithRemoteSlotNodeCalculateUtility(self):
        # First, create and set up the RemoteSlot node that will control the slot
        remote_slot_node = RemoteSlot(
            options={
                "slot_namespace": "/remote_slot/remote_slot",
                "wait_for_service_seconds": 5.0,
            }
        )

        remote_slot_node.setup()
        self.assertEqual(remote_slot_node.state, NodeMsg.IDLE)

        # No tree in the slot yet, so the node should short-circuit to
        # succeeding
        self.assertEqual(remote_slot_node.tick(), NodeMsg.SUCCEEDED)

        # Now load the tree, as above:
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Enough for the tree to be shoved over
        rospy.sleep(2.0)

        self.assertEqual(
            remote_slot_node.calculate_utility(),
            UtilityBounds(
                can_execute=False,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True,
            ),
        )

    @mock.patch("ros_bt_py.nodes.remote_slot.rosservice.get_service_type")
    def testRemoteExecutionWithRemoteSlotNodeCalculateUtilityException(
        self, mock_rosservice
    ):
        # First, create and set up the RemoteSlot node that will control the slot
        remote_slot_node = RemoteSlot(
            options={
                "slot_namespace": "/remote_slot/remote_slot",
                "wait_for_service_seconds": 5.0,
            }
        )

        remote_slot_node.setup()
        self.assertEqual(remote_slot_node.state, NodeMsg.IDLE)

        # No tree in the slot yet, so the node should short-circuit to
        # succeeding
        self.assertEqual(remote_slot_node.tick(), NodeMsg.SUCCEEDED)

        # Now load the tree, as above:
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Enough for the tree to be shoved over
        rospy.sleep(2.0)

        mock_rosservice.side_effect = rosservice.ROSServiceIOException

        self.assertEqual(remote_slot_node.calculate_utility(), UtilityBounds())

    def testRemoteExecutionWithRemoteSlotNodeCalculateUtilityServiceNotAvailable(self):
        # First, create and set up the RemoteSlot node that will control the slot
        remote_slot_node = RemoteSlot(
            options={
                "slot_namespace": "/does_not_exist",
                "wait_for_service_seconds": 0.5,
            }
        )

        self.assertRaises(rospy.exceptions.ROSException, remote_slot_node.setup)

        self.assertEqual(remote_slot_node.calculate_utility(), UtilityBounds())

    def testRemoteExecutionWithRemoteSlotNodeAsyncProxyError(self):
        # First, create and set up the RemoteSlot node that will control the slot
        remote_slot_node = RemoteSlot(
            options={
                "slot_namespace": "/remote_slot/remote_slot",
                "wait_for_service_seconds": 5.0,
            }
        )

        remote_slot_node.setup()
        self.assertEqual(remote_slot_node.state, NodeMsg.IDLE)

        # No tree in the slot yet, so the node should short-circuit to
        # succeeding
        self.assertEqual(remote_slot_node.tick(), NodeMsg.SUCCEEDED)

        # Now load the tree, as above:
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Enough for the tree to be shoved over
        rospy.sleep(2.0)

        remote_slot_node._service_proxy.get_state = mock.MagicMock()
        remote_slot_node._service_proxy.get_state.return_value = AsyncServiceProxy.ERROR
        self.assertEqual(remote_slot_node.tick(), NodeMsg.FAILED)

    def testRemoteExecutionWithRemoteSlotNodeAsyncProxyResponseReady(self):
        # First, create and set up the RemoteSlot node that will control the slot
        remote_slot_node = RemoteSlot(
            options={
                "slot_namespace": "/remote_slot/remote_slot",
                "wait_for_service_seconds": 5.0,
            }
        )

        remote_slot_node.setup()
        self.assertEqual(remote_slot_node.state, NodeMsg.IDLE)

        # No tree in the slot yet, so the node should short-circuit to
        # succeeding
        self.assertEqual(remote_slot_node.tick(), NodeMsg.SUCCEEDED)

        # Now load the tree, as above:
        load_res = self.load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(path="package://ros_bt_py/etc/trees/test_shove_tree.yaml")
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Enough for the tree to be shoved over
        rospy.sleep(2.0)

        remote_slot_node._service_proxy.get_state = mock.MagicMock()
        remote_slot_node._service_proxy.get_state.return_value = (
            AsyncServiceProxy.RESPONSE_READY
        )
        remote_slot_node._service_proxy.get_response = mock.MagicMock()
        remote_slot_node._service_proxy.get_response.return_value = (
            ControlTreeExecutionResponse(success=False)
        )
        self.assertEqual(remote_slot_node.tick(), NodeMsg.FAILED)


def get_root(tree_msg):
    return get_node_by_name(tree_msg, tree_msg.root_name)


def get_node_by_name(tree_msg, node_name):
    for node in tree_msg.nodes:
        if node.name == node_name:
            return node
    return None


def get_input(node_msg, key):
    for input_msg in node_msg.inputs:
        if input_msg.key == key:
            return json_decode(input_msg.serialized_value)
    return None


def get_output(node_msg, key):
    for output_msg in node_msg.outputs:
        if output_msg.key == key:
            return json_decode(output_msg.serialized_value)
    return None


def get_option(node_msg, key):
    for option_msg in node_msg.options:
        if option_msg.key == key:
            return json_decode(option_msg.serialized_value)
    return None


if __name__ == "__main__":
    rospy.init_node("test_shove_tree")
    import rostest
    import sys
    import os

    os.environ["COVERAGE_FILE"] = "%s.%s.coverage" % (PKG, "test_shove_tree")
    rostest.rosrun(PKG, "test_shove_tree", TestShoveTree, sysargs=sys.argv + ["--cov"])
