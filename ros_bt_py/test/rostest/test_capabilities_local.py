#!/usr/bin/env python
#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022-2023 FZI Forschungszentrum Informatik
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
from typing import Optional
import unittest
from ros_bt_py import tree_manager
from ros_bt_py.helpers import json_decode

from ros_bt_py.tree_manager import load_tree_from_file

try:
    import unittest.mock as mock
except ImportError:
    import mock

import os
import signal

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from ros_bt_py_msgs.srv import (
    ControlTreeExecution,
    ControlTreeExecutionRequest,
    LoadTree,
    LoadTreeRequest,
    LoadCapabilities,
    LoadCapabilitiesRequest,
    LoadCapabilitiesResponse,
)
from ros_bt_py_msgs.msg import Tree
from ros_bt_py_msgs.msg import Node as NodeMsg

PKG = "ros_bt_py"


class TestLocalCapabilities(unittest.TestCase):
    def setUp(self):
        self.first_load_tree_proxy = rospy.ServiceProxy(
            "/first/tree_node/load_tree", LoadTree
        )
        self.second_load_tree_proxy = rospy.ServiceProxy(
            "/second/tree_node/load_tree", LoadTree
        )

        self.first_control_tree_proxy = rospy.ServiceProxy(
            "/first/tree_node/control_tree_execution", ControlTreeExecution
        )
        self.second_control_tree_proxy = rospy.ServiceProxy(
            "/second/tree_node/control_tree_execution", ControlTreeExecution
        )

        # These throw an exception if they time out, failing the test
        self.first_load_tree_proxy.wait_for_service(1.0)
        self.first_control_tree_proxy.wait_for_service(1.0)

        self.first_tree_msg: Optional[Tree] = None
        self.first_tree_sub = rospy.Subscriber(
            "/first/tree_node/tree", Tree, self.update_first_tree_msg, queue_size=1
        )
        self.assertIsNotNone(self.first_tree_sub)

        self.second_tree_msg: Optional[Tree] = None
        self.second_tree_sub = rospy.Subscriber(
            "/second/tree_node/tree", Tree, self.update_second_tree_msg, queue_size=1
        )
        self.assertIsNotNone(self.second_tree_sub)

        self.first_load_capabilities_proxy = rospy.ServiceProxy(
            "/first/capability_repository/capabilities/load", LoadCapabilities
        )
        load_capabilities: LoadCapabilitiesResponse = (
            self.first_load_capabilities_proxy(
                LoadCapabilitiesRequest(package="ros_bt_py")
            )
        )
        self.assertTrue(load_capabilities.success, load_capabilities.error_message)
        self.assertGreaterEqual(len(load_capabilities.interfaces), 1)

    def tearDown(self):
        if self.first_tree_msg.state in [Tree.TICKING, Tree.ERROR, Tree.IDLE]:
            stop_tree_res = self.first_control_tree_proxy(
                ControlTreeExecutionRequest(
                    command=ControlTreeExecutionRequest.SHUTDOWN
                )
            )
            self.assertTrue(stop_tree_res.success, stop_tree_res.error_message)

        if self.second_tree_msg.state in [Tree.TICKING, Tree.ERROR, Tree.IDLE]:
            rospy.logerr(self.second_tree_msg.state)
            stop_tree_res = self.second_control_tree_proxy(
                ControlTreeExecutionRequest(
                    command=ControlTreeExecutionRequest.SHUTDOWN
                )
            )
            self.assertTrue(stop_tree_res.success, stop_tree_res.error_message)

        rospy.sleep(1.0)

    def update_first_tree_msg(self, msg):
        self.first_tree_msg = msg

    def update_second_tree_msg(self, msg):
        self.second_tree_msg = msg

    def testLoadTree(self):
        # First, load the test tree
        load_res = self.first_load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(
                    path="package://ros_bt_py/test/testdata/trees/capability_test_local.yaml"
                )
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

    def testLoadTreeWithoutCapabilityLoaded(self):
        # First, load the test tree
        load_res = self.second_load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(
                    path="package://ros_bt_py/test/testdata/trees/capability_test_local.yaml"
                )
            )
        )
        self.assertFalse(load_res.success, load_res.error_message)

    def testExecution(self):
        # First, load the test tree
        load_res = self.first_load_tree_proxy(
            LoadTreeRequest(
                tree=Tree(
                    path="package://ros_bt_py/test/testdata/trees/capability_test_local.yaml"
                )
            )
        )
        self.assertTrue(load_res.success, load_res.error_message)

        exec_res = self.first_control_tree_proxy(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=10.0,
            )
        )
        self.assertTrue(exec_res.success, exec_res.error_message)

        # Should be enough time for the tree to be shoved to the
        # remote_slot namespace and time out
        rospy.sleep(5.0)

        # Should be a given, since the tree message is updated even just at load
        self.assertIsNotNone(self.first_tree_msg)

        root_msg = get_root(self.first_tree_msg)
        self.assertIsNotNone(root_msg)

        capability_node_msg = get_node_by_name(self.first_tree_msg, "AddTwoCapability")
        self.assertIsNotNone(capability_node_msg)

        self.assertEqual(root_msg.state, NodeMsg.SUCCEEDED)
        self.assertEqual(capability_node_msg.state, NodeMsg.SUCCEEDED)
        self.assertEqual(get_output(capability_node_msg, "result"), 3)


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
    rospy.init_node("test_capabilities_local")
    import rostest

    rostest.rosrun(
        PKG,
        "test_capabilities_local",
        TestLocalCapabilities,
    )
