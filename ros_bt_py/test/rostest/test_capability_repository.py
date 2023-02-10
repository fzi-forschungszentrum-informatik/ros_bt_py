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
from ros_bt_py_msgs.msg import CapabilityInterface
from ros_bt_py_msgs.srv import (
    LoadCapabilities,
    LoadCapabilitiesRequest,
    LoadCapabilitiesResponse,
    GetCapabilityInterfaces,
    GetCapabilityInterfacesResponse,
    GetCapabilityInterfacesRequest,
)
from ros_bt_py_msgs.msg import Tree
from ros_bt_py_msgs.msg import Node as NodeMsg

PKG = "ros_bt_py"


class TestCapabilityRepository(unittest.TestCase):
    def setUp(self):

        self.first_get_capabilities_proxy = rospy.ServiceProxy(
            "/capability_repository/capabilities/interfaces/get",
            GetCapabilityInterfaces,
        )

        self.first_load_capabilities_proxy = rospy.ServiceProxy(
            "/capability_repository/capabilities/load", LoadCapabilities
        )
        load_capabilities: LoadCapabilitiesResponse = (
            self.first_load_capabilities_proxy(
                LoadCapabilitiesRequest(package="ros_bt_py")
            )
        )
        self.assertTrue(load_capabilities.success, load_capabilities.error_message)
        self.assertGreaterEqual(len(load_capabilities.interfaces), 1)

    def testCapabilitiesLoaded(self):
        # First, load the test tree
        get_capabilities: GetCapabilityInterfacesResponse = (
            self.first_get_capabilities_proxy(GetCapabilityInterfacesRequest())
        )
        self.assertTrue(get_capabilities.success, get_capabilities.error_message)
        self.assertGreaterEqual(len(get_capabilities.interfaces), 1)
        interface: CapabilityInterface = get_capabilities.interfaces[0]
        self.assertEqual(interface.name, "AddTwoCapability")
        self.assertEqual(len(interface.inputs), 2)
        self.assertEqual(len(interface.outputs), 1)


if __name__ == "__main__":
    rospy.init_node("test_capability_repository")
    import rostest

    rostest.rosrun(
        PKG,
        "test_capability_repository",
        TestCapabilityRepository,
    )
