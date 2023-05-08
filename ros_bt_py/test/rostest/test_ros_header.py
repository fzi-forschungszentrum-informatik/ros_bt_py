#!/usr/bin/env python
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


from threading import Lock
import unittest

try:
    import unittest.mock as mock
except ImportError:
    import mock

import rospy

from std_msgs.msg import Header
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.ros_header import GetStdHeader

PKG = "ros_bt_py"


class TestGetStdHeader(unittest.TestCase):
    def testHeader(self):
        ros_header = GetStdHeader(
            options={
                "header_type": Header,
            }
        )
        self.assertEqual(ros_header.state, NodeMsg.UNINITIALIZED)
        ros_header.setup()
        self.assertEqual(ros_header.state, NodeMsg.IDLE)

        self.assertEqual(ros_header.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_header.outputs["header"], ros_header.header)

        expected_bounds = UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )

        ros_header_utility = ros_header.calculate_utility()

        self.assertEqual(
            ros_header_utility.can_execute,
            expected_bounds.can_execute,
        )
        self.assertEqual(
            ros_header_utility.has_lower_bound_failure,
            expected_bounds.has_lower_bound_failure,
        )
        self.assertEqual(
            ros_header_utility.has_lower_bound_success,
            expected_bounds.has_lower_bound_success,
        )
        self.assertEqual(
            ros_header_utility.has_upper_bound_failure,
            expected_bounds.has_lower_bound_failure,
        )
        self.assertEqual(
            ros_header_utility.has_upper_bound_success,
            expected_bounds.has_upper_bound_success,
        )

        self.assertEqual(
            ros_header_utility.lower_bound_failure,
            expected_bounds.lower_bound_failure,
        )
        self.assertEqual(
            ros_header_utility.lower_bound_success,
            expected_bounds.lower_bound_success,
        )

        self.assertEqual(
            ros_header_utility.upper_bound_failure,
            expected_bounds.upper_bound_failure,
        )
        self.assertEqual(
            ros_header_utility.upper_bound_success,
            expected_bounds.upper_bound_success,
        )

        self.assertEqual(ros_header.untick(), NodeMsg.IDLE)
        self.assertEqual(ros_header.reset(), NodeMsg.IDLE)
        self.assertEqual(ros_header.shutdown(), NodeMsg.SHUTDOWN)

    def testBrokenHeader(self):
        ros_header = GetStdHeader(
            options={
                "header_type": int,
            }
        )
        self.assertEqual(ros_header.state, NodeMsg.UNINITIALIZED)
        ros_header.setup()
        self.assertEqual(ros_header.state, NodeMsg.IDLE)

        self.assertEqual(ros_header.tick(), NodeMsg.FAILED)


if __name__ == "__main__":
    rospy.init_node("test_ros_header")
    import rostest

    rostest.rosrun(PKG, "test_ros_header", TestGetStdHeader)
