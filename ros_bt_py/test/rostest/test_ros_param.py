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
from threading import Lock
import unittest

try:
    import unittest.mock as mock
except ImportError:
    import mock

import rospy

from std_msgs.msg import Int32
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.ros_param import RosParamOption, RosParamInput

PKG = "ros_bt_py"


class TestRosParam(unittest.TestCase):
    def testExistingParamOption(self):
        ros_param = RosParamOption(
            options={
                "param_name": "/param_int",
                "param_type": int,
            }
        )
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs["default_value"] = 23
        self.assertEqual(ros_param.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_param.outputs["param"], 42)

        expected_bounds = UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )

        self.assertEqual(ros_param.calculate_utility(), expected_bounds)

        self.assertEqual(ros_param.untick(), NodeMsg.IDLE)
        self.assertEqual(ros_param.reset(), NodeMsg.IDLE)
        self.assertEqual(ros_param.shutdown(), NodeMsg.SHUTDOWN)

    def testMissingParamOption(self):
        ros_param = RosParamOption(
            options={
                "param_name": "/param_missing",
                "param_type": int,
            }
        )
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs["default_value"] = 23
        self.assertEqual(ros_param.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_param.outputs["param"], 23)

        expected_bounds = UtilityBounds(
            can_execute=False,
            has_lower_bound_success=False,
            has_upper_bound_success=False,
            has_lower_bound_failure=False,
            has_upper_bound_failure=False,
        )

        self.assertEqual(ros_param.calculate_utility(), expected_bounds)

    def testWrongParamTypeOption(self):
        ros_param = RosParamOption(
            options={
                "param_name": "/param_int",
                "param_type": str,
            }
        )
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs["default_value"] = "default"
        self.assertEqual(ros_param.tick(), NodeMsg.FAILED)

    def testROSExceptionOption(self):
        ros_param = RosParamOption(
            options={
                "param_name": "/param_missing",
                "param_type": int,
            }
        )

        expected_bounds = UtilityBounds()
        with mock.patch("rospy.get_param_names") as mocked_get_param_names:
            mocked_get_param_names.side_effect = rospy.ROSException()
            self.assertEqual(ros_param.calculate_utility(), expected_bounds)

    def testExistingParamInput(self):
        ros_param = RosParamInput(options={"param_type": int, "default_value": 0})
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs["param_name"] = "/param_int"
        self.assertEqual(ros_param.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_param.outputs["param"], 42)

        self.assertEqual(ros_param.untick(), NodeMsg.IDLE)
        self.assertEqual(ros_param.reset(), NodeMsg.IDLE)
        self.assertEqual(ros_param.shutdown(), NodeMsg.SHUTDOWN)

    def testMissingParamInput(self):
        ros_param = RosParamInput(options={"param_type": int, "default_value": 0})

        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs["param_name"] = "/param_missing"
        self.assertEqual(ros_param.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_param.outputs["param"], 0)

        expected_bounds = UtilityBounds()
        expected_bounds.can_execute = True

        self.assertEqual(ros_param.calculate_utility(), expected_bounds)

    def testWrongParamTypeInput(self):
        ros_param = RosParamInput(
            options={
                "default_value": "toto",
                "param_type": str,
            }
        )
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs["param_name"] = "param_int"
        self.assertEqual(ros_param.tick(), NodeMsg.FAILED)


if __name__ == "__main__":
    rospy.init_node("test_ros_param")
    import rostest
    import sys
    import os

    os.environ["COVERAGE_FILE"] = "%s.%s.coverage" % (PKG, "test_ros_param")
    rostest.rosrun(PKG, "test_ros_param", TestRosParam, sysargs=sys.argv + ["--cov"])
