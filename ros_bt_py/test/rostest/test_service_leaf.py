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

import rospy

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.service import Service, WaitForService, ServiceInput

PKG = "ros_bt_py"


class TestServiceLeaf(unittest.TestCase):
    def setUp(self):
        self.delay_service_leaf = Service(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "service_name": "delay_1s_if_true",
                "wait_for_service_seconds": 0.5,
                "wait_for_response_seconds": 1.5,
            }
        )
        self.delay_service_leaf.setup()

        self.delay_service_input_leaf = ServiceInput(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "wait_for_response_seconds": 1.5,
            }
        )
        self.delay_service_input_leaf.setup()

    def tearDown(self):
        self.delay_service_leaf.shutdown()
        self.delay_service_input_leaf.shutdown()

    def testWaitForService(self):
        wait_for_service = WaitForService(
            options={
                "service_type": SetBool,
                "service_name": "delay_1s_if_true",
                "wait_for_service_seconds": 0.5,
            }
        )

        wait_for_service.setup()

        wait_for_service.tick()
        self.assertEqual(wait_for_service.state, NodeMsg.RUNNING)

        rospy.sleep(0.6)

        wait_for_service.tick()
        self.assertEqual(wait_for_service.state, NodeMsg.SUCCEEDED)

        wait_for_service.untick()
        self.assertEqual(wait_for_service.state, NodeMsg.IDLE)

        wait_for_service.reset()
        self.assertEqual(wait_for_service.state, NodeMsg.IDLE)

        wait_for_service.shutdown()
        self.assertEqual(wait_for_service.state, NodeMsg.SHUTDOWN)

    def testWaitForServiceNotAvailable(self):
        wait_for_service = WaitForService(
            options={
                "service_type": SetBool,
                "service_name": "this_service_is_not_available",
                "wait_for_service_seconds": 0.1,
            }
        )

        wait_for_service.setup()

        wait_for_service.tick()
        self.assertEqual(wait_for_service.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)

        wait_for_service.tick()
        self.assertEqual(wait_for_service.state, NodeMsg.FAILED)

    def noDelayServiceCall(self, node):
        node.inputs["request"] = SetBoolRequest(data=False)
        sleeps = 0
        while True:
            node.tick()
            self.assertNotEqual(node.state, NodeMsg.FAILED)
            if node.state == NodeMsg.SUCCEEDED:
                break
            rospy.sleep(0.1)
            sleeps += 1
            # If we don't get a response for half a second, something has gone wrong
            self.assertLess(sleeps, 5)
        self.assertEqual(node.state, NodeMsg.SUCCEEDED)

        # Ticking again should start a new service call i.e. the Leaf should be
        # RUNNING again
        node.inputs["request"] = SetBoolRequest(data=True)
        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        # no need to wait for the result here, we test that elsewhere
        node.untick()

        expected_bounds = UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )
        self.assertEqual(node.calculate_utility(), expected_bounds)

    def testNoDelayServiceCall(self):
        # No delay
        self.noDelayServiceCall(self.delay_service_leaf)

        self.delay_service_input_leaf.inputs["service_name"] = "delay_1s_if_true"
        self.noDelayServiceCall(self.delay_service_input_leaf)

    def delayServiceCall(self, node):
        node.inputs["request"] = SetBoolRequest(data=True)

        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)
        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        rospy.sleep(0.6)
        node.tick()
        self.assertEqual(node.state, NodeMsg.SUCCEEDED)

    def testDelayServiceCall(self):
        self.delayServiceCall(self.delay_service_leaf)

        self.delay_service_input_leaf.inputs["service_name"] = "delay_1s_if_true"
        self.delayServiceCall(self.delay_service_input_leaf)

    def crashServiceCall(self, node):
        node.inputs["request"] = SetBoolRequest(data=True)
        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)
        node.tick()
        self.assertEqual(node.state, NodeMsg.FAILED)

    def testCrashServiceCall(self):
        crash_service_leaf = Service(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "service_name": "crash",
                "wait_for_service_seconds": 0.5,
                "wait_for_response_seconds": 1.5,
            }
        )
        crash_service_leaf.setup()

        crash_service_input_leaf = ServiceInput(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "wait_for_response_seconds": 1.5,
            }
        )
        crash_service_input_leaf.setup()

        crash_service_input_leaf.inputs["service_name"] = "crash"

        self.crashServiceCall(crash_service_leaf)
        self.crashServiceCall(crash_service_input_leaf)

    def crashIfTrueServiceCall(self, node):
        node.inputs["request"] = SetBoolRequest(data=True)
        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)
        node.tick()
        self.assertEqual(node.state, NodeMsg.FAILED)

        self.assertIsNone(node.outputs["response"])

        node.inputs["request"] = SetBoolRequest(data=False)
        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)
        node.tick()
        self.assertEqual(node.state, NodeMsg.SUCCEEDED)

        self.assertEqual(node.outputs["response"], SetBoolResponse(True, ""))

    def testCrashIfTrueServiceCall(self):
        crash_if_true_service_leaf = Service(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "service_name": "crash_if_true",
                "wait_for_service_seconds": 0.5,
                "wait_for_response_seconds": 1.5,
            }
        )
        crash_if_true_service_leaf.setup()

        crash_if_true_service_input_leaf = ServiceInput(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "wait_for_response_seconds": 1.5,
            }
        )
        crash_if_true_service_input_leaf.setup()
        crash_if_true_service_input_leaf.inputs["service_name"] = "crash_if_true"

        self.crashIfTrueServiceCall(crash_if_true_service_leaf)
        self.crashIfTrueServiceCall(crash_if_true_service_input_leaf)

    def testReset(self):
        self.delay_service_leaf.inputs["request"] = SetBoolRequest(data=True)

        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.RUNNING)

        self.delay_service_leaf.reset()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.IDLE)

        self.delay_service_input_leaf.inputs["request"] = SetBoolRequest(data=True)
        self.delay_service_input_leaf.inputs["service_name"] = "delay_1s_if_true"

        self.delay_service_input_leaf.tick()
        self.assertEqual(self.delay_service_input_leaf.state, NodeMsg.RUNNING)

        self.delay_service_input_leaf.reset()
        self.assertEqual(self.delay_service_input_leaf.state, NodeMsg.IDLE)

    def testShutdown(self):
        delay_service = Service(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "service_name": "delay_1s_if_true",
                "wait_for_service_seconds": 0.5,
                "wait_for_response_seconds": 1.5,
            }
        )

        self.assertEqual(delay_service.state, NodeMsg.UNINITIALIZED)

        delay_service.setup()
        self.assertEqual(delay_service.state, NodeMsg.IDLE)

        delay_service.shutdown()
        self.assertEqual(delay_service.state, NodeMsg.SHUTDOWN)

        delay_input_service = ServiceInput(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "wait_for_response_seconds": 1.5,
            }
        )

        delay_input_service.inputs["service_name"] = "delay_1s_if_true"

        self.assertEqual(delay_input_service.state, NodeMsg.UNINITIALIZED)

        delay_input_service.setup()
        self.assertEqual(delay_input_service.state, NodeMsg.IDLE)

        delay_input_service.shutdown()
        self.assertEqual(delay_input_service.state, NodeMsg.SHUTDOWN)

    def untickTest(self, node):
        node.inputs["request"] = SetBoolRequest(data=True)

        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        rospy.sleep(0.1)
        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        node.untick()
        node.inputs["request"] = SetBoolRequest(data=False)
        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        for _ in range(10):
            rospy.sleep(0.1)
            node.tick()
            if node.state != NodeMsg.RUNNING:
                self.assertEqual(node.state, NodeMsg.SUCCEEDED)

    def testUntick(self):
        self.untickTest(self.delay_service_leaf)

        self.delay_service_input_leaf.inputs["service_name"] = "delay_1s_if_true"
        self.untickTest(self.delay_service_input_leaf)

    def timeout(self, node):
        node.inputs["request"] = SetBoolRequest(data=True)

        node.tick()
        self.assertEqual(node.state, NodeMsg.RUNNING)

        rospy.sleep(0.6)
        node.tick()
        self.assertEqual(node.state, NodeMsg.FAILED)

    def testTimeout(self):
        # Overwrite the leaf with one that has a shorter timeot
        delay_service_leaf = Service(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "service_name": "delay_1s_if_true",
                "wait_for_service_seconds": 0.5,
                "wait_for_response_seconds": 0.5,
            }
        )
        delay_service_leaf.setup()

        delay_service_input_leaf = ServiceInput(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "wait_for_response_seconds": 0.5,
            }
        )
        delay_service_input_leaf.setup()

        delay_service_input_leaf.inputs["service_name"] = "delay_1s_if_true"

        self.timeout(delay_service_leaf)
        self.timeout(delay_service_input_leaf)

    def testFailIfNotAvailable(self):
        service_leaf = Service(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "service_name": "this_service_is_not_available",
                "wait_for_service_seconds": 0.5,
                "wait_for_response_seconds": 1.5,
                "fail_if_not_available": True,
            }
        )

        service_leaf.setup()
        self.assertFalse(service_leaf._service_available)

        service_leaf.inputs["request"] = SetBoolRequest(data=True)

        self.assertEqual(service_leaf.tick(), NodeMsg.FAILED)

        expected_bounds = UtilityBounds()
        expected_bounds.can_execute = False
        self.assertEqual(
            service_leaf.calculate_utility().can_execute,
            expected_bounds.can_execute,
        )
        self.assertEqual(
            service_leaf.calculate_utility().has_lower_bound_failure,
            expected_bounds.has_lower_bound_failure,
        )
        self.assertEqual(
            service_leaf.calculate_utility().has_lower_bound_success,
            expected_bounds.has_lower_bound_success,
        )
        self.assertEqual(
            service_leaf.calculate_utility().has_upper_bound_failure,
            expected_bounds.has_lower_bound_failure,
        )
        self.assertEqual(
            service_leaf.calculate_utility().has_upper_bound_success,
            expected_bounds.has_upper_bound_success,
        )

        self.assertEqual(
            service_leaf.calculate_utility().lower_bound_failure,
            expected_bounds.lower_bound_failure,
        )
        self.assertEqual(
            service_leaf.calculate_utility().lower_bound_success,
            expected_bounds.lower_bound_success,
        )

        self.assertEqual(
            service_leaf.calculate_utility().upper_bound_failure,
            expected_bounds.upper_bound_failure,
        )
        self.assertEqual(
            service_leaf.calculate_utility().upper_bound_success,
            expected_bounds.upper_bound_success,
        )

    def testExceptionIfNotAvailable(self):
        service_leaf = Service(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "service_name": "this_service_is_not_available",
                "wait_for_service_seconds": 0.5,
                "wait_for_response_seconds": 1.5,
                "fail_if_not_available": False,
            }
        )

        self.assertRaises(rospy.ROSException, service_leaf.setup)

        expected_bounds = UtilityBounds()
        expected_bounds.can_execute = False
        self.assertEqual(
            service_leaf.calculate_utility().can_execute,
            expected_bounds.can_execute,
        )
        self.assertEqual(
            service_leaf.calculate_utility().has_lower_bound_failure,
            expected_bounds.has_lower_bound_failure,
        )
        self.assertEqual(
            service_leaf.calculate_utility().has_lower_bound_success,
            expected_bounds.has_lower_bound_success,
        )
        self.assertEqual(
            service_leaf.calculate_utility().has_upper_bound_failure,
            expected_bounds.has_lower_bound_failure,
        )
        self.assertEqual(
            service_leaf.calculate_utility().has_upper_bound_success,
            expected_bounds.has_upper_bound_success,
        )

        self.assertEqual(
            service_leaf.calculate_utility().lower_bound_failure,
            expected_bounds.lower_bound_failure,
        )
        self.assertEqual(
            service_leaf.calculate_utility().lower_bound_success,
            expected_bounds.lower_bound_success,
        )

        self.assertEqual(
            service_leaf.calculate_utility().upper_bound_failure,
            expected_bounds.upper_bound_failure,
        )
        self.assertEqual(
            service_leaf.calculate_utility().upper_bound_success,
            expected_bounds.upper_bound_success,
        )

    def testServiceInputUtility(self):
        service_input_leaf = ServiceInput(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "wait_for_response_seconds": 1.5,
            }
        )

        service_input_leaf.inputs["service_name"] = "this_service_is_not_available"
        expected_bounds = UtilityBounds()
        expected_bounds.can_execute = False
        self.assertEqual(
            service_input_leaf.calculate_utility().can_execute,
            expected_bounds.can_execute,
        )
        self.assertEqual(
            service_input_leaf.calculate_utility().has_lower_bound_failure,
            expected_bounds.has_lower_bound_failure,
        )
        self.assertEqual(
            service_input_leaf.calculate_utility().has_lower_bound_success,
            expected_bounds.has_lower_bound_success,
        )
        self.assertEqual(
            service_input_leaf.calculate_utility().has_upper_bound_failure,
            expected_bounds.has_lower_bound_failure,
        )
        self.assertEqual(
            service_input_leaf.calculate_utility().has_upper_bound_success,
            expected_bounds.has_upper_bound_success,
        )

        self.assertEqual(
            service_input_leaf.calculate_utility().lower_bound_failure,
            expected_bounds.lower_bound_failure,
        )
        self.assertEqual(
            service_input_leaf.calculate_utility().lower_bound_success,
            expected_bounds.lower_bound_success,
        )

        self.assertEqual(
            service_input_leaf.calculate_utility().upper_bound_failure,
            expected_bounds.upper_bound_failure,
        )
        self.assertEqual(
            service_input_leaf.calculate_utility().upper_bound_success,
            expected_bounds.upper_bound_success,
        )

    def testChangeServiceNameBetweenTicks(self):
        self.delay_service_input_leaf.inputs["service_name"] = "delay_1s_if_true"
        self.delay_service_input_leaf.inputs["request"] = SetBoolRequest(data=False)

        self.delay_service_input_leaf.tick()
        self.assertEqual(self.delay_service_input_leaf.state, NodeMsg.RUNNING)

        self.delay_service_input_leaf.inputs["service_name"] = "delay_1s"

        self.delay_service_input_leaf.tick()
        self.assertEqual(self.delay_service_input_leaf.state, NodeMsg.RUNNING)


if __name__ == "__main__":
    rospy.init_node("test_service_leaf")
    import rostest

    rostest.rosrun(PKG, "test_service_leaf", TestServiceLeaf)
