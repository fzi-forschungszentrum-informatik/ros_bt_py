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

from time import sleep

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.service import Service, ServiceInput

try:
    import unittest.mock as mock
except ImportError:
    import mock
from rospy import Time


class TestService(unittest.TestCase):
    def testUnavailableService(self):
        unavailable_service = Service(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "service_name": "this_service_does_not_exist",
                "wait_for_service_seconds": 0.5,
                "wait_for_response_seconds": 1.5,
            }
        )

        expected_bounds = UtilityBounds()
        self.assertEqual(unavailable_service.calculate_utility(), expected_bounds)


class TestServiceInput(unittest.TestCase):
    @mock.patch("ros_bt_py.nodes.decorators.rospy.Time.now")
    def testUnavailableService(self, mock_time_now):
        mock_time_now.return_value = Time.from_seconds(0.0)

        unavailable_service = ServiceInput(
            options={
                "service_type": SetBool,
                "request_type": SetBoolRequest,
                "response_type": SetBoolResponse,
                "wait_for_response_seconds": 0.1,
            }
        )
        unavailable_service.setup()
        unavailable_service.inputs["service_name"] = "this_service_does_not_exist"
        unavailable_service.inputs["request"] = SetBoolRequest()
        self.assertEqual(unavailable_service.tick(), NodeMsg.RUNNING)
        sleep(0.5)
        self.assertEqual(unavailable_service.tick(), NodeMsg.FAILED)

        expected_bounds = UtilityBounds()
        self.assertEqual(unavailable_service.calculate_utility(), expected_bounds)
