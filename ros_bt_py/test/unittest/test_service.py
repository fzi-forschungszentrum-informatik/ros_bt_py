import unittest

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.service import Service, ServiceInput

import unittest
try:
    import unittest.mock as mock
except ImportError:
    import mock
from rospy import Time


class TestService(unittest.TestCase):
    def testUnavailableService(self):
        unavailable_service = Service(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'service_name': 'this_service_does_not_exist',
            'wait_for_service_seconds': 0.5,
            'wait_for_response_seconds': 1.5
        })

        expected_bounds = UtilityBounds()
        self.assertEqual(
            unavailable_service.calculate_utility(), expected_bounds)


class TestServiceInput(unittest.TestCase):
    @mock.patch('ros_bt_py.nodes.decorators.rospy.Time.now')
    def testUnavailableService(self, mock_time_now):
        unavailable_service = ServiceInput(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'wait_for_response_seconds': 1.5
        })
        unavailable_service.setup()
        unavailable_service.inputs['service_name'] = 'this_service_does_not_exist'
        unavailable_service.inputs['request'] = SetBoolRequest()
        self.assertEqual(unavailable_service.tick(), NodeMsg.FAILED)

        expected_bounds = UtilityBounds()
        self.assertEqual(
            unavailable_service.calculate_utility(), expected_bounds)
