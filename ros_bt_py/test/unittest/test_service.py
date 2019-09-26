import unittest

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.service import Service


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
        self.assertEqual(unavailable_service.calculate_utility(), expected_bounds)
