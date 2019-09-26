#!/usr/bin/env python2.7
from threading import Lock
import unittest

import rospy

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.service import Service

PKG = 'ros_bt_py'


class TestServiceLeaf(unittest.TestCase):
    def setUp(self):
        self.delay_service_leaf = Service(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'service_name': 'delay_1s_if_true',
            'wait_for_service_seconds': 0.5,
            'wait_for_response_seconds': 1.5
        })
        self.delay_service_leaf.setup()

    def tearDown(self):
        self.delay_service_leaf.shutdown()

    def testNoDelayServiceCall(self):
        # No delay
        self.delay_service_leaf.inputs['request'] = SetBoolRequest(data=False)
        sleeps = 0
        while True:
            self.delay_service_leaf.tick()
            self.assertNotEqual(self.delay_service_leaf.state, NodeMsg.FAILED)
            if self.delay_service_leaf.state == NodeMsg.SUCCEEDED:
                break
            rospy.sleep(0.1)
            sleeps += 1
            # If we don't get a response for half a second, something has gone wrong
            self.assertLess(sleeps, 5)
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.SUCCEEDED)

        # Ticking again should start a new service call i.e. the Leaf should be
        # RUNNING again
        self.delay_service_leaf.inputs['request'] = SetBoolRequest(data=True)
        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.RUNNING)

        # no need to wait for the result here, we test that elsewhere
        self.delay_service_leaf.untick()

        expected_bounds = UtilityBounds(can_execute=True,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)
        self.assertEqual(self.delay_service_leaf.calculate_utility(), expected_bounds)

    def testDelayServiceCall(self):
        self.delay_service_leaf.inputs['request'] = SetBoolRequest(data=True)

        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)
        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.6)
        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.SUCCEEDED)

    def testCrashServiceCall(self):
        crash_service_leaf = Service(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'service_name': 'crash',
            'wait_for_service_seconds': 0.5,
            'wait_for_response_seconds': 1.5
        })
        crash_service_leaf.setup()

        crash_service_leaf.inputs['request'] = SetBoolRequest(data=True)
        crash_service_leaf.tick()
        self.assertEqual(crash_service_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)
        crash_service_leaf.tick()
        self.assertEqual(crash_service_leaf.state, NodeMsg.FAILED)

    def testCrashIfTrueServiceCall(self):
        crash_if_true_service_leaf = Service(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'service_name': 'crash_if_true',
            'wait_for_service_seconds': 0.5,
            'wait_for_response_seconds': 1.5
        })
        crash_if_true_service_leaf.setup()

        crash_if_true_service_leaf.inputs['request'] = SetBoolRequest(data=True)
        crash_if_true_service_leaf.tick()
        self.assertEqual(crash_if_true_service_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)
        crash_if_true_service_leaf.tick()
        self.assertEqual(crash_if_true_service_leaf.state, NodeMsg.FAILED)

        self.assertIsNone(crash_if_true_service_leaf.outputs['response'])

        crash_if_true_service_leaf.inputs['request'] = SetBoolRequest(data=False)
        crash_if_true_service_leaf.tick()
        self.assertEqual(crash_if_true_service_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.5)
        crash_if_true_service_leaf.tick()
        self.assertEqual(crash_if_true_service_leaf.state, NodeMsg.SUCCEEDED)

        self.assertEqual(crash_if_true_service_leaf.outputs['response'], SetBoolResponse(True, ''))

    def testReset(self):
        self.delay_service_leaf.inputs['request'] = SetBoolRequest(data=True)

        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.RUNNING)

        self.delay_service_leaf.reset()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.IDLE)

    def testShutdown(self):
        delay_service = Service(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'service_name': 'delay_1s_if_true',
            'wait_for_service_seconds': 0.5,
            'wait_for_response_seconds': 1.5
        })

        self.assertEqual(delay_service.state, NodeMsg.UNINITIALIZED)

        delay_service.setup()
        self.assertEqual(delay_service.state, NodeMsg.IDLE)

        delay_service.shutdown()
        self.assertEqual(delay_service.state, NodeMsg.SHUTDOWN)

    def testUntick(self):
        self.delay_service_leaf.inputs['request'] = SetBoolRequest(data=True)

        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.1)
        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.RUNNING)

        self.delay_service_leaf.untick()
        self.delay_service_leaf.inputs['request'] = SetBoolRequest(data=False)
        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.1)
        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.SUCCEEDED)

    def testTimeout(self):
        # Overwrite the leaf with one that has a shorter timeot
        self.delay_service_leaf = Service(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'service_name': 'delay_1s_if_true',
            'wait_for_service_seconds': 0.5,
            'wait_for_response_seconds': 0.5
        })
        self.delay_service_leaf.setup()

        self.delay_service_leaf.inputs['request'] = SetBoolRequest(data=True)

        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.RUNNING)

        rospy.sleep(0.6)
        self.delay_service_leaf.tick()
        self.assertEqual(self.delay_service_leaf.state, NodeMsg.FAILED)

    def testFailIfNotAvailable(self):
        service_leaf = Service(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'service_name': 'this_service_is_not_available',
            'wait_for_service_seconds': 0.5,
            'wait_for_response_seconds': 1.5,
            'fail_if_not_available': True
        })

        service_leaf.setup()
        self.assertFalse(service_leaf._service_available)

        service_leaf.inputs['request'] = SetBoolRequest(data=True)

        self.assertEqual(service_leaf.tick(), NodeMsg.FAILED)

        expected_bounds = UtilityBounds()
        self.assertEqual(service_leaf.calculate_utility(), expected_bounds)

    def testExceptionIfNotAvailable(self):
        service_leaf = Service(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'service_name': 'this_service_is_not_available',
            'wait_for_service_seconds': 0.5,
            'wait_for_response_seconds': 1.5,
            'fail_if_not_available': False
        })

        self.assertRaises(rospy.ROSException, service_leaf.setup)


if __name__ == '__main__':
    rospy.init_node('test_service_leaf')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_service_leaf')
    rostest.rosrun(PKG, 'test_service_leaf', TestServiceLeaf,
                   sysargs=sys.argv + ['--cov'])
