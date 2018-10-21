#!/usr/bin/env python2.7
import unittest

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from ros_bt_py.ros_helpers import AsyncServiceProxy

PKG = 'ros_bt_py'


class TestAsyncService(unittest.TestCase):
    def setUp(self):
        self.async_proxy = AsyncServiceProxy('delay_1s_if_true', SetBool)

    def testServiceCall(self):
        # The service returns immediately if data is False
        self.async_proxy.call_service(SetBoolRequest(data=False))
        # But since there's still some time between starting a process and that
        # process successfully setting the response, response should be None
        # immediately after calling the service
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.RUNNING)
        self.assertIsNone(self.async_proxy.get_response())

        # Wait a bit, however, and we're good
        rospy.sleep(0.1)
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.RESPONSE_READY)
        self.assertTrue(self.async_proxy.get_response().success)

    def testDelayedServiceCall(self):
        # The service returns after 1 second if data is Tre
        self.async_proxy.call_service(SetBoolRequest(data=True))

        # After half a second, the service is still running
        rospy.sleep(0.5)
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.RUNNING)
        self.assertIsNone(self.async_proxy.get_response())

        # One more second and we're certainly past the 1s of delay
        rospy.sleep(1.0)
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.RESPONSE_READY)
        self.assertTrue(self.async_proxy.get_response().success)

    def testAbortCall(self):
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.IDLE)
        # aborting with no active call shouldn't do anything
        self.async_proxy.stop_call()
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.IDLE)

        # This request will return after a second
        self.async_proxy.call_service(SetBoolRequest(data=True))
        rospy.sleep(0.1)

        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.RUNNING)
        self.assertIsNone(self.async_proxy.get_response())
        self.async_proxy.stop_call()
        self.assertIsNone(self.async_proxy._process)
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.ABORTED)

        rospy.sleep(1.0)
        # We cancelled the service call, so even after waiting, we should not
        # have a response
        self.assertIsNone(self.async_proxy.get_response())

    def testOverwriteCall(self):
        # This request will return after a second
        self.async_proxy.call_service(SetBoolRequest(data=True))
        rospy.sleep(0.1)
        self.assertIsNone(self.async_proxy.get_response())
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.RUNNING)

        # This new call should override the old one, and immediately return
        self.async_proxy.call_service(SetBoolRequest(data=False))

        for _ in range(10):
            rospy.sleep(0.1)
            # We cancelled the delayed service call, so we should have a response by
            # now
            if self.async_proxy.get_state() == AsyncServiceProxy.RESPONSE_READY:
                self.assertTrue(self.async_proxy.get_response().success)
                return

        raise Exception('Received no response after 1 second')

    def testCrashingService(self):
        crash_proxy = AsyncServiceProxy('crash', SetBool)

        crash_proxy.call_service(SetBoolRequest())
        rospy.sleep(0.1)

        self.assertEqual(crash_proxy.get_state(), AsyncServiceProxy.ERROR)


if __name__ == '__main__':
    rospy.init_node('test_async_service')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_async_service')
    rostest.rosrun(PKG, 'test_async_service', TestAsyncService,
                   sysargs=sys.argv + ['--cov'])
