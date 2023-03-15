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


import unittest

try:
    import unittest.mock as mock
except ImportError:
    import mock

import os
import signal

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from ros_bt_py.ros_helpers import (
    AsyncServiceProxy,
)

PKG = "ros_bt_py"


class TestAsyncService(unittest.TestCase):
    def setUp(self):
        self.async_proxy = AsyncServiceProxy("delay_1s_if_true", SetBool)

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
        self.assertIsNone(self.async_proxy._thread)
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

        raise Exception("Received no response after 1 second")

    def testCrashingService(self):
        crash_proxy = AsyncServiceProxy("crash", SetBool)

        crash_proxy.call_service(SetBoolRequest())

        for _ in range(10):
            rospy.sleep(0.1)
            if crash_proxy.get_state() != AsyncServiceProxy.RUNNING:
                self.assertEqual(crash_proxy.get_state(), AsyncServiceProxy.ERROR)
                break

        AsyncServiceProxy._call_service_impl(
            crash_proxy._data,
            lock=crash_proxy._data_lock,
            abort=crash_proxy._abort,
            claim_cb=crash_proxy._claim_service_proxy,
            unclaim_cb=crash_proxy._unclaim_service_proxy,
        )

        self.assertEqual(crash_proxy.get_state(), AsyncServiceProxy.ERROR)

    def testCrashIfTrueService(self):
        crash_proxy = AsyncServiceProxy("crash_if_true", SetBool)

        crash_proxy.call_service(SetBoolRequest(data=True))

        for _ in range(10):
            rospy.sleep(0.1)
            if crash_proxy.get_state() != AsyncServiceProxy.RUNNING:
                self.assertEqual(crash_proxy.get_state(), AsyncServiceProxy.ERROR)
                break

        AsyncServiceProxy._call_service_impl(
            crash_proxy._data,
            lock=crash_proxy._data_lock,
            abort=crash_proxy._abort,
            claim_cb=crash_proxy._claim_service_proxy,
            unclaim_cb=crash_proxy._unclaim_service_proxy,
        )

        self.assertEqual(crash_proxy.get_state(), AsyncServiceProxy.ERROR)

        crash_proxy.call_service(SetBoolRequest(data=False))
        for _ in range(10):
            rospy.sleep(0.1)
            if crash_proxy.get_state() != AsyncServiceProxy.RUNNING:
                self.assertEqual(
                    crash_proxy.get_state(), AsyncServiceProxy.RESPONSE_READY
                )
                break

        AsyncServiceProxy._call_service_impl(
            crash_proxy._data,
            lock=crash_proxy._data_lock,
            abort=crash_proxy._abort,
            claim_cb=crash_proxy._claim_service_proxy,
            unclaim_cb=crash_proxy._unclaim_service_proxy,
        )

        self.assertEqual(crash_proxy.get_state(), AsyncServiceProxy.RESPONSE_READY)

    def testCallServiceImpl(self):
        rospy.logfatal("Ping")
        self.async_proxy._data["req"] = SetBoolRequest()

        AsyncServiceProxy._call_service_impl(
            self.async_proxy._data,
            lock=self.async_proxy._data_lock,
            abort=self.async_proxy._abort,
            claim_cb=self.async_proxy._claim_service_proxy,
            unclaim_cb=self.async_proxy._unclaim_service_proxy,
        )

        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.RESPONSE_READY)

        rospy.logfatal("Pong")

    def testWaitForServiceImpl(self):
        self.async_proxy._data["req"] = SetBoolRequest()
        AsyncServiceProxy._wait_for_service_impl(
            self.async_proxy._data,
            lock=self.async_proxy._data_lock,
            abort=self.async_proxy._abort,
            claim_cb=self.async_proxy._claim_service_proxy,
            unclaim_cb=self.async_proxy._unclaim_service_proxy,
        )

        self.assertEqual(
            self.async_proxy.get_state(), AsyncServiceProxy.SERVICE_AVAILABLE
        )

        self.async_proxy._data["proxy"] = None
        claim_cb = mock.MagicMock()
        unclaim_cb = mock.MagicMock()
        AsyncServiceProxy._wait_for_service_impl(
            self.async_proxy._data,
            lock=self.async_proxy._data_lock,
            abort=self.async_proxy._abort,
            claim_cb=claim_cb,
            unclaim_cb=unclaim_cb,
        )
        claim_cb.assert_called_once()

        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.ERROR)

    def testWaitForServiceImplAfterCall(self):
        self.async_proxy.call_service(SetBoolRequest(data=True))
        self.async_proxy.wait_for_service(timeout=0.5)

        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.WAITING)

    @unittest.skip("Waiting does not work.")
    def testWaitForServiceTwice(self):
        self.async_proxy.wait_for_service(timeout=0.5)
        self.async_proxy.wait_for_service(timeout=0.5)
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.WAITING)

    def testWaitForServiceThenCall(self):
        self.async_proxy.wait_for_service(timeout=0.5)
        self.async_proxy.call_service(SetBoolRequest(data=True))
        self.assertEqual(self.async_proxy.get_state(), AsyncServiceProxy.RUNNING)

    @mock.patch("os.kill")
    def testOSErrorOnStop(self, mock_os_kill):
        mock_os_kill.side_effect = OSError()
        # This request will return after a second
        self.async_proxy.call_service(SetBoolRequest(data=True))
        rospy.sleep(0.1)

        self.async_proxy.stop_call()

    def testROSException(self):
        class Proxy(object):
            def __init__(self):
                self.wait_for_service = None
                self.call = None

        data = dict()
        data["timeout"] = None
        data["req"] = None
        data["proxy"] = Proxy()
        data["proxy_id"] = 0
        data["proxy"].wait_for_service = mock.MagicMock()
        data[
            "proxy"
        ].wait_for_service.side_effect = rospy.exceptions.ROSInterruptException()

        claim_cb = mock.MagicMock()
        unclaim_cb = mock.MagicMock()
        lock = mock.MagicMock()
        abort = mock.MagicMock()

        AsyncServiceProxy._wait_for_service_impl(
            data, lock, abort, claim_cb, unclaim_cb
        )

        data["proxy"].wait_for_service.side_effect = rospy.exceptions.ROSException()
        AsyncServiceProxy._wait_for_service_impl(
            data, lock, abort, claim_cb, unclaim_cb
        )

        data["proxy"].wait_for_service.side_effect = Exception()
        AsyncServiceProxy._wait_for_service_impl(
            data, lock, abort, claim_cb, unclaim_cb
        )

        data["proxy"].call = mock.MagicMock()
        data["proxy"].call.side_effect = rospy.exceptions.ROSInterruptException()

        AsyncServiceProxy._call_service_impl(data, lock, abort, claim_cb, unclaim_cb)


if __name__ == "__main__":
    rospy.init_node("test_async_service")
    import rostest

    rostest.rosrun(
        PKG,
        "test_async_service",
        TestAsyncService,
    )
