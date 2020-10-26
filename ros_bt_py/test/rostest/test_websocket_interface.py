#!/usr/bin/env python

import unittest
try:
    import unittest.mock as mock
except ImportError:
    import mock

import sys
import rospy

from ros_bt_py_msgs.srv import GetAvailableNodesRequest
from ros_bt_py_msgs.msg import Package, Packages

from ros_bt_py.helpers import json_encode, json_decode

from twisted.internet import reactor
from autobahn.twisted.websocket import (WebSocketClientProtocol,
                                        WebSocketClientFactory)

from twisted.python import log
log.startLogging(sys.stderr)

PKG = 'ros_bt_py'


class SubscribePackagesClientProtocol(WebSocketClientProtocol):
    received = []

    def onOpen(self):
        message = {
            'op': 'subscribe',
            'topic': '/tree_node/packages',
            'type': 'ros_bt_py_msgs/Packages',
            'queue_length': 1,
        }

        encoded_message = json_encode(message).encode('utf-8')
        self.sendMessage(encoded_message)

    def onMessage(self, payload, binary):
        self.__class__.received.append(payload.decode())


class ServiceCallGetAvailableNodesClientProtocol(WebSocketClientProtocol):
    received = []

    def onOpen(self):
        message = {
            'op': 'call_service',
            'service': '/tree_node/get_available_nodes',
            'type': 'ros_bt_py_msgs/GetAvailableNodes',
            'args': json_encode({
                'node_modules': ''
            })
        }

        encoded_message = json_encode(message).encode('utf-8')
        self.sendMessage(encoded_message)

    def onMessage(self, payload, binary):
        self.__class__.received.append(payload.decode())


class TestTreeNodeOverWebsocket(unittest.TestCase):
    def setUp(self):
        self.port = rospy.get_param('/rosbridge_websocket/actual_port')
        self.url = 'ws://127.0.0.1:' + str(self.port)

    def testPackagesTopicAndGetAvailableNodesService(self):
        factory = WebSocketClientFactory(self.url)
        factory.protocol = SubscribePackagesClientProtocol
        reactor.connectTCP('127.0.0.1', self.port, factory)

        factory = WebSocketClientFactory(self.url)
        factory.protocol = ServiceCallGetAvailableNodesClientProtocol
        reactor.connectTCP('127.0.0.1', self.port, factory)

        def shutdown_timer():
            rospy.sleep(5.0)
            reactor.stop()

        reactor.callInThread(shutdown_timer)
        reactor.run()

        for received in SubscribePackagesClientProtocol.received:
            msg = json_decode(received)
            self.assertEqual('publish', msg['op'])
            self.assertEqual('/tree_node/packages', msg['topic'])
        self.assertEqual(1, len(SubscribePackagesClientProtocol.received))

        for received in ServiceCallGetAvailableNodesClientProtocol.received:
            msg = json_decode(received)
            self.assertEqual('service_response', msg['op'])
            self.assertEqual(True, msg['result'])
        self.assertEqual(1, len(ServiceCallGetAvailableNodesClientProtocol.received))


if __name__ == '__main__':
    rospy.init_node('test_websocket_interface')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_websocket_interface')
    while (not rospy.is_shutdown()
           and not rospy.has_param('/rosbridge_websocket/actual_port')
           and '/tree_node/packages' not in dict(rospy.get_published_topics()).keys()):
        rospy.sleep(1.0)
    rostest.rosrun(PKG, 'test_websocket_interface', TestTreeNodeOverWebsocket,
                   sysargs=sys.argv + ['--cov'])
