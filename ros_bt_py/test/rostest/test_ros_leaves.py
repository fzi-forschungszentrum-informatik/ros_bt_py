#!/usr/bin/env python2.7
import unittest

import rospy

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node_config import NodeConfig, OptionRef
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



if __name__ == '__main__':
    rospy.init_node('test_ros_leaves')
    import rostest
    rostest.rosrun(PKG, 'test_service_leaf', TestServiceLeaf)
