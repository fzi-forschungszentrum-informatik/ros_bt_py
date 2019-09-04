#!/usr/bin/env python2.7
from threading import Lock
import unittest

import rospy

from std_msgs.msg import Int32
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.topic import TopicPublisher

PKG = 'ros_bt_py'


class TestTopicPublisherLeaf(unittest.TestCase):
    """This expects a test_topics_node.py instance running alongside

    That node will "reflect" anything we publish to /numbers_in - it's a
    separate node to avoid threading shenanigans in here.
    """
    def setUp(self):
        self.publisher_leaf = TopicPublisher(options={
            'topic_name': '/numbers_in',
            'topic_type': Int32
        })
        self.publisher_leaf.setup()
        self._lock = Lock()
        self.msg = None
        self.subscriber = rospy.Subscriber('/numbers_out', Int32, self.cb)
        rospy.wait_for_message('/ready', Int32)

    def tearDown(self):
        self.publisher_leaf.shutdown()

    def cb(self, msg):
        with self._lock:
            self.msg = msg

    def testSendsNumber(self):
        self.assertIsNone(self.msg)

        self.publisher_leaf.inputs['message'] = Int32(data=1)
        self.publisher_leaf.tick()
        # This should basically never fail - anything that can go wrong should
        # go wrong in the setup() method
        self.assertEqual(self.publisher_leaf.state, NodeMsg.SUCCEEDED)

        rospy.sleep(0.1)
        self.assertEqual(self.msg.data, 1)

        self.publisher_leaf.inputs['message'] = Int32(data=42)
        self.publisher_leaf.tick()
        # This should basically never fail - anything that can go wrong should
        # go wrong in the setup() method
        self.assertEqual(self.publisher_leaf.state, NodeMsg.SUCCEEDED)

        rospy.sleep(0.1)
        self.assertEqual(self.msg.data, 42)

        self.assertEqual(self.publisher_leaf.untick(), NodeMsg.IDLE)

        self.publisher_leaf.reset()
        self.publisher_leaf.inputs['message'] = Int32(data=23)
        self.publisher_leaf.tick()
        # This should basically never fail - anything that can go wrong should
        # go wrong in the setup() method
        self.assertEqual(self.publisher_leaf.state, NodeMsg.SUCCEEDED)

        rospy.sleep(0.1)
        self.assertEqual(self.msg.data, 23)


if __name__ == '__main__':
    rospy.init_node('test_topic_publish_leaf')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_topic_publish_leaf')
    rostest.rosrun(PKG, 'test_topic_publish_leaf', TestTopicPublisherLeaf,
                   sysargs=sys.argv + ['--cov'])
