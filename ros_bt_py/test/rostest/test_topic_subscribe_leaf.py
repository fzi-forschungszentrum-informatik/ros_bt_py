#!/usr/bin/env python2.7
import unittest

import rospy

from std_msgs.msg import Int32
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.topic import TopicSubscriber

PKG = 'ros_bt_py'


class TestTopicSubscriberLeaf(unittest.TestCase):
    """This expects a test_topics_node.py instance running alongside

    That node will "reflect" anything we publish to /numbers_in - it's a
    separate node to avoid threading shenanigans in here.
    """
    def setUp(self):
        self.subscriber_leaf = TopicSubscriber(options={
            'topic_name': '/numbers_out',
            'topic_type': Int32
            })
        self.subscriber_leaf.setup()
        self.publisher = rospy.Publisher('/numbers_in', Int32, latch=True, queue_size=1)
        rospy.wait_for_message('/ready', Int32)

    def tearDown(self):
        self.subscriber_leaf.shutdown()

    def testReceivesNumber(self):
        self.assertIsNone(self.subscriber_leaf.outputs['message'])

        self.subscriber_leaf.tick()
        # Should not have received any messages yet
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.RUNNING)

        self.publisher.publish(data=8)
        rospy.sleep(0.1)

        self.subscriber_leaf.tick()
        # Should have received a message with the 8 we published
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.subscriber_leaf.outputs['message'].data, 8)

        self.subscriber_leaf.reset()
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.IDLE)
        self.assertEqual(self.subscriber_leaf.outputs['message'], None)

        rospy.sleep(0.1)
        self.publisher.publish(data=9)
        rospy.sleep(0.1)

        self.subscriber_leaf.tick()
        # Same as before
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.subscriber_leaf.outputs['message'].data, 9)


if __name__ == '__main__':
    rospy.init_node('test_topic_subscribe_leaf')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_topic_subscribe_leaf')
    rostest.rosrun(PKG, 'test_topic_subscribe_leaf', TestTopicSubscriberLeaf,
                   sysargs=sys.argv + ['--cov'])
