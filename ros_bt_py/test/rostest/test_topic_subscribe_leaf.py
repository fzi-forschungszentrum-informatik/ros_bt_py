#!/usr/bin/env python2.7
import unittest

import rospy

from std_msgs.msg import Int32
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.topic import TopicSubscriber, TopicOnlineSubscriber

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

        sleeps = 0
        while True:
            self.subscriber_leaf.tick()
            self.assertNotEqual(self.subscriber_leaf.state, NodeMsg.FAILED)
            if self.subscriber_leaf.state == NodeMsg.SUCCEEDED:
                break
            rospy.sleep(0.1)
            sleeps += 1
            # If we don't get a response for half a second, something has gone wrong
            self.assertLess(sleeps, 5)

        self.assertEqual(self.subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.subscriber_leaf.outputs['message'].data, 8)

        self.assertEqual(self.subscriber_leaf.untick(), NodeMsg.IDLE)

        self.subscriber_leaf.reset()
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.IDLE)
        self.assertEqual(self.subscriber_leaf.outputs['message'], None)

        rospy.sleep(0.1)
        self.publisher.publish(data=9)

        sleeps = 0
        while True:
            self.subscriber_leaf.tick()
            self.assertNotEqual(self.subscriber_leaf.state, NodeMsg.FAILED)
            if self.subscriber_leaf.state == NodeMsg.SUCCEEDED:
                break
            rospy.sleep(0.1)
            sleeps += 1
            # If we don't get a response for half a second, something has gone wrong
            self.assertLess(sleeps, 5)

        self.subscriber_leaf.tick()
        # Same as before
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.subscriber_leaf.outputs['message'].data, 9)

    def testCalculateUtility(self):
        expected_bounds = UtilityBounds(can_execute=True,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)

        self.assertEqual(self.subscriber_leaf.calculate_utility(), expected_bounds)

    def testCalculateUtilityWithoutCorrectTopic(self):
        expected_bounds = UtilityBounds(can_execute=False,
                                        has_lower_bound_success=False,
                                        has_upper_bound_success=False,
                                        has_lower_bound_failure=False,
                                        has_upper_bound_failure=False)

        self.subscriber_leaf.options['topic_name'] = '/topic/does/not/exist'
        self.assertEqual(self.subscriber_leaf.calculate_utility(), expected_bounds)

    def testOnlineSubscriber(self):
        online_subscriber_leaf = TopicOnlineSubscriber(options={
            'topic_name': '/numbers_out',
            'topic_type': Int32
        })
        online_subscriber_leaf.setup()
        self.assertEqual(online_subscriber_leaf.state, NodeMsg.IDLE)
        self.assertIsNone(online_subscriber_leaf.outputs['message'])
        self.assertIsNone(online_subscriber_leaf._subscriber)

        online_subscriber_leaf.tick()
        # Should not have received any messages yet
        self.assertEqual(online_subscriber_leaf.state, NodeMsg.RUNNING)
        # subscriber should not be None anymore
        self.assertIsNotNone(online_subscriber_leaf._subscriber)

        self.publisher.publish(data=8)

        sleeps = 0
        while True:
            online_subscriber_leaf.tick()
            self.assertNotEqual(online_subscriber_leaf.state, NodeMsg.FAILED)
            if online_subscriber_leaf.state == NodeMsg.SUCCEEDED:
                break
            rospy.sleep(0.1)
            sleeps += 1
            # If we don't get a response for half a second, something has gone wrong
            self.assertLess(sleeps, 5)

        self.assertEqual(online_subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(online_subscriber_leaf.outputs['message'].data, 8)
        self.assertIsNone(online_subscriber_leaf._subscriber)

        self.assertEqual(online_subscriber_leaf.untick(), NodeMsg.IDLE)

        online_subscriber_leaf.reset()
        self.assertEqual(online_subscriber_leaf.state, NodeMsg.IDLE)
        self.assertEqual(online_subscriber_leaf.outputs['message'], None)

        self.assertEqual(online_subscriber_leaf.shutdown(), NodeMsg.SHUTDOWN)

        online_subscriber_leaf.setup()
        self.assertEqual(online_subscriber_leaf.state, NodeMsg.IDLE)

        self.assertEqual(online_subscriber_leaf.tick(), NodeMsg.RUNNING)
        self.assertEqual(online_subscriber_leaf.shutdown(), NodeMsg.SHUTDOWN)

        online_subscriber_leaf.setup()
        self.assertEqual(online_subscriber_leaf.state, NodeMsg.IDLE)
        self.assertEqual(online_subscriber_leaf.tick(), NodeMsg.RUNNING)
        self.assertEqual(online_subscriber_leaf.reset(), NodeMsg.IDLE)
        self.assertEqual(online_subscriber_leaf.shutdown(), NodeMsg.SHUTDOWN)

    def testCalculateUtilityOnlineSubscriber(self):
        online_subscriber_leaf = TopicOnlineSubscriber(options={
            'topic_name': '/numbers_out',
            'topic_type': Int32
        })
        expected_bounds = UtilityBounds(can_execute=True,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)
        self.assertEqual(online_subscriber_leaf.calculate_utility(), expected_bounds)

    def testCalculateUtilityWithoutCorrectTopicOnlineSubscriber(self):
        online_subscriber_leaf = TopicOnlineSubscriber(options={
            'topic_name': '/topic/does/not/exist',
            'topic_type': Int32
        })
        expected_bounds = UtilityBounds(can_execute=False,
                                        has_lower_bound_success=False,
                                        has_upper_bound_success=False,
                                        has_lower_bound_failure=False,
                                        has_upper_bound_failure=False)
        self.assertEqual(online_subscriber_leaf.calculate_utility(), expected_bounds)

if __name__ == '__main__':
    rospy.init_node('test_topic_subscribe_leaf')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_topic_subscribe_leaf')
    rostest.rosrun(PKG, 'test_topic_subscribe_leaf', TestTopicSubscriberLeaf,
                   sysargs=sys.argv + ['--cov'])
