#!/usr/bin/env python2.7
import unittest
try:
    import unittest.mock as mock
except ImportError:
    import mock


import rospy

from std_msgs.msg import Int32
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.topic import TopicSubscriber, TopicMemorySubscriber

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

    @mock.patch('ros_bt_py.nodes.topic.rospy.Time.now')
    def testMemorySubscriber(self, mock_time_now):
        memory_subscriber_leaf = TopicMemorySubscriber(options={
            'topic_name': '/numbers_out',
            'topic_type': Int32,
            'memory_delay': 100.
        })
        memory_subscriber_leaf.setup()

        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.IDLE)
        self.assertIsNone(memory_subscriber_leaf.outputs['message'])
        self.assertIsNotNone(memory_subscriber_leaf._subscriber)

        memory_subscriber_leaf.tick()
        # Should not have received any messages yet
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.FAILED)

        # Should receive a message at time t=0 and tick 1 second later
        mock_time_now.return_value = rospy.Time.from_seconds(0.)
        memory_subscriber_leaf._callback(Int32(8))
        mock_time_now.return_value = rospy.Time.from_seconds(1.)
        memory_subscriber_leaf.tick()

        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(memory_subscriber_leaf.outputs['message'].data, 8)

        # Should succeed again with the same message
        memory_subscriber_leaf.tick()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(memory_subscriber_leaf.outputs['message'].data, 8)

        # Should succeed again with a new message
        memory_subscriber_leaf._callback(Int32(3))
        memory_subscriber_leaf.tick()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(memory_subscriber_leaf.outputs['message'].data, 3)

        # Should fail if the message is too old
        mock_time_now.return_value = rospy.Time.from_seconds(105.)
        memory_subscriber_leaf.tick()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.FAILED)

        self.assertEqual(memory_subscriber_leaf.untick(), NodeMsg.IDLE)

        # Standard behavior
        memory_subscriber_leaf.reset()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.IDLE)
        self.assertEqual(memory_subscriber_leaf.outputs['message'], None)

        self.assertEqual(memory_subscriber_leaf.shutdown(), NodeMsg.SHUTDOWN)

        memory_subscriber_leaf.setup()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.IDLE)

        self.assertEqual(memory_subscriber_leaf.tick(), NodeMsg.FAILED)
        self.assertEqual(memory_subscriber_leaf.shutdown(), NodeMsg.SHUTDOWN)

        memory_subscriber_leaf.setup()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.IDLE)
        self.assertEqual(memory_subscriber_leaf.tick(), NodeMsg.FAILED)
        self.assertEqual(memory_subscriber_leaf.reset(), NodeMsg.IDLE)
        self.assertEqual(memory_subscriber_leaf.shutdown(), NodeMsg.SHUTDOWN)

    def testCalculateUtilityMemorySubscriber(self):
        memory_subscriber_leaf = TopicMemorySubscriber(options={
            'topic_name': '/numbers_out',
            'topic_type': Int32,
            'memory_delay': 100.
        })
        expected_bounds = UtilityBounds(can_execute=True,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)
        self.assertEqual(memory_subscriber_leaf.calculate_utility(), expected_bounds)

    def testCalculateUtilityWithoutCorrectTopicMemorySubscriber(self):
        memory_subscriber_leaf = TopicMemorySubscriber(options={
            'topic_name': '/topic/does/not/exist',
            'topic_type': Int32,
            'memory_delay': 100.
        })
        expected_bounds = UtilityBounds(can_execute=False,
                                        has_lower_bound_success=False,
                                        has_upper_bound_success=False,
                                        has_lower_bound_failure=False,
                                        has_upper_bound_failure=False)
        self.assertEqual(memory_subscriber_leaf.calculate_utility(), expected_bounds)

if __name__ == '__main__':
    rospy.init_node('test_topic_subscribe_leaf')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_topic_subscribe_leaf')
    rostest.rosrun(PKG, 'test_topic_subscribe_leaf', TestTopicSubscriberLeaf,
                   sysargs=sys.argv + ['--cov'])
