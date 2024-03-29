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
    import mock
except ImportError:
    import unittest.mock as mock

import rospy

from std_msgs.msg import Int32
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.topic import TopicSubscriber, TopicMemorySubscriber

PKG = "ros_bt_py"


class TestTopicSubscriberLeaf(unittest.TestCase):
    """
    Test the topic subscriber nodes.
    We call the callbacks directly instead of playing with ros asynchronous communication.
    """

    def setUp(self):
        self.subscriber_leaf = TopicSubscriber(
            options={"topic_name": "/numbers_out", "topic_type": Int32}
        )
        self.subscriber_leaf.setup()
        rospy.wait_for_message("/ready", Int32)

    def tearDown(self):
        self.subscriber_leaf.shutdown()

    def testReceivesNumber(self):
        self.assertIsNone(self.subscriber_leaf.outputs["message"])

        self.subscriber_leaf.tick()
        # Should not have received any messages yet
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.RUNNING)
        self.assertIsNotNone(self.subscriber_leaf._subscriber)
        self.assertEqual(
            self.subscriber_leaf._subscriber.callback, self.subscriber_leaf._callback
        )

        self.subscriber_leaf._callback(Int32(8))
        self.subscriber_leaf.tick()
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.subscriber_leaf.outputs["message"].data, 8)

        self.assertEqual(self.subscriber_leaf.untick(), NodeMsg.IDLE)

        self.subscriber_leaf.reset()
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.IDLE)
        self.assertEqual(self.subscriber_leaf.outputs["message"], None)

        self.subscriber_leaf._callback(Int32(9))
        self.subscriber_leaf.tick()
        # Same as before
        self.assertEqual(self.subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.subscriber_leaf.outputs["message"].data, 9)

    def testCalculateUtility(self):
        expected_bounds = UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )

        self.assertEqual(self.subscriber_leaf.calculate_utility(), expected_bounds)

    def testCalculateUtilityWithoutCorrectTopic(self):
        expected_bounds = UtilityBounds(
            can_execute=False,
            has_lower_bound_success=False,
            has_upper_bound_success=False,
            has_lower_bound_failure=False,
            has_upper_bound_failure=False,
        )

        self.subscriber_leaf.options["topic_name"] = "/topic/does/not/exist"
        self.assertEqual(self.subscriber_leaf.calculate_utility(), expected_bounds)

    @mock.patch("ros_bt_py.nodes.topic.rospy.Time.now")
    def testMemorySubscriber(self, mock_time_now):
        memory_subscriber_leaf = TopicMemorySubscriber(
            options={
                "topic_name": "/numbers_out",
                "topic_type": Int32,
                "memory_delay": 100.0,
            }
        )
        memory_subscriber_leaf.setup()

        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.IDLE)
        self.assertIsNone(memory_subscriber_leaf.outputs["message"])
        self.assertIsNotNone(memory_subscriber_leaf._subscriber)
        self.assertEqual(
            memory_subscriber_leaf._subscriber.callback,
            memory_subscriber_leaf._callback,
        )

        memory_subscriber_leaf.tick()
        # Should not have received any messages yet
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.FAILED)

        # Should receive a message at time t=0 and tick 1 second later
        mock_time_now.return_value = rospy.Time.from_seconds(0.0)
        memory_subscriber_leaf._callback(Int32(8))
        mock_time_now.return_value = rospy.Time.from_seconds(1.0)
        memory_subscriber_leaf.tick()

        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(memory_subscriber_leaf.outputs["message"].data, 8)

        # Should succeed again with the same message
        memory_subscriber_leaf.tick()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(memory_subscriber_leaf.outputs["message"].data, 8)

        # Should succeed again with a new message
        memory_subscriber_leaf._callback(Int32(3))
        memory_subscriber_leaf.tick()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.SUCCEEDED)
        self.assertEqual(memory_subscriber_leaf.outputs["message"].data, 3)

        # Should fail if the message is too old
        mock_time_now.return_value = rospy.Time.from_seconds(105.0)
        memory_subscriber_leaf.tick()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.FAILED)

        self.assertEqual(memory_subscriber_leaf.untick(), NodeMsg.IDLE)

        # Standard behavior
        memory_subscriber_leaf.reset()
        self.assertEqual(memory_subscriber_leaf.state, NodeMsg.IDLE)
        self.assertEqual(memory_subscriber_leaf.outputs["message"], None)

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
        memory_subscriber_leaf = TopicMemorySubscriber(
            options={
                "topic_name": "/numbers_out",
                "topic_type": Int32,
                "memory_delay": 100.0,
            }
        )
        expected_bounds = UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )
        self.assertEqual(memory_subscriber_leaf.calculate_utility(), expected_bounds)

    def testCalculateUtilityWithoutCorrectTopicMemorySubscriber(self):
        memory_subscriber_leaf = TopicMemorySubscriber(
            options={
                "topic_name": "/topic/does/not/exist",
                "topic_type": Int32,
                "memory_delay": 100.0,
            }
        )
        expected_bounds = UtilityBounds(
            can_execute=False,
            has_lower_bound_success=False,
            has_upper_bound_success=False,
            has_lower_bound_failure=False,
            has_upper_bound_failure=False,
        )
        self.assertEqual(memory_subscriber_leaf.calculate_utility(), expected_bounds)


if __name__ == "__main__":
    rospy.init_node("test_topic_subscribe_leaf")
    import rostest

    rostest.rosrun(
        PKG,
        "test_topic_subscribe_leaf",
        TestTopicSubscriberLeaf,
    )
