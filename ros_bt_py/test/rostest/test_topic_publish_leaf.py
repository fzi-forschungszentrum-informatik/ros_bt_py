#!/usr/bin/env python
#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022-2023 FZI Forschungszentrum Informatik
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
#    * Neither the name of the {copyright_holder} nor the names of its
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
#  -------- END LICENSE BLOCK --------
from threading import Lock
import unittest

import rospy

from std_msgs.msg import Int32
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.topic import TopicPublisher

PKG = "ros_bt_py"


class TestTopicPublisherLeaf(unittest.TestCase):
    """This expects a test_topics_node.py instance running alongside

    That node will "reflect" anything we publish to /numbers_in - it's a
    separate node to avoid threading shenanigans in here.
    """

    def setUp(self):
        self.publisher_leaf = TopicPublisher(
            options={"topic_name": "/numbers_in", "topic_type": Int32}
        )
        self.publisher_leaf.setup()
        self._lock = Lock()
        self.msg = None
        self.subscriber = rospy.Subscriber("/numbers_out", Int32, self.cb)
        rospy.wait_for_message("/ready", Int32)

    def tearDown(self):
        self.publisher_leaf.shutdown()

    def cb(self, msg):
        with self._lock:
            self.msg = msg

    def testSendsNumber(self):
        self.assertIsNone(self.msg)

        self.publisher_leaf.inputs["message"] = Int32(data=1)
        self.publisher_leaf.tick()
        # This should basically never fail - anything that can go wrong should
        # go wrong in the setup() method
        self.assertEqual(self.publisher_leaf.state, NodeMsg.SUCCEEDED)

        rospy.sleep(0.1)
        self.assertEqual(self.msg.data, 1)

        self.publisher_leaf.inputs["message"] = Int32(data=42)
        self.publisher_leaf.tick()
        # This should basically never fail - anything that can go wrong should
        # go wrong in the setup() method
        self.assertEqual(self.publisher_leaf.state, NodeMsg.SUCCEEDED)

        rospy.sleep(0.1)
        self.assertEqual(self.msg.data, 42)

        self.assertEqual(self.publisher_leaf.untick(), NodeMsg.IDLE)

        self.publisher_leaf.reset()
        self.publisher_leaf.inputs["message"] = Int32(data=23)
        self.publisher_leaf.tick()
        # This should basically never fail - anything that can go wrong should
        # go wrong in the setup() method
        self.assertEqual(self.publisher_leaf.state, NodeMsg.SUCCEEDED)

        rospy.sleep(0.1)
        self.assertEqual(self.msg.data, 23)


if __name__ == "__main__":
    rospy.init_node("test_topic_publish_leaf")
    import rostest

    rostest.rosrun(
        PKG,
        "test_topic_publish_leaf",
        TestTopicPublisherLeaf,
    )
