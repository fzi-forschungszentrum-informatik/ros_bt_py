#!/usr/bin/env python
#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
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

import unittest

from actionlib.simple_action_client import SimpleActionClient
import rospy

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from actionlib_tutorials.msg import (FibonacciAction, FibonacciGoal, FibonacciResult,
                                     FibonacciFeedback)

from ros_bt_py_msgs.msg import FindBestExecutorAction, FindBestExecutorGoal

from ros_bt_py.nodes.action import Action
from ros_bt_py.nodes.service import Service
from ros_bt_py.nodes.topic import TopicSubscriber
from ros_bt_py.nodes.sequence import Sequence

PKG = 'ros_bt_py'


class TestRosLeafUtility(unittest.TestCase):
    def setUp(self):
        self.ac = SimpleActionClient('find_best_executor', FindBestExecutorAction)

        # If find_best_executor isn't available within 2 seconds, fail
        # the test
        self.assertTrue(self.ac.wait_for_server(timeout=rospy.Duration(2.0)))

        self.topic = TopicSubscriber(options={
            'topic_type': Int32,
            'topic_name': 'numbers_out'})
        self.topic_2 = TopicSubscriber(
            name='Topic2',
            options={
                'topic_type': Int32,
                'topic_name': 'foo'})
        self.action = Action(options={
            'action_type': FibonacciAction,
            'goal_type': FibonacciGoal,
            'result_type': FibonacciResult,
            'feedback_type': FibonacciFeedback,
            'action_name': 'fibonacci',
            'wait_for_action_server_seconds': 1.0,
            'timeout_seconds': 1.0})
        self.service = Service(options={
            'service_type': SetBool,
            'request_type': SetBoolRequest,
            'response_type': SetBoolResponse,
            'service_name': 'delay_1s_if_true',
            'wait_for_service_seconds': 1.0,
            'wait_for_response_seconds': 1.0})

    def call_find_best_exec_with_node(self, node):
        goal_state = self.ac.send_goal_and_wait(
            node_to_goal(node),
            execute_timeout=rospy.Duration(2.0))
        self.assertEqual(goal_state, GoalStatus.SUCCEEDED)
        return self.ac.get_result()

    def testBestExecForSingleNodes(self):
        for node in [self.topic, self.action, self.service]:
            self.assertEqual(
                rospy.resolve_name(self.call_find_best_exec_with_node(node)
                                   .best_executor_namespace),
                rospy.resolve_name('has_stuff/good_slot/'),
                msg='Wrong namespace for node %s' % node.name)

        # Just to be sure, test one node that should be executed in
        # the other namespace
        self.assertEqual(
            rospy.resolve_name(self.call_find_best_exec_with_node(self.topic_2)
                               .best_executor_namespace),
            rospy.resolve_name('no_stuff/bad_slot/'))

    def testBestExecForSequence(self):
        seq = Sequence()\
            .add_child(self.topic)\
            .add_child(self.action)\
            .add_child(self.service)

        self.assertEqual(
            rospy.resolve_name(self.call_find_best_exec_with_node(seq)
                               .best_executor_namespace),
            rospy.resolve_name('has_stuff/good_slot/'))


def node_to_goal(node):
    goal = FindBestExecutorGoal()
    goal.tree = node.get_subtree_msg()[0]
    return goal


if __name__ == '__main__':
    rospy.init_node('test_action_leaf')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_ros_leaf_utility')
    rostest.rosrun(PKG, 'test_action_leaf', TestRosLeafUtility,
                   sysargs=sys.argv + ['--cov'])
