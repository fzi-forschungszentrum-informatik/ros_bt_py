#!/usr/bin/env python
import rospy

import actionlib

import actionlib_tutorials.msg


class FibonacciAction(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, actionlib_tutorials.msg.FibonacciAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        self._as.set_aborted()

if __name__ == '__main__':
    rospy.init_node('test_actions')
    server = FibonacciAction('fibonacci_fail')

    rospy.spin()
