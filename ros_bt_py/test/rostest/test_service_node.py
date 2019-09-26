#!/usr/bin/env python2.7
import rospy
from std_srvs.srv import SetBool, SetBoolResponse


def delay_if_true(req):
    if req.data:
        rospy.sleep(1.0)
    return SetBoolResponse(True, '')


def crash(req):
    # Raises an exception for the caller
    return None


def crash_if_true(req):
    if req.data:
        return
    return SetBoolResponse(True, '')


if __name__ == '__main__':
    rospy.init_node('test_services')
    delay = rospy.Service('delay_1s_if_true', SetBool, delay_if_true)
    crash = rospy.Service('crash', SetBool, crash)
    crash_if_true = rospy.Service('crash_if_true', SetBool, crash_if_true)

    rospy.spin()
