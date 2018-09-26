#! /usr/bin/env python2.7

import rospy

from ros_bt_py.find_best_executor_server import FindBestExecutorServer


if __name__ == '__main__':
    rospy.init_node('find_best_executor')

    FindBestExecutorServer()

    rospy.spin()
