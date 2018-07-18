#! /usr/bin/env python2

import rospy

from ros_bt_py.jsonpickle_server import get_jsonpickle_instance as service_handler
from ros_bt_py_msgs.srv import GetJsonpickleInstance

if __name__ == '__main__':
    rospy.init_node('get_jsonpickle_instance')

    s = rospy.Service('get_jsonpickle_instance',
                      GetJsonpickleInstance,
                      service_handler)

    rospy.spin()
