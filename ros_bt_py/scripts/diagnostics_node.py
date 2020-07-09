#!/usr/bin/env python

import rospy

from ros_bt_py_msgs.msg import Messages, Packages

from ros_bt_py_msgs.msg import NodeDiagnostics
from ros_bt_py_msgs.srv import SetExecutionMode, SetExecutionModeRequest


class DiagnosticsNode(object):
    def __init__(self):
        rospy.Subscriber('tree_node/debug/node_diagnostics',
                         NodeDiagnostics, self.diagnostics_callback)

    def diagnostics_callback(self, data):
        rospy.loginfo("%s (%s.%s) state: %s" % (
            ' > '.join(data.path),
            data.module,
            data.node_class,
            data.state
        ))


if __name__ == '__main__':
    rospy.init_node('diagnostics_node')
    rospy.loginfo("initializing diagnostics node...")

    node = DiagnosticsNode()

    rospy.wait_for_service('tree_node/debug/set_execution_mode')
    set_execution_mode = rospy.ServiceProxy('tree_node/debug/set_execution_mode', SetExecutionMode)
    response = set_execution_mode(SetExecutionModeRequest(collect_node_diagnostics=True))
    rospy.loginfo("initialized diagnostics node")

    rospy.spin()
    response = set_execution_mode(SetExecutionModeRequest(collect_node_diagnostics=False))
