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
