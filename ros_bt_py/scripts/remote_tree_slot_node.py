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

from actionlib.action_server import ActionServer

from ros_bt_py_msgs.msg import RemoteSlotState, RunTreeAction
from ros_bt_py_msgs.srv import ControlTreeExecution, EvaluateUtility

from ros_bt_py.remote_tree_slot import RemoteTreeSlot


def main():
    """Provide :class:`ros_bt_py.remote_tree_slot.RemoteTreeSlot` functionality.

    This ROS node offers a :class:`ros_bt_py_msgs.msg.RunTreeAction`
    server, and two service servers for
    :class:`ros_bt_py_msgs.srv.ControlTreeExecution` and
    :class:`ros_bt_py_msgs.srv.EvaluateUtility`.

    They are in the node's private namespace, making it possible to
    run multiple `RemoteTreeSlot` nodes in the same namespace.

    Most useful in conjunction with the
    :class:`ros_bt_py.nodes.remote_tree.RemoteTree` Behavior Tree
    node, which will send
    :class:`ros_bt_py_msgs.srv.ControlTreeExecution` requests to the
    slot.

    """
    rospy.init_node('remote_tree_slot')

    slot_state_pub = rospy.Publisher('~slot_state',
                                     RemoteSlotState,
                                     latch=True,
                                     queue_size=1)
    remote_slot = RemoteTreeSlot(publish_slot_state=slot_state_pub.publish)

    # Connect the action server's callbacks to the RemoteTreeSlot
    action_server = ActionServer('~run_tree',
                                 RunTreeAction,
                                 goal_cb=remote_slot.run_tree_handler,
                                 cancel_cb=remote_slot.cancel_run_tree_handler,
                                 auto_start=False)

    # Set up the service servers using the RemoteTreeSlot's handlers
    rospy.Service('~control_slot_execution',
                  ControlTreeExecution,
                  handler=remote_slot.control_tree_execution_handler)
    rospy.Service('~evaluate_utility',
                  EvaluateUtility,
                  handler=remote_slot.evaluate_utility_handler)

    # And go!
    action_server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
