#! /usr/bin/env python2.7

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
