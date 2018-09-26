#! /usr/bin/env python2.7

import rospy

from actionlib.action_server import ActionServer

from ros_bt_py_msgs.msg import RunTreeAction
from ros_bt_py_msgs.srv import ControlTreeExecution, EvaluateUtility

from ros_bt_py.remote_tree_slot import RemoteTreeSlot


def main():
    rospy.init_node('remote_tree_slot')

    remote_slot = RemoteTreeSlot()

    # Connect the action server's callbacks to the RemoteTreeSlot
    action_server = ActionServer(RunTreeAction,
                                 'run_tree_in_slot',
                                 goal_cb=remote_slot.run_tree_handler,
                                 cancel_cb=remote_slot.cancel_run_tree_handler,
                                 auto_start=False)

    # Set up the service servers using the RemoteTreeSlot's handlers
    rospy.Service('control_slot_execution',
                  ControlTreeExecution,
                  handler=remote_slot.control_tree_execution_handler)
    rospy.Service('evaluate_utility',
                  EvaluateUtility,
                  handler=remote_slot.evaluate_utility_handler)

    # And go!
    action_server.start()
    rospy.spin()


if __name__ == '__main__':
    main()
