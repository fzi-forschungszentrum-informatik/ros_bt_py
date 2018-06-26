#! /usr/bin/env python2.7

from threading import Lock

import rospy

from actionlib.simple_action_server import SimpleActionServer
from actionlib_msgs.msg import GoalStatus

from ros_bt_py_msgs.srv import EvaluateUtility, EvaluateUtilityRequest, EvaluateUtilityResponse
from ros_bt_py_msgs.srv import ControlTreeExecutionRequest, LoadTreeRequest
from ros_bt_py_msgs.msg import RunTreeAction, RunTreeGoal, RunTreeResult, TreeDataUpdate
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.tree_manager import TreeManager
from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

from ros_bt_py import ros_helpers


ACTION_NAMESPACE = '/run_tree_remote'


def _get_success(response):
    if isinstance(response, dict):
        return response['success']

    return response.success


def _get_error_message(response):
    if isinstance(response, dict):
        return response['error_message']

    return response.error_message


def eval_util_remote_cb(EvaluateUtilityRequest):
    return EvaluateUtilityResponse(
        local_is_best=False,
        best_executor_namespace=ACTION_NAMESPACE)


def eval_util_local_cb(EvaluateUtilityRequest):
    return EvaluateUtilityResponse(
        local_is_best=True)


class RunTreeActionServer(object):
    def __init__(self):
        self._lock = Lock()
        self._tree_manager = TreeManager(publish_tree_callback=self.tree_update)
        self._as = SimpleActionServer(ACTION_NAMESPACE + '/run_tree', RunTreeAction,
                                      execute_cb=self.run_tree,
                                      auto_start=False)
        rospy.Subscriber('update_tree_data', TreeDataUpdate, self.update_tree_data)

        self._as.start()
        self._tree_state = None
        self._tree = None

    def update_tree_data(self, msg):
        # TODO(nberg): Something
        pass

    def tree_update(self, msg):
        with self._lock:
            self._tree = msg
            if msg.root_name:
                for node in msg.nodes:
                    if node.name == msg.root_name:
                        self._tree_state = node.state

    def run_tree(self, goal):
        with self._lock:
            self._tree_state = None
        # TreeManager is robust against shutting down when nothing is
        # running. so just shut it down preemptively.
        self._tree_manager.control_execution(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.SHUTDOWN))

        load_response = self._tree_manager.load_tree(
            LoadTreeRequest(tree=goal.tree))

        if not _get_success(load_response):
            rospy.logerr(_get_error_message(load_response))
            self._as.set_aborted(_get_error_message(load_response))

            return

        start_response = self._tree_manager.control_execution(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                tick_frequency_hz=goal.tick_frequency_hz))

        if not _get_success(start_response):
            rospy.logerr(_get_error_message(start_response))
            self._as.set_aborted(result=None, text=_get_error_message(start_response))

            return

        sleep_rate = rospy.Rate(goal.tick_frequency_hz)


        while True:
            if self._as.is_preempt_requested():
                self._tree_manager.control_execution(
                ControlTreeExecutionRequest(
                    command=ControlTreeExecutionRequest.SHUTDOWN))

                self._as.set_preempted()
                break

            with self._lock:
                if self._tree_state not in [NodeMsg.RUNNING,
                                            NodeMsg.IDLE,
                                            NodeMsg.UNINITIALIZED]:
                    self._as.set_succeeded(RunTreeResult(final_tree=self._tree_manager.to_msg()))
                    break
            sleep_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('dummy_remote_tree')

    rospy.Service('evaluate_utility_remote', EvaluateUtility, eval_util_remote_cb)
    rospy.Service('evaluate_utility_local', EvaluateUtility, eval_util_local_cb)

    server = RunTreeActionServer()

    rospy.spin()
