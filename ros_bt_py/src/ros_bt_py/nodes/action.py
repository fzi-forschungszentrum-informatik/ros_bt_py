from threading import Lock
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'action_type': type,
             'goal_type': type,
             'feedback_type': type,
             'result_type': type,
             'action_name': str,
             'timeout_seconds': float},
    inputs={
        'goal': OptionRef('goal_type')},
    outputs={
        'feedback': OptionRef('feedback_type'),
        'result': OptionRef('result_type')},
    max_children=0))
class Action(Leaf):
    """Connects to a ROS action and sends the supplied goal.

    On every tick, outputs['feedback'] and outputs['result'] (if
    available) are updated.

    On untick, reset or shutdown, the goal is cancelled and will be
    re-sent on the next tick.
    """
    def _do_setup(self):
        self._lock = Lock()
        self._feedback = None
        self._has_active_goal = False

        self._ac = SimpleActionClient(self.options['action_name'],
                                      self.options['action_type'])
        return NodeMsg.IDLE

    def _feedback_cb(self, feedback):
        with self._lock:
            # TODO(nberg): Check if we need a deepcopy here
            self._feedback = feedback

    def _do_tick(self):
        current_state = self._ac.get_state()
        with self._lock:
            self.outputs['feedback'] = self._feedback
        if current_state is GoalStatus.LOST and not self._has_active_goal:
            # get_state returns LOST when the action client isn't tracking a
            # goal - so we can send a new one!
            self._ac.send_goal(self.inputs['goal'], feedback_cb=self._feedback_cb)
            self._has_active_goal = True
            return NodeMsg.RUNNING
        elif current_state in [
                GoalStatus.PREEMPTED,
                GoalStatus.SUCCEEDED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST
                ]:
            # we're done, one way or the other
            self.outputs['result'] = self._ac.get_result()
            # cancel goal to be sure, then stop tracking it so get_state()
            # returns LOST again
            self._ac.cancel_goal()
            self._ac.stop_tracking_goal()
            self._has_active_goal = False
            return NodeMsg.SUCCEEDED

        return NodeMsg.RUNNING

    def _do_untick(self):
        # stop the current goal but keep outputs
        self._ac.cancel_goal()
        self._has_active_goal = False
        self._feedback = None
        return NodeMsg.IDLE

    def _do_reset(self):
        # same as untick...
        self._do_untick()
        # but also clear the outputs
        self.outputs['feedback'] = None
        self.outputs['result'] = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        # nothing to do beyond what's done in reset
        self._do_reset()
