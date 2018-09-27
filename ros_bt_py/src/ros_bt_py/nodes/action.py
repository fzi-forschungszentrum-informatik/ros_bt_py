from copy import deepcopy
from threading import Lock

from roslib.message import get_message_class
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'action_type': type,
             'goal_type': type,
             'feedback_type': type,
             'result_type': type,
             'action_name': str,
             'wait_for_action_server_seconds': float,
             'timeout_seconds': float},
    inputs={
        'goal': OptionRef('goal_type')},
    outputs={
        'feedback': OptionRef('feedback_type'),
        'goal_status': int,
        'result': OptionRef('result_type')},
    max_children=0))
class Action(Leaf):
    """Connects to a ROS action and sends the supplied goal.

    Will always return RUNNING on the tick a new goal is sent, even if
    the server replies really quickly!

    On every tick, outputs['feedback'] and outputs['result'] (if
    available) are updated.

    On untick, reset or shutdown, the goal is cancelled and will be
    re-sent on the next tick.
    """
    def _do_setup(self):
        self._lock = Lock()
        self._feedback = None
        self._active_goal = None

        self._ac = SimpleActionClient(self.options['action_name'],
                                      self.options['action_type'])
        if not self._ac.wait_for_server(rospy.Duration.from_sec(
                self.options['wait_for_action_server_seconds'])):
            raise BehaviorTreeException(
                'Action server %s not available after waiting %f seconds!' % (
                    self.options['action_name'],
                    self.options['wait_for_action_server_seconds']))

        self._last_goal_time = None
        self.outputs['feedback'] = None
        self.outputs['goal_status'] = GoalStatus.LOST  # the default for no active goals
        self.outputs['result'] = None

        return NodeMsg.IDLE

    def _feedback_cb(self, feedback):
        with self._lock:
            # TODO(nberg): Check if we need a deepcopy here
            self._feedback = feedback

    def _do_tick(self):
        if self._active_goal is not None and self.inputs['goal'] != self._active_goal:
            # Goal message has changed since last tick, abort old goal
            # and return RUNNING
            self._ac.cancel_goal()
            self._active_goal = None

        if self._active_goal is None:
            # get_state returns LOST when the action client isn't tracking a
            # goal - so we can send a new one!
            self.logdebug('Sending goal: %s' % str(self.inputs['goal']))
            self._ac.send_goal(self.inputs['goal'], feedback_cb=self._feedback_cb)
            self._last_goal_time = rospy.Time.now()
            self._active_goal = deepcopy(self.inputs['goal'])
            return NodeMsg.RUNNING
        current_state = self._ac.get_state()
        self.logdebug('current_state: %s' % current_state)
        with self._lock:
            self.outputs['goal_status'] = current_state
            self.outputs['feedback'] = self._feedback

        if self.options['timeout_seconds'] != 0 and self._last_goal_time is not None:
            seconds_since_goal_start = (rospy.Time.now() - self._last_goal_time).to_sec()
            if seconds_since_goal_start > self.options['timeout_seconds']:
                self.logwarn('Stopping timed-out goal after %f seconds!' %
                             self.options['timeout_seconds'])
                self.outputs['goal_status'] = GoalStatus.LOST
                self._ac.cancel_goal()
                self._ac.stop_tracking_goal()
                self._last_goal_time = None
                self._active_goal = None

                return NodeMsg.FAILED

        if current_state in [
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
            self._active_goal = None

            # Fail if final goal status was not SUCCEEDED
            if current_state == GoalStatus.SUCCEEDED:
                return NodeMsg.SUCCEEDED
            else:
                return NodeMsg.FAILED

        return NodeMsg.RUNNING

    def _do_untick(self):
        # stop the current goal but keep outputs
        if self._active_goal is not None:
            self._ac.cancel_goal()
        self._last_goal_time = None
        self._active_goal = None
        self._feedback = None
        return NodeMsg.IDLE

    def _do_reset(self):
        # same as untick...
        self._do_untick()
        # but also clear the outputs
        self.outputs['feedback'] = None
        self.outputs['goal_status'] = GoalStatus.LOST  # the default for no active goals
        self.outputs['result'] = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        # nothing to do beyond what's done in reset
        self._do_reset()

    def _do_calculate_utility(self):
        resolved_topic = rospy.resolve_name(self.options['action_name'] + '/goal')

        for topic, topic_type_name in rospy.get_published_topics():
            topic_type = get_message_class(topic_type_name)
            if (topic == resolved_topic and
                    topic_type == self.options['goal_type']):
                # if the goal topic exists, we can execute the action, but
                # don't know much about the bounds, so set them all to
                # zero
                return UtilityBounds(
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True)
        return UtilityBounds()
