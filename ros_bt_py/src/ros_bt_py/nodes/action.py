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
from copy import deepcopy
from threading import Lock

from roslib.message import get_message_class
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus, GoalStatusArray

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import loglevel_is
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={
            "action_type": type,
            "goal_type": type,
            "feedback_type": type,
            "result_type": type,
            "action_name": str,
            "wait_for_action_server_seconds": float,
            "timeout_seconds": float,
            "fail_if_not_available": bool,
        },
        inputs={"goal": OptionRef("goal_type")},
        outputs={
            "feedback": OptionRef("feedback_type"),
            "goal_status": int,
            "result": OptionRef("result_type"),
        },
        max_children=0,
        optional_options=["fail_if_not_available"],
    )
)
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
        self._action_available = True
        self._shutdown = False

        self._ac = SimpleActionClient(
            self.options["action_name"], self.options["action_type"]
        )

        if not self._ac.wait_for_server(
            rospy.Duration.from_sec(self.options["wait_for_action_server_seconds"])
        ):
            self._action_available = False
            if (
                "fail_if_not_available" not in self.options
                or not self.options["fail_if_not_available"]
            ):
                raise BehaviorTreeException(
                    "Action server %s not available after waiting %f seconds!"
                    % (
                        self.options["action_name"],
                        self.options["wait_for_action_server_seconds"],
                    )
                )

        self._last_goal_time = None
        self.outputs["feedback"] = None
        self.outputs["goal_status"] = GoalStatus.LOST  # the default for no active goals
        self.outputs["result"] = None

        return NodeMsg.IDLE

    def _feedback_cb(self, feedback):
        with self._lock:
            # TODO(nberg): Check if we need a deepcopy here
            self._feedback = feedback

    def _do_tick(self):
        if not self._action_available:
            if (
                "fail_if_not_available" in self.options
                and self.options["fail_if_not_available"]
            ):
                return NodeMsg.FAILED

        if self._active_goal is not None and self.inputs["goal"] != self._active_goal:
            # Goal message has changed since last tick, abort old goal
            # and return RUNNING
            self._ac.cancel_goal()
            self._active_goal = None

        if self._active_goal is None:
            # get_state returns LOST when the action client isn't tracking a
            # goal - so we can send a new one!
            if loglevel_is(rospy.DEBUG):
                self.logdebug("Sending goal: %s" % str(self.inputs["goal"]))
            self._ac.send_goal(self.inputs["goal"], feedback_cb=self._feedback_cb)
            self._last_goal_time = rospy.Time.now()
            self._active_goal = deepcopy(self.inputs["goal"])
            return NodeMsg.RUNNING
        current_state = self._ac.get_state()
        if loglevel_is(rospy.DEBUG):
            self.logdebug("current_state: %s" % current_state)
        with self._lock:
            self.outputs["goal_status"] = current_state
            self.outputs["feedback"] = self._feedback

        if self.options["timeout_seconds"] != 0 and self._last_goal_time is not None:
            seconds_since_goal_start = (
                rospy.Time.now() - self._last_goal_time
            ).to_sec()
            if seconds_since_goal_start > self.options["timeout_seconds"]:
                self.logwarn(
                    "Stopping timed-out goal after %f seconds!"
                    % self.options["timeout_seconds"]
                )
                self.outputs["goal_status"] = GoalStatus.LOST
                self._ac.cancel_goal()
                self._ac.stop_tracking_goal()
                self._last_goal_time = None
                self._active_goal = None

                return NodeMsg.FAILED

        if current_state is GoalStatus.SUCCEEDED:
            result = self._ac.get_result()
            if self._ac.get_result() is None:
                return NodeMsg.RUNNING
            self.outputs["result"] = result
            # cancel goal to be sure, then stop tracking it so get_state()
            # returns LOST again
            self._active_goal = None

            # Fail if final goal status was not SUCCEEDED
            return NodeMsg.SUCCEEDED

        if current_state in [
            GoalStatus.PREEMPTED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        ]:
            # cancel goal to be sure, then stop tracking it so get_state()
            # returns LOST again
            self._ac.cancel_goal()
            self._ac.stop_tracking_goal()
            self._active_goal = None

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
        self.outputs["feedback"] = None
        self.outputs["goal_status"] = GoalStatus.LOST  # the default for no active goals
        self.outputs["result"] = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        # nothing to do beyond what's done in reset
        self._do_reset()
        self._action_available = False

    def _do_calculate_utility(self):
        resolved_topic = rospy.resolve_name(self.options["action_name"] + "/status")

        for topic, topic_type_name in rospy.get_published_topics(rospy.get_namespace()):
            topic_type = get_message_class(topic_type_name)
            if topic == resolved_topic and topic_type == GoalStatusArray:
                # if the goal topic exists, we can execute the action, but
                # don't know much about the bounds, so set them all to
                # zero
                return UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True,
                )
        return UtilityBounds()
