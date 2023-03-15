# Copyright 2018-2023 FZI Forschungszentrum Informatik
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
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
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


from copy import deepcopy
from threading import Lock
from abc import ABC, abstractmethod

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
        options={
            "action_name": str,
            "wait_for_action_server_seconds": float,
            "timeout_seconds": float,
            "fail_if_not_available": bool,
        },
        inputs={},
        outputs={},
        max_children=0,
        optional_options=["fail_if_not_available"],
    )
)
class ActionForSetType(ABC, Leaf):
    """Abstract ROS action class.

    This class can be inherited to create ROS action nodes with a defined action type.
    Supports building simple custom nodes.

    Will always return RUNNING on the tick a new goal is sent, even if
    the server replies really quickly!

    On every tick, outputs['feedback'] and outputs['result'] (if
    available) are updated.

    On untick, reset or shutdown, the goal is cancelled and will be
    re-sent on the next tick.

    Example:
        >>> @define_bt_node(NodeConfig(
                options={'MyOption': MyOptionsType},
                inputs={'MyInput': MyInputType},
                outputs={'MyOutput': MyOutputType}, # feedback, goal_status, result,..
                max_children=0))
        >>> class MyActionClass(ActionForSetType):
                # set all important action attributes
                def set_action_attributes(self):
                    self._action_type = MyAction
                    self._goal_type = MyActionGoal
                    self._feedback_type = MyActionFeedback
                    self._result_type = MyActionResult

                    self._action_name = self.options['MyAction']
                    self._output_feedback = self._action_name + '/feedback'
                    self._output_goal_status = self._action_name + '/status'
                    self._output_result = self._action_name + '/result'

                # set the action goal
                def set_goal(self):
                    self._input_goal = MyActionGoal()
                    self._input_goal.MyInput = self.inputs['MyImput']
    """

    @abstractmethod
    def set_action_attributes(self):
        """set all important action attributes"""
        self._action_type = "ENTER_ACTION_TYPE"
        self._goal_type = "ENTER_GOAL_TYPE"
        self._feedback_type = "ENTER_FEEDBACK_TYPE"
        self._result_type = "ENTER_RESULT_TYPE"

        self._action_name = self.options["action_name"]
        self._output_feedback = "ENTER_OUTPUT_FEEDBACK"
        self._output_goal_status = "ENTER_OUTPUT_GOAL_STATUS"
        self._output_result = "ENTER_OUTPUT_RESULT"

    def set_input(self):
        pass

    # overwrite, if there is more than one output key to be overwritten
    def set_output_none(self):
        self.outputs["feedback"] = None
        self.outputs["goal_status"] = GoalStatus.LOST  # the default for no active goals
        self.outputs["result"] = None

    @abstractmethod
    def set_goal(self):
        self._input_goal = "ENTER_GOAL_FROM_INPUT"

    def _do_setup(self):
        self._lock = Lock()
        self._feedback = None
        self._active_goal = None
        self._action_available = True
        self._shutdown = False

        self.set_action_attributes()
        self._ac = SimpleActionClient(self._action_name, self._action_type)

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
                        self._action_name,
                        self.options["wait_for_action_server_seconds"],
                    )
                )

        self._last_goal_time = None
        self.set_output_none()

        return NodeMsg.IDLE

    def _feedback_cb(self, feedback):
        with self._lock:
            # TODO(nberg): Check if we need a deepcopy here
            self._feedback = feedback

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug("Simulating tick. Action is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if not self._action_available:
            if (
                "fail_if_not_available" in self.options
                and self.options["fail_if_not_available"]
            ):
                return NodeMsg.FAILED

        self.set_input()
        self.set_goal()
        if self._active_goal is not None and self._input_goal != self._active_goal:
            # Goal message has changed since last tick, abort old goal
            # and return RUNNING
            self._ac.cancel_goal()
            self._active_goal = None

        if self._active_goal is None:
            # get_state returns LOST when the action client isn't tracking a
            # goal - so we can send a new one!
            if loglevel_is(rospy.DEBUG):
                self.logdebug(f"Sending goal: {str(self.inputs['goal'])}")
            self._ac.send_goal(self.inputs["goal"], feedback_cb=self._feedback_cb)
            self._last_goal_time = rospy.Time.now()
            self._active_goal = deepcopy(self._input_goal)
            return NodeMsg.RUNNING
        current_state = self._ac.get_state()

        if loglevel_is(rospy.DEBUG):
            self.logdebug(f"current_state: {current_state}")
        with self._lock:
            self.outputs["goal_status"] = current_state
            self.outputs["feedback"] = self._feedback

        if self.options["timeout_seconds"] != 0 and self._last_goal_time is not None:
            seconds_since_goal_start = (
                rospy.Time.now() - self._last_goal_time
            ).to_sec()
            if seconds_since_goal_start > self.options["timeout_seconds"]:
                self.logwarn(
                    f"Stopping timed-out goal after {self.options['timeout_seconds']:f} seconds!"
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

            return NodeMsg.SUCCEEDED

        if current_state in [
            GoalStatus.PREEMPTED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        ]:
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
        resolved_topic = rospy.resolve_name(f"{self.options['action_name']}/status")

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


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={
            "action_type": type,
            "goal_type": type,
            "feedback_type": type,
            "result_type": type,
        },
        inputs={"goal": OptionRef("goal_type")},
        outputs={
            "feedback": OptionRef("feedback_type"),
            "goal_status": int,
            "result": OptionRef("result_type"),
        },
        max_children=0,
    )
)
class Action(ActionForSetType):
    """Connects to a ROS action and sends the supplied goal.

    Will always return RUNNING on the tick a new goal is sent, even if
    the server replies really quickly!

    On every tick, outputs['feedback'] and outputs['result'] (if
    available) are updated.

    On untick, reset or shutdown, the goal is cancelled and will be
    re-sent on the next tick.
    """

    def set_action_attributes(self):
        """set all action attributes"""
        self._action_type = self.options["action_type"]
        self._goal_type = self.options["goal_type"]
        self._feedback_type = self.options["feedback_type"]
        self._result_type = self.options["result_type"]

        self._action_name = self.options["action_name"]
        self._output_feedback = self.options["action_name"] + "/feedback"
        self._output_goal_status = self.options["action_name"] + "/status"
        self._output_result = self.options["action_name"] + "/result"

    def set_goal(self):
        self._input_goal = self.inputs["goal"]
