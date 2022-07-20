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
from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from roslib.message import get_message_class
import rospy

from ros_bt_py_msgs.srv import EvaluateUtility, EvaluateUtilityRequest
from ros_bt_py_msgs.msg import (
    FindBestExecutorAction,
    FindBestExecutorGoal,
    FindBestExecutorResult,
)
from ros_bt_py_msgs.msg import RunTreeAction, RunTreeGoal, TreeDataUpdate
from ros_bt_py_msgs.msg import UtilityBounds
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

from ros_bt_py.helpers import json_encode, json_decode
from ros_bt_py.ros_helpers import AsyncServiceProxy
from ros_bt_py.exceptions import BehaviorTreeException


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={
            "find_best_executor_action": str,
            "wait_for_find_best_executor_seconds": float,
            "find_best_executor_timeout_seconds": float,
            "remote_tick_frequency_hz": float,
            "run_tree_action_timeout_seconds": float,
            "wait_for_run_tree_seconds": float,
        },
        inputs={},
        outputs={"running_remotely": bool},
        max_children=1,
    )
)
class Shovable(Decorator):
    """Marks the subtree below this decorator as remote-executable.

    It will first send the subtree to `find_best_executor_action' (a
    :class:`ros_bt_py_msgs.msg.FindBestExecutorAction` server) to see
    who is best suited to execute it.

    Depending on the answer, it will either tick the subtree itself of
    use the action server from the action result to execute it
    remotely.

    The negotiation induces some delay, so expect a few ticks' wait
    before the subtree is actually executed. Consequently, make sure
    that the subtree takes long enough to arrive at a final state
    (`SUCCEEDED` or `FAILED`) that this overhead is justified.

    Note that thise node will not even `setup` successfully if
    `find_best_executor_action` is unavailable. This is by design,
    because although we *could* fall back to always executing locally,
    that would essentially hide any errors in the ROS graph
    (i.e. nodes launched in the wrong namespace, nodes that did not
    launch or crashed, etc.)

    """

    # States to structure the do_tick() method
    IDLE = 0
    WAIT_FOR_UTILITY_RESPONSE = 1
    ACTION_CLIENT_INIT = 2
    START_REMOTE_EXEC_ACTION = 3
    EXECUTE_LOCAL = 4
    EXECUTE_REMOTE = 5

    def _do_setup(self):
        self._state = Shovable.IDLE
        self._remote_namespace = ""
        self._find_best_executor_start_time = None
        self._subtree_action_start_time = None
        self._subtree_action_client = None
        self._subtree_action_client_creation_time = None
        # Save this so we can be sure we're actually executing the same tree we
        # evaluated the utility value for
        self._subtree_msg = None

        # Don't setup subtree here - we don't know yet if it is going
        # to be executed locally, or if it even can be!
        #
        # for child in self.children:
        #     child.setup()
        if not self.children:
            msg = "Trying to set up Shovable decorator without a child!"
            self.logwarn(msg)
            raise BehaviorTreeException(msg)

        _, incoming_conns, outgoing_conns = self.children[0].get_subtree_msg()

        # These should be references to all the children with external inputs.
        #
        # It's fine to cache this list: Our children can only change
        # when the tree is shutdown, so setup() will be called again
        # when they do.
        self._children_with_external_outputs = {
            child_node.name: child_node
            for child_node in self.get_children_recursive()
            if child_node.name in [conn.source.node_name for conn in outgoing_conns]
        }

        self._external_outputs_by_name = {}
        for conn in outgoing_conns:
            if conn.source.node_name not in self._external_outputs_by_name:
                self._external_outputs_by_name[conn.source.node_name] = []
            self._external_outputs_by_name[conn.source.node_name].append(
                conn.source.data_key
            )

        # Create an action client for FindBestExecutor
        self._find_best_executor_ac = SimpleActionClient(
            self.options["find_best_executor_action"], FindBestExecutorAction
        )

        if not self._find_best_executor_ac.wait_for_server(
            timeout=rospy.Duration(self.options["wait_for_find_best_executor_seconds"])
        ):
            raise BehaviorTreeException(
                "Action server %s not available after waiting %f seconds!"
                % (
                    self.options["find_best_executor_action"],
                    self.options["wait_for_find_best_executor_seconds"],
                )
            )

    def _do_tick(self):  # noqa: C901 # TODO: Simplify the method.
        # If the child is not currently running, check where best to execute it
        if self._state == Shovable.IDLE:
            # Kick off a service call
            self._subtree_msg, _, _ = self.children[0].get_subtree_msg()

            self._find_best_executor_ac.send_goal(
                FindBestExecutorGoal(tree=self._subtree_msg)
            )
            self._find_best_executor_start_time = rospy.Time.now()

            self._state = Shovable.WAIT_FOR_UTILITY_RESPONSE

        # Note that this and the following are if's, not elif's!
        # This allows execution to "fall through" multiple states in a single tick
        if self._state == Shovable.WAIT_FOR_UTILITY_RESPONSE:
            if rospy.Time.now() - self._find_best_executor_start_time > rospy.Duration(
                self.options["find_best_executor_timeout_seconds"]
            ):
                self.logerr(
                    "FindBestExecutor action timed out after %.2f seconds"
                    % self.options["find_best_executor_timeout_seconds"]
                )
                self._state = Shovable.IDLE
                return NodeMsg.FAILED

            find_best_executor_state = self._find_best_executor_ac.get_state()

            if find_best_executor_state in [
                GoalStatus.PREEMPTED,
                GoalStatus.SUCCEEDED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ]:
                # Fail if final goal status was not SUCCEEDED
                if find_best_executor_state == GoalStatus.SUCCEEDED:
                    # Figure out whether we're executing locally or remotely
                    find_best_executor_result = self._find_best_executor_ac.get_result()
                    if find_best_executor_result.local_is_best:
                        self.outputs["running_remotely"] = False
                        self._state = Shovable.EXECUTE_LOCAL
                    else:
                        if not find_best_executor_result.best_executor_namespace:
                            self.loginfo("Unable to execute subtree anywhere!")
                            self.cleanup()
                            return NodeMsg.FAILED
                        self.outputs["running_remotely"] = True
                        namespace = find_best_executor_result.best_executor_namespace

                        # No need to re-initialize the ActionClient, skip ahead
                        if namespace == self._remote_namespace:
                            self._state = Shovable.START_REMOTE_EXEC_ACTION
                        else:
                            self._remote_namespace = namespace

                            self.logdebug(f"Building ActionClient for action {self._remote_namespace}/run_tree")
                            self._subtree_action_client = SimpleActionClient(
                                f"{self._remote_namespace}/run_tree", RunTreeAction)

                            self._subtree_action_client_creation_time = rospy.Time.now()
                            self._state = Shovable.ACTION_CLIENT_INIT
                else:
                    self.logerr(
                        "FindBestExecutor action ended without succeeding (state ID: %d)"
                        % find_best_executor_state
                    )
                    self.cleanup()
                    return NodeMsg.FAILED
            # If goal is still running, return running
            else:
                return NodeMsg.RUNNING

        if self._state == Shovable.ACTION_CLIENT_INIT:
            # The timeout can't be 0, because that is interpreted as "infinite"
            if self._subtree_action_client.wait_for_server(
                timeout=rospy.Duration.from_sec(0.0001)
            ):
                self._state = Shovable.START_REMOTE_EXEC_ACTION
            else:
                self._state = Shovable.ACTION_CLIENT_INIT

            # After the set timeout, give up
            if (rospy.Time.now() - self._subtree_action_client_creation_time
                    > rospy.Duration(self.options['wait_for_run_tree_seconds'])):
                self.logerr(('Remote RunTree ActionClient for %s did not finish loading '
                             'after %.2f seconds. giving up.') %
                            (f"{self._remote_namespace}/run_tree",
                             self.options['wait_for_run_tree_seconds']))

                self.cleanup()
                return NodeMsg.FAILED

        if self._state == Shovable.START_REMOTE_EXEC_ACTION:
            # Ensure the "updated" state of children with outputs is
            # set correctly when executing remotely
            for child_node in self._children_with_external_outputs.values():
                child_node.outputs.reset_updated()

            self.logdebug(f"Sending goal to action server at {self._remote_namespace}/run_tree")
            self._subtree_action_start_time = rospy.Time.now()
            self._subtree_action_client.send_goal(
                RunTreeGoal(
                    tree=self._subtree_msg,
                    tick_frequency_hz=self.options["remote_tick_frequency_hz"],
                )
            )
            self._state = Shovable.EXECUTE_REMOTE

        if self._state == Shovable.EXECUTE_LOCAL:
            # Now that we know we're executing locally, setup subtree
            # if necessary
            if self.children[0].state == NodeMsg.UNINITIALIZED:
                self.children[0].setup()
            child_result = self.children[0].tick()

            return child_result

        if self._state == Shovable.EXECUTE_REMOTE:
            seconds_since_goal = (
                rospy.Time.now() - self._subtree_action_start_time
            ).to_sec()
            if seconds_since_goal > self.options["run_tree_action_timeout_seconds"]:
                self.logerr("Remote subtree execution timed out")
                self._subtree_action_client.cancel_goal()
                # Send another RunTree goal next tick. To switch to
                # another executor, we first have to reset()
                self._state = Shovable.START_REMOTE_EXEC_ACTION
                return NodeMsg.FAILED

            action_state = self._subtree_action_client.get_state()
            if action_state == GoalStatus.SUCCEEDED:
                final_tree = self._subtree_action_client.get_result().final_tree
                final_state = NodeMsg.BROKEN

                # Set and handle external outputs
                for node in final_tree.nodes:
                    if node.name in self._external_outputs_by_name:
                        for output in node.outputs:
                            if output.key in self._external_outputs_by_name[node.name]:
                                self._children_with_external_outputs[node.name].outputs[
                                    output.key
                                ] = json_decode(output.serialized_value)
                        self._children_with_external_outputs[
                            node.name
                        ]._handle_outputs()
                    if node.name == final_tree.root_name:
                        final_state = node.state

                # Send a new RunTree action goal on the next tick
                self._state = Shovable.START_REMOTE_EXEC_ACTION
                return final_state
            elif action_state in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                return NodeMsg.RUNNING
            else:
                # Send a new RunTree action goal on the next tick
                self._state = Shovable.START_REMOTE_EXEC_ACTION
                self._subtree_action_client.cancel_goal()
                return NodeMsg.FAILED

        return NodeMsg.RUNNING

    def _do_untick(self):
        """Pause execution of the decorated subtree, but preserve the executor

        This means a new
        :class:`ros_bt_py_msgs.msg.FindBestExecutorAction` will
        **not** be sent on the next tick. Rather, the same executor
        (either local or remote) as before is re-used.

        To force a new
        :class:`ros_bt_py_msgs.msg.FindBestExecutorAction`, use
        :meth:`_do_reset()`

        """
        # If we're in an EXECUTE state, stop the execution. otherwise,
        # just clean up
        new_state = NodeMsg.PAUSED
        if self._state == Shovable.WAIT_FOR_UTILITY_RESPONSE:
            self._find_best_executor_ac.cancel_goal()
        elif self._state == Shovable.EXECUTE_LOCAL:
            for child in self.children:
                new_state = child.untick()
        elif self._state == Shovable.EXECUTE_REMOTE:
            self._subtree_action_client.cancel_goal()
            # On the next tick, another goal is sent to the same
            # action server as before
            self._state = Shovable.START_REMOTE_EXEC_ACTION

        return new_state

    def _do_reset(self):
        if self._state == Shovable.EXECUTE_LOCAL:
            for child in self.children:
                child.reset()
        elif self._subtree_action_client is not None:
            self._subtree_action_client.cancel_goal()
        self.cleanup()

        return NodeMsg.IDLE

    def _do_shutdown(self):
        for child in self.children:
            child.shutdown()
        self._find_best_executor_ac.cancel_goal()
        if self._subtree_action_client is not None:
            self._subtree_action_client.cancel_goal()
        self.cleanup()

    def _do_calculate_utility(self):
        # We cannot just forward our child subtree's utility score,
        # because we might not be the one to execute it.  The only
        # case where we would want to just forward it is if
        # find_best_executor_action is not available. But in that
        # case, the tree will just fail to run, which is preferable
        # because it does not hide errors
        resolved_topic = rospy.resolve_name(
            f"{self.options['find_best_executor_action']}/status")

        for topic, topic_type_name in rospy.get_published_topics(rospy.get_namespace()):
            topic_type = get_message_class(topic_type_name)
            if topic == resolved_topic and topic_type == GoalStatusArray:
                # if the goal topic exists, we can execute the action, but
                # don't know much about the bounds, so set them all to
                # zero
                return UtilityBounds(
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True,
                )
        return UtilityBounds()

    def cleanup(self):
        self._remote_namespace = ""
        self._find_best_executor_start_time = None
        self._subtree_action_client = None
        self._subtree_action_client_creation_time = None
        self._subtree_msg = None
        self._state = Shovable.IDLE
