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


"""BT node allowing the execution of remote subtrees using the shovable decorator."""
from threading import Lock

import rospy

from ros_bt_py_msgs.srv import EvaluateUtilityResponse, LoadTreeRequest
from ros_bt_py_msgs.srv import ControlTreeExecutionRequest, ControlTreeExecutionResponse
from ros_bt_py_msgs.msg import RunTreeResult
from ros_bt_py_msgs.msg import RemoteSlotState
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.tree_manager import TreeManager, get_success, get_error_message


class RemoteTreeSlot(object):
    """Provides an interface for a Behavior Tree to be 'shoved' into.

    This encompasses the following:

    A handler for :class:`ros_bt_py_msgs.srv.EvaluateUtility` to
    provide a utility estimate by using the `calculate_utility`
    methods of the BT in the Service request.

    A handler for :class:`ros_bt_py_msgs.msg.RunTreeAction`. This
    takes a remote tree, loads it, and returns when it is finished
    executing. Note that the remote tree may only run when it is
    allowed to (see below). Also, we may reject a goal, meaning the
    outside `ActionServer` implementation cannot be a
    `SimpleActionServer`!

    A handler for :class:`ros_bt_py_msgs.srv.ControlTreeExecution`, which
    can be called from a node in the "main" BT. This way, the main BT
    can determine when the remote tree in this slot (if any) is
    allowed to execute.

    """

    def __init__(self, publish_slot_state):
        """Initialize the `RemoteTreeSlot`.

        :param function publish_slot_state:

        A callback that will be called with the current state of the
        RemoteTreeSlot whenever it changes

        """
        self.run_tree_gh = None
        self.latest_tree = None
        self.slot_state = RemoteSlotState(
            tree_in_slot=False, tree_running=False, tree_finished=False
        )

        self._lock = Lock()

        self.publish_slot_state = publish_slot_state
        self.publish_slot_state(self.slot_state)
        self.tree_manager = TreeManager(publish_tree_callback=self.update_tree_msg)

    def evaluate_utility_handler(self, request):
        """Service handler for :class:`ros_bt_py_msgs.srv.EvaluateUtility`.

        Load the current tree and evaluate it. May respond with an
        empty response if there is another remote tree loaded and
        executing already.

        Note, however, that multiple instances of this class can exist
        per robot, so if *this slot* is filled, another one may be
        available on the same robot.

        """
        if self.run_tree_gh:
            # defaults to a message that has no bounds set
            rospy.logwarn(
                "Trying to evaluate utility when another tree is already loaded."
            )
            return EvaluateUtilityResponse()

        # shut down and clear TreeManager, just to be sure
        self.tree_manager.control_execution(
            ControlTreeExecutionRequest(command=ControlTreeExecutionRequest.SHUTDOWN)
        )
        self.tree_manager.clear(None)

        rospy.loginfo(f"Loading tree: {str(request.tree)}")
        res = self.tree_manager.load_tree(LoadTreeRequest(tree=request.tree))
        if not get_success(res):
            rospy.logerr(get_error_message(res))
            return EvaluateUtilityResponse()

        return EvaluateUtilityResponse(
            utility=self.tree_manager.find_root().calculate_utility()
        )

    def run_tree_handler(self, goal_handle):
        """`goal_callback` for the `RunTree` Action.

        The given behavior tree is loaded, but not executed until a
        :class:`ros_bt_py_msgs.srv.ControlTreeExecution` call lets us
        know we are allowed to execute.

        :param actionlib.server_goal_handle.ServerGoalHandle goal_handle:

        This has methods to communicate changes in goal status to the
        server (i.e. accepting/rejecting the goal, finishing it or
        accepting cancelling/other preemption)
        """
        rospy.loginfo("Got RunTree goal")
        if self.run_tree_gh:
            rospy.loginfo("Rejected goal because we already have a tree loaded")
            goal_handle.set_rejected()
            return

        stop_res = self.tree_manager.control_execution(
            ControlTreeExecutionRequest(command=ControlTreeExecutionRequest.SHUTDOWN)
        )
        if not get_success(stop_res):
            rospy.loginfo(
                "Rejected goal because shutting down the old tree failed with error "
                f"{get_error_message(stop_res)}"
            )
            goal_handle.set_rejected(
                text=f"Failed to shutdown old tree: {get_error_message(stop_res)}"
            )
            return

        res = self.tree_manager.load_tree(
            LoadTreeRequest(tree=goal_handle.get_goal().tree)
        )
        if not get_success(res):
            rospy.loginfo(
                "Rejected goal because loading the tree failed with error "
                f"{get_error_message(res)}"
            )
            goal_handle.set_rejected(
                text=f"Failed to load tree: {get_error_message(res)}"
            )
            return

        self.run_tree_gh = goal_handle
        self.latest_tree = None

        rospy.loginfo("Successfully loaded tree")
        self.slot_state.tree_in_slot = True
        self.slot_state.tree_finished = False
        self.publish_slot_state(self.slot_state)
        goal_handle.set_accepted()

    def control_tree_execution_handler(self, request):
        """Service handler for :class:`ros_bt_py_msgs.srv.ControlTreeExecution`.

        The request is forwarded to the current loaded remote subtree,
        if any.

        """
        if not self.run_tree_gh:
            rospy.logdebug(
                "Received a ControlTreeExecution request with no tree loaded. "
                "Nothing to do, succeeding."
            )
            return ControlTreeExecutionResponse(success=True)

        # No matter what happens, the tree is not finished.
        self.slot_state.tree_finished = False

        if request.command not in [
            ControlTreeExecutionRequest.TICK_ONCE,
            ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
            ControlTreeExecutionRequest.STOP,
            ControlTreeExecutionRequest.RESET,
            ControlTreeExecutionRequest.SHUTDOWN,
        ]:
            rospy.loginfo(f"Received invalid command: {str(request.command)}")
            return ControlTreeExecutionResponse(
                success=False,
                error_message=(
                    "RemoteTreeSlot does not allow ControlTreeExecution command "
                    f'"{request.command}"'
                ),
            )
        rospy.loginfo("Sending command %d to tree", request.command)
        res = self.tree_manager.control_execution(request)
        rospy.loginfo("ControlTreeExec result: %s", res)
        if not get_success(res):
            return res

        if request.command in [
            ControlTreeExecutionRequest.TICK_ONCE,
            ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
        ]:
            rospy.loginfo("started ticking loaded tree")
            self.slot_state.tree_running = True
        else:
            # combined with the if above, hitting this else means
            # the command was either STOP, RESET or SHUTDOWN, so
            # the tree is not running any more
            self.slot_state.tree_running = False

        self.publish_slot_state(self.slot_state)
        return res

    def cancel_run_tree_handler(self, goal_handle):
        """`cancel_callback` for the `RunTree` Action.

        If this is called, the remote tree is stopped and the Action
        call marked as preempted.

        :param actionlib.server_goal_handle.ServerGoalHandle goal_handle:

        This has methods to communicate changes in goal status to the
        server (i.e. accepting/rejecting the goal, finishing it or
        accepting cancelling/other preemption)

        """
        if (
            self.run_tree_gh
            and self.run_tree_gh.get_goal_id() == goal_handle.get_goal_id()
        ):
            rospy.logdebug(
                "Received cancel request for current goal, "
                "stopping and clearing tree"
            )

            stop_res = self.tree_manager.control_execution(
                ControlTreeExecutionRequest(
                    command=ControlTreeExecutionRequest.SHUTDOWN
                )
            )
            if not get_success(stop_res):
                raise Exception(
                    f"Failed to stop tree in RemoteTreeSlot: {get_success(stop_res)}"
                )

            self.tree_manager.clear(None)

            self.slot_state.tree_in_slot = False
            self.slot_state.tree_running = False
            self.slot_state.tree_finished = False
            self.publish_slot_state(self.slot_state)

            with self._lock:
                if self.latest_tree:
                    goal_handle.set_canceled(
                        result=RunTreeResult(final_tree=self.latest_tree)
                    )
                else:
                    goal_handle.set_canceled()
            # Remove existing goal handle and empty latest_tree
            self.run_tree_gh = None
            self.latest_tree = None
        else:
            goal_handle.set_canceled()

    def update_tree_msg(self, msg):
        """Update the tree message with the current state of the tree."""
        if self.run_tree_gh is None:
            return
        with self._lock:
            self.latest_tree = msg
            for node in self.latest_tree.nodes:
                if node.name == self.latest_tree.root_name:
                    if node.state in [
                        NodeMsg.SUCCEEDED,
                        NodeMsg.FAILED,
                        NodeMsg.SHUTDOWN,
                    ]:
                        # We got a result, send it back
                        self.slot_state.tree_running = False
                        self.slot_state.tree_finished = True
                        self.slot_state.tree_in_slot = False
                        self.publish_slot_state(self.slot_state)

                        # TODO(nberg): Can't shutdown and clear the
                        # tree from here, because that would deadlock
                        # the TreeManager. Is that okay, or do we need
                        # to figure out a way aruond that deadlock?

                        self.run_tree_gh.set_succeeded(
                            result=RunTreeResult(final_tree=self.latest_tree)
                        )
                        self.run_tree_gh = None
                        self.latest_tree = None

                        break
