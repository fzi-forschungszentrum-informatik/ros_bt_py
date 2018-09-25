from threading import Lock

import rospy

from ros_bt_py_msgs.srv import EvaluateUtilityResponse, LoadTreeRequest
from ros_bt_py_msgs.srv import ControlTreeExecutionRequest, ControlTreeExecutionResponse
from ros_bt_py_msgs.msg import RunTreeResult
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.tree_manager import TreeManager


class RemoteTreeSlot(object):
    """Provides an interface for a Behavior Tree to be 'shoved' into

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
    def __init__(self):
        """Initialize the `RemoteTreeSlot`

        """
        self.run_tree_gh = None
        self.latest_tree = None

        self._lock = Lock()

        self.tree_manager = TreeManager(publish_tree_callback=self.update_tree_msg)

    def evaluate_utility_handler(self, request):
        """A service handler for :class:`ros_bt_py_msgs.srv.EvaluateUtility`

        Load the current tree and evaluate it. May respond with an
        empty response if there is another remote tree loaded and
        executing already.

        Note, however, that multiple instances of this class can exist
        per robot, so if *this slot* is filled, another one may be
        available on the same robot.

        """
        if self.run_tree_gh:
            # defaults to a message that has no bounds set
            rospy.logwarn('Trying to evaluate utility when another tree is already loaded.')
            return EvaluateUtilityResponse()

        # shut down and clear TreeManager, just to be sure
        self.tree_manager.control_execution(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.SHUTDOWN))
        self.tree_manager.clear(None)

        # rospy.loginfo('Loading tree: %s' % str(request.tree))
        res = self.tree_manager.load_tree(LoadTreeRequest(tree=request.tree))
        if not res.success:
            rospy.logerr(res.error_message)

        return EvaluateUtilityResponse(
            utility=self.tree_manager.find_root().calculate_utility())

    def run_tree_handler(self, goal_handle):
        """This is the `goal_callback` for the `RunTree` Action.

        The given behavior tree is loaded, but not executed until a
        :class:`ros_bt_py_msgs.srv.ControlTreeExecution` call lets us
        know we are allowed to execute.

        :param actionlib.server_goal_handle.ServerGoalHandle goal_handle:

        This has methods to communicate changes in goal status to the
        server (i.e. accepting/rejecting the goal, finishing it or
        accepting cancelling/other preemption)
        """
        if self.run_tree_gh:
            goal_handle.set_rejected()
            return

        self.run_tree_gh = goal_handle
        self.latest_tree = None
        # TODO(nberg): Check result
        self.tree_manager.load_tree(
            LoadTreeRequest(tree=goal_handle.get_goal().tree))
        goal_handle.set_accepted()

    def control_tree_execution_handler(self, request):
        """A Service handler for :class:`ros_bt_py_msgs.srv.ControlTreeExecution`

        The request is forwarded to the current loaded remote subtree,
        if any.

        """
        if not self.run_tree_gh:
            rospy.logdebug('Received a ControlTreeExecution request with no tree loaded. '
                           'Nothing to do, succeeding.')
            return ControlTreeExecutionRequest(success=True)

        if request.command not in [
                ControlTreeExecutionRequest.TICK_ONCE,
                ControlTreeExecutionRequest.TICK_UNTIL_RESULT,
                ControlTreeExecutionRequest.STOP,
                ControlTreeExecutionRequest.RESET,
                ControlTreeExecutionRequest.SHUTDOWN]:
            return ControlTreeExecutionResponse(
                success=False,
                error_message=('RemoteTreeSlot does not allow ControlTreeExecution command "%s"'
                               % request.command))
        return self.tree_manager.control_execution(request)

    def cancel_run_tree_handler(self, goal_handle):
        """A `cancel_callback` for the `RunTree` Action.

        If this is called, the remote tree is stopped and the Action
        call marked as preempted.

        :param actionlib.server_goal_handle.ServerGoalHandle goal_handle:

        This has methods to communicate changes in goal status to the
        server (i.e. accepting/rejecting the goal, finishing it or
        accepting cancelling/other preemption)

        """
        if self.run_tree_gh and self.run_tree_gh.get_goal_id() == goal_handle.get_goal_id():
            rospy.logdebug('Received cancel request for current goal, '
                           'stopping and clearing tree')

            self.tree_manager.control_execution(ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.SHUTDOWN))
            self.tree_manager.clear(None)

            with self._lock:
                if self.latest_tree:
                    goal_handle.set_canceled(result=RunTreeResult(final_tree=self.latest_tree))
                else:
                    goal_handle.set_canceled()
            # Remove existing goal handle and empty latest_tree
            self.run_tree_gh = None
            self.latest_tree = None
        else:
            goal_handle.set_canceled()

    def update_tree_msg(self, msg):
        with self._lock:
            self.latest_tree = msg
            for node in self.latest_tree.nodes:
                if node.name == self.latest_tree.root_name:
                    if node.state in [
                            NodeMsg.SUCCEEDED,
                            NodeMsg.FAILED,
                            NodeMsg.SHUTDOWN]:
                        # We got a result, send it back
                        self.run_tree_gh.set_succeeded(result=RunTreeResult(
                            final_tree=self.latest_tree))
                        self.run_tree_gh = None
                        self.latest_tree = None

                        break
