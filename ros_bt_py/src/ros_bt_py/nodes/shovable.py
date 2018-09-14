import jsonpickle

from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
import rospy

from ros_bt_py_msgs.srv import EvaluateUtility, EvaluateUtilityRequest
from ros_bt_py_msgs.msg import RunTreeAction, RunTreeGoal, TreeDataUpdate
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

from ros_bt_py.ros_helpers import AsyncServiceProxy
from ros_bt_py.exceptions import BehaviorTreeException


@define_bt_node(NodeConfig(
    options={
        'utility_evaluator_service': str,
        'remote_tick_frequency_hz': float,
        'wait_for_service_seconds': float,
        'action_timeout_seconds': float},
    inputs={},
    outputs={'running_remotely': bool},
    max_children=1))
class Shovable(Decorator):
    """Marks the subtree below this decorator as remote-executable.

    It will first send the subtree to `utility_evaluator_service' (a
    :class:`ros_bt_py_msgs.srv.EvaluateUtility` server) to see who is
    best suited to execute it.

    Depending on the answer, it will either tick the subtree itself of
    use the action server from the service response to execute it
    remotely.

    The negotiation induces some delay, so expect a few ticks' wait
    before the subtree is actually executed. Consequently, make sure
    that the subtree takes long enough to arrive at a final state
    (`SUCCEEDED` or `FAILED`) that this overhead is justified.
    """
    # States to structure the do_tick() method
    IDLE = 0
    WAIT_FOR_UTILITY_RESPONSE = 1
    ACTION_CLIENT_INIT = 2
    START_REMOTE_EXEC_ACTION = 3
    EXECUTE_LOCAL = 4
    EXECUTE_REMOTE = 5

    def _do_setup(self):
        self._evaluator_client = AsyncServiceProxy(
            self.options['utility_evaluator_service'],
            EvaluateUtility)
        # Throws an exception after timing out
        rospy.wait_for_service(
            self.options['utility_evaluator_service'],
            timeout=self.options['wait_for_service_seconds'])

        self._state = Shovable.IDLE
        self._remote_namespace = ''
        self._subtree_action_start_time = None
        self._subtree_action_client = None
        self._subtree_data_update_publisher = None
        # Save this so we can be sure we're actually executing the same tree we
        # evaluated the utility value for
        self._subtree_msg = None

        for child in self.children:
            child.setup()
        if not self.children:
            msg = 'Trying to set up Shovable decorator without a child!'
            self.logwarn(msg)
            raise BehaviorTreeException(msg)

        _, incoming_conns, outgoing_conns = \
            self.children[0].get_subtree_msg()

        # These should be references to all the children with external inputs.
        #
        # It's fine to cache this list: Our children can only change
        # when the tree is shutdown, so setup() will be called again
        # when they do.
        self._children_with_external_outputs = {
            child_node.name: child_node
            for child_node in self.get_children_recursive()
            if child_node.name in [conn.source.node_name for conn in outgoing_conns]}

        self._external_outputs_by_name = {}
        for conn in outgoing_conns:
            if conn.source.node_name not in self._external_outputs_by_name:
                self._external_outputs_by_name[conn.source.node_name] = []
            self._external_outputs_by_name[conn.source.node_name].append(conn.source.data_key)

    def _do_tick(self):
        # If the child is not currently running, check where best to execute it
        if self._state == Shovable.IDLE:
            # Kick off a service call
            self._subtree_msg, _, _ = \
                self.children[0].get_subtree_msg()

            self._evaluator_client.call_service(
                EvaluateUtilityRequest(tree=self._subtree_msg))

            self._state = Shovable.WAIT_FOR_UTILITY_RESPONSE

        if self._state == Shovable.WAIT_FOR_UTILITY_RESPONSE:
            eval_state = self._evaluator_client.get_state()

            if eval_state == AsyncServiceProxy.RUNNING:
                return NodeMsg.RUNNING

            elif eval_state == AsyncServiceProxy.RESPONSE_READY:
                eval_response = self._evaluator_client.get_response()

                self.outputs['running_remotely'] = not eval_response.local_is_best

                if eval_response.local_is_best:
                    self._state = Shovable.EXECUTE_LOCAL
                else:
                    # No need to re-initialize the ActionClient, skip ahead
                    if eval_response.best_executor_namespace == self._remote_namespace:
                        self._state = Shovable.START_REMOTE_EXEC_ACTION
                    else:
                        self._remote_namespace = eval_response.best_executor_namespace

                        self.logdebug('Building ActionClient for action %s'
                                      % self._remote_namespace + '/run_tree')
                        self._subtree_action_client = SimpleActionClient(
                            self._remote_namespace + '/run_tree', RunTreeAction)

                        self._subtree_data_update_publisher = rospy.Publisher(
                            self._remote_namespace + '/update_data', TreeDataUpdate,
                            queue_size=1)

                        self._state = Shovable.ACTION_CLIENT_INIT

            elif eval_state == AsyncServiceProxy.ERROR:
                self.logerr('Cannot determine best evaluator for subtree')
                self._state = Shovable.IDLE
                return NodeMsg.FAILED

        # Note that this and the following are if's, not elif's!
        # This allows execution to "fall through" in a single tick
        if self._state == Shovable.ACTION_CLIENT_INIT:
            # The timeout can't be 0, because that is interpreted as "infinite"
            if self._subtree_action_client.wait_for_server(
                    timeout=rospy.Duration.from_sec(0.0001)):
                self._state = Shovable.START_REMOTE_EXEC_ACTION
            else:
                self._state = Shovable.ACTION_CLIENT_INIT

        if self._state == Shovable.START_REMOTE_EXEC_ACTION:
            # Ensure the "updated" state of children with outputs is
            # set correctly when executing remotely
            for child_node in self._children_with_external_outputs.itervalues():
                child_node.outputs.reset_updated()

            self.logdebug('Sending goal to action server at %s'
                          % self._remote_namespace + '/run_tree')
            self._subtree_action_start_time = rospy.Time.now()
            self._subtree_action_client.send_goal(
                RunTreeGoal(tree=self._subtree_msg,
                            tick_frequency_hz=self.options['remote_tick_frequency_hz']))
            self._state = Shovable.EXECUTE_REMOTE

        if self._state == Shovable.EXECUTE_LOCAL:
            child_result = self.children[0].tick()

            return child_result

        if self._state == Shovable.EXECUTE_REMOTE:
            seconds_since_goal = (rospy.Time.now() - self._subtree_action_start_time).to_sec()
            if seconds_since_goal > self.options['action_timeout_seconds']:
                self.logerr('Remote subtree execution timed out')
                self._subtree_action_client.cancel_goal()
                self.cleanup()
                self._state = Shovable.IDLE
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
                                self._children_with_external_outputs[node.name]\
                                    .outputs[output.key] = jsonpickle.decode(
                                        output.serialized_value)
                        self._children_with_external_outputs[node.name]._handle_outputs()
                    if node.name == final_tree.root_name:
                        final_state = node.state

                self.cleanup()
                return final_state
            elif action_state in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                return NodeMsg.RUNNING
            else:
                self.cleanup()
                return NodeMsg.FAILED
        else:
            return NodeMsg.RUNNING

    def _do_untick(self):
        # TODO(nberg): Pause remote subtree execution

        return NodeMsg.IDLE

    def _do_reset(self):
        if self._state == Shovable.EXECUTE_LOCAL:
            for child in self.children:
                child.reset()
        elif self._subtree_action_client is not None:
            self._subtree_action_client.cancel_goal()
        self.cleanup()

        return NodeMsg.IDLE

    def cleanup(self):
        self._remote_namespace = ''
        self._subtree_action_client = None
        if self._subtree_data_update_publisher is not None:
            self._subtree_data_update_publisher.unregister()
            self._subtree_data_update_publisher = None
        self._subtree_msg = None
        self._state = Shovable.IDLE
        self._children_with_external_outputs = {}
        self._external_outputs_by_name = {}
