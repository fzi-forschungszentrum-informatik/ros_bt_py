from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
import rospy

from ros_bt_py_msgs.srv import EvaluateUtility, EvaluateUtilityRequest
from ros_bt_py_msgs.msg import RunTreeAction, RunTreeGoal, TreeDataUpdate
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

from ros_bt_py.ros_helpers import AsyncServiceProxy


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

    def _do_tick(self):
        # If the child is not currently running, check where best to execute it
        if self._state == Shovable.IDLE:
            # Kick off a service call
            self._subtree_msg, _ = self.children[0].get_subtree_msg()

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
                        self._state = START_REMOTE_EXEC_ACTION
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
                self._state = IDLE
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
            # Forward subtree inputs/outputs - not needed if the subtree is
            # finished already, but it doesn't hurt either

            action_state = self._subtree_action_client.get_state()
            if action_state == GoalStatus.SUCCEEDED:
                self.cleanup()
                return NodeMsg.SUCCEEDED
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
            for child in children:
                child.reset()
        elif self._subtree_action_client is not None:
            self._subtree_action_client.cancel_goal()
        self.cleanup()

    def cleanup(self):
        self._remote_namespace = ''
        self._subtree_action_client = None
        self._subtree_data_update_publisher.unregister()
        self._subtree_data_update_publisher = None
        self._subtree_msg = None
        self._state = Shovable.IDLE
