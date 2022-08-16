"""
Module used to implement mission control components.
This includes the management of capabilities and their implementations as well as the forwarding to
other nodes by using an auction protocol.
"""
# pylint: disable=no-name-in-module,import-error
import threading
from threading import RLock
from typing import Dict, List, Tuple, Optional

import actionlib
import rospy
from actionlib import SimpleActionServer, SimpleActionClient, ActionServer
from actionlib_msgs.msg import GoalStatus
from rospy import ROSException
from rosservice import rosservice_find
from ros_bt_py_msgs.msg import (
    CapabilityImplementation, GetLocalBidGoal, GetLocalBidAction, GetLocalBidResult,
    ExecuteCapabilityGoal, ExecuteCapabilityResult, ExecuteCapabilityAction, FindBestCapabilityExecutorAction,
    FindBestCapabilityExecutorGoal, FindBestCapabilityExecutorResult, Precondition, ExecuteRemoteCapabilityAction,
    ExecuteRemoteCapabilityGoal, ExecuteRemoteCapabilityActionGoal, ExecuteCapabilityImplementationGoal,
    RemoteCapabilitySlotStatus, ExecuteRemoteCapabilityActionResult, ExecuteRemoteCapabilityResult,
    ExecuteCapabilityImplementationAction, ExecuteCapabilityImplementationResult,
)
from ros_bt_py_msgs.srv import (
    GetCapabilityImplementationsRequest, GetCapabilityImplementationsResponse, GetCapabilityInterfaces,
    GetCapabilityImplementations, ControlTreeExecutionRequest, LoadTreeRequest, PrepareLocalImplementationRequest,
    PrepareLocalImplementationResponse,
    NotifyCapabilityExecutionStatusRequest, NotifyCapabilityExecutionStatusResponse, LoadTreeResponse, AddNodeRequest,
    AddNodeResponse, MoveNodeRequest, AddNodeAtIndexRequest,
    CheckPreconditionStatusRequest, CheckPreconditionStatusResponse, CheckPreconditionStatus, ClearTreeRequest,
    MoveNodeResponse, AddNodeAtIndexResponse, PrepareLocalImplementation,
)

import ros_bt_py.nodes.sequence
from ros_bt_py.capability import (
    capability_node_class_from_capability_interface,
)
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.helpers import HashableCapabilityInterface
from ros_bt_py.migration import MigrationManager
from ros_bt_py.tree_manager import TreeManager


class RemoteCapabilitySlotTask:

    def __init__(
            self, remote_capability_slot_name,
            goal: ExecuteCapabilityImplementationGoal,
            publish_feedback_cb,
            done_cb,
            aborted_cb
            ):
        self.remote_capability_slot_name = remote_capability_slot_name

        self.remote_tree_slot_action_client = SimpleActionClient(
            f"~/remote_tree_slot/{self.remote_capability_slot_name}",
            ExecuteCapabilityAction
        )

        self.lock = RLock()
        self.cancel_execution_flag = False

        self.goal = goal
        self.publish_feedback_cb = publish_feedback_cb
        self.done_cb = done_cb
        self.aborted_cb = aborted_cb

    def exec(self):

        start_time = rospy.Time.now()
        while not self.remote_tree_slot_action_client.wait_for_server(timeout=rospy.Duration(nsecs=500)):
            if rospy.Time.now() - start_time > rospy.Duration.from_sec(5) or self.cancel_execution_flag:
                self.aborted_cb()

        self.remote_tree_slot_action_client.send_goal(
            goal=self.goal,
            feedback_cb=self.publish_feedback_cb
        )

    def cancel_execution(self):
        with self.lock:
            self.cancel_execution_flag = True


class MissionControl:
    """
    Class to manage the capability implementations and interfaces on the local node.
    Additionally, it allows to exchange interface definitions with remote nodes.
    """

    # pylint: disable=too-many-instance-attributes

    def nop(self, msg):
        """
        No operation method
        :return: None
        """

    def __init__(
            self,
            auction_manager: Type[AssignmentManager],
            local_capability_topic_prefix: str,
            global_capability_topic_prefix: str
    ):
        """
        Creates a new capability repository.
        The repository has to major interfaces, it communicates with other repositories and
        offers services geared towards its local subscribers.

        :param local_capability_topic_prefix: Prefix used for local communication.
        This topic only relays information relevant for the local robot, thus foreign implementations,
        etc. are not used.

        :param global_capability_topic_prefix: Prefix used for communication with other ros_bt_py nodes.
        This is used to sync available interfaces across all nodes.
        """

        self.local_capability_topic_prefix = local_capability_topic_prefix
        self.global_capability_topic_prefix = global_capability_topic_prefix

        self.executing_capabilities: Dict[HashableCapabilityInterface, Dict[str, int]] = {}

        self.staging_tree_manager = TreeManager(
            name="StagingManager",
            publish_tree_callback=self.nop,
            publish_debug_info_callback=self.nop,
            publish_debug_settings_callback=self.nop,
            publish_node_diagnostics_callback=self.nop,
            debug_manager=DebugManager()
        )
        """Tree manager used to stage trees for local deployment in a capability."""



        self.staging_migration_manager = MigrationManager(tree_manager=self.staging_tree_manager)
        """Migration manager used for preparing the trees in the staging tree manager"""

        self.__get_local_bid_action_server: SimpleActionServer = SimpleActionServer(
            "~get_local_bid",
            GetLocalBidAction,
            execute_cb=self.get_local_bid_cb
        )
        self.__get_local_bid_action_server.start()

        self.__get_local_bid_action_client = SimpleActionClient(
            "~get_local_bid",
            GetLocalBidAction
        )

        self.__find_best_capability_executor_action_client = SimpleActionClient(
            "~assignment_system/find_best_capability_executor",
            FindBestCapabilityExecutorAction
        )

        self.__execute_capability_action_server: SimpleActionServer = SimpleActionServer(
            "~execute_capability",
            ExecuteCapabilityAction,
            execute_cb=self.execute_capability_cb
        )
        self.__execute_capability_action_server.start()

        self.__get_capability_interfaces_proxy = rospy.ServiceProxy(
            "~capabilities/interface/get",
            GetCapabilityInterfaces
        )

        self.__get_capability_implementations_proxy = rospy.ServiceProxy(
            "/capability_repository_node/capabilities/implementations/get",
            GetCapabilityImplementations
        )

        self.__check_precondition_status_service = rospy.Service(
            "~check_precondition_status",
            CheckPreconditionStatus,
            handler=self.check_precondition_status
        )

        self.__execute_remote_capability_action_server = ActionServer(
            "~execute_remote_capability",
            ExecuteRemoteCapabilityAction,
            goal_cb=self.execute_remote_capability_goal_cb,
            cancel_cb=self.execute_remote_capability_cancel_cb
        )

        self.__prepare_local_implementation_service = rospy.Service(
            "~prepare_local_implementation",
            PrepareLocalImplementation,
            self.prepare_local_capability_implementation
        )

        self._local_remote_slots_status_subscriber = rospy.Subscriber(
            "~remote_slot_status",
            RemoteCapabilitySlotStatus,
            callback=self.remote_capability_slot_status_callback
        )

        self._remote_capability_slot_lock = RLock()

        self._remote_capability_slot_status: Dict[str, str] = {}
        """
        Status of all known remote capability slots.
        It stores the name of the executor together with the current status.
        """

        self._remote_capability_slot_goals: Dict[str, Tuple[str, bool]] = {}
        """
        Dict associating goal ids with the related remote capability slot and the boolean variable to signal
        cancellation.
        """

    def remote_capability_slot_status_callback(self, msg: RemoteCapabilitySlotStatus):
        rospy.loginfo(f"Updating status of remote capability slot: {msg.name} to {msg.status}")
        with self._remote_capability_slot_lock:
            self._remote_capability_slot_status[msg.name] = msg.status

    def execute_remote_capability_exec(self, remote_slot_name: str, goal_handle: ExecuteRemoteCapabilityActionGoal):
        rospy.loginfo(f"Starting execution of {goal_handle.goal_id} on {remote_slot_name}")

        response: PrepareLocalImplementationResponse = self.prepare_local_capability_implementation(
            PrepareLocalImplementationRequest(
                interface=goal_handle.goal.interface,
                implementation_name=goal_handle.goal.implementation_name
            )
        )

        if not response.success:
            rospy.logwarn(f"Could not prepare local implementation for {goal_handle.goal_id}: {response.error_message}")
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.goal_id,
                    status=GoalStatus.SUCCEEDED
                ),
                result=ExecuteRemoteCapabilityResult(
                    success=False,
                    error_message=response.error_message
                )
            )
            return

        remote_slot_action_client = actionlib.SimpleActionClient(
            f"~remote_capability_slots/{remote_slot_name}",
            ExecuteCapabilityImplementationAction
        )
        if not remote_slot_action_client.wait_for_server(timeout=rospy.Duration(secs=5)):
            rospy.logwarn("Could not contact remote_capability slot before timeout")
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.goal_id,
                    status=GoalStatus.SUCCEEDED
                ),
                result=ExecuteRemoteCapabilityResult(
                    success=False,
                    error_message="Could not contact remote_capability slot before timeout"
                )
            )
            return

        remote_slot_action_client.send_goal(ExecuteCapabilityImplementationGoal(
            implementation=response.implementation_subtree,
            interface=goal_handle.goal.interface,
            input_topic=goal_handle.goal.input_topic,
            output_topic=goal_handle.goal.output_topic
        ))

        while not remote_slot_action_client.wait_for_result(timeout=rospy.Duration(nsecs=10)):
            if self._remote_capability_slot_goals[goal_handle.goal_id][1]:
                rospy.logerr(f"Cancellation of goal {goal_handle.goal_id} requested!")
                remote_slot_action_client.cancel_goal()
                return

        result: Optional[ExecuteCapabilityImplementationResult] = remote_slot_action_client.get_result()

        self.__execute_remote_capability_action_server.publish_result(
            status=GoalStatus(
                goal_id=goal_handle.goal_id,
                status=GoalStatus.SUCCEEDED
            ),
            result=ExecuteRemoteCapabilityResult(
                success=result.success,
                error_message=result.error_message
            )
        )

        # TODO: Start action on remote slot!

    def execute_remote_capability_goal_cb(self, goal_handle: ExecuteRemoteCapabilityActionGoal) -> None:
        """
        Execute remote capability action goal callback.
        This callback manages the arrival of a new goal to the action server.

        It will look for a new remote capability slot where the implementation in the goal can be executed.
        If a executor is found, a new thread will be started that processes the goal.

        :param goal_handle: The goal handle that contains all the necessary information for the new request.
        :return: None
        """
        with self._remote_capability_slot_lock:
            idle_executors: List[Tuple[str, str]] = list(
                filter(lambda x: x[1] == RemoteCapabilitySlotStatus.IDLE,
                       self._remote_capability_slot_status.items())
            )

        if len(idle_executors) < 1:
            rospy.logerr(f"No idle remote_capability_slot found, cannot execute remote capability.")
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.goal_id,
                    status=GoalStatus.SUCCEEDED
                ),
                result=ExecuteRemoteCapabilityResult(
                    success=False,
                    error_message="No remote capability slot executor available!"
                )
            )
            return

        selected_executor = idle_executors.pop()
        self.remote_capability_slot_status_callback(RemoteCapabilitySlotStatus(
            name=selected_executor[0],
            status=RemoteCapabilitySlotStatus.RUNNING,
            timestamp=rospy.Time.now()
        ))

        self._remote_capability_slot_goals[goal_handle.goal_id] = (selected_executor[0], False)

        thread = threading.Thread(
            name=f"Thread-RemoteCapabilitySlot-{selected_executor[0]}",
            target=self.execute_remote_capability_exec,
            args=(selected_executor[0], goal_handle)
        )
        thread.start()
        rospy.loginfo("Started thread for execution of remote capability!")

    def execute_remote_capability_cancel_cb(self, goal_handle: ExecuteRemoteCapabilityActionGoal) -> None:
        """
        ExecuteRemoteCapability cancel callback.
        This callback manages the request for the cancellation of a running goal.
        The method will send a failed status back if the goal id is not associated with a running goal.

        :param goal_handle: The goal handle containing all the information about the goal to cancel.
        :return: None
        """
        if goal_handle.goal_id not in self._remote_capability_slot_goals:
            rospy.logwarn("Requested cancellation of unknown remote capability goal!")
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.goal_id,
                    status=GoalStatus.ABORTED,
                )
            )
            return
        current_status = self._remote_capability_slot_goals[goal_handle.goal_id]
        self._remote_capability_slot_goals[goal_handle.goal_id] = (current_status[0], True)
        return

    def notify_capability_status_change(
            self,
            request: NotifyCapabilityExecutionStatusRequest
    ) -> NotifyCapabilityExecutionStatusResponse:
        """
        ROS service handler allowing to notify the MissionControl node about changes in the
        capability execution states.
        :param request:
        :return:
        """
        response = NotifyCapabilityExecutionStatusResponse()
        hashable_interface = HashableCapabilityInterface(request.interface)
        if not self.executing_capabilities[hashable_interface]:
            self.executing_capabilities[hashable_interface] = {}

        self.executing_capabilities[hashable_interface][request.node_name] = request.status
        response.success = True
        return response

    def get_local_bid_cb(self, goal: GetLocalBidGoal) -> None:
        """
        ROS actionlib callback that calculates the bid for the local node to perform a certain action.
        This method receives all the local capability implementations, and loads them into the
        staging tree manager.
        Then the local bid is calculated for each of the implementations using the calculate_utility function.

        :param goal: The actionlib goal containing the information about the required interface for which the bids
        should be calculated
        :return: None
        """
        result = GetLocalBidResult()

        try:
            self.__get_capability_implementations_proxy.wait_for_service(timeout=rospy.Duration.from_sec(1.0))
        except ROSException:
            error_msg = f"Failed to find local implementation service"
            rospy.logwarn(error_msg)
            self.__get_local_bid_action_server.set_aborted(text=error_msg)
            return

        response: GetCapabilityImplementationsResponse = self.__get_capability_implementations_proxy.call(
            GetCapabilityImplementationsRequest(
                interface=goal.interface
            )
        )

        if not response.success:
            error_msg = f"Failed to get local implementations: {response.error_message}"
            rospy.logwarn(error_msg)
            self.__get_local_bid_action_server.set_aborted(
                text=error_msg
            )
            return

        if len(response.implementations) < 1:
            result.executable = False
            self.__get_local_bid_action_server.set_succeeded(result=result)
            return

        implementation_utility: Dict[str, float] = {}
        for implementation in response.implementations:
            if self.__get_local_bid_action_server.is_preempt_requested():
                rospy.loginfo("GetLocalBid service is preempted")
                self.__get_local_bid_action_server.set_preempted()
                return

            self.staging_tree_manager.control_execution(
                ControlTreeExecutionRequest(
                    command=ControlTreeExecutionRequest.SHUTDOWN
                )
            )
            self.staging_tree_manager.clear(ClearTreeRequest())

            res = self.staging_tree_manager.load_tree(LoadTreeRequest(tree=implementation.tree))
            if not res.success:
                rospy.logerr(res.error_message)
                continue

            calculated_utility = \
                self.staging_tree_manager.find_root().calculate_utility()

            success_upper_lower_difference = 0
            failure_upper_lower_difference = 0
            if calculated_utility.has_upper_bound_success and calculated_utility.has_lower_bound_success:
                success_upper_lower_difference = \
                    calculated_utility.upper_bound_success - calculated_utility.lower_bound_success
            if calculated_utility.has_upper_bound_failure and calculated_utility.has_lower_bound_failure:
                failure_upper_lower_difference = \
                    calculated_utility.upper_bound_failure - calculated_utility.lower_bound_failure
            implementation_utility[
                implementation.name] = success_upper_lower_difference - failure_upper_lower_difference

        if len(implementation_utility) < 1:
            rospy.logwarn("No utilities could be calculated for any available implementation.")
            self.__get_local_bid_action_server.set_aborted("Could not calculate the local bid.")
            return

        sorted_implementation_bids: List[Tuple[str, float]] = sorted(
            [(n, v) for n, v in implementation_utility.items()], key=lambda v: v[1]
        )

        (best_implementation_name, best_implementation_bid) = sorted_implementation_bids[0]
        result.bid = best_implementation_bid
        result.implementation_name = best_implementation_name

        result.executable = True
        self.__get_local_bid_action_server.set_succeeded(result=result)

    def execute_capability_cb(self, goal: ExecuteCapabilityGoal) -> None:
        result = ExecuteCapabilityResult()
        rospy.loginfo("Received request to execute capability!")

        if self.__find_best_capability_executor_action_client.wait_for_server(
                timeout=rospy.Duration(secs=5)
        ):
            self.__find_best_capability_executor_action_client.send_goal(
                goal=FindBestCapabilityExecutorGoal(capability=goal.capability)
            )
            while not self.__find_best_capability_executor_action_client.wait_for_result(
                    timeout=rospy.Duration(secs=1)
            ):
                if self.__execute_capability_action_server.is_preempt_requested():
                    self.__find_best_capability_executor_action_client.cancel_goal()
                    self.__execute_capability_action_server.set_preempted()

            find_best_executor_result: FindBestCapabilityExecutorResult = \
                self.__find_best_capability_executor_action_client.get_result()

            if find_best_executor_result.success:
                rospy.loginfo("Found a suitable implementation!")
                result.success = True
                result.implementation_name = find_best_executor_result.implementation_name
                result.execute_local = find_best_executor_result.execute_local
                result.remote_mission_controller_topic = find_best_executor_result.executor_mission_control_topic
                self.__execute_capability_action_server.set_succeeded(result=result)
                return

        rospy.loginfo("Assignment  system is currently offline. We will use the local implementation!")

        self.__get_local_bid_action_client.send_goal(
            goal=GetLocalBidGoal(
                interface=goal.capability
            )
        )

        while not self.__get_local_bid_action_client.wait_for_result(timeout=rospy.Duration(secs=1)):
            if self.__execute_capability_action_server.is_preempt_requested():
                self.__get_local_bid_action_client.cancel_goal()
                self.__execute_capability_action_server.set_preempted()
                return

        local_bid_result: GetLocalBidResult = self.__get_local_bid_action_client.get_result()
        rospy.logwarn("Got local bid result")

        if local_bid_result.executable:
            result.implementation_name = local_bid_result.implementation_name
            result.success = True
            result.execute_local = True
        else:
            result.success = False
            result.error_message = "Could not find a suitable implementation!"

        self.__execute_capability_action_server.set_succeeded(result=result)
        return

    def check_precondition_status(
            self,
            request: CheckPreconditionStatusRequest
    ) -> CheckPreconditionStatusResponse:
        """
        Service to check if the tree tied to this mission controller fulfills the precondition.

        A precondition counts as fulfilled if there is a local capability node that is running the required
        capability.
        This includes both capabilities and remote capability slots.

        :param request: Request containing the precondition that should be checked.
        :return: True if fulfilled or False if not.
        """
        # TODO: Include the remote capability nodes.
        response = CheckPreconditionStatusResponse()
        hashable_capability = HashableCapabilityInterface(request.interface)

        response.available = hashable_capability in self.executing_capabilities and any(
            s in [
                NotifyCapabilityExecutionStatusRequest.IDLE,
                NotifyCapabilityExecutionStatusRequest.EXECUTING,
                NotifyCapabilityExecutionStatusRequest.SUCCEEDED
            ] for n, s in self.executing_capabilities[hashable_capability].items()
        )
        return response

    def prepare_local_capability_implementation(
            self,
            request: PrepareLocalImplementationRequest
    ) -> PrepareLocalImplementationResponse:
        """
        Service that prepares a local capability implementation for execution.

        This includes receiving it from the local capability storage, check and fulfill preconditions and then
        prepare it to be send to the executor.

        The service will return unsuccessfully if the implementation cannot be found, the preconditions can not be
        fulfilled or the implementation is not a valid tree.
        :param request: The request containing all the information about the implementation.
        :return: A response containing the result status, as well as the potential resulting tree.
        """
        response = PrepareLocalImplementationResponse()

        self.staging_tree_manager.control_execution(
            ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.SHUTDOWN
            )
        )
        self.staging_tree_manager.clear(ClearTreeRequest())

        implementations_response: GetCapabilityImplementationsResponse = \
            self.__get_capability_implementations_proxy.call(
                GetCapabilityImplementationsRequest(
                    interface=request.interface
                )
            )
        implementation: Optional[CapabilityImplementation] = next(
            filter(lambda x: x.name == request.implementation_name, implementations_response.implementations), None
        )

        if implementation is None:
            response.success = False
            response.error_message = "Could not get local implementation!"
            return response
        service_response: LoadTreeResponse = self.staging_tree_manager.load_tree(
            LoadTreeRequest(
                tree=implementation.tree
            )
        )
        if not service_response.success:
            response.success = False
            response.error_message = f"Failed to load tree: {service_response.error_message}"
            rospy.logerr(response.error_message)
            return response

        root_node = self.staging_tree_manager.find_root()
        root_sequence_node = ros_bt_py.nodes.sequence.Sequence(name="RootCapabilitySequence")
        service_response: AddNodeResponse = self.staging_tree_manager.add_node(
            AddNodeRequest(
                parent_name=None,
                node=root_sequence_node.to_msg(),
                allow_rename=False
            )
        )

        if not service_response.success:
            response.success = False
            response.error_message = f"Failed to add sequence for preconditions: {service_response.error_message}"
            rospy.logerr(response.error_message)
            return response

        service_response: MoveNodeResponse = self.staging_tree_manager.move_node(
            MoveNodeRequest(
                node_name=root_node.name,
                new_parent_name=root_sequence_node.name,
                new_child_index=0
            )
        )

        if not service_response.success:
            response.success = False
            response.error_message = f"Failed to move implementation under sequence node:" \
                                     f"{service_response.error_message}"
            rospy.logerr(response.error_message)
            return response

        for precondition in reversed(implementation.preconditions):

            if precondition.kind == Precondition.REMOTE:
                available_services = rosservice_find('CheckRemotePrecondition')

                for service_name in available_services:
                    service_client = rospy.ServiceProxy(
                        service_name,
                        CheckPreconditionStatus
                    )
                    service_client.wait_for_service(1.0)
                    service_response: CheckPreconditionStatusResponse = service_client(
                        CheckPreconditionStatusRequest(
                            interface=precondition.capability
                        )
                    )
                    if service_response.available:
                        rospy.loginfo(f"Remote precondition: {precondition} is fulfilled")
                        continue

            if not self.check_precondition_status(CheckPreconditionStatusRequest(interface=precondition.capability)):
                # Add a hashable capability to the tree
                node = capability_node_class_from_capability_interface(
                    capability_interface=precondition.capability,
                    mission_control_topic=rospy.resolve_name("~")
                )
                service_response: AddNodeAtIndexResponse = self.staging_tree_manager.add_node_at_index(
                    AddNodeAtIndexRequest(
                        parent_name="RootCapabilitySequence",
                        node=node,
                        allow_rename=True,
                        new_child_index=0
                    )
                )

                if not service_response.success:
                    response.success = False
                    response.error_message = f"Failed to add needed precondition: {service_response.error_message}"
                    rospy.logerr(response.error_message)
                    return response

            rospy.loginfo(f"Precondition: {precondition} is fulfilled")

        response.success = True
        response.implementation_subtree = self.staging_tree_manager.tree_msg
        return response
