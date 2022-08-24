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
from actionlib import ActionServer, ServerGoalHandle
from actionlib_msgs.msg import GoalStatus
from rospy import ROSException, Service, ServiceProxy
from rosservice import rosservice_find
from ros_bt_py_msgs.msg import (
    CapabilityImplementation, Precondition, ExecuteRemoteCapabilityAction,
    ExecuteRemoteCapabilityActionGoal, ExecuteCapabilityImplementationGoal,
    RemoteCapabilitySlotStatus, ExecuteRemoteCapabilityResult,
    ExecuteCapabilityImplementationAction, ExecuteCapabilityImplementationResult,
    ExecuteCapabilityImplementationActionResult, ExecuteRemoteCapabilityGoal, ExecuteRemoteCapabilityActionResult,
)
from ros_bt_py_msgs.srv import (
    GetCapabilityImplementationsRequest, GetCapabilityImplementationsResponse,
    GetCapabilityImplementations, ControlTreeExecutionRequest, LoadTreeRequest, PrepareLocalImplementationRequest,
    PrepareLocalImplementationResponse,
    NotifyCapabilityExecutionStatusRequest, NotifyCapabilityExecutionStatusResponse, LoadTreeResponse, AddNodeRequest,
    AddNodeResponse, MoveNodeRequest, AddNodeAtIndexRequest,
    CheckPreconditionStatusRequest, CheckPreconditionStatusResponse, CheckPreconditionStatus, ClearTreeRequest,
    MoveNodeResponse, AddNodeAtIndexResponse, PrepareLocalImplementation, RequestCapabilityExecutionRequest,
    RequestCapabilityExecutionResponse, GetLocalBidResponse, GetLocalBidRequest, GetLocalBid,
    FindBestCapabilityExecutor, FindBestCapabilityExecutorRequest, MigrateTreeRequest, RequestCapabilityExecution,
    NotifyCapabilityExecutionStatus,
)

import ros_bt_py.nodes.sequence
from ros_bt_py.capability import (
    capability_node_class_from_capability_interface,
)
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.helpers import HashableCapabilityInterface
from ros_bt_py.migration import MigrationManager, check_node_versions
from ros_bt_py.ros_helpers import AsyncServiceProxy
from ros_bt_py.tree_manager import TreeManager


class MissionControl:
    """
    Class to manage the capability implementations and interfaces on the local node.
    Additionally, it allows to exchange interface definitions with remote nodes.
    """

    # pylint: disable=too-many-instance-attributes

    def __init__(
            self
    ):
        """
        Creates a new capability repository.
        The repository has to major interfaces, it communicates with other repositories and
        offers services geared towards its local subscribers.
        """
        self.executing_capabilities: Dict[HashableCapabilityInterface, Dict[str, int]] = {}
        self.capability_repository_topic = f"{rospy.get_namespace()}/capability_repository"

        self.__request_capability_execution_service: Service = Service(
            "~execute_capability",
            RequestCapabilityExecution,
            self.request_capability_execution
        )

        self.__get_local_bid_service: Service = Service(
            "~get_local_bid",
            GetLocalBid,
            self.get_local_bid
        )

        self.__get_local_bid_service_client = AsyncServiceProxy(
            "~get_local_bid",
            GetLocalBid,
        )

        self.__check_precondition_status_service = rospy.Service(
            "~check_precondition_status",
            CheckPreconditionStatus,
            handler=self.check_precondition_status
        )

        self._notify_capability_execution_status_service = rospy.Service(
            "~notify_capability_execution_status",
            NotifyCapabilityExecutionStatus,
            self.notify_capability_execution_status
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

    def execute_remote_capability_exec(self, remote_slot_name: str, goal_handle: ServerGoalHandle):
        goal_id = goal_handle.get_goal_id().id

        rospy.logfatal(f"Starting execution of {goal_id} on {remote_slot_name}" , logger_name="mission_control_remote_exec")
        goal: ExecuteRemoteCapabilityGoal = goal_handle.get_goal()

        prepare_local_capability_implementation_service_proxy = ServiceProxy(
            "~prepare_local_implementation",
            PrepareLocalImplementation
        )

        response: PrepareLocalImplementationResponse = prepare_local_capability_implementation_service_proxy.call(
            PrepareLocalImplementationRequest(
                interface=goal.interface,
                implementation_name=goal.implementation_name
            )
        )
        rospy.logfatal("service call finished")

        if not response.success:
            rospy.logwarn(f"Could not prepare local implementation for {goal_id}: {response.error_message}", logger_name="mission_control_remote_exec")
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.get_goal_id(),
                    status=GoalStatus.SUCCEEDED
                ),
                result=ExecuteRemoteCapabilityResult(
                    success=False,
                    error_message=response.error_message
                )
            )
            return

        rospy.logfatal("Resolving executor server!", logger_name="mission_control_remote_exec")

        remote_tree_slot_executor_topic = rospy.resolve_name(
            f"~remote_capability_slot/{remote_slot_name}"
        )

        remote_slot_action_client = actionlib.SimpleActionClient(
            remote_tree_slot_executor_topic,
            ExecuteCapabilityImplementationAction
        )
        
        if not remote_slot_action_client.wait_for_server(timeout=rospy.Duration(secs=5)):
            rospy.logwarn("Could not contact remote_capability slot before timeout", logger_name="mission_control_remote_exec")
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.get_goal_id(),
                    status=GoalStatus.SUCCEEDED
                ),
                result=ExecuteRemoteCapabilityResult(
                    success=False,
                    error_message="Could not contact remote_capability slot before timeout"
                )
            )

            return

        rospy.logfatal("Send goal!", logger_name="mission_control_remote_exec")
        remote_slot_action_client.cancel_all_goals()

        remote_slot_action_client.send_goal(
            ExecuteCapabilityImplementationGoal(
                implementation_tree=response.implementation_subtree,
                interface=goal.interface,
                input_topic=goal.input_topic,
                output_topic=goal.output_topic
            )
        )

        while not remote_slot_action_client.wait_for_result(timeout=rospy.Duration(nsecs=10)):
            if self._remote_capability_slot_goals[goal_id][1]:
                rospy.logerr(f"Cancellation of goal {goal_id} requested!", logger_name="mission_control_remote_exec")
                remote_slot_action_client.cancel_goal()
                return

        result: Optional[ExecuteCapabilityImplementationResult] = remote_slot_action_client.get_result()

        self.__execute_remote_capability_action_server.publish_result(
            status=GoalStatus(
                goal_id=goal_handle.get_goal_id(),
                status=GoalStatus.SUCCEEDED
            ),
            result=ExecuteRemoteCapabilityResult(
                success=result.success,
                error_message=result.error_message
            )
        )

    def execute_remote_capability_goal_cb(self, goal_handle: ExecuteRemoteCapabilityActionGoal) -> None:
        """
        Execute remote capability action goal callback.
        This callback manages the arrival of a new goal to the action server.

        It will look for a new remote capability slot where the implementation in the goal can be executed.
        If an executor is found, a new thread will be started that processes the goal.

        :param goal_handle: The goal handle that contains all the necessary information for the new request.
        :return: None
        """
        goal_id = goal_handle.get_goal_id().id
        with self._remote_capability_slot_lock:
            idle_executor_ids: List[str] = list(
                filter(
                    lambda x: self._remote_capability_slot_status[x] == RemoteCapabilitySlotStatus.IDLE,
                    self._remote_capability_slot_status
                )
            )

        if len(idle_executor_ids) < 1:
            rospy.logerr(f"No idle remote_capability_slot found, cannot execute remote capability.", logger_name="mission_control_remote_exec")
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.get_goal_id(),
                    status=GoalStatus.SUCCEEDED
                ),
                result=ExecuteRemoteCapabilityResult(
                    success=False,
                    error_message="No remote capability slot executor available!"
                )
            )
            return

        selected_executor_id = idle_executor_ids.pop()
        rospy.logfatal(f"Selected executor: {selected_executor_id}", logger_name="mission_control_remote_exec")
        self.remote_capability_slot_status_callback(
            RemoteCapabilitySlotStatus(
                name=selected_executor_id,
                status=RemoteCapabilitySlotStatus.RUNNING,
                timestamp=rospy.Time.now()
            )
        )

        self._remote_capability_slot_goals[goal_id] = (selected_executor_id, False)
        rospy.logfatal("Starting thread!", logger_name="mission_control_remote_exec")
        thread = threading.Thread(
            name=f"Thread-RemoteCapabilitySlot-{selected_executor_id}",
            target=self.execute_remote_capability_exec,
            args=(selected_executor_id, goal_handle)
        )
        thread.start()
        rospy.loginfo("Started thread for execution of remote capability!", logger_name="mission_control_remote_exec")

    def execute_remote_capability_cancel_cb(self, goal_handle: ServerGoalHandle) -> None:
        """
        ExecuteRemoteCapability cancel callback.
        This callback manages the request for the cancellation of a running goal.
        The method will send a failed status back if the goal id is not associated with a running goal.

        :param goal_handle: The goal handle containing all the information about the goal to cancel.
        :return: None
        """
        goal_id = goal_handle.get_goal_id().id
        if goal_id not in self._remote_capability_slot_goals:
            rospy.logwarn("Requested cancellation of unknown remote capability goal!")
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.get_goal_id(),
                    status=GoalStatus.ABORTED,
                ),
                result=ExecuteCapabilityImplementationActionResult()
            )
            return
        current_status = self._remote_capability_slot_goals[goal_id]
        self._remote_capability_slot_goals[goal_id] = (current_status[0], True)
        return

    def notify_capability_execution_status(
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

        if hashable_interface not in self.executing_capabilities:
            self.executing_capabilities[hashable_interface] = {}

        self.executing_capabilities[hashable_interface][request.node_name] = request.status

        rospy.loginfo(f"Set status of {request.node_name} to {request.status}")

        response.success = True
        return response

    def request_capability_execution(
            self,
            goal: RequestCapabilityExecutionRequest
    ) -> RequestCapabilityExecutionResponse:
        response = RequestCapabilityExecutionResponse()
        rospy.loginfo("Received request to execute capability!")

        assignment_system_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/assignment_manager/find_best_executor"
        )
        find_best_executor_service = ServiceProxy(
            assignment_system_topic,
            FindBestCapabilityExecutor
        )

        try:
            find_best_executor_service.wait_for_service(
                timeout=rospy.Duration(secs=5)
            )
            find_best_executor_response = find_best_executor_service.call(
                FindBestCapabilityExecutorRequest(
                    capability=goal.capability,
                    mission_control_name=rospy.get_node_uri()
                )
            )

            if find_best_executor_response.success:
                rospy.logfatal("Found a suitable implementation!")
                response.success = True
                response.implementation_name = find_best_executor_response.implementation_name
                response.execute_local = find_best_executor_response.execute_local
                response.remote_mission_controller_topic = find_best_executor_response.executor_mission_control_topic
                return response

        except ROSException:
            rospy.logwarn("Assignment system could not be contacted before timeout!")

        rospy.loginfo("Assignment  system is currently offline. We will use the local implementation!")

        local_bid_response = self.get_local_bid(
            GetLocalBidRequest(
                interface=goal.capability
            )
        )

        if local_bid_response.success:
            response.implementation_name = local_bid_response.implementation_name
            response.success = True
            response.execute_local = True
        else:
            response.success = False
            response.error_message = "Could not find a suitable implementation!"

        return response

    @staticmethod
    def get_local_bid(request: GetLocalBidRequest) -> GetLocalBidResponse:
        """
        ROS service callback that calculates the bid for the local node to perform a certain action.
        This method receives all the local capability implementations, and loads them into the
        staging tree manager.
        Then the local bid is calculated for each of the implementations using the calculate_utility function.

        :param request: The actionlib goal containing the information about the required interface for which the bids
        should be calculated
        :return: None
        """
        service_response = GetLocalBidResponse()

        capability_repository_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/capability_repository/capabilities/implementations/get"
        )

        get_capability_implementations_proxy = rospy.ServiceProxy(
            capability_repository_topic,
            GetCapabilityImplementations
        )

        try:
            get_capability_implementations_proxy.wait_for_service(timeout=rospy.Duration.from_sec(1.0))
        except ROSException:
            service_response.error_message = \
                f"Failed to find local implementation service at {capability_repository_topic}"
            rospy.logwarn(service_response.error_message)
            service_response.success = False

            return service_response

        response: GetCapabilityImplementationsResponse = get_capability_implementations_proxy.call(
            GetCapabilityImplementationsRequest(
                interface=request.interface
            )
        )

        if not response.success:
            service_response.error_message = f"Failed to get local implementations: {response.error_message}"
            rospy.logwarn(service_response.error_message)
            service_response.success = response.success
            return service_response

        if len(response.implementations) < 1:
            service_response.success = False
            service_response.error_message = "No suitable implementation found!"
            return service_response

        get_local_bid_tree_manager = TreeManager(
            name="LocalBidStagingManager",
            publish_tree_callback=nop,
            publish_debug_info_callback=nop,
            publish_debug_settings_callback=nop,
            publish_node_diagnostics_callback=nop,
            debug_manager=DebugManager()
        )
        get_local_bid_migration_manager = MigrationManager(tree_manager=get_local_bid_tree_manager)

        implementation_utility: Dict[str, float] = {}
        for implementation in response.implementations:
            get_local_bid_tree_manager.control_execution(
                ControlTreeExecutionRequest(
                    command=ControlTreeExecutionRequest.SHUTDOWN
                )
            )
            get_local_bid_tree_manager.clear(ClearTreeRequest())

            tree = implementation.tree

            migration_request = MigrateTreeRequest(tree=tree)
            migration_response = check_node_versions(migration_request)

            if migration_response.migrated:
                migration_response = get_local_bid_migration_manager.migrate_tree(migration_request)
                if migration_response.success:
                    tree = migration_response.tree

            res = get_local_bid_tree_manager.load_tree(LoadTreeRequest(tree=tree))
            if not res.success:
                rospy.logerr(res.error_message)
                continue

            calculated_utility = \
                get_local_bid_tree_manager.find_root().calculate_utility()

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
            service_response.error_message = "No utilities could be calculated for any available implementation."
            service_response.success = False
            rospy.logwarn(service_response.error_message)
            return service_response

        sorted_implementation_bids: List[Tuple[str, float]] = sorted(
            [(n, v) for n, v in implementation_utility.items()], key=lambda v: v[1]
        )

        (best_implementation_name, best_implementation_bid) = sorted_implementation_bids[0]
        service_response.bid = best_implementation_bid
        service_response.implementation_name = best_implementation_name
        service_response.success = True
        return service_response

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

        response = CheckPreconditionStatusResponse()
        hashable_capability = HashableCapabilityInterface(request.interface)

        capability_was_executed = hashable_capability in self.executing_capabilities
        if not capability_was_executed:
            response.available = False
        else:
            response.available = any(
                s in [
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
        prepare it to be sent to the executor.

        The service will return unsuccessfully if the implementation cannot be found, the preconditions can not be
        fulfilled or the implementation is not a valid tree.
        :param request: The request containing all the information about the implementation.
        :return: A response containing the result status, as well as the potential resulting tree.
        """
        response = PrepareLocalImplementationResponse()

        prepare_local_implementation_tree_manager = TreeManager(
            name="LocalBidStagingManager",
            publish_tree_callback=nop,
            publish_debug_info_callback=nop,
            publish_debug_settings_callback=nop,
            publish_node_diagnostics_callback=nop,
            debug_manager=DebugManager()
        )
        prepare_local_implementation_migration_manager = MigrationManager(
            tree_manager=prepare_local_implementation_tree_manager
        )

        capability_repository_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/capability_repository/capabilities/implementations/get"
        )

        get_capability_implementation_service_proxy = ServiceProxy(
            capability_repository_topic,
            GetCapabilityImplementations
        )

        implementations_response: GetCapabilityImplementationsResponse = \
            get_capability_implementation_service_proxy.call(
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

        tree = implementation.tree

        migration_request = MigrateTreeRequest(tree=tree)
        migration_response = check_node_versions(migration_request)

        if migration_response.migrated:
            migration_response = prepare_local_implementation_migration_manager.migrate_tree(migration_request)
            if migration_response.success:
                tree = migration_response.tree

        service_response: LoadTreeResponse = prepare_local_implementation_tree_manager.load_tree(
            LoadTreeRequest(
                tree=tree
            )
        )
        if not service_response.success:
            response.success = False
            response.error_message = f"Failed to load tree: {service_response.error_message}"
            rospy.logerr(response.error_message)
            return response

        root_node = prepare_local_implementation_tree_manager.find_root()
        root_sequence_node = ros_bt_py.nodes.sequence.Sequence(name="RootCapabilitySequence")
        service_response: AddNodeResponse = prepare_local_implementation_tree_manager.add_node(
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

        service_response: MoveNodeResponse = prepare_local_implementation_tree_manager.move_node(
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
                available_services = rosservice_find('CheckPreconditionStatus')

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

            local_precondition_fulfilled = self.check_precondition_status(
                CheckPreconditionStatusRequest(interface=precondition.capability)
            ).available

            if not local_precondition_fulfilled:
                rospy.loginfo("Adding local precondition to tree!")
                # Add a hashable capability to the tree
                node_type = capability_node_class_from_capability_interface(
                    capability_interface=precondition.capability,
                    mission_control_topic=rospy.resolve_name("~")
                )

                node = node_type()

                service_response: AddNodeAtIndexResponse = prepare_local_implementation_tree_manager.add_node_at_index(
                    AddNodeAtIndexRequest(
                        parent_name=root_sequence_node.name,
                        node=node.to_msg(),
                        allow_rename=True,
                        new_child_index=0
                    )
                )

                if not service_response.success:
                    response.success = False
                    response.error_message = f"Failed to add needed precondition: {service_response.error_message}"
                    rospy.logerr(response.error_message)
                    return response
                rospy.loginfo(f"Added {node} to tree as {service_response.actual_node_name}")

            rospy.loginfo(f"Precondition: {precondition} is fulfilled")

        response.success = True
        response.implementation_subtree = prepare_local_implementation_tree_manager.tree_msg
        return response


def nop(msg):
    """
    No operation method
    :return: None
    """
