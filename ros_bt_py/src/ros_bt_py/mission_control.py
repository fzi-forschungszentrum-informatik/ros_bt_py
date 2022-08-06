"""
Module used to implement mission control components.
This includes the management of capabilities and their implementations as well as the forwarding to
other nodes by using an auction protocol.
"""
# pylint: disable=no-name-in-module,import-error
from threading import RLock
from typing import Dict, List, Tuple, Optional

import rospy
from actionlib import SimpleActionServer, SimpleActionClient, ActionServer
from rospy import ROSException
from rosservice import rosservice_find
from ros_bt_py_msgs.msg import (
    CapabilityImplementation, GetLocalBidGoal, GetLocalBidAction, GetLocalBidResult,
    ExecuteCapabilityGoal, ExecuteCapabilityResult, ExecuteCapabilityAction, FindBestCapabilityExecutorAction,
    FindBestCapabilityExecutorGoal, FindBestCapabilityExecutorResult, Precondition, ExecuteRemoteCapabilityAction,
    ExecuteRemoteCapabilityGoal, ExecuteRemoteCapabilityActionGoal, ExecuteCapabilityImplementationGoal,
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

    def __init__(self, remote_capability_slot_name,
                 goal: ExecuteCapabilityImplementationGoal,
                 publish_feedback_cb,
                 done_cb,
                 aborted_cb):
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

        self.migration_manager = MigrationManager(tree_manager=self.staging_tree_manager)

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

        self.__remote_slot: Dict[str, int] = {}

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

    def execute_remote_capability_exec(self, remote_slot_name: str, goal: ExecuteRemoteCapabilityGoal):
        pass

    def execute_remote_capability_goal_cb(self, goal_handle: ExecuteRemoteCapabilityActionGoal) -> None:
        pass

    def execute_remote_capability_cancel_cb(self, goal_handle: ExecuteRemoteCapabilityActionGoal) -> None:
        pass

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
        :param goal:
        :return:
        """
        rospy.logwarn("Calculating local bid!")
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
