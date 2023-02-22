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

"""
Module used to implement mission control components.

This includes the management of capabilities and their implementations as well as the forwarding to
other nodes by using an auction protocol.
"""

# pylint: disable=no-name-in-module,import-error

import dataclasses
import threading
from threading import RLock
from typing import Dict, List, Tuple, Optional

import rospy
import std_msgs.msg

from actionlib import ActionServer, ServerGoalHandle
from actionlib_msgs.msg import GoalStatus
from rospy import (
    ROSException,
    Service,
    ServiceProxy,
    ServiceException,
    ROSInterruptException,
)
from rosservice import rosservice_find
from ros_bt_py_msgs.msg import (
    CapabilityImplementation,
    Precondition,
    ExecuteRemoteCapabilityAction,
    RemoteCapabilitySlotStatus,
    ExecuteRemoteCapabilityResult,
    ExecuteRemoteCapabilityGoal,
    CapabilityExecutionStatus,
)
from ros_bt_py_msgs.srv import (
    GetCapabilityImplementationsRequest,
    GetCapabilityImplementationsResponse,
    GetCapabilityImplementations,
    LoadTreeRequest,
    PrepareLocalImplementationRequest,
    PrepareLocalImplementationResponse,
    LoadTreeResponse,
    AddNodeRequest,
    AddNodeResponse,
    MoveNodeRequest,
    CheckPreconditionStatusRequest,
    CheckPreconditionStatusResponse,
    CheckPreconditionStatus,
    ClearTreeRequest,
    MoveNodeResponse,
    PrepareLocalImplementation,
    RequestCapabilityExecutionRequest,
    RequestCapabilityExecutionResponse,
    GetLocalBidResponse,
    GetLocalBidRequest,
    GetLocalBid,
    FindBestCapabilityExecutor,
    FindBestCapabilityExecutorRequest,
    MigrateTreeRequest,
    RequestCapabilityExecution,
    RunRemoteCapabilitySlot,
    RunRemoteCapabilitySlotRequest,
    CancelRemoteCapabilitySlot,
    RunRemoteCapabilitySlotResponse,
    GetAvailableRemoteCapabilitySlotsRequest,
    GetAvailableRemoteCapabilitySlotsResponse,
    GetAvailableRemoteCapabilitySlots,
    ReserveRemoteCapabilitySlotRequest,
    ReserveRemoteCapabilitySlotResponse,
    ReserveRemoteCapabilitySlot,
    CancelRemoteCapabilitySlotRequest,
)

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.capability import set_capability_io_bridge_id
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.helpers import HashableCapabilityInterface
from ros_bt_py.migration import MigrationManager, check_node_versions
from ros_bt_py.ros_helpers import AsyncServiceProxy
from ros_bt_py.tree_manager import TreeManager


@dataclasses.dataclass
class RemoteCapabilitySlotStatusData:
    """Data class holding all information about the remote capability slots current status."""

    status: str = RemoteCapabilitySlotStatus.IDLE
    reserved: bool = False
    timestamp: Optional[rospy.Time] = None
    reauction_threshold: float = float("inf")
    implementation_name: str = ""
    remote_mission_control_topic: str = ""


@dataclasses.dataclass
class RemoteCapabilityGoalStatus:
    """Represents the status of a remote capability execution running in a local slot."""

    status: str
    cancellation_requested: bool = False


class MissionControl:
    """
    Class to manage the capability implementations and interfaces on the local node.

    Additionally, it allows to exchange interface definitions with remote nodes.
    """

    # pylint: disable=too-many-instance-attributes

    def __init__(self):
        """
        Create a new capability repository.

        The repository has to major interfaces, it communicates with other repositories and
        offers services geared towards its local subscribers.
        """
        self.__capability_execution_status: Dict[
            HashableCapabilityInterface, Dict[str, int]
        ] = {}
        self.capability_repository_topic = (
            f"{rospy.get_namespace()}/capability_repository"
        )

        capability_repository_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/capability_repository/capabilities/implementations/get"
        )

        self.__get_capability_implementations_proxy = rospy.ServiceProxy(
            capability_repository_topic, GetCapabilityImplementations
        )
        self.__get_capability_implementations_proxy_lock = RLock()

        self.__request_capability_execution_service: Service = Service(
            "~execute_capability",
            RequestCapabilityExecution,
            self.request_capability_execution,
        )

        self.__get_local_bid_service: Service = Service(
            "~get_local_bid", GetLocalBid, self.get_local_bid
        )

        self.__get_local_bid_service_client = AsyncServiceProxy(
            "~get_local_bid",
            GetLocalBid,
        )

        self.__check_precondition_status_service = rospy.Service(
            "~check_precondition_status",
            CheckPreconditionStatus,
            handler=self.check_precondition_status,
        )

        self.__prepare_local_implementation_service = rospy.Service(
            "~prepare_local_implementation",
            PrepareLocalImplementation,
            self.prepare_local_capability_implementation,
        )
        self.__prepare_local_implementation_service_lock = RLock()

        self._capability_execution_status_subscriber = rospy.Subscriber(
            "~notify_capability_execution_status",
            CapabilityExecutionStatus,
            callback=self.notify_capability_execution_status,
        )

        self.__execute_remote_capability_action_server = ActionServer(
            "~execute_remote_capability",
            ExecuteRemoteCapabilityAction,
            goal_cb=self.execute_remote_capability_goal_cb,
            cancel_cb=self.execute_remote_capability_cancel_cb,
            auto_start=False,
        )

        self._local_remote_slots_status_subscriber = rospy.Subscriber(
            "~remote_slot_status",
            RemoteCapabilitySlotStatus,
            callback=self.remote_capability_slot_status_callback,
        )

        self.__available_remote_capability_slots_service = rospy.Service(
            "~get_available_remote_slots",
            GetAvailableRemoteCapabilitySlots,
            self.get_available_remote_capability_slots,
        )

        self.__reserve_remote_capability_slot_service = rospy.Service(
            "~reserve_remote_capability_slot",
            ReserveRemoteCapabilitySlot,
            self.reserve_remote_capability_executor,
        )

        self._remote_capability_slot_lock = RLock()
        self._get_local_bid_lock = RLock()

        self._remote_capability_slot_status: Dict[
            str, RemoteCapabilitySlotStatusData
        ] = {}
        """
        Status of all known remote capability slots.
        It stores the name of the executor together with the current status.
        """

        self._remote_capability_slot_goals: Dict[str, RemoteCapabilityGoalStatus] = {}
        """
        Dict associating goal ids with the related remote capability slot and
        the boolean variable to signal cancellation.
        """

        self._remote_capability_slot_reservation_duration = rospy.Duration.from_sec(
            rospy.get_param("~remote_capability_slot_reservation_duration_sec", 5.0)
        )

        self.__execute_remote_capability_action_server.start()

        team_join_topic = rospy.resolve_name(
            rospy.get_param("mission_control_join_topic", "/team_join")
        )
        self.__team_join_publisher = rospy.Publisher(
            team_join_topic, std_msgs.msg.Time, queue_size=1
        )

        self.__team_join_subscriber = rospy.Subscriber(
            team_join_topic, std_msgs.msg.Time, self.team_join_callback
        )

        self.__team_join_publisher.publish(std_msgs.msg.Time(data=rospy.Time.now()))

    def team_join_callback(self, msg: std_msgs.msg.Time) -> None:
        """
        Handle team composition changes as callback.

        :param msg: Timestamp when the team composition change occurred.
        :return: None
        """
        with self._remote_capability_slot_lock:
            for goal_id in self._remote_capability_slot_goals:
                rospy.logdebug(
                    f"Team composition has changed at {msg}. Canceling goal {goal_id}!"
                )
                self._remote_capability_slot_goals[
                    goal_id
                ].cancellation_requested = True

    def shutdown(self):
        """
        Shut down all the services and pub/sub of this node.

        :return: None
        """
        self.__team_join_publisher.publish(rospy.Time.now())
        self.__team_join_subscriber.unregister()
        self.__team_join_publisher.unregister()
        self.__execute_remote_capability_action_server.stop()
        self.__request_capability_execution_service.shutdown()
        self.__get_local_bid_service.shutdown()
        self.__get_local_bid_service_client.shutdown()
        self.__check_precondition_status_service.shutdown()
        self.__prepare_local_implementation_service.shutdown()

    @staticmethod
    def __handle_async_service_call(service_proxy: AsyncServiceProxy):
        """
        Manage a call to a service via the AsyncServiceProxy.

        The call method needs to be called beforehand, this method just handles
        the possible states until the result is received.
        It checks the states the service_proxy can be in and returns results
        depending on the current execution state.
        This method in nonblocking.

        :param service_proxy: The service proxy the call has been placed for.

        :raises RuntimeError: Should the call to the AsyncServiceProxy be
        aborted or not placed before calling this method a BehaviorTreeException will be raised.
        :return: The response of the call if available, otherwise None.
        """
        state = service_proxy.get_state()

        if state in [AsyncServiceProxy.IDLE, AsyncServiceProxy.SERVICE_AVAILABLE]:
            return None

        if state is AsyncServiceProxy.TIMEOUT:
            raise RuntimeError(
                f"Service call timed out before succeeding" f" (state ID: {state})"
            )

        if state is AsyncServiceProxy.ABORTED:
            raise RuntimeError(
                f"Service call was aborted before succeeding" f" (state ID: {state})"
            )

        if state is AsyncServiceProxy.ERROR:
            raise RuntimeError(
                f"Service call returned a error before succeeding"
                f" (state ID: {state})"
            )

        if state is AsyncServiceProxy.RUNNING:
            return None

        if state is AsyncServiceProxy.RESPONSE_READY:
            response = service_proxy.get_response()
            if response.success:
                return response
            raise RuntimeError(f"Service call failed {response.error_message})")
        raise RuntimeError(f"Async proxy in invalid state {state})")

    def get_local_bid(self, request: GetLocalBidRequest) -> GetLocalBidResponse:
        """
        Calculate the bid for the local node to perform a certain action as a callback.

        This method receives all the local capability implementations, and loads them into the
        staging tree manager.
        Then the local bid is calculated for each of the implementations using
        the calculate_utility function.

        :param request: The actionlib goal containing the information about
        the required interface for which the bids should be calculated
        :return: None
        """
        # pylint: disable=too-many-statements,too-many-locals, too-many-return-statements
        service_response = GetLocalBidResponse()

        if request.node_id == "":
            service_response.success = False
            service_response.error_message = (
                "GetLocalBid failed as the node_id is not populated!"
            )
            rospy.logerr(service_response.error_message)
            return service_response

        with self.__get_capability_implementations_proxy_lock:
            try:
                self.__get_capability_implementations_proxy.wait_for_service(
                    timeout=rospy.Duration.from_sec(1.0)
                )
            except ROSException:
                service_response.error_message = (
                    "Failed to find local implementation service"
                )
                rospy.logwarn(service_response.error_message)
                service_response.success = False

                return service_response

            try:
                response: GetCapabilityImplementationsResponse = (
                    self.__get_capability_implementations_proxy.call(
                        GetCapabilityImplementationsRequest(interface=request.interface)
                    )
                )
            except (ROSInterruptException, ROSException) as exc:
                service_response.error_message = (
                    f"Failed to get local implementations: {exc}"
                )
                rospy.logwarn(service_response.error_message)
                service_response.success = False
                return service_response

        if not response.success:
            service_response.error_message = (
                f"Failed to get local implementations: {response.error_message}"
            )
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
            debug_manager=DebugManager(),
            simulate_tick=True,
            succeed_always=False,
        )
        get_local_bid_migration_manager = MigrationManager(
            tree_manager=get_local_bid_tree_manager
        )

        implementation_utility: Dict[str, float] = {}
        for implementation in response.implementations:
            tree = implementation.tree

            migration_request = MigrateTreeRequest(tree=tree)
            migration_response = check_node_versions(migration_request)

            if migration_response.migrated:
                migration_response = get_local_bid_migration_manager.migrate_tree(
                    migration_request
                )
                if migration_response.success:
                    tree = migration_response.tree

            res = get_local_bid_tree_manager.load_tree(LoadTreeRequest(tree=tree))
            if not res.success:
                rospy.logerr(
                    "Failed to load implementation for calculating the local bid: "
                    f"{res.error_message}"
                )
                continue

            try:
                set_capability_io_bridge_id(
                    tree_manager=get_local_bid_tree_manager,
                    interface=request.interface,
                    io_bridge_id=request.node_id,
                )

                get_local_bid_tree_manager.find_root().setup()
                get_local_bid_tree_manager.find_root().tick()

                calculated_utility = (
                    get_local_bid_tree_manager.find_root().calculate_utility()
                )
            except (BehaviorTreeException, ROSException) as exc:
                rospy.logerr(f"Failed to calculate utility value from tree: {exc}")
                continue

            if calculated_utility.lower_bound_success == 0:
                implementation_utility[implementation.name] = (
                    calculated_utility.upper_bound_failure
                    - calculated_utility.lower_bound_failure
                )
            else:
                implementation_utility[implementation.name] = (
                    calculated_utility.lower_bound_success
                    + (
                        calculated_utility.upper_bound_failure
                        - calculated_utility.lower_bound_failure
                    )
                    / calculated_utility.lower_bound_success
                )

            try:
                get_local_bid_tree_manager.find_root().shutdown()
                get_local_bid_tree_manager.clear(ClearTreeRequest())
            except BehaviorTreeException as exc:
                rospy.logerr(f"Failed to shutdown and clear the tree: {exc}")
                continue

        if len(implementation_utility) < 1:
            service_response.error_message = (
                "No utilities could be calculated for any available implementation."
            )
            service_response.success = False
            rospy.logwarn(service_response.error_message)
            return service_response

        sorted_implementation_bids: List[Tuple[str, float]] = sorted(
            list(implementation_utility.items()), key=lambda v: v[1]
        )

        (
            best_implementation_name,
            best_implementation_bid,
        ) = sorted_implementation_bids[0]
        service_response.bid = best_implementation_bid
        service_response.implementation_name = best_implementation_name
        service_response.success = True
        return service_response

    def remote_capability_slot_status_callback(self, msg: RemoteCapabilitySlotStatus):
        """
        Update the status of remote capability slots as subscriber.

        :param msg: The new slot status.
        :return: None
        """
        with self._remote_capability_slot_lock:
            try:
                self._remote_capability_slot_status[msg.name]
            except KeyError:
                self._remote_capability_slot_status[
                    msg.name
                ] = RemoteCapabilitySlotStatusData(
                    status=msg.IDLE, timestamp=rospy.Time.now(), reserved=False
                )
            self._remote_capability_slot_status[msg.name].status = msg.status

            if self._remote_capability_slot_status[msg.name].reserved:
                timestamp = self._remote_capability_slot_status[msg.name].timestamp
                if (
                    rospy.Time.now()
                    > timestamp + self._remote_capability_slot_reservation_duration
                ):
                    self._remote_capability_slot_status[msg.name].reserved = False

    def reserve_remote_capability_executor(
        self, request: ReserveRemoteCapabilitySlotRequest
    ) -> ReserveRemoteCapabilitySlotResponse:
        """
        Reserve a remote capability slot in the local tree for a remote execution.

        :param request: Contains the information on the reservation.
        :return: None
        """
        response = ReserveRemoteCapabilitySlotResponse()

        with self._remote_capability_slot_lock:
            idle_executor_ids: List[str] = list(
                filter(
                    lambda x: (
                        lambda status_data: status_data.status
                        == RemoteCapabilitySlotStatus.IDLE
                        and not (
                            status_data.reserved
                            and rospy.Time.now()
                            < status_data.timestamp
                            + self._remote_capability_slot_reservation_duration
                        )
                    )(self._remote_capability_slot_status[x]),
                    self._remote_capability_slot_status,
                )
            )

        if len(idle_executor_ids) < 1:
            rospy.logerr(
                "No idle remote_capability_slot found, cannot reserve remote capability slot.",
                logger_name="remote_capability_execution",
            )
            response.success = False
            response.error = "No idle remote_capability_slot found!"

            return response

        reserved_remote_capability_slot = idle_executor_ids.pop()
        # rospy.logdebug(
        #     f"Reserving slot {reserved_remote_capability_slot} for "
        #     f"{request.implementation_name} {request.reauction_threshold}"
        # )
        with self._remote_capability_slot_lock:

            self._remote_capability_slot_status[
                reserved_remote_capability_slot
            ].reserved = True
            self._remote_capability_slot_status[
                reserved_remote_capability_slot
            ].timestamp = rospy.Time.now()
            self._remote_capability_slot_status[
                reserved_remote_capability_slot
            ].remote_mission_control_topic = request.remote_mission_control
            self._remote_capability_slot_status[
                reserved_remote_capability_slot
            ].reauction_threshold = request.reauction_threshold
            self._remote_capability_slot_status[
                reserved_remote_capability_slot
            ].implementation_name = request.implementation_name

        response.success = True
        return response

    def execute_remote_capability_exec(
        self,
        remote_slot_name: str,
        goal_handle: ServerGoalHandle,
        reauction_threshold: float,
    ):
        """
        Control the execution of a remote capability in a local slot.

        This method is intended to run in a separate thread to allow multiple parallel executions.
        :param remote_slot_name: The remote slot name where the capability should be executed.
        This must be a local capability slot.
        :param goal_handle: The goal definition containing all the information about the execution.
        :param reauction_threshold: Threshold when a reauction should occur.
        :return: None
        """
        goal_id = goal_handle.get_goal_id().id

        rospy.loginfo(
            f"Starting execution of {goal_id} on {remote_slot_name}",
            logger_name="remote_capability_execution",
        )
        goal: ExecuteRemoteCapabilityGoal = goal_handle.get_goal()

        prepare_local_capability_implementation_service_proxy = ServiceProxy(
            "~prepare_local_implementation", PrepareLocalImplementation
        )

        response: PrepareLocalImplementationResponse = (
            prepare_local_capability_implementation_service_proxy.call(
                PrepareLocalImplementationRequest(
                    interface=goal.interface,
                    implementation_name=goal.implementation_name,
                )
            )
        )

        if not response.success:
            rospy.logwarn(
                f"Could not prepare local implementation for {goal_id}: {response.error_message}",
                logger_name="remote_capability_execution",
            )
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.get_goal_id(), status=GoalStatus.SUCCEEDED
                ),
                result=ExecuteRemoteCapabilityResult(
                    success=False, error_message=response.error_message
                ),
            )
            return

        run_remote_capability_slot_topic = rospy.resolve_name(
            f"~remote_capability_slot/{remote_slot_name}/run"
        )

        run_remote_capability_slot_service_proxy = AsyncServiceProxy(
            run_remote_capability_slot_topic, RunRemoteCapabilitySlot
        )

        cancel_remote_capability_slot_topic = rospy.resolve_name(
            f"~remote_capability_slot/{remote_slot_name}/cancel"
        )

        cancel_remote_capability_slot_service_proxy = ServiceProxy(
            cancel_remote_capability_slot_topic, CancelRemoteCapabilitySlot
        )

        try:
            run_remote_capability_slot_service_proxy.wait_for_service(
                timeout=rospy.Duration(secs=5)
            )
        except ROSException:
            rospy.logwarn(
                "Could not contact remote_capability slot before timeout",
                logger_name="remote_capability_execution",
            )
            self.__execute_remote_capability_action_server.publish_result(
                status=GoalStatus(
                    goal_id=goal_handle.get_goal_id(), status=GoalStatus.SUCCEEDED
                ),
                result=ExecuteRemoteCapabilityResult(
                    success=False,
                    error_message="Could not contact remote_capability slot before timeout",
                ),
            )
            return

        rospy.loginfo(
            f"Send goal to {run_remote_capability_slot_topic}",
            logger_name="remote_capability_execution",
        )
        run_remote_capability_slot_service_proxy.call_service(
            RunRemoteCapabilitySlotRequest(
                implementation_tree=response.implementation_subtree,
                interface=goal.interface,
                node_id=goal.node_id,
            )
        )
        try:
            response: Optional[
                RunRemoteCapabilitySlotResponse
            ] = self.__handle_async_service_call(
                run_remote_capability_slot_service_proxy
            )
        except RuntimeError as exc:
            rospy.logerr(
                f"Failed to execute remote capability: {exc}",
                logger_name="remote_capability_execution",
            )
            return

        expended_costs = 0

        local_bid_response = self.get_local_bid(
            GetLocalBidRequest(interface=goal.interface, node_id=goal.node_id)
        )
        if local_bid_response.success:
            previous_cost_projection = local_bid_response.bid
        else:
            previous_cost_projection = reauction_threshold * 0.8

        changed_reauction_threshold = reauction_threshold - previous_cost_projection

        while response is None:

            local_bid_response = self.get_local_bid(
                GetLocalBidRequest(interface=goal.interface, node_id=goal.node_id)
            )
            if local_bid_response.success:
                expended_costs += abs(previous_cost_projection - local_bid_response.bid)
                previous_cost_projection = local_bid_response.bid
                if expended_costs > changed_reauction_threshold:
                    try:
                        cancel_remote_capability_slot_service_proxy.call(
                            CancelRemoteCapabilitySlotRequest()
                        )
                    except ServiceException as exc:
                        rospy.logerr(
                            f"Failed to cancel the remote capability slot execution: {exc}"
                        )
                    run_remote_capability_slot_service_proxy.stop_call()
                    goal_handle.set_canceled(text="Reauction started!")
                    return

            if (
                self._remote_capability_slot_goals[goal_id].cancellation_requested
                or rospy.is_shutdown()
            ):
                rospy.loginfo(
                    f"Cancellation of goal {goal_id} requested!",
                    logger_name="remote_capability_execution",
                )
                try:
                    cancel_remote_capability_slot_service_proxy.call(
                        CancelRemoteCapabilitySlotRequest()
                    )
                except ServiceException as exc:
                    rospy.logerr(
                        f"Failed to cancel the remote capability slot execution: {exc}"
                    )

                run_remote_capability_slot_service_proxy.stop_call()
                goal_handle.set_canceled(text="Cancellation of goal requested!")
                return

            try:
                response: Optional[
                    RunRemoteCapabilitySlotResponse
                ] = self.__handle_async_service_call(
                    run_remote_capability_slot_service_proxy
                )
            except RuntimeError as exc:
                rospy.logerr(
                    f"Failed to execute remote capability: {exc}",
                    logger_name="remote_capability_execution",
                )
                return

        self.__execute_remote_capability_action_server.publish_result(
            status=GoalStatus(
                goal_id=goal_handle.get_goal_id(), status=GoalStatus.SUCCEEDED
            ),
            result=ExecuteRemoteCapabilityResult(
                success=response.success, error_message=response.error_message
            ),
        )

        del self._remote_capability_slot_goals[goal_id]

    def execute_remote_capability_goal_cb(self, goal_handle: ServerGoalHandle) -> None:
        """
        Execute remote capability action goal callback.

        This callback manages the arrival of a new goal to the action server.

        It will look for a new remote capability slot where the implementation
        in the goal can be executed. If an executor is found, a new thread will be started
        that processes the goal.

        :param goal_handle: The goal handle that contains all the necessary
        information for the new request.
        :return: None
        """
        goal_id = goal_handle.get_goal_id().id
        goal: ExecuteRemoteCapabilityGoal = goal_handle.get_goal()

        with self._remote_capability_slot_lock:
            idle_executor_ids: List[str] = list(
                filter(
                    lambda x: self._remote_capability_slot_status[x].status
                    == RemoteCapabilitySlotStatus.IDLE,
                    self._remote_capability_slot_status,
                )
            )
            reserved_executors = list(
                filter(
                    lambda x: (
                        lambda slot_status: slot_status.reserved
                        and slot_status.implementation_name == goal.implementation_name
                    )(self._remote_capability_slot_status[x]),
                    idle_executor_ids,
                )
            )

            # rospy.logdebug(f"Reserved executors: {reserved_executors}")

            if len(reserved_executors) >= 1:
                selected_executor_id = reserved_executors.pop()
                self._remote_capability_slot_status[
                    selected_executor_id
                ].reserved = False
                reauction_threshold = self._remote_capability_slot_status[
                    selected_executor_id
                ].reauction_threshold
                self._remote_capability_slot_status[
                    selected_executor_id
                ].reauction_threshold = float("inf")
            else:
                rospy.logerr(
                    "No reserved remote_capability_slot found, cannot execute remote capability.",
                    logger_name="remote_capability_execution",
                )
                self.__execute_remote_capability_action_server.publish_result(
                    status=GoalStatus(
                        goal_id=goal_handle.get_goal_id(), status=GoalStatus.SUCCEEDED
                    ),
                    result=ExecuteRemoteCapabilityResult(
                        success=False,
                        no_executor_available=True,
                        error_message="No reserved remote capability slot executor available!",
                    ),
                )
                return

            self.remote_capability_slot_status_callback(
                RemoteCapabilitySlotStatus(
                    name=selected_executor_id,
                    status=RemoteCapabilitySlotStatus.RUNNING,
                    timestamp=rospy.Time.now(),
                )
            )

            self._remote_capability_slot_goals[goal_id] = RemoteCapabilityGoalStatus(
                status=selected_executor_id, cancellation_requested=False
            )
            # rospy.logdebug("Starting thread!", logger_name="remote_capability_execution")
            thread = threading.Thread(
                name=f"Thread-RemoteCapabilitySlot-{selected_executor_id}",
                target=self.execute_remote_capability_exec,
                args=(selected_executor_id, goal_handle, reauction_threshold),
            )
            thread.start()
            rospy.loginfo(
                "Started thread for execution of remote capability!",
                logger_name="remote_capability_execution",
            )

    def execute_remote_capability_cancel_cb(
        self, goal_handle: ServerGoalHandle
    ) -> None:
        """
        Cancel the execution of a remote capability.

        This callback manages the request for the cancellation of a running goal.
        The method will send a failed status back if the goal id is not
        associated with a running goal.

        :param goal_handle: The goal handle containing all the information about
        the goal to cancel.
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
                result=ExecuteRemoteCapabilityResult(),
            )
            return
        current_status = self._remote_capability_slot_goals[goal_id]
        self._remote_capability_slot_goals[goal_id] = RemoteCapabilityGoalStatus(
            status=current_status.status, cancellation_requested=True
        )
        return

    # pylint: disable=unused-argument

    def notify_capability_execution_status(
        self, msg: CapabilityExecutionStatus
    ) -> None:
        """
        Notify the MissionControl node about changes in the capability execution status.

        :param msg: Update msg containing the new status.
        :return: None
        """
        hashable_interface = HashableCapabilityInterface(msg.interface)

        if hashable_interface not in self.__capability_execution_status:
            self.__capability_execution_status[hashable_interface] = {}

        self.__capability_execution_status[hashable_interface][
            msg.node_name
        ] = msg.status

    def request_capability_execution(
        self, request: RequestCapabilityExecutionRequest
    ) -> RequestCapabilityExecutionResponse:
        """
        Trigger the execution of a capability implementation via a remote capability executor.

        :param request: The request that should be archived.
        :return: Response containing the result of the remote execution.
        """
        response = RequestCapabilityExecutionResponse()
        rospy.loginfo("Received request to execute capability!")

        if not request.require_local_execution:
            assignment_system_topic = rospy.resolve_name(
                f"{rospy.get_namespace()}/assignment_manager/find_best_executor"
            )
            find_best_executor_service = ServiceProxy(
                assignment_system_topic, FindBestCapabilityExecutor
            )

            try:
                find_best_executor_service.wait_for_service(
                    timeout=rospy.Duration(secs=1)
                )

                find_best_executor_response = find_best_executor_service.call(
                    FindBestCapabilityExecutorRequest(
                        capability=request.capability,
                        node_id=request.node_id,
                        mission_control_name=rospy.get_name(),
                    )
                )

                if find_best_executor_response.success:
                    rospy.loginfo("Found a suitable implementation!")
                    response.success = True
                    response.implementation_name = (
                        find_best_executor_response.implementation_name
                    )
                    response.execute_local = find_best_executor_response.execute_local
                    response.remote_mission_controller_topic = (
                        find_best_executor_response.executor_mission_control_topic
                    )
                    return response

            except ROSException:
                rospy.logwarn(
                    "Assignment system could not be contacted before timeout!"
                )

            rospy.loginfo(
                "Assignment  system is currently offline. We will use the local implementation!"
            )
        else:
            rospy.loginfo("Local execution required!")

        local_bid_response = self.get_local_bid(
            GetLocalBidRequest(interface=request.capability, node_id=request.node_id)
        )

        if local_bid_response.success:
            response.implementation_name = local_bid_response.implementation_name
            response.success = True
            response.execute_local = True
        else:
            response.success = False
            response.error_message = "Could not find a suitable implementation!"

        return response

    def check_precondition_status(
        self, request: CheckPreconditionStatusRequest
    ) -> CheckPreconditionStatusResponse:
        """
        Service to check if the tree tied to this mission controller fulfills the precondition.

        A precondition counts as fulfilled if there is a local capability node that
        is running the required capability.
        This includes both capabilities and remote capability slots.

        :param request: Request containing the precondition that should be checked.
        :return: True if fulfilled or False if not.
        """
        response = CheckPreconditionStatusResponse()
        hashable_capability = HashableCapabilityInterface(request.interface)

        capability_was_executed = (
            hashable_capability in self.__capability_execution_status
        )
        if not capability_was_executed:
            response.available = False
        else:
            response.available = any(
                s
                in [
                    CapabilityExecutionStatus.EXECUTING,
                    CapabilityExecutionStatus.SUCCEEDED,
                ]
                for n, s in self.__capability_execution_status[
                    hashable_capability
                ].items()
            )

        return response

    def prepare_local_capability_implementation(
        self, request: PrepareLocalImplementationRequest
    ) -> PrepareLocalImplementationResponse:
        """
        Service that prepares a local capability implementation for execution.

        This includes receiving it from the local capability storage, check and
        fulfill preconditions and then prepare it to be sent to the executor.

        The service will return unsuccessfully if the implementation cannot be found,
        the preconditions can not be fulfilled or the implementation is not a valid tree.

        :param request: The request containing all the information about the implementation.
        :return: A response containing the result status, as well as the potential resulting tree.
        """
        # pylint: disable=too-many-locals, too-many-statements
        response = PrepareLocalImplementationResponse()

        with self.__prepare_local_implementation_service_lock:

            prepare_local_implementation_tree_manager = TreeManager(
                name="LocalBidStagingManager",
                publish_tree_callback=nop,
                publish_debug_info_callback=nop,
                publish_debug_settings_callback=nop,
                publish_node_diagnostics_callback=nop,
                debug_manager=DebugManager(),
            )
            prepare_local_implementation_migration_manager = MigrationManager(
                tree_manager=prepare_local_implementation_tree_manager
            )

            with self.__get_capability_implementations_proxy_lock:
                implementations_response: GetCapabilityImplementationsResponse = (
                    self.__get_capability_implementations_proxy.call(
                        GetCapabilityImplementationsRequest(interface=request.interface)
                    )
                )
            implementation: Optional[CapabilityImplementation] = next(
                filter(
                    lambda x: x.name == request.implementation_name,
                    implementations_response.implementations,
                ),
                None,
            )

            if implementation is None:
                response.success = False
                response.error_message = "Could not get local implementation!"
                return response

            tree = implementation.tree

            migration_request = MigrateTreeRequest(tree=tree)
            migration_response = check_node_versions(migration_request)

            if migration_response.migrated:
                migration_response = (
                    prepare_local_implementation_migration_manager.migrate_tree(
                        migration_request
                    )
                )
                if migration_response.success:
                    tree = migration_response.tree

            service_response: LoadTreeResponse = (
                prepare_local_implementation_tree_manager.load_tree(
                    LoadTreeRequest(tree=tree)
                )
            )
            if not service_response.success:
                response.success = False
                response.error_message = (
                    f"Failed to load tree: {service_response.error_message}"
                )
                rospy.logerr(response.error_message)
                return response

            root_node = prepare_local_implementation_tree_manager.find_root()
            root_sequence_node = Sequence(name="RootCapabilitySequence")
            service_response: AddNodeResponse = (
                prepare_local_implementation_tree_manager.add_node(
                    AddNodeRequest(
                        parent_name=None,
                        node=root_sequence_node.to_msg(),
                        allow_rename=False,
                    )
                )
            )

            if not service_response.success:
                response.success = False
                response.error_message = (
                    "Failed to add sequence for preconditions: "
                    f"{service_response.error_message}"
                )
                rospy.logerr(response.error_message)
                return response

            service_response: MoveNodeResponse = (
                prepare_local_implementation_tree_manager.move_node(
                    MoveNodeRequest(
                        node_name=root_node.name,
                        new_parent_name=root_sequence_node.name,
                        new_child_index=0,
                    )
                )
            )

            if not service_response.success:
                response.success = False
                response.error_message = (
                    f"Failed to move implementation under sequence node:"
                    f"{service_response.error_message}"
                )
                rospy.logerr(response.error_message)
                return response

            for precondition in reversed(implementation.preconditions):

                if precondition.type == Precondition.REMOTE:
                    available_services = rosservice_find("CheckPreconditionStatus")

                    for service_name in available_services:
                        service_client = rospy.ServiceProxy(
                            service_name, CheckPreconditionStatus
                        )
                        service_client.wait_for_service(1.0)
                        service_response: CheckPreconditionStatusResponse = (
                            service_client(
                                CheckPreconditionStatusRequest(
                                    interface=precondition.capability
                                )
                            )
                        )
                        if service_response.available:
                            rospy.loginfo(
                                f"Remote precondition: {precondition} is fulfilled"
                            )
                            continue

                local_precondition_fulfilled = self.check_precondition_status(
                    CheckPreconditionStatusRequest(interface=precondition.capability)
                ).available

                if not local_precondition_fulfilled:
                    response.success = False
                    response.error_message = "Local precondition not fulfilled!"
                    rospy.logerr(response.error_message)
                    return response

                rospy.logdebug(f"Precondition: {precondition} is fulfilled")

            response.success = True
            response.implementation_subtree = (
                prepare_local_implementation_tree_manager.tree_msg
            )
            return response

    def get_available_remote_capability_slots(
        self, requests: GetAvailableRemoteCapabilitySlotsRequest
    ) -> GetAvailableRemoteCapabilitySlotsResponse:
        """
        Return the amount of available remote capability slots.

        :param requests: None
        :return: Amount of available remote capability slots.
        """
        response = GetAvailableRemoteCapabilitySlotsResponse()

        response.available_remote_capability_slots = len(
            list(
                filter(
                    lambda x: self._remote_capability_slot_status[x].status
                    == RemoteCapabilitySlotStatus.IDLE
                    and not (
                        self._remote_capability_slot_status[x].reserved
                        and rospy.Time.now()
                        < self._remote_capability_slot_status[x].timestamp
                        + self._remote_capability_slot_reservation_duration
                    ),
                    self._remote_capability_slot_status,
                )
            )
        )
        return response


def nop(msg):
    """
    No operation method.

    :return: None
    """
