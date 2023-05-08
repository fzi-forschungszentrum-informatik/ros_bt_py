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
Module for handling BT nodes related to capabilities.

It defines the base ABC for Capability nodes
and functions to create class definitions for capability interfaces
at runtime.
"""
# pylint: disable=no-name-in-module,import-error,too-many-lines

import secrets
import threading
from abc import ABC
from typing import Optional, List, Type, Dict

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from rospy import ROSSerializationException, ROSException, Publisher, Subscriber
from std_msgs.msg import Time

from ros_bt_py_msgs.msg import (
    CapabilityInterface,
    Node as NodeMsg,
    Tree,
    ExecuteRemoteCapabilityAction,
    ExecuteRemoteCapabilityGoal,
    CapabilityIOBridgeData,
    CapabilityExecutionStatus,
    ExecuteRemoteCapabilityResult,
    NodeData as NodeDataMsg,
    UtilityBounds,
    PingMsg,
)
from ros_bt_py_msgs.srv import (
    PrepareLocalImplementation,
    PrepareLocalImplementationRequest,
    LoadTreeRequest,
    ClearTreeRequest,
    ControlTreeExecutionRequest,
    GetCapabilityInterfacesRequest,
    GetCapabilityInterfacesResponse,
    RequestCapabilityExecution,
    RequestCapabilityExecutionRequest,
    RequestCapabilityExecutionResponse,
    GetLocalBid,
    GetLocalBidRequest,
    GetLocalBidResponse,
)

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import BehaviorTreeException, TreeTopologyError
from ros_bt_py.helpers import json_decode, json_encode, rgetattr
from ros_bt_py.node import define_bt_node, Leaf, Node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.ros_helpers import AsyncServiceProxy
from ros_bt_py.tree_manager import TreeManager


class CapabilityDataBridge(ABC, Leaf):
    """Data bridge that allows the exchange of data between capability interfaces and nodes."""

    # pylint: disable=too-few-public-methods

    def __init__(
        self,
        debug_manager=None,
        name=None,
        options=None,
        simulate_tick=False,
        succeed_always=False,
    ):
        """
        Create a new capability data bridge node.

        :param debug_manager: The debug manager to use.
        :param name: The name of the node.
        :param options: The node options.
        """
        super(CapabilityDataBridge, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            simulate_tick=simulate_tick,
            succeed_always=succeed_always,
        )
        self._io_bridge_id: Optional[str] = None
        self.capability_io_topic: str = rospy.get_param(
            "capability_io_topic", "~/capabilities/io"
        )

    @property
    def capability_bridge_id(self) -> str:
        """
        Unique id used to identify the information relevant for this IO bridge.

        :return: string containing the topic
        """
        return self._io_bridge_id

    @capability_bridge_id.setter
    def capability_bridge_id(self, new_id: str):
        """
        Set the unique id used to identify the information relevant for this IO bridge.

        :param new_topic: The new topic to use.
        :return: None
        """
        self._io_bridge_id = new_id


class CapabilityInputDataBridge(CapabilityDataBridge):
    """CapabilityDataBridge that transfers the interface inputs to the implementations inputs."""

    # pylint: disable=too-few-public-methods

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: str = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(CapabilityInputDataBridge, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            simulate_tick=simulate_tick,
            succeed_always=succeed_always,
        )
        self._source_capability_inputs_subscriber: Optional[rospy.Subscriber] = None
        self._current_received_msg: Optional[CapabilityIOBridgeData] = None
        self._lock = threading.RLock()
        self._message_lock = threading.RLock()

    def _publish_inputs_cb(self, msg: CapabilityIOBridgeData):
        """
        Callback to store the received input message internally to be processed on the next tick.

        :param msg: The received message.
        :return: None
        """
        with self._message_lock:
            if (msg.node_id == self.capability_bridge_id) and (
                msg.type == CapabilityIOBridgeData.INPUT
            ):
                rospy.logdebug("Recieved msg relevant to this input bridge!")
                self._current_received_msg = msg

    def _do_setup(self):
        with self._lock:

            self._source_capability_inputs_subscriber = rospy.Subscriber(
                self.capability_io_topic,
                CapabilityIOBridgeData,
                callback=self._publish_inputs_cb,
            )
            try:
                rospy.wait_for_message(
                    self.capability_io_topic,
                    CapabilityIOBridgeData,
                    timeout=rospy.Duration.from_sec(10),
                )
            except ROSException as exc:
                rospy.logwarn(
                    f"Failed to receive an initial message for the input bridge: {self.name}"
                )
                raise BehaviorTreeException(
                    f"Input bridge {self.name} messages not populated!"
                ) from exc

            while self._current_received_msg is None:
                rospy.sleep(rospy.Duration.from_sec(0.1))

            for bridge_data in self._current_received_msg.bridge_data:
                node_data: NodeDataMsg = bridge_data
                try:
                    self.outputs[node_data.key] = json_decode(
                        node_data.serialized_value
                    )
                except TypeError as exc:
                    self.logwarn(f"Could not set output {node_data.key}: {exc}")
                    continue
            self._handle_outputs()

    def _do_tick(self):
        with self._lock:
            if self.state == NodeMsg.IDLE:
                self.state = NodeMsg.RUNNING

            if self.state == NodeMsg.RUNNING:
                with self._message_lock:
                    if self._current_received_msg is not None:
                        for bridge_data in self._current_received_msg.bridge_data:
                            node_data: NodeDataMsg = bridge_data
                            try:
                                self.outputs[node_data.key] = json_decode(
                                    node_data.serialized_value
                                )
                            except TypeError as exc:
                                self.logwarn(
                                    f"Could not set output {node_data.key}: {exc}"
                                )
                                continue

                        return NodeMsg.SUCCEEDED
                    return NodeMsg.RUNNING
            return self.state

    def _do_untick(self):
        with self._lock:
            with self._message_lock:
                self._current_received_msg = None
            return NodeMsg.IDLE

    def _do_reset(self):
        with self._lock:
            with self._message_lock:
                self._current_received_msg = None
            return NodeMsg.IDLE

    def _do_shutdown(self):
        with self._lock:
            with self._message_lock:
                self._current_received_msg = None

            if self._source_capability_inputs_subscriber is not None:
                try:
                    self._source_capability_inputs_subscriber.unregister()
                except AttributeError as exc:
                    self.logfatal(f"Attribute error during unregister: {exc}")

                self._source_capability_inputs_subscriber = None


class CapabilityOutputDataBridge(CapabilityDataBridge):
    """
    CapabilityDataBridge that transfers the outputs
    of the implementation as an output to the interface.
    """

    # pylint: disable=too-few-public-methods

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: str = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super().__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )
        self._source_capability_outputs_publisher: Optional[rospy.Publisher] = None
        self._lock = threading.RLock()

    def _do_setup(self):
        with self._lock:
            self._source_capability_outputs_publisher = rospy.Publisher(
                self.capability_io_topic, CapabilityIOBridgeData, queue_size=10
            )

    def _do_tick(self):
        with self._lock:
            if self.state == NodeMsg.IDLE:
                return NodeMsg.RUNNING
            if self.state == NodeMsg.RUNNING:
                if self._source_capability_outputs_publisher is not None:
                    outputs = []
                    for key in self.inputs:
                        output = NodeDataMsg()
                        output.key = key
                        output.serialized_value = self.inputs.get_serialized(key=key)
                        output.serialized_type = self.inputs.get_serialized_type(
                            key=key
                        )
                        outputs.append(output)

                    self._source_capability_outputs_publisher.publish(
                        CapabilityIOBridgeData(
                            node_id=self.capability_bridge_id,
                            type=CapabilityIOBridgeData.OUTPUT,
                            bridge_data=outputs,
                            timestamp=rospy.Time.now(),
                        )
                    )
                    return NodeMsg.SUCCEEDED
                return NodeMsg.FAILED
            return NodeMsg.FAILED

    def _do_untick(self):
        with self._lock:
            return NodeMsg.IDLE

    def _do_reset(self):
        with self._lock:
            return NodeMsg.IDLE

    def _do_shutdown(self):
        with self._lock:
            if self._source_capability_outputs_publisher is not None:
                try:
                    self._source_capability_outputs_publisher.unregister()
                except AttributeError as exc:
                    self.logfatal(f"Attribute error during unregister: {exc}")
                self._source_capability_outputs_publisher = None


class Capability(ABC, Leaf):
    """
    ABC for Capability nodes that implements the necessary logic for capabilities to function.

    The class manages the lifecycle of capabilities and performs the necessary
    communications to allow the capability to function.
    """

    # pylint: disable=too-many-instance-attributes, too-few-public-methods

    # States to structure the do_tick() method
    IDLE = 0
    WAITING_FOR_ASSIGNMENT = 1
    EXECUTE_LOCAL = 2
    EXECUTE_REMOTE = 3
    WAITING_FOR_OUTPUT_VALUES = 4

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: str = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super().__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        self.capability_io_topic: str = rospy.get_param(
            "capability_io_topic", "~/capabilities/io"
        )
        self.wait_for_local_mission_control_timeout_sec: int = rospy.get_param(
            "wait_for_local_mission_control_timeout_sec", 2
        )
        self.wait_for_implementation_assignment_timeout_sec: int = rospy.get_param(
            "wait_for_implementation_assignment_timeout_sec", 2
        )
        self.wait_for_remote_mission_control_tries: str = rospy.get_param(
            "wait_for_remote_mission_control_tries", 3
        )
        self.wait_for_outputs_timeout_sec: str = rospy.get_param(
            "wait_for_outputs_timeout_sec", 1
        )

        if hasattr(self, "__capability_interface"):
            self.capability_interface = getattr(self, "__capability_interface")
        else:
            rospy.logwarn("Created Capability without interface! This is not intended!")
            raise BehaviorTreeException(
                "Created Capability without interface! This is not intended!"
            )

        if hasattr(self, "__local_mc_topic"):
            self.local_mc_topic = getattr(self, "__local_mc_topic")
        else:
            rospy.logwarn("Created Capability without interface! This is not intended!")
            raise BehaviorTreeException(
                "Created Capability without interface! This is not intended!"
            )

        self._internal_state = self.IDLE
        """Internal state used to track the execution progress of the capability."""

        self.tree_manager = TreeManager(
            name=name,
            publish_tree_callback=self.nop,
            publish_debug_info_callback=self.nop,
            publish_debug_settings_callback=self.nop,
            publish_node_diagnostics_callback=self.nop,
            debug_manager=self.debug_manager,
        )
        """Tree manager used to run the local capability implementations."""

        self._old_state = self.state
        """Backup of the node state before the node enters the PAUSED state"""

        self._capability_lock = threading.RLock()
        """Lock used to avoid race conditions when setting up local implementations in a tree."""

        self._implementation_name: Optional[str] = None

        # Action and service clients
        self._request_capability_execution_service_proxy: Optional[
            AsyncServiceProxy
        ] = None
        self._prepare_local_implementation_service_proxy: Optional[
            AsyncServiceProxy
        ] = None
        self._execute_capability_remote_client: Optional[SimpleActionClient] = None
        self._capability_execution_status_publisher: Optional[Publisher] = None

        # Input and output data bridge information
        self._io_bridge_id: Optional[str] = None
        self._io_publisher: Optional[rospy.Publisher] = None
        self._io_subscriber: Optional[rospy.Subscriber] = None
        self._last_received_outputs: Optional[CapabilityIOBridgeData] = None

        # Ping used to indicate that a remote capability is still running!
        self._ping_subscriber: Optional[Subscriber] = None
        self._last_ping_timestamp: Optional[rospy.Time] = None

        # Result variables used to track if the implementation is finished
        self._result_status: Optional[str] = None
        self._result_timestamp: Optional[rospy.Time] = None

        # Variables for remote trees
        self._remote_mission_control_topic: Optional[str] = None
        self._execute_capability_remote_client_connection_tries: int = 0
        # Variables for local trees
        self._prepare_local_tree_thread: Optional[threading.Thread] = None
        self._prepare_local_implementation_start_time: Optional[rospy.Time] = None

        self._local_implementation_tree: Optional[Tree] = None
        self._local_implementation_tree_root: Optional[Node] = None
        self._setup_local_implementation_tree_status: str = NodeMsg.IDLE

        local_bid_service_topic = rospy.resolve_name(
            f"{self.local_mc_topic}/get_local_bid"
        )
        self.__get_local_bid_service: rospy.ServiceProxy = rospy.ServiceProxy(
            local_bid_service_topic, GetLocalBid
        )

    def _do_calculate_utility(self):
        if self._io_publisher is not None:
            input_data = []
            for key in self.inputs:
                input_data_msg = NodeDataMsg()
                input_data_msg.key = key
                input_data_msg.serialized_value = self.inputs.get_serialized(key=key)
                input_data_msg.serialized_type = self.inputs.get_serialized_type(
                    key=key
                )
                input_data.append(input_data_msg)

            try:
                self._io_publisher.publish(
                    CapabilityIOBridgeData(
                        node_id=self._io_bridge_id,
                        type=CapabilityIOBridgeData.INPUT,
                        bridge_data=input_data,
                        timestamp=rospy.Time.now(),
                    )
                )
            except ROSSerializationException as exc:
                error_msg = f"Could not serialize inputs to publish: {exc}"
                self.logerr(error_msg)
                raise BehaviorTreeException(error_msg) from exc
            except ROSException as exc:
                error_msg = f"Input data bridge publisher could not publish: {exc}"
                self.logerr(error_msg)
                raise BehaviorTreeException(error_msg) from exc

        response: GetLocalBidResponse = self.__get_local_bid_service.call(
            GetLocalBidRequest(
                interface=self.capability_interface, node_id=self._io_bridge_id
            )
        )
        if not response.success:
            rospy.logerr(
                f"Could not calculate the utility value for the capability {self.name}:"
                f" {response.error_message}"
            )
            return UtilityBounds(can_execute=False)
        return UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            lower_bound_success=0,
            has_upper_bound_success=True,
            upper_bound_success=response.bid,
            has_lower_bound_failure=True,
            lower_bound_failure=0,
            has_upper_bound_failure=True,
            upper_bound_failure=response.bid,
        )

    @staticmethod
    def nop(msg):
        """no op "publisher" """

    @staticmethod
    def __handle_async_service_call(service_proxy: AsyncServiceProxy):
        """
        Helper method to manage a call to a service via the AsyncServiceProxy.
        The call method needs to be called beforehand,
        this method just handles the possible states until the result is received.
        It checks the states the service_proxy can be in and returns results depending
        on the current execution state.
        This method in nonblocking.

        :param service_proxy: The service proxy the call has been placed for.

        :raises BehaviorTreeException: Should the call to the AsyncServiceProxy be aborted
        or not placed before calling
        this method a BehaviorTreeException will be raised.
        :return: The response of the call if available, otherwise None.
        """
        state = service_proxy.get_state()

        if state in [AsyncServiceProxy.IDLE, AsyncServiceProxy.SERVICE_AVAILABLE]:
            raise BehaviorTreeException(f"Service is still idle (state ID: {state})")

        if state is AsyncServiceProxy.TIMEOUT:
            raise BehaviorTreeException(
                f"Service call timed out before succeeding (state ID: {state})"
            )

        if state is AsyncServiceProxy.ABORTED:
            raise BehaviorTreeException(
                f"Service call was aborted before succeeding (state ID: {state})"
            )

        if state is AsyncServiceProxy.ERROR:
            raise BehaviorTreeException(
                f"Service call returned a error before succeeding (state ID: {state})"
            )

        if state is AsyncServiceProxy.RUNNING:
            return None

        if state is AsyncServiceProxy.RESPONSE_READY:
            response = service_proxy.get_response()
            if response.success:
                return response
            raise BehaviorTreeException(
                f"Service call failed {response.error_message})"
            )
        raise RuntimeError(f"Async proxy in invalid state {state})")

    @staticmethod
    def _execute_action(action_client: SimpleActionClient):
        """
        Checks the execution status of a simple action client action and provides the result.
        Non-blocking method that checks possible states and returns the result.
        :param action_client: The action client to use for the action.
        :raises BehaviorTreeException: If the action enters a failed state or is not successful.
        :return: None if no result is yet present, result if action succeeded.
        """
        action_state = action_client.get_state()

        if action_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
            raise BehaviorTreeException(
                f"Action ended without succeeding (state ID: {action_state})"
            )

        if action_state in [GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
            raise BehaviorTreeException(
                f"Action was willingly canceled before succeeding (state ID: {action_state})"
            )

        if action_state == GoalStatus.SUCCEEDED:
            action_result = action_client.get_result()
            return action_result

        return None

    def _publish_outputs_cb(self, msg: CapabilityIOBridgeData):
        """
        Callback to store the received output values locally and allow them to be
        processed in the next tick.
        :param msg: The received output values.
        :return:None
        """
        if (
            msg.type == CapabilityIOBridgeData.OUTPUT
            and msg.node_id == self._io_bridge_id
        ):
            with self._capability_lock:
                self._last_received_outputs = msg

    def _do_setup(self):
        try:
            prepare_local_implementation_name = rospy.resolve_name(
                f"{self.local_mc_topic}/prepare_local_implementation"
            )

            rospy.wait_for_service(
                prepare_local_implementation_name,
                timeout=rospy.Duration(
                    secs=self.wait_for_local_mission_control_timeout_sec
                ),
            )
        except rospy.ROSException as exc:
            raise BehaviorTreeException(
                f'Service "{self.local_mc_topic}/prepare_local_implementation" not available'
                f" after waiting f{self.wait_for_local_mission_control_timeout_sec} seconds!"
            ) from exc

        self._prepare_local_implementation_service_proxy = AsyncServiceProxy(
            prepare_local_implementation_name, PrepareLocalImplementation
        )

        # Create an action client for FindBestExecutor
        self._request_capability_execution_service_proxy = AsyncServiceProxy(
            f"{self.local_mc_topic}/execute_capability", RequestCapabilityExecution
        )

        try:
            self._request_capability_execution_service_proxy.wait_for_service(
                timeout=rospy.Duration(
                    secs=self.wait_for_local_mission_control_timeout_sec
                )
            )
        except rospy.ROSException as exc:
            raise BehaviorTreeException(
                f'Service "{self.local_mc_topic}/execute_capability" not available'
                f" after waiting f{self.wait_for_local_mission_control_timeout_sec} seconds!"
            ) from exc

        self._io_bridge_id = f"{self.name}_{secrets.randbelow(100000)}"

        self._io_publisher = rospy.Publisher(
            self.capability_io_topic, CapabilityIOBridgeData, queue_size=1, latch=True
        )
        self._output_bridge_subscriber = rospy.Subscriber(
            self.capability_io_topic,
            CapabilityIOBridgeData,
            callback=self._publish_outputs_cb,
        )

        self._ping_subscriber = rospy.Subscriber(
            "~/capabilities/ping",
            PingMsg,
            self._update_remote_exec_ping,
            queue_size=1,
        )

        notify_capability_execution_status_topic = rospy.resolve_name(
            f"{self.local_mc_topic}/notify_capability_execution_status"
        )

        self._capability_execution_status_publisher = Publisher(
            notify_capability_execution_status_topic,
            CapabilityExecutionStatus,
            queue_size=1,
        )

    def _update_remote_exec_ping(self, msg: PingMsg):
        if self._io_bridge_id == msg.node_id:
            with self._capability_lock:
                self._last_ping_timestamp = msg.timestamp

    def _tick_unassigned(self) -> str:
        """
        Function performing the tick operation while in the UNASSIGNED state.
        :return: The new state as a NodeMsg constant.
        """
        # pylint: disable=too-many-return-statements
        if self._internal_state == self.IDLE:
            try:
                require_local_execution = self.options["require_local_execution"]
            except KeyError:
                require_local_execution = False

            tags: List[str] = self.options["inputs_names_for_implementation_tags"]
            implementation_tags_dict = dict()
            for tag in tags:
                input_tags = tag.split(".", 1)
                if len(input_tags) < 1:
                    self.logerr(f"Specified attribute {tag} is not valid")
                    self._cleanup()
                    return NodeMsg.FAILED

                input = input_tags[0]
                try:
                    input_value = self.inputs[input]
                except KeyError:
                    self.logerr(
                        f"Specified attribute {tag} could not be found on inputs"
                    )
                    self._cleanup()

                if len(input_tags) == 2 and input_tags[1] != "":
                    try:
                        implementation_tags_dict[tag] = rgetattr(
                            input_value, input_tags[1]
                        )
                    except AttributeError:
                        self.logerr(
                            f"Specified attribute {input_tags[1]} could not be found on inputs"
                        )
                        self._cleanup()
                        return NodeMsg.FAILED
                else:
                    implementation_tags_dict[tag] = input_value

            self._request_capability_execution_service_proxy.call_service(
                RequestCapabilityExecutionRequest(
                    capability=self.capability_interface,
                    node_id=self._io_bridge_id,
                    require_local_execution=require_local_execution,
                    implementation_tags_dict=json_encode(implementation_tags_dict),
                )
            )
            self._internal_state = self.WAITING_FOR_ASSIGNMENT

        if self._internal_state == self.WAITING_FOR_ASSIGNMENT:
            try:
                request_capability_execution_response: Optional[
                    RequestCapabilityExecutionResponse
                ] = self.__handle_async_service_call(
                    self._request_capability_execution_service_proxy
                )
            except BehaviorTreeException as exc:
                self.logerr(
                    "Failed to request execute capability service, no implementation will be"
                    f" selected: {exc}"
                )
                self._cleanup()
                return NodeMsg.FAILED

            if request_capability_execution_response is None:
                return NodeMsg.UNASSIGNED

            self._implementation_name = (
                request_capability_execution_response.implementation_name
            )

            if request_capability_execution_response.execute_local:
                self.loginfo("Executing capability locally!")

                self._prepare_local_implementation_service_proxy.call_service(
                    req=PrepareLocalImplementationRequest(
                        implementation_name=self._implementation_name,
                        interface=self.capability_interface,
                    )
                )
                self._prepare_local_implementation_start_time = rospy.Time.now()

                self._internal_state = self.EXECUTE_LOCAL

            else:
                self.loginfo(
                    "Executing capability remotely on"
                    f" {request_capability_execution_response.remote_mission_controller_topic}"
                )
                self._internal_state = self.EXECUTE_REMOTE
                self._remote_mission_control_topic = (
                    f"{request_capability_execution_response.remote_mission_controller_topic}"
                    f"/mission_control"
                )
            return NodeMsg.ASSIGNED
        return NodeMsg.FAILED

    def _tick_assigned(self) -> str:
        """
        Performs ticking operation while the node is in the assigned state.
        :return: The new node status.
        """
        if self._internal_state == self.EXECUTE_LOCAL:
            return self._tick_prepare_local_execution()

        if self._internal_state == self.EXECUTE_REMOTE:
            return self._tick_prepare_remote_execution()

        return NodeMsg.BROKEN

    def _setup_local_implementation_tree(self, tree: Tree):
        """
        Function to run in a separate thread used to set up the local implementation tree.
        :param tree: The tree to set up.
        :return: None
        """
        with self._capability_lock:
            self._setup_local_implementation_tree_status = NodeMsg.RUNNING

            load_tree_response = self.tree_manager.load_tree(
                request=LoadTreeRequest(tree=tree), prefix=self.name + "_"
            )

            if not load_tree_response.success:
                self.logerr(f"Error loading capability tree: {load_tree_response}")
                self._setup_local_implementation_tree_status = NodeMsg.FAILED
                return

            set_capability_io_bridge_id(
                tree_manager=self.tree_manager,
                interface=self.capability_interface,
                io_bridge_id=self._io_bridge_id,
            )

            try:
                self._local_implementation_tree_root = self.tree_manager.find_root()
            except TreeTopologyError:
                self.logerr("Failed to get root of implementation tree!")
                self._setup_local_implementation_tree_status = NodeMsg.FAILED
                return

            self._local_implementation_tree_root.shutdown()
            self._local_implementation_tree_root.setup()

            if self.debug_manager and self.debug_manager.get_publish_subtrees():
                self.debug_manager.add_subtree_info(
                    self.name, self.tree_manager.to_msg()
                )

            self._setup_local_implementation_tree_status = NodeMsg.SUCCESS
            return

    def _tick_prepare_local_execution(self) -> str:
        """
        Perform tick operation while the node is prepared for local capability execution.
        :return: The status of the node while the preparation is running.
        """

        # pylint: disable=too-many-return-statements

        if self._prepare_local_tree_thread is not None:
            if self._setup_local_implementation_tree_status is NodeMsg.RUNNING:
                return NodeMsg.ASSIGNED

            if self._setup_local_implementation_tree_status in [
                NodeMsg.IDLE,
                NodeMsg.FAILED,
            ]:
                return NodeMsg.FAILED

            self._capability_execution_status_publisher.publish(
                CapabilityExecutionStatus(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=CapabilityExecutionStatus.EXECUTING,
                )
            )
            return NodeMsg.RUNNING
        try:
            prepare_local_implementation_response = self.__handle_async_service_call(
                self._prepare_local_implementation_service_proxy
            )
        except BehaviorTreeException as exc:
            self.logerr(f"Failed to receive prepared tree from mission control: {exc}")
            return NodeMsg.FAILED

        if prepare_local_implementation_response is None:
            if (
                rospy.Time.now() - self._prepare_local_implementation_start_time
                > rospy.Duration(secs=10)
            ):
                self.logerr(
                    "Timeout when waiting for local implementation to be prepared!"
                )
                return NodeMsg.FAILED
            return NodeMsg.ASSIGNED

        if not prepare_local_implementation_response.success:
            error_msg = (
                "Prepare local implementation request failed: "
                f"{prepare_local_implementation_response.error_message}"
            )
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

        self._prepare_local_tree_thread = threading.Thread(
            target=self._setup_local_implementation_tree,
            args=(prepare_local_implementation_response.implementation_subtree,),
        )
        self._prepare_local_tree_thread.start()

        return NodeMsg.ASSIGNED

    def _tick_prepare_remote_execution(self) -> str:
        """
        Ticks the node while remote execution is prepared.
        :return: the new status after the node was prepared.
        """
        if self._execute_capability_remote_client is None:
            action_name = rospy.resolve_name(
                f"{self._remote_mission_control_topic}/execute_remote_capability"
            )
            # self.logdebug(f"Running on: {self._remote_mission_control_topic} {action_name}")
            self._execute_capability_remote_client = SimpleActionClient(
                action_name, ExecuteRemoteCapabilityAction
            )

        if self._execute_capability_remote_client_connection_tries > 10:
            self._cleanup()

            raise BehaviorTreeException(
                f'Action server "{self._remote_mission_control_topic}/execute_remote_capability"'
                " not available after trying 10 times."
            )

        if not self._execute_capability_remote_client.wait_for_server(
            timeout=rospy.Duration(nsecs=10)
        ):
            self._execute_capability_remote_client_connection_tries += 1
            return NodeMsg.ASSIGNED

        self._execute_capability_remote_client.send_goal(
            ExecuteRemoteCapabilityGoal(
                interface=self.capability_interface,
                implementation_name=self._implementation_name,
                node_id=self._io_bridge_id,
            )
        )
        with self._capability_lock:
            self._last_ping_timestamp = rospy.Time.now()

        self._capability_execution_status_publisher.publish(
            CapabilityExecutionStatus(
                interface=self.capability_interface,
                node_name=self.name,
                status=CapabilityExecutionStatus.EXECUTING,
            )
        )

        return NodeMsg.RUNNING

    def __tick_executing_local(self) -> str:
        """
        Performs the ticking operation if the capability is executed locally.
        It retrieves the local implementation and performs the ticking operations.

        :raise BehaviorTreeException: If the implementation cannot be received
        or nodes cannot be instantiated.
        :return: The statue of the node from NodeMsg.
        """
        # No current implementation is present, one needs to be received from
        # the mission controller.

        new_state = self._local_implementation_tree_root.tick()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.debug_manager.add_subtree_info(self.name, self.tree_manager.to_msg())

        if new_state in [NodeMsg.ASSIGNED, NodeMsg.UNASSIGNED]:
            return NodeMsg.RUNNING

        if new_state in [NodeMsg.SUCCEEDED, NodeMsg.FAILED]:
            if new_state == NodeMsg.SUCCEEDED:
                status = CapabilityExecutionStatus.SUCCEEDED
            else:
                status = CapabilityExecutionStatus.FAILED
            self._capability_execution_status_publisher.publish(
                CapabilityExecutionStatus(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=status,
                )
            )
            self._local_implementation_tree_root.shutdown()
            self._shutdown_local_implementation_tree()
            # Not sure why this would be needed,
            # as reset should be called which performs this action too.
            # self._cleanup()
        return new_state

    def __tick_executing_remote(self) -> str:
        """
        Performs the ticking operation when operating on a remote node.

        :return: The status of the node as a NodeMsg status.
        """

        # Check if the action ran into a timeout.
        if rospy.Time.now() - self._last_ping_timestamp > rospy.Duration(
            self.options["execution_timeout_sec"]
        ):
            self.logerr(
                "ExecuteRemoteCapability action timed out after"
                f' {self.options["execution_timeout_sec"]} seconds'
            )
            self._execute_capability_remote_client.cancel_goal()
            self._cleanup()
            return NodeMsg.FAILED

        try:
            execute_remote_implementation_result: Optional[
                ExecuteRemoteCapabilityResult
            ] = self._execute_action(self._execute_capability_remote_client)
        except BehaviorTreeException as exc:
            self.logerr(f"Failed to execute action: {exc}")
            self._cleanup()
            return NodeMsg.FAILED
        if execute_remote_implementation_result is None:
            return NodeMsg.RUNNING

        if not execute_remote_implementation_result.success:
            if execute_remote_implementation_result.no_executor_available:
                self.logerr("No executor available at the moment! Try again later!")
                self._cleanup()
                return NodeMsg.UNASSIGNED

            self.logerr(
                f"Failed to execute action: {execute_remote_implementation_result.error_message}"
            )
            self._cleanup()
            return NodeMsg.FAILED

        return NodeMsg.SUCCEEDED

    def _tick_running(self) -> str:
        """
        Performs ticking operations for capabilities during the NodeMsg.RUNNING status.
        :return: Either NodeMsg.RUNNING, NodeMsg.SUCCESS or NodeMsg.FAILED
        """
        new_state = NodeMsg.RUNNING

        if self._internal_state == self.WAITING_FOR_OUTPUT_VALUES:
            if self._last_received_outputs is not None:
                for bridge_data in self._last_received_outputs.bridge_data:
                    node_data: NodeDataMsg = bridge_data

                    try:
                        self.outputs[node_data.key] = json_decode(
                            node_data.serialized_value
                        )
                    except TypeError as exc:
                        self.logwarn(f"Could not set output {node_data.key}: {exc}")
                        continue
                return self._result_status

            if rospy.Time.now() - self._result_timestamp > rospy.Duration(
                secs=self.wait_for_outputs_timeout_sec
            ):
                self.logwarn(
                    "Timed out while waiting for results! Check if the implementation contains a"
                    " validOutputDataBridge!"
                )
                return self._result_status

        if self._internal_state == self.EXECUTE_LOCAL:
            new_state = self.__tick_executing_local()

        if self._internal_state == self.EXECUTE_REMOTE:
            new_state = self.__tick_executing_remote()

        if new_state in [NodeMsg.FAILED, NodeMsg.SUCCEEDED]:
            self._result_status = new_state
            self._internal_state = self.WAITING_FOR_OUTPUT_VALUES
            self._result_timestamp = rospy.Time.now()
            new_state = NodeMsg.RUNNING

        return new_state

    def _do_tick(self):
        if self._io_publisher is not None:
            input_data = []
            for key in self.inputs:
                input_data_msg = NodeDataMsg()
                input_data_msg.key = key
                input_data_msg.serialized_value = self.inputs.get_serialized(key=key)
                input_data_msg.serialized_type = self.inputs.get_serialized_type(
                    key=key
                )
                input_data.append(input_data_msg)
            try:
                self._io_publisher.publish(
                    CapabilityIOBridgeData(
                        node_id=self._io_bridge_id,
                        type=CapabilityIOBridgeData.INPUT,
                        bridge_data=input_data,
                        timestamp=rospy.Time.now(),
                    )
                )
            except ROSSerializationException as exc:
                error_msg = f"Could not serialize inputs to publish: {exc}"
                self.logerr(error_msg)
                raise BehaviorTreeException(error_msg) from exc
            except ROSException as exc:
                error_msg = f"Input data bridge publisher could not publish: {exc}"
                self.logerr(error_msg)
                raise BehaviorTreeException(error_msg) from exc

        if self.simulate_tick:
            # self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if self.state == NodeMsg.PAUSED:
            self.state = self._old_state

        if self.state == NodeMsg.IDLE:
            self._capability_execution_status_publisher.publish(
                CapabilityExecutionStatus(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=CapabilityExecutionStatus.IDLE,
                )
            )
            return NodeMsg.UNASSIGNED

        if self.state == NodeMsg.UNASSIGNED:
            return self._tick_unassigned()

        if self.state == NodeMsg.ASSIGNED:
            return self._tick_assigned()

        if self.state == NodeMsg.RUNNING:
            return self._tick_running()

        # TODO: Fix this issue!
        return NodeMsg.FAILED

    def _do_untick(self):
        if self.state == NodeMsg.PAUSED:
            return NodeMsg.PAUSED

        self._old_state = self.state

        if self.state in [NodeMsg.IDLE, NodeMsg.SUCCEEDED, NodeMsg.FAILED]:
            return NodeMsg.PAUSED

        if self.state == NodeMsg.UNASSIGNED:
            self._internal_state = self.IDLE
            if self._request_capability_execution_service_proxy is not None:
                self._request_capability_execution_service_proxy.stop_call()

        if self.state == NodeMsg.ASSIGNED:
            if self._internal_state is self.EXECUTE_LOCAL:
                self._prepare_local_implementation_service_proxy.stop_call()

            if self._internal_state is self.EXECUTE_REMOTE:
                if self._execute_capability_remote_client is not None:
                    self._execute_capability_remote_client.cancel_goal()

        if not self._shutdown_local_implementation_tree():
            return NodeMsg.FAILED

        return NodeMsg.PAUSED

    def _do_reset(self):
        self._stop_calls_action_clients_async_service_clients()
        self._shutdown_local_implementation_tree()
        self._cleanup()

        return NodeMsg.IDLE

    def _do_shutdown(self):
        if self._capability_execution_status_publisher is not None:
            self._capability_execution_status_publisher.publish(
                CapabilityExecutionStatus(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=CapabilityExecutionStatus.SHUTDOWN,
                )
            )
        self._shutdown_local_implementation_tree()
        self._unregister_io_bridge_publishers_subscribers()

        if self._ping_subscriber is not None:
            self._ping_subscriber.unregister()
            del self._ping_subscriber

        self._stop_calls_action_clients_async_service_clients()

        self._request_capability_execution_service_proxy.shutdown()
        self._prepare_local_implementation_service_proxy.shutdown()

        self._cleanup()

        self._ping_subscriber = None
        self._io_subscriber = None
        self._io_publisher = None

        self._request_capability_execution_service_proxy = None
        self._prepare_local_implementation_service_proxy = None
        self._capability_execution_status_publisher = None

    # Cleanup operations

    def _unregister_io_bridge_publishers_subscribers(self):
        """
        Helper method that unregisters all the io bridge publishers if not none.
        :return: None
        """
        if self._io_subscriber is not None:
            self._io_subscriber.unregister()
            del self._io_subscriber

        if self._io_publisher is not None:
            self._io_publisher.unregister()
            del self._io_publisher

    def _stop_calls_action_clients_async_service_clients(self):
        """
        Shuts down the running goals of action clients and Async Service clients.
        :return: None
        """
        if self._request_capability_execution_service_proxy is not None:
            self._request_capability_execution_service_proxy.stop_call()

        if self._prepare_local_implementation_service_proxy is not None:
            self._prepare_local_implementation_service_proxy.stop_call()

        if (
            self._execute_capability_remote_client is not None
            and self._execute_capability_remote_client.get_state()
            in [
                GoalStatus.ACTIVE,
                GoalStatus.PREEMPTING,
            ]
        ):
            self._execute_capability_remote_client.cancel_goal()

    def _shutdown_local_implementation_tree(self) -> bool:
        """
        Shutdown the local implementation tree.

        :return: True if the tree could be shutdown, False otherwise.
        """
        if self._local_implementation_tree is not None:
            with self._capability_lock:
                self._local_implementation_tree_root.shutdown()

                response = self.tree_manager.control_execution(
                    ControlTreeExecutionRequest(
                        command=ControlTreeExecutionRequest.SHUTDOWN
                    )
                )
                if not response.success:
                    self.logwarn(
                        f"Could not shutdown local capability: {response.error_message}"
                    )
                    return False

        return True

    def _clear_local_implementation_tree(self) -> bool:
        """
        Helper method that clears the local tree.
        :return: True if successful, false otherwise.
        """
        if self._local_implementation_tree is not None:
            with self._capability_lock:
                response = self.tree_manager.clear(ClearTreeRequest())
                if not response.success:
                    self.logwarn(
                        f"Could not clear local capability tree: {response.error_message}"
                    )
                    return False
        return True

    def _cleanup(self):
        """
        Cleans up the internal states of the capability and resets them to the default values.
        :return: None
        """
        # Wait for prepare local tree thread to finish
        with self._capability_lock:
            if self._prepare_local_tree_thread is not None:
                self._prepare_local_tree_thread.join()
            self._prepare_local_tree_thread: Optional[threading.Thread] = None

            self._internal_state = self.IDLE

            self._implementation_name: Optional[str] = None
            self._result_status = None
            self._result_timestamp = None

            # Variables for remote trees
            self._remote_mission_control_topic: Optional[str] = None
            self._execute_capability_remote_client_connection_tries: int = 0

            # Variables for local trees
            self._clear_local_implementation_tree()

            self._local_implementation_tree: Optional[Tree] = None
            self._local_implementation_tree_root: Optional[Node] = None
            self._setup_local_implementation_tree_status = NodeMsg.IDLE

            del self._execute_capability_remote_client
            self._execute_capability_remote_client = None


def capability_node_class_from_capability_interface(
    capability_interface: CapabilityInterface, mission_control_topic: str
) -> Type[Capability]:
    """
    Uses a capability interface in combination with a mission control topic to create a new class
    definition for the defined interface.
    The new class is a subclass to the Capability class.

    This function uses the python type(...) metaclass to create a new subclass for the provided
    interface definition.
    This class is then decorated with the define_bt_node
    decorator to create a valid node class definition.

    All created classes are placed in the `ros_bt_py.capability` module.
    Their names are currently used as identifier.

    :param capability_interface: The capability interface to use.
    :param mission_control_topic: The topic the local mission controller is running on.
    :return: Class definition for the capability that can be instanced like any other node.
    """
    inputs_dict = {}
    for input_node_data in capability_interface.inputs:
        inputs_dict[input_node_data.key] = json_decode(input_node_data.serialized_type)

    outputs_dict = {}
    for output_node_data in capability_interface.outputs:
        outputs_dict[output_node_data.key] = json_decode(
            output_node_data.serialized_type
        )

    capability_node_class = type(
        capability_interface.name,
        (Capability,),
        {
            "__capability_interface": capability_interface,
            "__local_mc_topic": mission_control_topic,
            "__module__": Capability.__module__,
        },
    )

    return define_bt_node(
        NodeConfig(
            options={
                "require_local_execution": bool,
                "execution_timeout_sec": float,
                "inputs_names_for_implementation_tags": list,
            },
            inputs=inputs_dict,
            outputs=outputs_dict,
            max_children=0,
        )
    )(capability_node_class)


def capability_input_data_bridge_from_interface(
    capability_interface: CapabilityInterface,
) -> Type[CapabilityInputDataBridge]:
    """
    Create an in memory version of a CapabilityInputDataBridge from a CapabilityInterface.

    :param capability_interface: The capability interface to create a representation for.
    :return: The class definition for a capability input bridge
    """
    inputs_dict = {}
    for input_node_data in capability_interface.inputs:
        inputs_dict[input_node_data.key] = json_decode(input_node_data.serialized_type)

    capability_input_bridge_node_class = type(
        f"{capability_interface.name}_InputDataBridge",
        (CapabilityInputDataBridge,),
        {
            "__capability_interface": capability_interface,
            "__module__": f"{CapabilityInputDataBridge.__module__}.bridge",
        },
    )

    return define_bt_node(
        NodeConfig(options={}, inputs={}, outputs=inputs_dict, max_children=0)
    )(capability_input_bridge_node_class)


def capability_output_data_bridge_from_interface(
    capability_interface: CapabilityInterface,
) -> Type[CapabilityOutputDataBridge]:
    """
    Creates an in memory representation of a CapabilityOutputDataBridge.

    :param capability_interface: The capability interface to generate the class from.
    :return: The type definition of the capability output data bridge.
    """

    outputs_dict = {}
    for output_node_data in capability_interface.outputs:
        outputs_dict[output_node_data.key] = json_decode(
            output_node_data.serialized_type
        )

    capability_output_bridge_node_class = type(
        f"{capability_interface.name}_OutputDataBridge",
        (CapabilityOutputDataBridge,),
        {
            "__capability_interface": capability_interface,
            "__module__": f"{CapabilityOutputDataBridge.__module__}.bridge",
        },
    )

    return define_bt_node(
        NodeConfig(options={}, inputs=outputs_dict, outputs={}, max_children=0)
    )(capability_output_bridge_node_class)


def capability_node_class_from_capability_interface_callback(msg: Time, args: List):
    # pylint: disable=unused-argument
    """
    Rospy service callback used to register capability interfaces with the local Node class.

    :param msg: The capability interface to register.
    :param args: The additional arguments passed to the callback.
    :return: Nothing
    """

    if len(args) < 2:
        rospy.logerr(
            "Create capability node callback received less then one extra args."
        )
        return
    if not isinstance(args[0], str):
        rospy.logerr("First args is not a string!")
        return

    if not isinstance(args[1], rospy.ServiceProxy):
        rospy.logerr("Second args is not a server proxy!")
        return

    rospy.loginfo(f"Received new capability information at {msg}")

    service_proxy: rospy.ServiceProxy = args[1]
    response: GetCapabilityInterfacesResponse = service_proxy.call(
        GetCapabilityInterfacesRequest()
    )
    if not response.success:
        rospy.logwarn(f"Failed to get capability interfaces: {response.error_message}!")
    for interface in response.interfaces:
        capability_node_class_from_capability_interface(interface, args[0])
        capability_input_data_bridge_from_interface(interface)
        capability_output_data_bridge_from_interface(interface)


def set_capability_io_bridge_id(
    tree_manager: TreeManager, interface: CapabilityInterface, io_bridge_id: str
) -> None:
    """
    Set the input and output bridge topic id to a specific value.

    :param tree_manager: The tree manager where the tree is stored
    :param interface: The interface for which the input and output bridge values should be set.
    :param io_bridge_id: The unique id that should be used for communication.
    :return: None
    """
    for node in tree_manager.nodes.values():
        if (
            hasattr(node, "__capability_interface")
            and getattr(node, "__capability_interface") == interface
        ):
            if isinstance(node, CapabilityInputDataBridge):
                node.capability_bridge_id = io_bridge_id
                continue
            if isinstance(node, CapabilityOutputDataBridge):
                node.capability_bridge_id = io_bridge_id
