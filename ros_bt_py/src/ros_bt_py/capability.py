#  -------- BEGIN LICENSE BLOCK --------
#  Copyright 2022 FZI Forschungszentrum Informatik
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the {copyright_holder} nor the names of its
#        contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
"""
Module for handling BT nodes related to capabilities.
It defines the base ABC for Capability nodes and functions to create class definitions for capability interfaces
at runtime.
"""
# pylint: disable=no-name-in-module,import-error,too-many-lines

import secrets
import threading
from abc import ABC
from typing import Optional, List, Type

import rospy
from std_msgs.msg import Time
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from ros_bt_py_msgs.msg import (
    CapabilityInterface, Node as NodeMsg, Tree, ExecuteRemoteCapabilityAction,
    ExecuteRemoteCapabilityGoal, CapabilityIOBridgeData,
)
from ros_bt_py_msgs.msg import NodeData as NodeDataMsg
from ros_bt_py_msgs.srv import (
    PrepareLocalImplementation, PrepareLocalImplementationRequest,
    LoadTreeRequest, ClearTreeRequest, ControlTreeExecutionRequest, GetCapabilityInterfacesRequest,
    GetCapabilityInterfacesResponse, ControlTreeExecutionResponse, RequestCapabilityExecution,
    RequestCapabilityExecutionRequest, RequestCapabilityExecutionResponse, NotifyCapabilityExecutionStatus,
    NotifyCapabilityExecutionStatusRequest,
)
from rospy import ROSSerializationException, ROSException, ServiceProxy

from ros_bt_py.exceptions import BehaviorTreeException, TreeTopologyError
from ros_bt_py.helpers import json_decode
from ros_bt_py.node import define_bt_node, Leaf, Node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.ros_helpers import AsyncServiceProxy
from ros_bt_py.tree_manager import TreeManager

WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS = 2
WAIT_FOR_IMPLEMENTATION_ASSIGNMENT_TIMEOUT_SECONDS = 2

WAIT_FOR_REMOTE_MISSION_CONTROL_TRIES = 3
WAIT_FOR_REMOTE_EXECUTION_TIMEOUT_SECONDS = 30
WAIT_FOR_OUTPUTS_TIMEOUT_SECONDS = 1


class CapabilityDataBridge(ABC, Leaf):
    """
    Data bridge that allows the exchange of inputs and outputs between capability interfaces and nodes.
    """

    def __init__(
            self,
            debug_manager=None,
            name=None,
            options=None
    ):
        """
        Creates a new capability data bridge node.

        :param debug_manager: The debug manager to use.
        :param name: The name of the node.
        :param options: The node options.
        """
        super().__init__(options=options, debug_manager=debug_manager, name=name)
        self._source_capability_bridge_topic: Optional[str] = None

    @property
    def capability_bridge_topic(self) -> str:
        """
        The topic to use for communication with the capability.
        :return: string containing the topic
        """
        return self._source_capability_bridge_topic

    @capability_bridge_topic.setter
    def capability_bridge_topic(self, new_topic: str):
        """
        Sets the topic used for communication with the capability.
        :param new_topic: The new topic to use.
        :return: None
        """
        self._source_capability_bridge_topic = new_topic


class CapabilityInputDataBridge(CapabilityDataBridge):
    """
    CapabilityDataBridge that transfers the inputs of the interface as an output to the implementation.
    """

    def __init__(
            self,
            debug_manager=None,
            name=None,
            options=None
    ):
        super().__init__(options=options, debug_manager=debug_manager, name=name)
        self._source_capability_inputs_subscriber: Optional[rospy.Subscriber] = None
        self._current_received_msg: Optional[CapabilityIOBridgeData] = None
        self._lock = threading.RLock()

    def _publish_inputs_cb(self, msg: CapabilityIOBridgeData):
        """
        Callback to store the received input message internally to be processed on the next tick.
        :param msg: The received message.
        :return: None
        """
        with self._lock:
            self._current_received_msg = msg

    def _do_setup(self):
        if self.capability_bridge_topic is None:
            raise BehaviorTreeException(f"{self.name}: No input topic set!")
        self._source_capability_inputs_subscriber = rospy.Subscriber(
            self.capability_bridge_topic,
            CapabilityIOBridgeData,
            callback=self._publish_inputs_cb
        )

    def _do_tick(self):
        if self.state == NodeMsg.IDLE:
            return NodeMsg.RUNNING

        if self.state == NodeMsg.RUNNING:
            with self._lock:
                if self._current_received_msg is not None:
                    for bridge_data in self._current_received_msg.bridge_data:
                        node_data: NodeDataMsg = bridge_data
                        try:
                            self.outputs[node_data.key] = json_decode(node_data.serialized_value)
                        except TypeError as exc:
                            self.logwarn(f"Could not set output {node_data.key}: {exc}")
                            continue

                    return NodeMsg.SUCCEEDED
            return NodeMsg.RUNNING
        return self.state

    def _do_untick(self):
        self._current_received_msg = None
        return NodeMsg.IDLE

    def _do_reset(self):
        self._current_received_msg = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self._current_received_msg = None
        if self._source_capability_inputs_subscriber is not None:
            self._source_capability_inputs_subscriber.unregister()
            self._source_capability_inputs_subscriber = None


class CapabilityOutputDataBridge(CapabilityDataBridge):
    """
    CapabilityDataBridge that transfers the outputs of the implementation as an output to the interface.
    """

    def __init__(
            self,
            debug_manager=None,
            name=None,
            options=None
    ):
        super().__init__(options=options, debug_manager=debug_manager, name=name)
        self._source_capability_outputs_publisher: Optional[rospy.Publisher] = None

    def _do_setup(self):
        if self.capability_bridge_topic is None:
            raise BehaviorTreeException(f"{self.name}: No input topic set!")

        self._source_capability_outputs_publisher = rospy.Publisher(
            self.capability_bridge_topic,
            CapabilityIOBridgeData,
            queue_size=1000
        )
        print(f"SETUP {self.name} - {self.state}")

    def _do_tick(self):
        print(f"TICK {self.name} - {self.state}")
        if self.state == NodeMsg.IDLE:
            return NodeMsg.RUNNING
        if self.state == NodeMsg.RUNNING:
            if self._source_capability_outputs_publisher is not None:

                outputs = []
                for key in self.inputs:
                    output = NodeDataMsg()
                    output.key = key
                    output.serialized_value = self.inputs.get_serialized(key=key)
                    output.serialized_type = self.inputs.get_serialized_type(key=key)
                    outputs.append(output)

                self._source_capability_outputs_publisher.publish(
                    CapabilityIOBridgeData(
                        bridge_data=outputs,
                        timestamp=rospy.Time.now()
                    )
                )
                return NodeMsg.SUCCEEDED
            return NodeMsg.FAILED
        return NodeMsg.FAILED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        if self._source_capability_outputs_publisher is not None:
            self._source_capability_outputs_publisher.unregister()
            self._source_capability_outputs_publisher = None


class Capability(ABC, Leaf):
    """
    ABC for Capability nodes that implements the necessary logic for capabilities to function.

    The class manages the lifecycle of capabilities and performs the necessary communications to allow the
    capability to function.
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
            debug_manager=None,
            name=None,
            options=None
    ):
        super().__init__(options=options, debug_manager=debug_manager, name=name)

        if hasattr(self, '__capability_interface'):
            self.capability_interface = getattr(self, '__capability_interface')
        else:
            rospy.logwarn("Created Capability without interface! This is not intended!")
            raise BehaviorTreeException("Created Capability without interface! This is not intended!")

        if hasattr(self, '__local_mc_topic'):
            self.local_mc_topic = getattr(self, '__local_mc_topic')
        else:
            rospy.logwarn("Created Capability without interface! This is not intended!")
            raise BehaviorTreeException("Created Capability without interface! This is not intended!")

        self._internal_state = self.IDLE
        """Internal state used to track the execution progress of the capability."""

        self.tree_manager = TreeManager(
            name=name,
            publish_tree_callback=self.nop,
            publish_debug_info_callback=self.nop,
            publish_debug_settings_callback=self.nop,
            publish_node_diagnostics_callback=self.nop,
            debug_manager=self.debug_manager
        )
        """Tree manager used to run the local capability implementations."""

        self._old_state = self.state
        """Backup of the node state before the node enters the PAUSED state"""

        self._capability_lock = threading.RLock()
        """Lock used to avoid race conditions when setting up local implementations in a tree."""

        self._implementation_name: Optional[str] = None

        # Action and service clients
        self._request_capability_execution_service_proxy: Optional[AsyncServiceProxy] = None
        self._prepare_local_implementation_service_proxy: Optional[AsyncServiceProxy] = None
        self._execute_capability_remote_client: Optional[SimpleActionClient] = None
        self._notify_capability_execution_status_client: Optional[ServiceProxy] = None

        # Input and output data bridge information
        self._input_bridge_topic: Optional[str] = None
        self._input_bridge_publisher: Optional[rospy.Publisher] = None
        self._output_bridge_topic: Optional[str] = None
        self._output_bridge_subscriber: Optional[rospy.Subscriber] = None
        self._last_received_outputs: Optional[CapabilityIOBridgeData] = None

        # Result variables used to track if the implementation is finished
        self._result_status: Optional[str] = None
        self._result_timestamp: Optional[rospy.Time] = None

        # Variables for remote trees
        self._remote_mission_control_topic: Optional[str] = None
        self._execute_capability_remote_client_connection_tries: int = 0
        self._execute_capability_remote_client_start_time: Optional[rospy.Time] = None

        # Variables for local trees
        self._prepare_local_tree_thread: Optional[threading.Thread] = None
        self._prepare_local_implementation_start_time: Optional[rospy.Time] = None

        self._local_implementation_tree: Optional[Tree] = None
        self._local_implementation_tree_root: Optional[Node] = None
        self._local_implementation_tree_status: str = NodeMsg.IDLE

    @staticmethod
    def nop(msg):
        """no op "publisher" to stop tree_manager from spamming complaints for not-set publishers
        """

    @staticmethod
    def __handle_async_service_call(service_proxy: AsyncServiceProxy):
        """
        Helper method to manage a call to a service via the AsyncServiceProxy.
        The call method needs to be called beforehand, this method just handles the possible states until the result
        is received.
        It checks the states the service_proxy can be in and returns results depending on the current execution state.
        This method in nonblocking.

        :param service_proxy: The service proxy the call has been placed for.

        :raises BehaviorTreeException: Should the call to the AsyncServiceProxy be aborted or not placed before calling
        this method a BehaviorTreeException will be raised.
        :return: The response of the call if available, otherwise None.
        """
        state = service_proxy.get_state()

        if state in [AsyncServiceProxy.IDLE, AsyncServiceProxy.SERVICE_AVAILABLE]:
            raise BehaviorTreeException(
                f'Service is still idle'
                f' (state ID: {state})'
            )

        if state is AsyncServiceProxy.TIMEOUT:
            raise BehaviorTreeException(
                f'Service call timed out before succeeding'
                f' (state ID: {state})'
            )

        if state is AsyncServiceProxy.ABORTED:
            raise BehaviorTreeException(
                f'Service call was aborted before succeeding'
                f' (state ID: {state})'
            )

        if state is AsyncServiceProxy.ERROR:
            raise BehaviorTreeException(
                f'Service call returned a error before succeeding'
                f' (state ID: {state})'
            )

        if state is AsyncServiceProxy.RUNNING:
            return None

        if state is AsyncServiceProxy.RESPONSE_READY:
            response = service_proxy.get_response()
            if response.success:
                return response
            raise BehaviorTreeException(
                f'Service call failed {response.error_message})'
            )
        raise RuntimeError(
            f'Async proxy in invalid state {state})'
        )

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
                f'Action ended without succeeding'
                f' (state ID: {action_state})'
            )

        if action_state in [GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
            raise BehaviorTreeException(
                f'Action was willingly canceled before succeeding'
                f' (state ID: {action_state})'
            )

        if action_state == GoalStatus.SUCCEEDED:
            action_result = action_client.get_result()

            if not action_result.success:
                raise BehaviorTreeException(
                    f'Action did not finish successfully:'
                    f' "{action_result.error_message}"'
                )
            return action_result

        return None

    def _publish_outputs_cb(self, msg: CapabilityIOBridgeData):
        """
        Callback to store the received output values locally and allow them to be processed in the next tick.
        :param msg: The received output values.
        :return:None
        """
        with self._capability_lock:
            self._last_received_outputs = msg

    def _do_setup(self):
        try:
            rospy.wait_for_service(
                f'{self.local_mc_topic}/prepare_local_implementation',
                timeout=rospy.Duration(secs=WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS)
            )
        except rospy.ROSException as exc:
            raise BehaviorTreeException(
                f'Service "{self.local_mc_topic}/prepare_local_implementation" not available'
                f' after waiting f{WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS} seconds!'
            ) from exc

        self._prepare_local_implementation_service_proxy = AsyncServiceProxy(
            f'{self.local_mc_topic}/prepare_local_implementation',
            PrepareLocalImplementation
        )

        # Create an action client for FindBestExecutor
        self._request_capability_execution_service_proxy = AsyncServiceProxy(
            f'{self.local_mc_topic}/execute_capability',
            RequestCapabilityExecution
        )

        try:
            self._request_capability_execution_service_proxy.wait_for_service(
                timeout=rospy.Duration(secs=WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS)
            )
        except rospy.ROSException as exc:
            raise BehaviorTreeException(
                f'Service "{self.local_mc_topic}/execute_capability" not available'
                f' after waiting f{WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS} seconds!'
            ) from exc

        capability_topic_prefix = rospy.resolve_name(f"~/capabilities/{self.name}_{secrets.randbelow(100000)}")

        self._input_bridge_topic = f"{capability_topic_prefix}/inputs"
        self._output_bridge_topic = f"{capability_topic_prefix}/outputs"

        self._input_bridge_publisher = rospy.Publisher(
            self._input_bridge_topic,
            CapabilityIOBridgeData,
            queue_size=1,
            latch=True
        )
        self._output_bridge_subscriber = rospy.Subscriber(
            self._output_bridge_topic,
            CapabilityIOBridgeData,
            callback=self._publish_outputs_cb
        )

        notify_capability_execution_status_topic = rospy.resolve_name(
            f"{self.local_mc_topic}/notify_capability_execution_status"
        )

        self._notify_capability_execution_status_client = ServiceProxy(
            notify_capability_execution_status_topic,
            NotifyCapabilityExecutionStatus,
            persistent=False
        )
        try:
            self._notify_capability_execution_status_client.wait_for_service(
                timeout=rospy.Duration(secs=WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS)
            )
        except rospy.ROSException as exc:
            raise BehaviorTreeException(
                f'Service "{self.local_mc_topic}/notify_capability_execution_status" not available'
                f' after waiting f{WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS} seconds!'
            ) from exc

    def _tick_unassigned(self) -> str:
        """
        Function performing the tick operation while in the UNASSIGNED state.
        :return: The new state as a NodeMsg constant.
        """
        # pylint: disable=too-many-return-statements
        if self._internal_state == self.IDLE:
            self._request_capability_execution_service_proxy.call_service(
                RequestCapabilityExecutionRequest(
                    capability=self.capability_interface
                )
            )
            self._internal_state = self.WAITING_FOR_ASSIGNMENT

        if self._internal_state == self.WAITING_FOR_ASSIGNMENT:
            try:
                request_capability_execution_response: Optional[RequestCapabilityExecutionResponse] = \
                    self.__handle_async_service_call(self._request_capability_execution_service_proxy)
            except BehaviorTreeException as exc:
                self.logerr(f"Failed to request execute capability service, no implementation will be selected: {exc}")
                self._cleanup()
                return NodeMsg.FAILED

            if request_capability_execution_response is None:
                return NodeMsg.UNASSIGNED

            self._implementation_name = request_capability_execution_response.implementation_name

            if request_capability_execution_response.execute_local:
                self.loginfo("Executing capability locally!")

                self._prepare_local_implementation_service_proxy.call_service(
                    req=PrepareLocalImplementationRequest(
                        implementation_name=self._implementation_name,
                        interface=self.capability_interface
                    )
                )
                self._prepare_local_implementation_start_time = rospy.Time.now()

                self._internal_state = self.EXECUTE_LOCAL

            else:
                self.loginfo(
                    f"Executing capability remotely on"
                    f" {request_capability_execution_response.remote_mission_controller_topic}"
                )
                self._internal_state = self.EXECUTE_REMOTE
                self._remote_mission_control_topic = \
                    request_capability_execution_response.remote_mission_controller_topic
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
            self._local_implementation_tree_status = NodeMsg.RUNNING

            load_tree_response = self.tree_manager.load_tree(
                request=LoadTreeRequest(tree=tree),
                prefix=self.name + "_"
            )

            if not load_tree_response.success:
                self.logerr(f"Error loading capability tree: {load_tree_response}")
                self._local_implementation_tree_status = NodeMsg.FAILED
                return

            set_capability_io_bridge_topics(self.tree_manager, self.capability_interface, self._input_bridge_topic, self._output_bridge_topic)

            try:
                self._local_implementation_tree_root = self.tree_manager.find_root()
            except TreeTopologyError:
                self.logerr("Failed to get root of implementation tree!")
                self._local_implementation_tree_status = NodeMsg.FAILED
                return

            self._local_implementation_tree_root.shutdown()
            self._local_implementation_tree_root.setup()

            if self.debug_manager and self.debug_manager.get_publish_subtrees():
                self.debug_manager.add_subtree_info(
                    self.name, self.tree_manager.to_msg()
                )

            self._local_implementation_tree_status = NodeMsg.SUCCESS
            return

    def _tick_prepare_local_execution(self) -> str:
        """
        Perform tick operation while the node is prepared for local capability execution.
        :return: The status of the node while the preparation is running.
        """

        if self._prepare_local_tree_thread is not None:
            if self._local_implementation_tree_status is NodeMsg.RUNNING:
                return NodeMsg.ASSIGNED

            if self._local_implementation_tree_status in [NodeMsg.IDLE, NodeMsg.FAILED]:
                return NodeMsg.FAILED

            self._notify_capability_execution_status_client.call(
                NotifyCapabilityExecutionStatusRequest(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=NotifyCapabilityExecutionStatusRequest.EXECUTING
                )
            )
            return NodeMsg.RUNNING
        try:
            prepare_local_implementation_response = \
                self.__handle_async_service_call(self._prepare_local_implementation_service_proxy)
        except BehaviorTreeException as exc:
            self.logerr(f"Failed to receive prepared tree from mission control: {exc}")
            return NodeMsg.FAILED

        if prepare_local_implementation_response is None:
            if rospy.Time.now() - self._prepare_local_implementation_start_time > rospy.Duration(secs=10):
                self.logerr("Timeout when waiting for local implementation to be prepared!")
                return NodeMsg.FAILED

            return NodeMsg.ASSIGNED

        if not prepare_local_implementation_response.success:
            error_msg = f'Prepare local implementation request failed: ' \
                        f'{prepare_local_implementation_response.error_message}'
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

        self._prepare_local_tree_thread = threading.Thread(
            target=self._setup_local_implementation_tree,
            args=(prepare_local_implementation_response.implementation_subtree,)
        )
        self._prepare_local_tree_thread.start()

        return NodeMsg.ASSIGNED

    def _tick_prepare_remote_execution(self) -> str:
        """
        Ticks the node while remote execution is prepared.
        :return: the new status after the node was prepared.
        """
        if self._execute_capability_remote_client is None:
            self._execute_capability_remote_client = SimpleActionClient(
                f'{self._remote_mission_control_topic}/execute_remote_capability',
                ExecuteRemoteCapabilityAction
            )

        if self._execute_capability_remote_client_connection_tries > 10:
            self._cleanup()

            raise BehaviorTreeException(
                f'Action server "{self._remote_mission_control_topic}/execute_remote_capability" not available'
                f' after trying 10 times.'
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
            )
        )
        self._execute_capability_remote_client_start_time = rospy.Time.now()

        self._notify_capability_execution_status_client.call(
            NotifyCapabilityExecutionStatusRequest(
                interface=self.capability_interface,
                node_name=self.name,
                status=NotifyCapabilityExecutionStatusRequest.EXECUTING
            )
        )

        return NodeMsg.RUNNING

    def __tick_executing_local(self) -> str:
        """
        Performs the ticking operation if the capability is executed locally.
        It retrieves the local implementation and performs the ticking operations.

        :raise BehaviorTreeException: If the implementation cannot be received or nodes cannot be instantiated.
        :return: The statue of the node from NodeMsg.
        """
        # No current implementation is present, one needs to be received from the mission controller.

        new_state = self._local_implementation_tree_root.tick()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.debug_manager.add_subtree_info(
                self.name, self.tree_manager.to_msg()
            )

        if new_state in [NodeMsg.ASSIGNED, NodeMsg.UNASSIGNED]:
            return NodeMsg.RUNNING

        if new_state in [NodeMsg.SUCCEEDED, NodeMsg.FAILED]:
            if new_state == NodeMsg.SUCCEEDED:
                status = NotifyCapabilityExecutionStatusRequest.SUCCEEDED
            else:
                status = NotifyCapabilityExecutionStatusRequest.FAILED
            self._notify_capability_execution_status_client.call(
                NotifyCapabilityExecutionStatusRequest(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=status
                )
            )
            self._local_implementation_tree_root.shutdown()
            self._shutdown_local_implementation_tree()
            # Not sure why this would be needed, as reset should be called which performs this action too.
            # self._cleanup()
        return new_state

    def __tick_executing_remote(self) -> str:
        """
        Performs the ticking operation when operating on a remote node.

        :return: The status of the node as a NodeMsg status.
        """

        # Check if the action ran into a timeout.
        if (rospy.Time.now() - self._execute_capability_remote_client_start_time
                > rospy.Duration(WAIT_FOR_REMOTE_EXECUTION_TIMEOUT_SECONDS)):
            self.logerr(
                f'ExecuteRemoteCapability action timed out after'
                f' {WAIT_FOR_REMOTE_EXECUTION_TIMEOUT_SECONDS} seconds'
            )
            self._execute_capability_remote_client.cancel_goal()
            self._cleanup()
            return NodeMsg.FAILED

        try:
            execute_remote_implementation_result = self._execute_action(self._execute_capability_remote_client)
        except BehaviorTreeException as exc:
            self.logerr(f"Failed to execute action: {exc}")
            self._cleanup()
            return NodeMsg.FAILED
        if execute_remote_implementation_result is None:
            return NodeMsg.RUNNING

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
                        self.outputs[node_data.key] = json_decode(node_data.serialized_value)
                    except TypeError as exc:
                        self.logwarn(f"Could not set output {node_data.key}: {exc}")
                        continue
                return self._result_status

            if rospy.Time.now() - self._result_timestamp > rospy.Duration(secs=WAIT_FOR_OUTPUTS_TIMEOUT_SECONDS):
                self.logwarn(
                    "Timed out while waiting for results! Check if the implementation contains a valid"
                    "OutputDataBridge!"
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
        if self._input_bridge_publisher is not None:
            input_data = []
            for key in self.inputs:
                input_data_msg = NodeDataMsg()
                input_data_msg.key = key
                input_data_msg.serialized_value = self.inputs.get_serialized(key=key)
                input_data_msg.serialized_type = self.inputs.get_serialized_type(key=key)
                input_data.append(input_data_msg)
            try:
                self._input_bridge_publisher.publish(
                    CapabilityIOBridgeData(
                        bridge_data=input_data,
                        timestamp=rospy.Time.now()
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

        if self.state == NodeMsg.PAUSED:
            self.state = self._old_state

        if self.state == NodeMsg.IDLE:
            self._notify_capability_execution_status_client.call(
                NotifyCapabilityExecutionStatusRequest(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=NotifyCapabilityExecutionStatusRequest.IDLE
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
        if self._notify_capability_execution_status_client is not None:
            self._notify_capability_execution_status_client.call(
                NotifyCapabilityExecutionStatusRequest(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=NotifyCapabilityExecutionStatusRequest.SHUTDOWN
                )
            )
        self._shutdown_local_implementation_tree()
        self._unregister_io_bridge_publishers_subscribers()
        self._stop_calls_action_clients_async_service_clients()

        self._request_capability_execution_service_proxy.shutdown()
        self._prepare_local_implementation_service_proxy.shutdown()

        self._cleanup()

        self._input_bridge_publisher = None
        self._output_bridge_subscriber = None
        self._request_capability_execution_service_proxy = None
        self._prepare_local_implementation_service_proxy = None
        self._execute_capability_remote_client = None
        self._notify_capability_execution_status_client = None

    # Cleanup operations

    def _unregister_io_bridge_publishers_subscribers(self):
        """
        Helper method that unregisters all the io bridge publishers if not none.
        :return: None
        """
        if self._output_bridge_subscriber is not None:
            self._output_bridge_subscriber.unregister()

        if self._input_bridge_publisher is not None:
            self._input_bridge_publisher.unregister()

    def _stop_calls_action_clients_async_service_clients(self):
        """
        Shuts down the running goals of action clients and Async Service clients.
        :return: None
        """
        if self._request_capability_execution_service_proxy is not None:
            self._request_capability_execution_service_proxy.stop_call()

        if self._prepare_local_implementation_service_proxy is not None:
            self._prepare_local_implementation_service_proxy.stop_call()

        if self._execute_capability_remote_client is not None and \
                self._execute_capability_remote_client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PREEMPTING]:
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
                    ControlTreeExecutionRequest(command=ControlTreeExecutionRequest.SHUTDOWN)
                )
                if not response.success:
                    self.logwarn(f"Could not shutdown local capability: {response.error_message}")
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
                    self.logwarn(f"Could not clear local capability tree: {response.error_message}")
                    return False
        return True

    def _cleanup(self):
        """
        Cleans up the internal states of the capability and resets them to the default values.
        :return: None
        """
        # Wait for prepare local tree thread to finish
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
        self._execute_capability_remote_client_start_time: Optional[rospy.Time] = None

        # Variables for local trees
        self._clear_local_implementation_tree()

        self._local_implementation_tree: Optional[Tree] = None
        self._local_implementation_tree_root: Optional[Node] = None
        self._local_implementation_tree_status = NodeMsg.IDLE


def capability_node_class_from_capability_interface(
        capability_interface: CapabilityInterface,
        mission_control_topic: str
) -> Type[Capability]:
    """
    Uses a capability interface in combination with a mission control topic to create a new class
    definition for the defined interface.
    The new class is a subclass to the Capability class.

    This function uses the python type(...) metaclass to create a new subclass for the provided
    interface definition.
    This class is then decorated with the define_bt_node decorator to create a valid node class definition.

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
        outputs_dict[output_node_data.key] = json_decode(output_node_data.serialized_type)

    capability_node_class = type(
        capability_interface.name,
        (Capability,),
        {
            '__capability_interface': capability_interface,
            '__local_mc_topic'      : mission_control_topic,
            '__module__'            : Capability.__module__
        }
    )

    return define_bt_node(
        NodeConfig(
            options={},
            inputs=inputs_dict,
            outputs=outputs_dict,
            max_children=0
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
            '__capability_interface': capability_interface,
            '__module__'            : f"{CapabilityInputDataBridge.__module__}.bridge"
        }
    )

    return define_bt_node(
        NodeConfig(
            options={},
            inputs={},
            outputs=inputs_dict,
            max_children=0
        )
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
        outputs_dict[output_node_data.key] = json_decode(output_node_data.serialized_type)

    capability_output_bridge_node_class = type(
        f"{capability_interface.name}_OutputDataBridge",
        (CapabilityOutputDataBridge,),
        {
            '__capability_interface': capability_interface,
            '__module__'            : f"{CapabilityOutputDataBridge.__module__}.bridge"
        }
    )

    return define_bt_node(
        NodeConfig(
            options={},
            inputs=outputs_dict,
            outputs={},
            max_children=0
        )
    )(capability_output_bridge_node_class)


def capability_node_class_from_capability_interface_callback(
        msg: Time,
        args: List
):
    # pylint: disable=unused-argument
    """
    Rospy service callback used to register capability interfaces with the local Node class.

    :param msg: The capability interface to register.
    :param args: The additional arguments passed to the callback.
    :return: Nothing
    """

    if len(args) < 2:
        rospy.logerr("Create capability node callback received less then one extra args.")
        return
    if not isinstance(args[0], str):
        rospy.logerr("First args is not a string!")
        return

    if not isinstance(args[1], rospy.ServiceProxy):
        rospy.logerr("Second args is not a server proxy!")
        return

    rospy.loginfo(f"Received new capability information at {msg}")

    service_proxy: rospy.ServiceProxy = args[1]
    response: GetCapabilityInterfacesResponse = service_proxy.call(GetCapabilityInterfacesRequest())
    if not response.success:
        rospy.logwarn(f"Failed to get capability interfaces: {response.error_message}!")
    for interface in response.interfaces:
        capability_node_class_from_capability_interface(interface, args[0])
        capability_input_data_bridge_from_interface(interface)
        capability_output_data_bridge_from_interface(interface)


def set_capability_io_bridge_topics(
        tree_manager: TreeManager,
        interface: CapabilityInterface,
        input_topic: str,
        output_topic: str
) -> None:
    for node in tree_manager.nodes.values():
        if hasattr(node, "__capability_interface") and \
                getattr(node, "__capability_interface") == interface:
            if isinstance(node, CapabilityInputDataBridge):
                node.capability_bridge_topic = input_topic
                continue
            if isinstance(node, CapabilityOutputDataBridge):
                node.capability_bridge_topic = output_topic
