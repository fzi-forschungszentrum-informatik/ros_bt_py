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
#     * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
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
Module describing the RemoteCapabilitySlot class.

Its purpose is to execute capability implementations upon remote requests.
"""

# pylint: disable=import-error, no-name-in-module
from threading import RLock
from typing import Optional

import rospy
from actionlib import SimpleActionServer
from ros_bt_py_msgs.msg import (
    ExecuteCapabilityImplementationGoal, ExecuteCapabilityImplementationAction,
    ExecuteCapabilityImplementationFeedback, ExecuteCapabilityImplementationResult, Node as NodeMsg,
    CapabilityIOBridgeData,
)
from ros_bt_py_msgs.srv import (
    ControlTreeExecutionRequest, LoadTreeRequest, LoadTreeResponse, ClearTreeRequest,
    NotifyCapabilityExecutionStatus, NotifyCapabilityExecutionStatusRequest,
)
from rospy import Publisher, Subscriber, ServiceProxy

from ros_bt_py.capability import CapabilityInputDataBridge, CapabilityOutputDataBridge
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.tree_manager import TreeManager


@define_bt_node(
    NodeConfig(
        max_children=0,
        version="0.1.0",
        inputs={},
        outputs={},
        options={},
        optional_options=[],
        tags=['capabilities']
    )
)
class RemoteCapabilitySlot(Node):
    """
    Node class that allows the execution of Capabilities upon remote requests.
    """

    def __init__(self, options=None, debug_manager=None, name=None):
        super().__init__(options, debug_manager, name)

        self._tree_manager = TreeManager(
            name="RemoteTreeSlotTreeManager",
            publish_tree_callback=self.nop,
            publish_debug_info_callback=self.nop,
            publish_debug_settings_callback=self.nop,
            publish_node_diagnostics_callback=self.nop,
            debug_manager=DebugManager()
        )
        self._tree_root: Optional[Node] = None
        self._tree_manager_lock = RLock()
        self._tree_loaded: bool = False

        self._remote_slot_executor_action_server: Optional[SimpleActionServer] = None
        self._notify_capability_execution_status_client: Optional[ServiceProxy] = None

    def _do_setup(self):
        self._remote_slot_executor_action_server = SimpleActionServer(
            f"~/remote_capability_slot/{self.name}",
            ExecuteCapabilityImplementationAction,
            execute_cb=self._remote_slot_executor_cb
        )

        notify_capability_execution_status_topic = rospy.resolve_name(
            f"{self.local_mc_topic}/notify_capability_execution_status"
        )

        self._notify_capability_execution_status_client = ServiceProxy(
            notify_capability_execution_status_topic,
            NotifyCapabilityExecutionStatus,
            persistent=True
        )
        try:
            self._notify_capability_execution_status_client.wait_for_service(
                timeout=rospy.Duration(secs=2)
            )
        except rospy.ROSException as exc:
            raise BehaviorTreeException(
                f'Service "{self.local_mc_topic}/notify_capability_execution_status" not available'
                f' after waiting 2 seconds!'
            ) from exc

    def _remote_slot_executor_cb(self, goal: ExecuteCapabilityImplementationGoal) -> None:
        if self._tree_loaded:
            with self._tree_manager_lock:
                self._tree_manager.control_execution(
                    ControlTreeExecutionRequest(
                        command=ControlTreeExecutionRequest.SHUTDOWN
                    )
                )
                self._tree_manager.clear(request=ClearTreeRequest())
                self._tree_loaded = False
        with self._tree_manager_lock:
            service_response: LoadTreeResponse = self._tree_manager.load_tree(
                LoadTreeRequest(tree=goal.implementation_tree))
        if not service_response.success:
            self._remote_slot_executor_action_server \
                .set_aborted(text=f"Could not load implementation tree: {service_response.error_message}")

        for node in self._tree_manager.nodes.values():
            if hasattr(node, "__capability_interface") and \
                    getattr(node, "__capability_interface") == goal.interface:
                if isinstance(node, CapabilityInputDataBridge):
                    node.capability_bridge_topic = self._input_bridge_topic
                    continue
                if isinstance(node, CapabilityOutputDataBridge):
                    node.capability_bridge_topic = self._output_bridge_topic

        with self._tree_manager_lock:
            self._tree_loaded = True
            self._tree_root = self._tree_manager.find_root()

    def _do_tick(self):

        if self.state is NodeMsg.IDLE:
            self._notify_capability_execution_status_client.call(
                NotifyCapabilityExecutionStatusRequest(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=NotifyCapabilityExecutionStatusRequest.IDLE
                )
            )
            return NodeMsg.UNASSIGNED

        if self.state is NodeMsg.UNASSIGNED:
            if self._tree_loaded:
                self._notify_capability_execution_status_client.call(
                    NotifyCapabilityExecutionStatusRequest(
                        interface=self.capability_interface,
                        node_name=self.name,
                        status=NotifyCapabilityExecutionStatusRequest.EXECUTING
                    )
                )
                return NodeMsg.RUNNING
            return NodeMsg.UNASSIGNED

        if self.state is NodeMsg.RUNNING:
            if self._remote_slot_executor_action_server.is_preempt_requested():
                self.cleanup()
                self._remote_slot_executor_action_server.set_preempted()
                self._notify_capability_execution_status_client.call(
                    NotifyCapabilityExecutionStatusRequest(
                        interface=self.capability_interface,
                        node_name=self.name,
                        status=NotifyCapabilityExecutionStatusRequest.SHUTDOWN
                    )
                )
                return NodeMsg.UNASSIGNED
            new_state = self._tree_root.tick()
            if self.debug_manager and self.debug_manager.get_publish_subtrees():
                self.debug_manager.add_subtree_info(
                    self.name, self._tree_manager.to_msg()
                )
            self._remote_slot_executor_action_server.publish_feedback(
                ExecuteCapabilityImplementationFeedback(
                    execution_status=new_state
                )
            )
            if new_state in [NodeMsg.SUCCEEDED, NodeMsg.FAILED]:
                self._remote_slot_executor_action_server.set_succeeded(
                    result=ExecuteCapabilityImplementationResult(
                        success=True
                    )
                )
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

                self.cleanup()
                return NodeMsg.UNASSIGNED
        return NodeMsg.RUNNING

    def _do_untick(self):
        if self._tree_loaded:
            with self._tree_manager_lock:
                self._tree_manager.control_execution(
                    ControlTreeExecutionRequest(
                        command=ControlTreeExecutionRequest.STOP
                    )
                )
        return NodeMsg.PAUSED

    def _do_reset(self):
        self.cleanup()
        if self._remote_slot_executor_action_server.is_active():
            self._remote_slot_executor_action_server.set_aborted(text="RemoteCapabilitySlot has been reset!")

    def _do_shutdown(self):
        self.cleanup()
        if self._remote_slot_executor_action_server.is_active():
            self._remote_slot_executor_action_server.set_aborted(text="RemoteCapabilitySlot has been reset!")

        self._remote_slot_executor_action_server = None
        self._notify_capability_execution_status_client.close()

    @staticmethod
    def nop(msg) -> None:
        """
        No operation method.

        :return: None
        """

    def cleanup(self) -> None:
        """
        Cleans up any state changes that the node has and restores it to its original state.
        :return: None
        """
        if self._tree_loaded:
            with self._tree_manager_lock:
                self._tree_manager.control_execution(
                    ControlTreeExecutionRequest(
                        command=ControlTreeExecutionRequest.SHUTDOWN
                    )
                )
                self._tree_manager.clear(request=ClearTreeRequest())
                self._tree_loaded = False
