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
from threading import RLock, Event
from typing import Optional

import rospy
from actionlib import SimpleActionServer
from ros_bt_py_msgs.msg import (
    ExecuteCapabilityImplementationGoal, ExecuteCapabilityImplementationAction,
    ExecuteCapabilityImplementationFeedback, ExecuteCapabilityImplementationResult, Node as NodeMsg,
    RemoteCapabilitySlotStatus,
)
from ros_bt_py_msgs.srv import (
    ControlTreeExecutionRequest, LoadTreeRequest, LoadTreeResponse, ClearTreeRequest,
    NotifyCapabilityExecutionStatus, NotifyCapabilityExecutionStatusRequest,
)
from rospy import ServiceProxy, Publisher

from ros_bt_py.capability import CapabilityInputDataBridge, CapabilityOutputDataBridge, set_capability_io_bridge_topics
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
            debug_manager=self.debug_manager
        )
        self._tree_root: Optional[Node] = None
        self._tree_manager_lock = RLock()
        self._tree_loaded_event: Event = Event()
        self._is_finished_event: Event = Event()

        self._callback_lock = RLock()

        self.capability_interface = None

        self._remote_slot_executor_action_server: Optional[SimpleActionServer] = None
        self._notify_capability_execution_status_client: Optional[ServiceProxy] = None
        self._remote_capability_slot_status_publisher: Optional[Publisher] = None

    def _do_setup(self):

        remote_tree_slot_executor_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/remote_capability_slot/{self.name}"
        )
        rospy.logfatal(remote_tree_slot_executor_topic)

        self._remote_slot_executor_action_server = SimpleActionServer(
            remote_tree_slot_executor_topic,
            ExecuteCapabilityImplementationAction,
            execute_cb=self._remote_slot_executor_cb,
            auto_start=False
        )

        notify_capability_execution_status_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/notify_capability_execution_status"
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
                f'Service "/notify_capability_execution_status" not available'
                f' after waiting 2 seconds!'
            ) from exc

        remote_capability_slot_status_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/remote_slot_status"
        )

        self._remote_capability_slot_status_publisher = Publisher(
            remote_capability_slot_status_topic,
            RemoteCapabilitySlotStatus,
            queue_size=1
        )

        self._remote_slot_executor_action_server.start()

    def _remote_slot_executor_cb(self, goal: ExecuteCapabilityImplementationGoal) -> None:
        with self._callback_lock:
            self.logfatal("Remote Slot CB starting")

            self._is_finished_event.clear()
            with self._tree_manager_lock:
                if self._tree_loaded_event.is_set():
                    self._tree_root.shutdown()
                    self._tree_manager.clear(request=ClearTreeRequest())
                    self._tree_loaded_event.clear()

                rospy.logfatal("Loading impl tree")
                service_response: LoadTreeResponse = self._tree_manager.load_tree(
                    LoadTreeRequest(tree=goal.implementation_tree),
                    prefix=self.name + "_"
                )

            if not service_response.success:
                self._remote_slot_executor_action_server \
                    .set_aborted(text=f"Could not load implementation tree: {service_response.error_message}")

            with self._tree_manager_lock:
                set_capability_io_bridge_topics(self._tree_manager, goal.interface, goal.input_topic, goal.output_topic)
                self._tree_root = self._tree_manager.find_root()
                self._tree_root.shutdown()
                self._tree_root.setup()

            rospy.logfatal("Set tree loaded event!")
            self._tree_loaded_event.set()

            while not self._is_finished_event.wait(timeout=1):
                if self._remote_slot_executor_action_server.is_preempt_requested():
                    self._remote_slot_executor_action_server.set_preempted()
                    self.logwarn("Preempt requested!")
                    self.cleanup()
                    return

            self._remote_slot_executor_action_server.set_succeeded(
                result=ExecuteCapabilityImplementationResult(
                    success=True
                )
            )


    def _do_tick(self):
        if self.state is NodeMsg.IDLE:
            self._remote_capability_slot_status_publisher.publish(
                RemoteCapabilitySlotStatus(
                    name=self.name,
                    status=RemoteCapabilitySlotStatus.IDLE,
                    timestamp=rospy.Time.now()
                )
            )

            self._notify_capability_execution_status_client.call(
                NotifyCapabilityExecutionStatusRequest(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=NotifyCapabilityExecutionStatusRequest.IDLE
                )
            )
            return NodeMsg.UNASSIGNED

        if self.state is NodeMsg.UNASSIGNED:
            if self._tree_loaded_event.is_set():
                self._remote_capability_slot_status_publisher.publish(
                    RemoteCapabilitySlotStatus(
                        name=self.name,
                        status=RemoteCapabilitySlotStatus.RUNNING,
                        timestamp=rospy.Time.now()
                    )
                )
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
            if not self._tree_loaded_event.is_set():
                self.logerr("No tree loaded; returning to UNASSIGNED state!")
                return NodeMsg.UNASSIGNED

            self._remote_capability_slot_status_publisher.publish(
                RemoteCapabilitySlotStatus(
                    name=self.name,
                    status=RemoteCapabilitySlotStatus.RUNNING,
                    timestamp=rospy.Time.now()
                )
            )

            new_state = self._tree_root.tick()
            if self.debug_manager and self.debug_manager.get_publish_subtrees():
                self.debug_manager.add_subtree_info(
                    self.name, self._tree_manager.to_msg()
                )

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
                self._is_finished_event.set()
                self.cleanup()
                return NodeMsg.UNASSIGNED

        return NodeMsg.RUNNING

    def _do_untick(self):
        if self._tree_loaded_event.is_set():
            with self._tree_manager_lock:
                self._tree_manager.control_execution(
                    ControlTreeExecutionRequest(
                        command=ControlTreeExecutionRequest.STOP
                    )
                )
        return NodeMsg.PAUSED

    def _do_reset(self):
        self._remote_capability_slot_status_publisher.publish(
            RemoteCapabilitySlotStatus(
                name=self.name,
                status=RemoteCapabilitySlotStatus.SHUTDOWN,
                timestamp=rospy.Time.now()
            )
        )

        self.cleanup()
        if self._remote_slot_executor_action_server.is_active():
            self._remote_slot_executor_action_server.set_aborted(text="RemoteCapabilitySlot has been reset!")

    def _do_shutdown(self):
        self._remote_capability_slot_status_publisher.publish(
            RemoteCapabilitySlotStatus(
                name=self.name,
                status=RemoteCapabilitySlotStatus.SHUTDOWN,
                timestamp=rospy.Time.now()
            )
        )

        if self._remote_slot_executor_action_server.is_active():
            self._remote_slot_executor_action_server.set_aborted(text="RemoteCapabilitySlot has been reset!")

        self._remote_capability_slot_status_publisher.unregister()
        self._remote_capability_slot_status_publisher = None

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
        with self._tree_manager_lock:
            rospy.logfatal("Cleaning up!")
            if self._tree_loaded_event.is_set():
                    self._tree_root.shutdown()
                    self._tree_manager.clear(request=ClearTreeRequest())
            self._tree_loaded_event.clear()
