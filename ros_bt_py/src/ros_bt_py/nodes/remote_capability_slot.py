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
Module describing the RemoteCapabilitySlot class.

Its purpose is to execute capability implementations upon remote requests.
"""

# pylint: disable=import-error, no-name-in-module
from threading import RLock, Event
from typing import Optional, Dict

import rospy
import std_msgs.msg
from rospy import Publisher, Service, ROSException
from ros_bt_py_msgs.msg import (
    Node as NodeMsg,
    RemoteCapabilitySlotStatus,
    CapabilityExecutionStatus,
    CapabilityInterface,
    PingMsg,
)
from ros_bt_py_msgs.srv import (
    LoadTreeRequest,
    LoadTreeResponse,
    ClearTreeRequest,
    RunRemoteCapabilitySlot,
    RunRemoteCapabilitySlotRequest,
    RunRemoteCapabilitySlotResponse,
    CancelRemoteCapabilitySlotRequest,
    CancelRemoteCapabilitySlotResponse,
    CancelRemoteCapabilitySlot,
)

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.tree_manager import TreeManager
from ros_bt_py.capability import set_capability_io_bridge_id


@define_bt_node(
    NodeConfig(
        max_children=0,
        inputs={},
        outputs={},
        options={},
        optional_options=[],
        tags=["capabilities", "remote"],
    )
)
class RemoteCapabilitySlot(Node):
    """Node class that allows the execution of Capabilities upon remote requests."""

    # pylint: disable=too-many-instance-attributes

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: str = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        """Instanciate a new RemoteCapabilitySlot BT node."""
        super(RemoteCapabilitySlot, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            simulate_tick=simulate_tick,
            succeed_always=succeed_always,
        )

        self._tree_manager = TreeManager(
            name="RemoteTreeSlotTreeManager",
            publish_tree_callback=self.nop,
            publish_debug_info_callback=self.nop,
            publish_debug_settings_callback=self.nop,
            publish_node_diagnostics_callback=self.nop,
            debug_manager=self.debug_manager,
        )
        self._tree_root: Optional[Node] = None
        self._ping_publisher: Optional[Publisher] = None
        self.node_id: Optional[str] = None

        self._run_remote_capability_service: Optional[Service] = None
        self._cancel_remote_capability_service: Optional[Service] = None

        self._capability_execution_status_publisher: Optional[Publisher] = None
        self._remote_capability_slot_status_publisher: Optional[Publisher] = None

        self._capability_implementation_available_event: Event = Event()
        self._is_finished_event: Event = Event()
        self._tree_loaded_event: Event = Event()

        self._canceled_event: Event = Event()
        self._canceled_event.clear()
        self._request_cancellation_event: Event = Event()
        self._request_cancellation_event.clear()

        self._callback_lock = RLock()

        self.capability_interface: Optional[CapabilityInterface] = None

    def run_remote_capability_callback(
        self, req: RunRemoteCapabilitySlotRequest
    ) -> RunRemoteCapabilitySlotResponse:
        """
        ROS service to execute a capability implementation on the remote capability slot.

        :param req: The request containing all the needed information to run the request.
        :return: Response if the execution was a success.
        """
        with self._callback_lock:
            response = RunRemoteCapabilitySlotResponse()
            self._canceled_event.clear()

            self.capability_interface = req.interface
            self.node_id = req.node_id
            self._ping_publisher.publish(
                PingMsg(node_id=self.node_id, timestamp=rospy.Time.now())
            )

            if self._tree_loaded_event.is_set():
                self._tree_root.shutdown()
                self._tree_manager.clear(request=ClearTreeRequest())
                self._tree_root = None
                self._tree_loaded_event.clear()

            self._capability_implementation_available_event.set()

            # Setup tree

            service_response: LoadTreeResponse = self._tree_manager.load_tree(
                LoadTreeRequest(tree=req.implementation_tree), prefix=self.name + "_"
            )

            if not service_response.success:
                self._capability_implementation_available_event.clear()
                response.success = False
                response.error_message = (
                    "Could not load implementation tree: "
                    f"{service_response.error_message}"
                )
                return response

            set_capability_io_bridge_id(
                tree_manager=self._tree_manager,
                interface=req.interface,
                io_bridge_id=req.node_id,
            )

            self._tree_root = self._tree_manager.find_root()
            self._tree_root.shutdown()
            self._tree_root.setup()

            self._tree_loaded_event.set()

            while not self._is_finished_event.wait(timeout=0.001):
                if self._request_cancellation_event.is_set():
                    self._canceled_event.set()
                    self._request_cancellation_event.clear()

                    self._capability_implementation_available_event.clear()
                    self._tree_loaded_event.clear()
                    self.cleanup()

                    response.success = False
                    response.error_message = "Execution was cancelled!"
                    return response

            self.cleanup()
            self._is_finished_event.clear()
            self._capability_implementation_available_event.clear()
            self._tree_loaded_event.clear()

            response.success = True
            return response

    def cancel_remote_capability_callback(
        self, req: CancelRemoteCapabilitySlotRequest
    ) -> CancelRemoteCapabilitySlotResponse:
        """
        Cancel the currently running capability execution.

        :param req: None
        :return: True if the cancel request was a success, false if there was an error.
        """
        response = CancelRemoteCapabilitySlotResponse()
        self.logdebug(f"Cancellation request received: {req}")
        self._request_cancellation_event.set()
        if not self._canceled_event.wait(timeout=0.1):
            response.success = False
            response.error_message = "No confirmation of cancellation was received!"
        else:
            response.success = True
        self._request_cancellation_event.clear()
        return response

    def _do_setup(self):

        remote_tree_slot_executor_run_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/remote_capability_slot/{self.name}/run"
        )

        remote_tree_slot_executor_cancel_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/remote_capability_slot/{self.name}/cancel"
        )

        capability_execution_status_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/notify_capability_execution_status"
        )

        remote_capability_slot_status_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/mission_control/remote_slot_status"
        )

        self._ping_publisher = Publisher(
            "~/capabilities/ping", data_class=PingMsg, queue_size=1
        )

        self._run_remote_capability_service = Service(
            remote_tree_slot_executor_run_topic,
            RunRemoteCapabilitySlot,
            self.run_remote_capability_callback,
        )

        self._cancel_remote_capability_service = Service(
            remote_tree_slot_executor_cancel_topic,
            CancelRemoteCapabilitySlot,
            self.cancel_remote_capability_callback,
        )

        self._capability_execution_status_publisher = Publisher(
            capability_execution_status_topic,
            CapabilityExecutionStatus,
            queue_size=1,
            latch=True,
        )

        self._remote_capability_slot_status_publisher = Publisher(
            remote_capability_slot_status_topic,
            RemoteCapabilitySlotStatus,
            queue_size=1,
            latch=False,
        )

    def _tick_idle(self) -> str:
        self._remote_capability_slot_status_publisher.publish(
            RemoteCapabilitySlotStatus(
                name=self.name,
                status=RemoteCapabilitySlotStatus.IDLE,
                timestamp=rospy.Time.now(),
            )
        )
        self._capability_execution_status_publisher.publish(
            CapabilityExecutionStatus(
                interface=self.capability_interface,
                node_name=self.name,
                status=CapabilityExecutionStatus.IDLE,
            )
        )
        return NodeMsg.UNASSIGNED

    def _tick_unassigned(self) -> str:
        self._remote_capability_slot_status_publisher.publish(
            RemoteCapabilitySlotStatus(
                name=self.name,
                status=RemoteCapabilitySlotStatus.IDLE,
                timestamp=rospy.Time.now(),
            )
        )

        if self._capability_implementation_available_event.is_set():
            self._remote_capability_slot_status_publisher.publish(
                RemoteCapabilitySlotStatus(
                    name=self.name,
                    status=RemoteCapabilitySlotStatus.RUNNING,
                    timestamp=rospy.Time.now(),
                )
            )
            self._capability_execution_status_publisher.publish(
                CapabilityExecutionStatus(
                    interface=self.capability_interface,
                    node_name=self.name,
                    status=CapabilityExecutionStatus.EXECUTING,
                )
            )
            return NodeMsg.ASSIGNED

        return NodeMsg.UNASSIGNED

    def _tick_assigned(self) -> str:
        self._remote_capability_slot_status_publisher.publish(
            RemoteCapabilitySlotStatus(
                name=self.name,
                status=RemoteCapabilitySlotStatus.RUNNING,
                timestamp=rospy.Time.now(),
            )
        )
        self._capability_execution_status_publisher.publish(
            CapabilityExecutionStatus(
                interface=self.capability_interface,
                node_name=self.name,
                status=CapabilityExecutionStatus.EXECUTING,
            )
        )

        if (
            self._canceled_event.is_set()
            or not self._capability_implementation_available_event.is_set()
        ):
            return NodeMsg.UNASSIGNED

        if self._tree_loaded_event.is_set():
            return NodeMsg.RUNNING

        return NodeMsg.ASSIGNED

    def _tick_running(self):

        if (
            self._canceled_event.is_set()
            or not self._capability_implementation_available_event.is_set()
        ):
            return NodeMsg.UNASSIGNED

        self._remote_capability_slot_status_publisher.publish(
            RemoteCapabilitySlotStatus(
                name=self.name,
                status=RemoteCapabilitySlotStatus.RUNNING,
                timestamp=rospy.Time.now(),
            )
        )

        new_state = self._tree_root.tick()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.debug_manager.add_subtree_info(self.name, self._tree_manager.to_msg())

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

            self._remote_capability_slot_status_publisher.publish(
                RemoteCapabilitySlotStatus(
                    name=self.name,
                    status=RemoteCapabilitySlotStatus.IDLE,
                    timestamp=rospy.Time.now(),
                )
            )
            self._capability_implementation_available_event.clear()
            self._is_finished_event.set()
            return NodeMsg.UNASSIGNED
        return NodeMsg.RUNNING

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if self._ping_publisher is not None and self.node_id is not None:
            try:
                self._ping_publisher.publish(
                    PingMsg(node_id=self.node_id, timestamp=rospy.Time.now())
                )
            except ROSException as exc:
                self.logwarn(f"Exception when publishing ping: {exc}")

        if self.state is NodeMsg.SHUTDOWN:
            raise BehaviorTreeException("Ticking shutdown node!")

        if self.state is NodeMsg.IDLE:
            return self._tick_idle()

        if self.state is NodeMsg.UNASSIGNED:
            return self._tick_unassigned()

        if self.state is NodeMsg.ASSIGNED:
            return self._tick_assigned()

        if self.state is NodeMsg.RUNNING:
            return self._tick_running()

        return self.state

    def _do_untick(self):
        return NodeMsg.PAUSED

    def _do_reset(self):
        if self._capability_implementation_available_event.is_set():
            self.cancel_remote_capability_callback(CancelRemoteCapabilitySlotRequest())

        self._remote_capability_slot_status_publisher.publish(
            RemoteCapabilitySlotStatus(
                name=self.name,
                status=RemoteCapabilitySlotStatus.SHUTDOWN,
                timestamp=rospy.Time.now(),
            )
        )

    def _do_shutdown(self):
        if self._capability_implementation_available_event.is_set():
            self.cancel_remote_capability_callback(CancelRemoteCapabilitySlotRequest())

        if self._remote_capability_slot_status_publisher is not None:
            self._remote_capability_slot_status_publisher.publish(
                RemoteCapabilitySlotStatus(
                    name=self.name,
                    status=RemoteCapabilitySlotStatus.SHUTDOWN,
                    timestamp=rospy.Time.now(),
                )
            )
        rospy.sleep(rospy.Duration.from_sec(1))

        self._run_remote_capability_service.shutdown()
        self._run_remote_capability_service = None

        self._cancel_remote_capability_service.shutdown()
        self._cancel_remote_capability_service = None

        self._remote_capability_slot_status_publisher.unregister()
        self._remote_capability_slot_status_publisher = None

        self._capability_execution_status_publisher.unregister()
        self._capability_execution_status_publisher = None

        self._capability_implementation_available_event.clear()
        self._tree_loaded_event.clear()
        self._is_finished_event.clear()

    @staticmethod
    def nop(msg) -> None:
        """
        No operation method.

        :return: None
        """

    def cleanup(self) -> None:
        """
        Clean up any state changes that the node has and restores it to its original state.

        :return: None
        """
        self.logdebug("Cleaning up!")
        if self._tree_root is not None:
            self._tree_root.shutdown()
            self._tree_root = None
        self._tree_manager.clear(request=ClearTreeRequest())
        self._tree_loaded_event.clear()

        self.node_id = None
