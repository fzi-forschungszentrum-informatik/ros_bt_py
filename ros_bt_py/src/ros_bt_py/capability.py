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
# pylint: disable=no-name-in-module,import-error
import threading
import uuid
from typing import Optional, List, Type

from abc import ABC
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from more_itertools import first_true
from ros_bt_py_msgs.msg import (
    CapabilityInterface, Node as NodeMsg,
    ExecuteCapabilityAction, ExecuteCapabilityGoal, ExecuteCapabilityResult, Tree, ExecuteRemoteCapabilityAction,
    ExecuteRemoteCapabilityGoal, ExecuteRemoteCapabilityResult,
)
from ros_bt_py_msgs.srv import (
    GetCapabilityImplementations, GetCapabilityImplementationsResponse,
    PrepareLocalImplementation, PrepareLocalImplementationRequest, PrepareLocalImplementationResponse,
    MigrateTreeRequest, LoadTreeRequest,
)

from ros_bt_py.helpers import json_decode
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import define_bt_node, Leaf, Node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.ros_helpers import AsyncServiceProxy
from ros_bt_py.tree_manager import TreeManager
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.migration import MigrationManager

WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS = 5
WAIT_FOR_IMPLEMENTATION_ASSIGNMENT_TIMEOUT_SECONDS = 5

WAIT_FOR_REMOTE_MISSION_CONTROL_TRIES = 10
WAIT_FOR_REMOTE_EXECUTION_TIMEOUT_SECONDS = 10


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

        self.tree_manager = TreeManager(
            name=name,
            publish_tree_callback=self.nop,
            publish_debug_info_callback=self.nop,
            publish_debug_settings_callback=self.nop,
            publish_node_diagnostics_callback=self.nop,
            debug_manager=DebugManager()
        )

        self.migration_manager = MigrationManager(tree_manager=self.tree_manager)
        self.capability_write_lock = threading.RLock()
        self.__local_tree_setup_thread: Optional[threading.Thread] = None

        self.__internal_state = self.IDLE

        self.__implementation_name: Optional[str] = None

        # Variables for remote trees
        self.__remote_mission_control_topic: Optional[str] = None
        self.__execute_capability_remote_client: Optional[SimpleActionClient] = None
        self.__execute_capability_remote_client_connection_tries: int = 0
        self.__execute_capability_remote_client_start_time: Optional[rospy.Time] = None

        # Variables for local trees
        self.__prepared_local_implementation_tree: Optional[Tree] = None
        self.__prepared_local_implementation_tree_root: Optional[Node] = None

        self.__execute_capability_action_client: Optional[SimpleActionClient] = None
        self.__prepare_local_implementation: Optional[AsyncServiceProxy] = None

    def __nop(self, msg):
        """no op "publisher" to stop tree_manager from spamming complaints for not-set publishers
        """
        pass

    def _do_setup(self):

        try:
            rospy.wait_for_service(
                f'{self.local_mc_topic}/prepare_local_implementation',
                timeout=rospy.Duration(WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS)
            )
        except rospy.ROSException as exc:
            raise BehaviorTreeException(
                f'Service "{self.local_mc_topic}/prepare_local_implementation" not available'
                f' after waiting f{WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS} seconds!'
            ) from exc

        self.__prepare_local_implementation = AsyncServiceProxy(
            f'{self.local_mc_topic}/prepare_local_implementation',
            PrepareLocalImplementation
        )

        # Create an action client for FindBestExecutor
        self.__execute_capability_action_client = SimpleActionClient(
            f'{self.local_mc_topic}/execute_capability',
            ExecuteCapabilityAction
        )

        if not self.__execute_capability_action_client.wait_for_server(
                timeout=rospy.Duration(WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS)
        ):
            raise BehaviorTreeException(
                f'Action server "{self.local_mc_topic}/execute_capability" not available'
                f' after waiting f{WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS} seconds!'
            )

    def __tick_unassigned(self) -> str:
        """
        Function performing the tick operation while in the UNASSIGNED state.
        :return: The new state as a NodeMsg constant.
        """
        # pylint: disable=too-many-return-statements

        if self.__internal_state == self.IDLE:
            rospy.loginfo("Starting implementation search!")
            self.__execute_capability_action_client.send_goal(
                ExecuteCapabilityGoal(
                    capability=self.capability_interface
                )
            )
            self.__internal_state = self.WAITING_FOR_ASSIGNMENT

        if self.__internal_state == self.WAITING_FOR_ASSIGNMENT:
            execute_capability_state = self.__execute_capability_action_client.get_state()

            if execute_capability_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
                self.logerr(
                    f'ExecuteCapability action ended without succeeding'
                    f' (state ID: {execute_capability_state})'
                )
                self.cleanup()
                return NodeMsg.FAILED

            if execute_capability_state in [GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
                self.logerr(
                    f'ExecuteCapability action was willingly canceled before succeeding'
                    f' (state ID: {execute_capability_state})'
                )
                self.cleanup()
                return NodeMsg.FAILED

            if execute_capability_state == GoalStatus.SUCCEEDED:
                execute_capability_result: ExecuteCapabilityResult \
                    = self.__execute_capability_action_client.get_result()

                if not execute_capability_result.success:
                    self.logerr(
                        f'ExecuteCapability action returned a failed result. No implementation could be found!'
                        f'{execute_capability_result.error_message}'
                    )
                    self.cleanup()
                    return NodeMsg.FAILED
                self.__implementation_name = execute_capability_result.implementation_name
                if execute_capability_result.execute_local:
                    self.loginfo("Executing capability locally!")
                    self.__internal_state = self.EXECUTE_LOCAL

                else:
                    self.loginfo(
                        f"Executing capability remotely on"
                        f" {execute_capability_result.remote_mission_controller_topic}"
                        )
                    self.__internal_state = self.EXECUTE_REMOTE
                    self.__remote_mission_control_topic = execute_capability_result.remote_mission_controller_topic
                return NodeMsg.ASSIGNED

        return NodeMsg.UNASSIGNED

    def __tick_assigned(self) -> str:
        if self.__internal_state == self.EXECUTE_LOCAL:
            return self.__tick_get_local_implementation()

        if self.__internal_state == self.EXECUTE_REMOTE:
            return self.__tick_start_remote_execution()

        return NodeMsg.BROKEN

    def __prepare_loaded_tree(self, tree):
        migration_request = MigrateTreeRequest(tree=tree)
        migration_response = self.migration_manager.check_node_versions(migration_request)

        if migration_response.migrated:
            migration_response = self.migration_manager.migrate_tree(migration_request)
            if migration_response.success:
                tree = migration_response.tree
        with self.capability_write_lock:
            load_tree_response = self.tree_manager.load_tree(
                request=LoadTreeRequest(tree=tree),
                prefix=self.name + "."
            )

        if not load_tree_response.success:
            self.logerr(f"Error loading capability tree: {load_tree_response}")
            return NodeMsg.FAILED

        # TODO: Forward inputs and outputs to the tree running in the tree manager

        with self.capability_write_lock:
            self.__prepared_local_implementation_tree_root = self.tree_manager.find_root()
            self.__prepared_local_implementation_tree_root.setup()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.debug_manager.add_subtree_info(
                self.name, self.manager.to_msg()
            )

    def __tick_get_local_implementation(self) -> str:
        if self.__local_tree_setup_thread is not None:
            if self.__local_tree_setup_thread.isAlive():
                return NodeMsg.ASSIGNED
            else:
                self.__local_tree_setup_thread = None
                return NodeMsg.RUNNING

        if self.__prepare_local_implementation.get_state() == AsyncServiceProxy.IDLE:
            self.__prepare_local_implementation.call_service(
                req=PrepareLocalImplementationRequest(
                    implementation_name=self.__implementation_name,
                    interface=self.capability_interface
                )
            )
        if self.__prepare_local_implementation.get_state() == AsyncServiceProxy.RESPONSE_READY:

            prepare_local_implementation_response: PrepareLocalImplementationResponse = \
                self.__prepare_local_implementation.get_response()

            if not prepare_local_implementation_response.success:
                error_msg = f'Prepare local implementation request failed: ' \
                            f'{prepare_local_implementation_response.error_message}'
                self.logerr(error_msg)
                raise BehaviorTreeException(error_msg)

            local_implementation_tree: Tree = \
                prepare_local_implementation_response.implementation_subtree

            self.__local_tree_setup_thread = threading.Thread(
                self.__prepared_local_implementation_tree_root,
                args=(local_implementation_tree,)
            )

        return NodeMsg.ASSIGNED

    def __tick_start_remote_execution(self) -> str:
        if self.__execute_capability_remote_client is None:
            self.__execute_capability_remote_client = SimpleActionClient(
                f'{self.__remote_mission_control_topic}/execute_remote_capability',
                ExecuteRemoteCapabilityAction
            )

        if self.__execute_capability_remote_client_connection_tries > 10:
            self.cleanup()
            raise BehaviorTreeException(
                f'Action server "{self.__remote_mission_control_topic}/execute_remote_capability" not available'
                f' after trying 10 times.'
            )

        if not self.__execute_capability_remote_client.wait_for_server(
                timeout=rospy.Duration(nsecs=10)
        ):
            self.__execute_capability_remote_client_connection_tries += 1
            return NodeMsg.ASSIGNED

        return NodeMsg.RUNNING

    def __tick_executing_local(self) -> str:
        """
        Performs the ticking operation if the capability is executed locally.
        It retrieves the local implementation and performs the ticking operations.

        :raise BehaviorTreeException: If the implementation cannot be received or nodes cannot be instantiated.
        :return: The statue of the node from NodeMsg.
        """
        # No current implementation is present, one needs to be received from the mission controller.
        new_state = self.__prepared_local_implementation_tree_root.tick()
        if self.debug_manager and self.debug_manager.get_publish_subtrees():
            self.debug_manager.add_subtree_info(
                self.name, self.tree_manager.to_msg()
            )
        return new_state

    def __tick_executing_remote(self) -> str:
        """
        Performs the ticking operation when operating on a remote node.

        :return: The status of the node as a NodeMsg status.
        """

        self.__execute_capability_remote_client.send_goal(
            ExecuteRemoteCapabilityGoal(
                interface=self.capability_interface,
                implementation_name=self.__implementation_name,
            )
        )
        self.__execute_capability_remote_client_start_time = rospy.Time.now()

        # Check if the action ran into a timeout.
        if (rospy.Time.now() - self.__execute_capability_remote_client_start_time
                > rospy.Duration(WAIT_FOR_REMOTE_EXECUTION_TIMEOUT_SECONDS)):
            self.logerr(
                f'ExecuteRemoteCapability action timed out after'
                f' {WAIT_FOR_REMOTE_EXECUTION_TIMEOUT_SECONDS} seconds'
            )
            self.cleanup()
            return NodeMsg.FAILED

        execute_remote_implementation_state = self.__execute_capability_remote_client_start_time.get_state()

        if execute_remote_implementation_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
            self.logerr(
                f'ExecuteRemoteCapability action ended without succeeding'
                f' (state ID: {execute_remote_implementation_state})'
            )
            self.cleanup()
            return NodeMsg.FAILED

        if execute_remote_implementation_state in [GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
            self.logerr(
                f'ExecuteRemoteCapability action was willingly canceled before succeeding'
                f' (state ID: {execute_remote_implementation_state})'
            )
            self.cleanup()
            return NodeMsg.FAILED

        if execute_remote_implementation_state == GoalStatus.SUCCEEDED:
            execute_remote_implementation_result: ExecuteRemoteCapabilityResult \
                = self.__execute_implementation_action.get_result()

            if not execute_remote_implementation_result.success:
                self.logerr(
                    f'ExecuteRemoteCapability action did not finish successfully:'
                    f' "{execute_remote_implementation_result.error_message}"'
                )
                self.cleanup()
                return NodeMsg.FAILED

            # TODO: Copy results and publish to the subscribers
            return NodeMsg.SUCCEEDED
        return NodeMsg.RUNNING

    def __tick_running(self) -> str:

        if self.__internal_state == self.EXECUTE_LOCAL:
            return self.__tick_executing_local()

        if self.__internal_state == self.EXECUTE_REMOTE:
            return self.__tick_executing_remote()

        return NodeMsg.RUNNING

    def _do_tick(self):
        if self.state == NodeMsg.IDLE:
            return NodeMsg.UNASSIGNED

        if self.state == NodeMsg.UNASSIGNED:
            return self.__tick_unassigned()

        if self.state == NodeMsg.ASSIGNED:
            return self.__tick_assigned()

        if self.state == NodeMsg.RUNNING:
            return self.__tick_running()

        # TODO: Fix this issue!
        return NodeMsg.FAILED

    def _do_untick(self):
        """Pause execution of the decorated subtree, but preserve the executor

        This means a new
        :class:`ros_bt_py_msgs.msg.FindBestExecutorAction` will
        **not** be sent on the next tick. Rather, the same executor
        (either local or remote) as before is re-used.

        To force a new
        :class:`ros_bt_py_msgs.msg.FindBestExecutorAction`, use
        :meth:`_do_reset()`

        """
        # If we're in an EXECUTE state, stop the execution. otherwise,
        # just clean up
        if self.state == NodeMsg.IDLE:
            return NodeMsg.PAUSED

        if self.state == NodeMsg.UNASSIGNED:
            return self.__tick_unassigned()

        if self.state == NodeMsg.ASSIGNED:
            return self.__tick_assigned()

        if self.state == NodeMsg.RUNNING:
            return

        new_state = NodeMsg.PAUSED
        if self.__internal_state == self.WAIT_FOR_ASSIGNMENT_RESPONSE:
            self.__find_implementation_action.cancel_goal()
        elif self.__internal_state == self.EXECUTE_LOCAL:
            # TODO: Pause local execution
            pass
        elif self.__internal_state == self.EXECUTE_REMOTE:
            self.__execute_implementation_action.cancel_goal()
            # On the next tick, another goal is sent to the same
            # action server as before
            self.__internal_state = self.WAIT_FOR_EXECUTION_SERVER

        return new_state

    def _do_reset(self):
        if self.__internal_state == self.EXECUTE_LOCAL:
            # TODO: Reset on the local
            pass
        elif self.__execute_implementation_action is not None:
            self.__execute_implementation_action.cancel_goal()
        self.cleanup()

        return NodeMsg.IDLE

    def _do_shutdown(self):
        # TODO: Shutdown local children
        self.__find_implementation_action.cancel_goal()
        if self.__execute_implementation_action is not None:
            self.__execute_implementation_action.cancel_goal()
        self.cleanup()

    def cleanup(self):
        """
        Cleans up the internal states of the capability and resets them to the default values.
        :return: None
        """
        self.__execute_implementation_action = None
        self.__execute_implementation_action_start_time = None

        self.__local_implementation = None

        self.__find_implementation_action_start_time = None
        self.__execute_implementation_mc_namespace = None
        self.__execute_implementation_name = None
        self.__internal_state = self.IDLE
        self.__execute_implementation_mc_connection_attempts = 0


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

    options_dict = {}
    for options_node_data in capability_interface.options:
        options_dict[options_node_data.key] = json_decode(options_node_data.serialized_type)

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
            version='1.0.0',
            options=options_dict,
            inputs=inputs_dict,
            outputs=outputs_dict,
            max_children=0
        )
    )(capability_node_class)


def capability_node_class_from_capability_interface_callback(
        msg: CapabilityInterface,
        args: List
):
    """
    Rospy service callback used to register capability interfaces with the local Node class.

    :param msg: The capability interface to register.
    :param args: The additional arguments passed to the callback.
    :return: Nothing
    """
    rospy.logwarn("Capability callback")

    if len(args) < 1:
        rospy.logerr("Create capability node callback received less then one extra args.")
        return
    if not isinstance(args[0], str):
        rospy.logerr("First args is not a string!")
        return

    if msg.name == '':
        rospy.logerr("Capability interface message has empty name.")
        return

    capability_node_class_from_capability_interface(msg, args[0])
