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

from typing import Optional, List, Type

from abc import ABC
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from more_itertools import first_true
from ros_bt_py_msgs.msg import (
    CapabilityInterface, FindBestCapabilityImplementationAction,
    FindBestCapabilityImplementationGoal, FindBestCapabilityImplementationResult, ExecuteImplementationGoal,
    ExecuteImplementationAction, ExecuteImplementationResult, CapabilityImplementation, Node as NodeMsg,
)
from ros_bt_py_msgs.srv import GetCapabilityImplementations

from ros_bt_py.helpers import json_decode
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import define_bt_node, Leaf
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.ros_helpers import AsyncServiceProxy

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
    WAIT_FOR_ASSIGNMENT_RESPONSE = 1
    WAIT_FOR_EXECUTION_SERVER = 2
    EXECUTE_LOCAL = 5
    EXECUTE_REMOTE = 6

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

        self.__execute_implementation_action: Optional[SimpleActionClient] = None
        self.__execute_implementation_action_start_time: Optional[rospy.Duration] = None
        self.__execute_implementation_mc_namespace: Optional[str] = None
        self.__execute_implementation_uuid: Optional[str] = None
        self.__execute_implementation_mc_connection_attempts: int = 0

        self.__find_implementation_action: Optional[SimpleActionClient] = None
        self.__find_implementation_action_start_time: Optional[rospy.Duration] = None

        self.__get_local_implementations_service: Optional[AsyncServiceProxy] = None
        self.__internal_state = self.IDLE
        self.__local_implementation = None

    def _do_setup(self):

        try:
            rospy.wait_for_service(
                f'{self.local_mc_topic}/capabilities/implementations/get',
                timeout=rospy.Duration(WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS)
            )
        except rospy.ROSException as exc:
            raise BehaviorTreeException(
                f'Service "{self.local_mc_topic}/capabilities/implementations/get" not available'
                f' after waiting f{WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS} seconds!'
            ) from exc

        self.__get_local_implementations_service = AsyncServiceProxy(
            f'{self.local_mc_topic}/capabilities/implementations/get',
            GetCapabilityImplementations
        )

        # Create an action client for FindBestExecutor
        self.__find_implementation_action = SimpleActionClient(
            f'{self.local_mc_topic}/find_capability_implementation',
            FindBestCapabilityImplementationAction
        )

        if not self.__find_implementation_action.wait_for_server(
                timeout=rospy.Duration(WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS)
        ):
            raise BehaviorTreeException(
                f'Action server "{self.local_mc_topic}/find_capability_implementation" not available'
                f' after waiting f{WAIT_FOR_LOCAL_MISSION_CONTROL_TIMEOUT_SECONDS} seconds!'
            )

    def __tick_unassigned(self) -> str:
        """
        Function performing the tick operation while in the UNASSIGNED state.
        :return: The new state as a NodeMsg constant.
        """
        # pylint: disable=too-many-return-statements

        if self.__internal_state == self.IDLE:
            self.__find_implementation_action.send_goal(
                FindBestCapabilityImplementationGoal(
                    capability=self.capability_interface
                )
            )
            self.__find_implementation_action_start_time = rospy.Time.now()
            self.__internal_state = self.WAIT_FOR_ASSIGNMENT_RESPONSE

        if self.__internal_state == self.WAIT_FOR_ASSIGNMENT_RESPONSE:
            # Check if the action ran into a timeout.
            if (rospy.Time.now() - self.__find_implementation_action_start_time
                    > rospy.Duration(WAIT_FOR_IMPLEMENTATION_ASSIGNMENT_TIMEOUT_SECONDS)):
                self.logerr(
                    f'FindImplementation action timed out after'
                    f' {WAIT_FOR_IMPLEMENTATION_ASSIGNMENT_TIMEOUT_SECONDS} seconds'
                )
                self.__internal_state = self.IDLE
                return NodeMsg.FAILED

            find_implementation_action_state = self.__find_implementation_action.get_state()

            if find_implementation_action_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
                self.logerr(
                    f'FindImplementation action ended without succeeding'
                    f' (state ID: {find_implementation_action_state})'
                )
                self.cleanup()
                return NodeMsg.FAILED

            if find_implementation_action_state in [GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
                self.logerr(
                    f'FindImplementation action was willingly canceled before succeeding'
                    f' (state ID: {find_implementation_action_state})'
                )
                self.cleanup()
                return NodeMsg.FAILED

            if find_implementation_action_state == GoalStatus.SUCCEEDED:
                find_implementation_action_result: FindBestCapabilityImplementationResult \
                    = self.__find_implementation_action.get_result()

                if not find_implementation_action_result.success:
                    self.logerr(
                        f'FindImplementation action did not find a valid implementation:'
                        f' "{find_implementation_action_result.error_message}"'
                    )
                    self.cleanup()
                    return NodeMsg.UNASSIGNED

                self.loginfo(
                    f'Planning to execute capability {self.capability_interface.uuid} '
                    f'using implementation {find_implementation_action_result.implementation_uuid} '
                    f'on mission control "{find_implementation_action_result.executor_mission_control_topic}"'
                )

                self.__execute_implementation_mc_namespace = \
                    find_implementation_action_result.executor_mission_control_topic
                self.__execute_implementation_uuid = find_implementation_action_result.implementation_uuid

                if self.__execute_implementation_mc_namespace == self.local_mc_topic:
                    self.__internal_state = self.EXECUTE_LOCAL
                    return NodeMsg.RUNNING

                self.__execute_implementation_action = SimpleActionClient(
                    f'{self.__execute_implementation_mc_namespace}/execute_implementation',
                    ExecuteImplementationAction
                )
                self.__internal_state = self.WAIT_FOR_EXECUTION_SERVER

        if self.__internal_state == self.WAIT_FOR_EXECUTION_SERVER:
            if self.__execute_implementation_action.wait_for_server(
                    timeout=rospy.Duration.from_sec(0.0001)
            ):
                self.__internal_state = self.EXECUTE_REMOTE
                return NodeMsg.RUNNING

            self.__execute_implementation_mc_connection_attempts += 1
            if self.__execute_implementation_mc_connection_attempts > WAIT_FOR_REMOTE_MISSION_CONTROL_TRIES:
                self.logerr(
                    f'Exeeded connection ties {WAIT_FOR_REMOTE_MISSION_CONTROL_TRIES}, '
                    f'cannot connect to selected remote MC, reassigning!'
                )
                self.cleanup()
            else:
                self.__internal_state = self.WAIT_FOR_EXECUTION_SERVER

        return NodeMsg.UNASSIGNED

    def __get_local_implementation(self) -> Optional[CapabilityImplementation]:
        """
        Get local implementation from the mission controller and add it below the current node.

        :return: None if the implementation is not yet received from the remote server. A capability implementation, if
        a value was received and is correct.
        """
        if self.__get_local_implementations_service.get_state() == AsyncServiceProxy.IDLE:
            self.__get_local_implementations_service.call_service(
                self.capability_interface.uuid
            )
        if self.__get_local_implementations_service.get_state() == AsyncServiceProxy.RESPONSE_READY:
            local_implementations: List[CapabilityImplementation] = \
                self.__get_local_implementations_service.get_response()

            if len(local_implementations) < 1:
                self.logerr('No local implementation returned for the capability!')
                raise BehaviorTreeException('No local implementation returned for the capability!')

            local_implementation: Optional[CapabilityImplementation] = first_true(
                local_implementations,
                None,
                lambda impl: impl.implementation_uuid == self.__execute_implementation_uuid,
            )

            if local_implementation is None:
                self.logerr("Assigned local implementation not returned by Mission Control!")
                raise BehaviorTreeException('Assigned local implementation not returned by Mission Control!')

            return local_implementation

        if self.__get_local_implementations_service.get_state() in [AsyncServiceProxy.ABORTED, AsyncServiceProxy.ERROR,
                                                                    AsyncServiceProxy.TIMEOUT]:
            self.logerr("Service call to the local Mission Control failed!")
            raise BehaviorTreeException("Service call to the local Mission Control failed!")
        return None

    def __tick_executing_local(self) -> str:
        """
        Performs the ticking operation if the capability is executed locally.
        It retrieves the local implementation and performs the ticking operations.

        :raise BehaviorTreeException: If the implementation cannot be received or nodes cannot be instantiated.
        :return: The statue of the node from NodeMsg.
        """
        # No current implementation is present, one needs to be received from the mission controller.
        if self.__local_implementation is None:

            try:
                self.__local_implementation = self.__get_local_implementation()
            except BehaviorTreeException as exc:
                self.logerr("Cannot receive local implementation from mission control!")
                raise exc

            # We have not yet received the implementation, but the service is running.
            if self.__local_implementation is None:
                return NodeMsg.RUNNING

            # Implementation is present and we can add it below the capability.
            children_names = {}

            while len(children_names) != len(self.__local_implementation.nodes):
                added = 0
                # find nodes whose children are all in the tree already, then add them
                for node in (node for node in self.__local_implementation.nodes
                             if (node.name not in children_names
                                 and all((name in children_names for name in node.child_names)))):
                    try:
                        node_instance = self.from_msg(
                            node, debug_manager=self.debug_manager, permissive=False
                        )
                        for child_name in node.child_names:
                            node_instance.add_child(children_names[child_name])
                        children_names[node_instance.name] = node_instance

                        added += 1
                    except TypeError as exc:
                        raise BehaviorTreeException(str(exc)) from exc
                    except AttributeError as exc:
                        raise BehaviorTreeException(str(exc)) from exc
                if added == 0:
                    rospy.logerr("Empty local implementation provided!")
                    return NodeMsg.FAILED

                orphans = list(filter(lambda child: child.parent is None, children_names.values()))
                if len(orphans) > 1:
                    rospy.logwarn("More than one sub node for the capability detected.")
                if len(orphans) < 1:
                    rospy.logwarn("No root for capability subtree detected!")
                    self.cleanup()
                    return NodeMsg.UNASSIGNED

                for orphan in orphans:
                    self.add_child(orphan)

        return self.children[0].tick()

    def __tick_executing_remote(self) -> str:
        """
        Performs the ticking operation when operating on a remote node.

        :return: The status of the node as a NodeMsg status.
        """
        self.__execute_implementation_action.send_goal(
            ExecuteImplementationGoal(
                capability_uuid=self.capability_interface.uuid,
                implementation_uuid=self.__execute_implementation_uuid,
            )
        )
        self.__execute_implementation_action_start_time = rospy.Time.now()

        # Check if the action ran into a timeout.
        if (rospy.Time.now() - self.__execute_implementation_action_start_time
                > rospy.Duration(WAIT_FOR_REMOTE_EXECUTION_TIMEOUT_SECONDS)):
            self.logerr(
                f'ExecuteImplementation action timed out after'
                f' {WAIT_FOR_REMOTE_EXECUTION_TIMEOUT_SECONDS} seconds'
            )
            self.__internal_state = self.IDLE
            return NodeMsg.FAILED

        execute_implementation_state = self.__execute_implementation_action.get_state()

        if execute_implementation_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
            self.logerr(
                f'ExecuteImplementation action ended without succeeding'
                f' (state ID: {execute_implementation_state})'
            )
            self.cleanup()
            return NodeMsg.FAILED

        if execute_implementation_state in [GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
            self.logerr(
                f'ExecuteImplementation action was willingly canceled before succeeding'
                f' (state ID: {execute_implementation_state})'
            )
            self.cleanup()
            return NodeMsg.FAILED

        if execute_implementation_state == GoalStatus.SUCCEEDED:
            execute_implementation_action_result: ExecuteImplementationResult \
                = self.__execute_implementation_action.get_result()

            if not execute_implementation_action_result.success:
                self.logerr(
                    f'ExecuteImplementation action did not finish successfully:'
                    f' "{execute_implementation_action_result.error_message}"'
                )
                self.cleanup()
                return NodeMsg.FAILED

            # TODO: Copy results and publish to the subscribers
            return NodeMsg.SUCCEEDED
        return NodeMsg.RUNNING

    def __tick_running(self) -> NodeMsg:

        if self.__internal_state == self.EXECUTE_LOCAL:
            return self.__tick_executing_local()

        if self.__internal_state == self.EXECUTE_REMOTE:
            return self.__tick_executing_remote()

        return NodeMsg.RUNNING

    def _do_tick(self):

        if self.state == NodeMsg.UNASSIGNED:
            return self.__tick_unassigned()

        if self.state == NodeMsg.RUNNING:
            return self.__tick_running()

        return NodeMsg.BROKEN

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
        self.__execute_implementation_uuid = None
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
