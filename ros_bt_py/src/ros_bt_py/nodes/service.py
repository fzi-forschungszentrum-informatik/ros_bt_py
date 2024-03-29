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


from roslib.message import get_service_class
import rospy
import rosservice

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.ros_helpers import AsyncServiceProxy

from abc import ABC, abstractmethod
from typing import Optional, Dict


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={
            "service_type": type,
            "request_type": type,
            "response_type": type,
            "wait_for_response_seconds": float,
        },
        inputs={
            "request": OptionRef("request_type"),
            "service_name": str,
        },
        outputs={"response": OptionRef("response_type")},
        max_children=0,
    )
)
class ServiceInput(Leaf):
    """Calls a ROS service with the provided Request data.

    To make sure the service call cannot block the :meth:`tick()`
    method, this uses a :class:`ros_bt_py.ros_helpers.AsyncServiceProxy`
    behind the scenes.

    Due to the way that class works (it uses the `multiprocessing`
    module), the first tick of this node will almost certainly leave it
    in the RUNNING state, even if the service responds very quicly.

    If this node is ticked again after having returned SUCCEEDED or
    FAILED, it will call the service again with the now current request
    data.
    """

    def _do_setup(self):
        self._service_proxy = None
        self._last_service_call_time = None
        self._last_request = None
        self._reported_result = False
        self.outputs["response"] = None

        return NodeMsg.IDLE

    def _do_reset(self):
        if self._service_proxy is not None:
            self._service_proxy.shutdown()
            self._service_proxy = None
        self._last_service_call_time = None
        self._last_request = None
        self._reported_result = False
        self.outputs["response"] = None

        return NodeMsg.IDLE

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        # If the service name changed
        if self.inputs.is_updated("service_name"):
            if self._service_proxy is not None:
                self._do_reset()
        if self._service_proxy is None:
            self._service_proxy = AsyncServiceProxy(
                self.inputs["service_name"], self.options["service_type"]
            )

        # If theres' no service call in-flight, and we have already reported
        # the result (see below), start a new call and save the request
        if (
            self._reported_result
            or self._service_proxy.get_state() == AsyncServiceProxy.IDLE
            or self._service_proxy.get_state() == AsyncServiceProxy.ABORTED
        ):
            self._last_service_call_time = rospy.Time.now()
            self._last_request = self.inputs["request"]
            self._reported_result = False
            self._service_proxy.call_service(self._last_request)

        if self._service_proxy.get_state() == AsyncServiceProxy.RUNNING:
            # If the call takes longer than the specified timeout, abort the
            # call and return FAILED
            seconds_since_call = (
                rospy.Time.now() - self._last_service_call_time
            ).to_sec()
            if seconds_since_call > self.options["wait_for_response_seconds"]:
                self.logerr(
                    "Service call to %s with request %s timed out"
                    % (self.inputs["service_name"], self._last_request)
                )
                self._service_proxy.stop_call()
                return NodeMsg.FAILED

            return NodeMsg.RUNNING
        else:
            new_state = NodeMsg.SUCCEEDED
            if self._service_proxy.get_state() == AsyncServiceProxy.RESPONSE_READY:
                self.outputs["response"] = self._service_proxy.get_response()
            if self._service_proxy.get_state() == AsyncServiceProxy.ERROR:
                # TODO(nberg): Leave old response or set to None?
                self._service_proxy.stop_call()
                new_state = NodeMsg.FAILED

            self._reported_result = True
            return new_state

    def _do_untick(self):
        if self._service_proxy is not None:
            self._service_proxy.stop_call()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        if self._service_proxy is not None:
            self._service_proxy.shutdown()

    def _do_calculate_utility(self):
        resolved_service = rospy.resolve_name(self.inputs["service_name"])

        try:
            service_type_name = rosservice.get_service_type(resolved_service)
        except rosservice.ROSServiceIOException as exc:
            # Defaults to no bounds set, dragging down the utility
            # score
            self.loginfo(f"Unable to check for service {resolved_service}: {str(exc)}")
            return UtilityBounds()

        if service_type_name:
            service_type = get_service_class(service_type_name)

            if service_type == self.options["service_type"]:
                self.loginfo(
                    (
                        "Found service %s with correct type, returning "
                        "filled out UtilityBounds"
                    )
                    % resolved_service
                )
                return UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True,
                )

        self.loginfo(f"Service {resolved_service} is unavailable or has wrong type.")
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={
            "service_type": type,
            "service_name": str,
            "wait_for_service_seconds": float,
        },
        inputs={},
        outputs={},
        max_children=0,
        optional_options=[],
    )
)
class WaitForService(Leaf):
    """Waits for a service to be available, fails if this wait times out"""

    def _do_setup(self):
        self._service_proxy = AsyncServiceProxy(
            self.options["service_name"], self.options["service_type"]
        )

    def _do_tick(self):
        if (
            self._service_proxy.get_state() == AsyncServiceProxy.IDLE
            or self._service_proxy.get_state() == AsyncServiceProxy.ABORTED
        ):
            self._last_service_call_time = rospy.Time.now()
            self._service_proxy.wait_for_service(
                self.options["wait_for_service_seconds"]
            )
        if self._service_proxy.get_state() == AsyncServiceProxy.WAITING:
            return NodeMsg.RUNNING
        if self._service_proxy.get_state() == AsyncServiceProxy.SERVICE_AVAILABLE:
            return NodeMsg.SUCCEEDED
        return NodeMsg.FAILED

    def _do_untick(self):
        self._service_proxy.stop_call()
        return NodeMsg.IDLE

    def _do_reset(self):
        self._service_proxy.stop_call()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self._service_proxy.shutdown()


@define_bt_node(
    NodeConfig(
        options={
            "service_name": str,
            "wait_for_service_seconds": float,
            "wait_for_response_seconds": float,
            "fail_if_not_available": bool,
        },
        inputs={},
        outputs={},
        max_children=0,
        optional_options=["fail_if_not_available"],
    )
)
class ServiceForSetType(ABC, Leaf):
    """Abstract ROS service class.

    Inherit form this class to create a ROS service node with
    a defined service type and build a cleaner Behavior Tree.

    To make sure the service call cannot block the :meth:`tick()`
    method, this uses a :class:`ros_bt_py.ros_helpers.AsyncServiceProxy`
    behind the scenes.

    Due to the way that class works (it uses the `multiprocessing`
    module), the first tick of this node will almost certainly leave it
    in the RUNNING state, even if the service responds very quickly.

    If this node is ticked again after having returned SUCCEEDED or
    FAILED, it will call the service again with the now current request
    data.

    Example:
        >>> @define_bt_node(NodeConfig(
                options={'MyOption' : MyOptionType},
                inputs ={'MyInput' : MyInputType},
                outputs={'MyOutput' : MyOutputType},
                max_children=0,
                optional_options=['fail_if_not_available']))
        >>> class CallMyService(ServiceForSetType):

                # Set the service type
                def set_service_type(self):
                    self._service_type = MyServiceType

                # Set all outputs to none (define output key while overwriting)
                def set_output_none(self):
                    self.outputs['MyServiceOutput'] = None

                # Sets the service request message, sent to the service.
                def set_request(self):
                    self._last_request = MyServiceRequest()

                # Sets the output (in relation to the response)
                # (define output key while overwriting)
                # it should return True, if the node state should be SUCCEEDED after receiving
                #the message and False. if it should be in the FAILED state
                def set_outputs(self):
                    self.outputs['MyServiceOutput'] =\
                    self._service_proxy.get_response().MyServiceOutput
                    return self.outputs[’MyServiceOutput']
    """

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: Optional[str] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ) -> None:
        super().__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        self._service_name = self.options["service_name"]
        self.set_service_type()

    # Sets all outputs none (define output key while overwriting)
    @abstractmethod
    def set_output_none(self):
        self.outputs["OUTPUT_KEY"] = None

    # Returns the service request message that should be send to the service.
    @abstractmethod
    def set_request(self):
        pass

    # Sets the output (in relation to the response) (define output key while overwriting)
    # Should return True, if the node state should be SUCCEEDED after receiving the message
    # and False, if it's in the FAILED state
    @abstractmethod
    def set_outputs(self):
        self.outputs["OUTPUT_KEY"] = self._service_proxy.get_response()
        return "TRUTHVALUE"

    # Sets the service type
    @abstractmethod
    def set_service_type(self):
        self._service_type = "SERVICE_TYPE"

    def _do_setup(self):
        self._service_available = True
        # Exception if service is not available
        try:
            rospy.wait_for_service(
                self._service_name, self.options["wait_for_service_seconds"]
            )
        except rospy.ROSException as e:
            if (
                "fail_if_not_available" in self.options
                and self.options["fail_if_not_available"]
            ):
                self._service_available = False
            else:
                raise e

        self._service_proxy = AsyncServiceProxy(self._service_name, self._service_type)

        self._last_service_call_time = None
        self._last_request = None
        self._reported_result = False
        self.set_output_none()
        return NodeMsg.IDLE

    def _do_reset(self):
        self._service_proxy.stop_call()
        self._last_service_call_time = None
        self._last_request = None
        self._reported_result = False
        self.set_output_none()
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if not self._service_available:
            return NodeMsg.FAILED
        # If theres' no service call in-flight, and we have already reported
        # the result (see below), start a new call and save the request
        if (
            self._reported_result
            or self._service_proxy.get_state() == AsyncServiceProxy.IDLE
            or self._service_proxy.get_state() == AsyncServiceProxy.ABORTED
        ):
            self._last_service_call_time = rospy.Time.now()
            self.set_request()
            self._reported_result = False
            self._service_proxy.call_service(self._last_request)

        if self._service_proxy.get_state() == AsyncServiceProxy.RUNNING:
            # If the call takes longer than the specified timeout, abort the
            # call and return FAILED
            seconds_since_call = (
                rospy.Time.now() - self._last_service_call_time
            ).to_sec()
            if seconds_since_call > self.options["wait_for_response_seconds"]:
                self.logerr(
                    "Service call to %s with request %s timed out"
                    % (self._service_name, self._last_request)
                )
                self._service_proxy.stop_call()
                return NodeMsg.FAILED

            return NodeMsg.RUNNING
        else:
            # returns failed except the set.ouput() method returns True
            new_state = NodeMsg.FAILED
            if self._service_proxy.get_state() == AsyncServiceProxy.RESPONSE_READY:
                if self.set_outputs():
                    new_state = NodeMsg.SUCCEEDED

            if self._service_proxy.get_state() == AsyncServiceProxy.ERROR:
                self._service_proxy.stop_call()

            self._reported_result = True
            return new_state

    def _do_untick(self):
        self._service_proxy.stop_call()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self._service_proxy.shutdown()

    def _do_calculate_utility(self):
        resolved_service = rospy.resolve_name(self.options["service_name"])

        try:
            service_type_name = rosservice.get_service_type(resolved_service)
        except rosservice.ROSServiceIOException as exc:
            # Defaults to no bounds set, dragging down the utility
            # score
            self.loginfo(f"Unable to check for service {resolved_service}: {str(exc)}")
            return UtilityBounds()

        if service_type_name:
            service_type = get_service_class(service_type_name)

            if service_type == self._service_type:
                self.loginfo(
                    (
                        "Found service %s with correct type, returning "
                        "filled out UtilityBounds"
                    )
                    % resolved_service
                )
                return UtilityBounds(
                    can_execute=True,
                    has_lower_bound_success=True,
                    has_upper_bound_success=True,
                    has_lower_bound_failure=True,
                    has_upper_bound_failure=True,
                )

        self.loginfo(f"Service {resolved_service} is unavailable or has wrong type.")
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"service_type": type, "wait_for_service_seconds": float},
        inputs={"service_name": str},
        outputs={},
        max_children=0,
        optional_options=[],
    )
)
class WaitForServiceInput(Leaf):
    """Waits for a service to be available, fails if this wait times out"""

    def _do_setup(self):
        self._service_proxy = None

    def _do_tick(self):
        if self.simulate_tick:
            self.logdebug(f"Simulating tick. {self.name} is not executing!")
            if self.succeed_always:
                return NodeMsg.SUCCEEDED

            return NodeMsg.RUNNING

        if self._service_proxy is None:
            self._service_proxy = AsyncServiceProxy(
                self.inputs["service_name"], self.options["service_type"]
            )

        if (
            self._service_proxy.get_state() == AsyncServiceProxy.IDLE
            or self._service_proxy.get_state() == AsyncServiceProxy.ABORTED
        ):
            self._service_proxy.wait_for_service(
                self.options["wait_for_service_seconds"]
            )
        if self._service_proxy.get_state() == AsyncServiceProxy.WAITING:
            return NodeMsg.RUNNING
        if self._service_proxy.get_state() == AsyncServiceProxy.SERVICE_AVAILABLE:
            self._do_reset()
            return NodeMsg.SUCCEEDED
        self._do_reset()
        return NodeMsg.FAILED

    def _do_untick(self):
        if self._service_proxy is not None:
            self._service_proxy.stop_call()
        return NodeMsg.IDLE

    def _do_reset(self):
        if self._service_proxy is not None:
            self._service_proxy.stop_call()
            self._service_proxy.shutdown()
        self._service_proxy = None
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self._do_reset()


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={"service_type": type, "request_type": type, "response_type": type},
        inputs={"request": OptionRef("request_type")},
        outputs={"response": OptionRef("response_type")},
        max_children=0,
    )
)
class Service(ServiceForSetType):
    """Calls a ROS service with the provided Request data.

    To make sure the service call cannot block the :meth:`tick()`
    method, this uses a :class:`ros_bt_py.ros_helpers.AsyncServiceProxy`
    behind the scenes.
    Due to the way that class works (it uses the `multiprocessing`
    module), the first tick of this node will almost certainly leave it
    in the RUNNING state, even if the service responds very quicly.

    If this node is ticked again after having returned SUCCEEDED or
    FAILED, it will call the service again with the now current request
    data.
    """

    def set_service_type(self):
        self._service_type = self.options["service_type"]

    # Set all outputs to none (define output key while overwriting)
    def set_output_none(self):
        self.outputs["response"] = None

    # Sets the service request message, sent to the service.
    def set_request(self):
        self._last_request = self.inputs["request"]

    # Sets the output (in relation to the response) (define output key while overwriting)
    # it should return True, if the node state should be SUCCEEDED after receiving the message and
    # False. if it should be in the FAILED state
    def set_outputs(self):
        self.outputs["response"] = self._service_proxy.get_response()
        print(f"{self.outputs['response']}")
        return self.outputs["response"]
