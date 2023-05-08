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


import inspect
from threading import Event, Lock, Thread
from typing import Callable, Dict, Optional, Tuple, Type

import genpy

import rospy

from ros_bt_py.exceptions import BehaviorTreeException


class AsyncServiceProxy:
    """
    Implementation of an asynchronous service proxy for ROS services.

    For each service call a new thread is spawned to handle the ROS ServiceProxy.
    ROS ServiceProxy's are only created when required and are reused on subsequent calls to the
    same service.
    All AsyncServiceProxies share a list of allocated ServiceProxies via the Singleton pattern.
    """

    # define call states
    IDLE = 0
    RUNNING = 1
    RESPONSE_READY = 2
    ERROR = 3
    ABORTED = 4
    WAITING = 5
    TIMEOUT = 6
    SERVICE_AVAILABLE = 7

    class SerivceProxyInstance:
        """Allocated ROS ServiceProxy that can be claimed by AsyncServiceProxies on demand."""

        def __init__(self, service_name: str, service_type: str):
            """Allocate a new ServiceProxy for a given service.

            :param service_name: The ROS URI for the requested service.
            :type service_name: str
            :param service_type: The type of the service.
            :type service_type: str
            """
            self._service_proxy = rospy.ServiceProxy(service_name, service_type)
            self._claimed = False

        @property
        def claimed(self) -> bool:
            """If a AsyncServiceProxy was claimed for a service call.

            :return: Claimed status.
            :rtype: bool
            """
            return self._claimed

        @claimed.setter
        def claimed(self, new: bool):
            """Set the claimed status of the ServiceProxy.

            :param new: The new claimed status.
            :type new: bool
            """
            self._claimed = new

        @property
        def service_proxy(self) -> rospy.ServiceProxy:
            """The ServiceProxy allocated by this instance.

            :return: The manager ServiceProxy.
            :rtype: rospy.ServiceProxy
            """
            return self._service_proxy

    _service_proxies: Dict[Tuple[str, str], Dict[int, SerivceProxyInstance]] = {}
    """Shared dictornary of all allocated service proxies."""
    _singleton_lock = Lock()
    """Lock to ensure safe write access to the __service_proxies"""
    _id_counter = 0
    """Id counter used to identify allocated ServiceProxies."""

    def __new__(cls, *args, **kwargs):
        """Allocate a new instance of the AsyncServiceProxy.

        This sets up the singleton variables.

        :return: The newly created class instance object.
        :rtype: AsyncServiceProxy
        """
        obj = super(AsyncServiceProxy, cls).__new__(cls)
        obj.service_proxies = cls._service_proxies
        obj.singleton_lock = cls._singleton_lock
        obj.id_counter = cls._id_counter
        return obj

    def __init__(self, service_name: str, service_type: str):
        """Initialize the AsyncServiceProxy instance.

        :param service_name: The ROS URI for the requested service.
        :type service_name: str
        :param service_type: The type of the service.
        :type service_type: str
        """

        self._service_name: str = rospy.resolve_name(service_name)
        self._service_type: str = service_type
        self._data_lock: Lock = Lock()
        self._abort: Event = Event()
        self._thread: Optional[Thread] = None
        self._data: Dict = {
            "state": self.IDLE,
            "req": None,
            "res": None,
            "proxy": None,
            "timeout": None,
            "proxy_id": None,
        }

    def _claim_service_proxy(self) -> None:
        """Get the currently claimed service proxy or claim a new service proxy.

        :return: Return a service proxy for this service that is only used by this instance.
        :rtype: AsyncSerivceProxyInstance
        """
        with self.singleton_lock:
            if self._data["proxy_id"] is None:
                try:
                    service_proxies = self.service_proxies[
                        (self._service_name, self._service_type)
                    ]
                    try:
                        free_id: int = next(
                            filter(
                                lambda x: {not service_proxies[x].claimed},
                                service_proxies,
                            )
                        )
                        with self._data_lock:
                            self._data["proxy_id"] = free_id

                    except StopIteration:
                        rospy.logwarn(
                            "No free service handler found, allocating new one!"
                        )
                        proxy_record = self.SerivceProxyInstance(
                            service_name=self._service_name,
                            service_type=self._service_type,
                        )
                        proxy_record.claimed = True
                        self.service_proxies[(self._service_name, self._service_type)][
                            self.id_counter
                        ] = proxy_record
                        with self._data_lock:
                            self._data["proxy_id"] = self.id_counter
                        self.id_counter += 1

                except KeyError:
                    rospy.logwarn("Allocating initial service handler!")
                    proxy_record = self.SerivceProxyInstance(
                        service_name=self._service_name, service_type=self._service_type
                    )
                    proxy_record.claimed = True
                    self.service_proxies[(self._service_name, self._service_type)] = {
                        self.id_counter: proxy_record
                    }
                    with self._data_lock:
                        self._data["proxy_id"] = self.id_counter
                    self.id_counter += 1

            self.service_proxies[(self._service_name, self._service_type)][
                self._data["proxy_id"]
            ].claimed = True
            with self._data_lock:
                self._data["proxy"] = self.service_proxies[
                    (self._service_name, self._service_type)
                ][self._data["proxy_id"]].service_proxy

    def _unclaim_service_proxy(self):
        """Unclaim the currently claimed service proxy."""
        if self._data["proxy_id"] is not None:
            with self.singleton_lock:
                self.service_proxies[(self._service_name, self._service_type)][
                    self._data["proxy_id"]
                ].claimed = False

            with self._data_lock:
                self._data["proxy"] = None
                self._data["proxy_id"] = None

    def wait_for_service(self, timeout=None):
        """Async implementation of rospy.wait_for_service to be used in tick() methods"""

        if self._data["state"] == self.WAITING:
            rospy.logwarn("Already waiting on %s", self._service_name)
            return
        if self._thread is not None:
            # try to join the process, if that doesn't work, KILL IT WITH FIRE
            self._thread.join(0)
            if self._thread.is_alive():
                self.stop_call()
            self._thread = None

        if self._thread is None:
            with self._data_lock:
                self._data["state"] = self.WAITING
                self._data["timeout"] = timeout
            self._abort.clear()
            self._thread = Thread(
                target=AsyncServiceProxy._wait_for_service_impl,
                args=(
                    self._data,
                    self._data_lock,
                    self._abort,
                    self._claim_service_proxy,
                    self._unclaim_service_proxy,
                ),
            )
            self._thread.start()

    def shutdown(self):
        self.stop_call()
        self._unclaim_service_proxy()

    def stop_call(self):
        if self._thread is not None:
            self._abort.set()
            self._thread = None
            with self._data_lock:
                self._data["state"] = self.ABORTED
                self._data["timeout"] = None

    def call_service(self, req):

        if self._data["state"] == self.RUNNING:
            rospy.logwarn("Aborting previous call to %s", self._service_name)
            self.stop_call()
        if self._thread is not None:
            # try to join the process, if that doesn't work, KILL IT WITH FIRE
            self._thread.join(0)
            if self._thread.is_alive():
                self.stop_call()
            self._thread = None
        if self._thread is None:
            with self._data_lock:
                self._data["req"] = req
                self._data["res"] = None
                self._data["state"] = self.RUNNING

            self._abort.clear()
            self._thread = Thread(
                target=AsyncServiceProxy._call_service_impl,
                args=(
                    self._data,
                    self._data_lock,
                    self._abort,
                    self._claim_service_proxy,
                    self._unclaim_service_proxy,
                ),
            )
            self._thread.start()

    def get_response(self):
        if self._data["state"] == self.RESPONSE_READY:
            with self._data_lock:
                self._data["state"] = self.IDLE
        return self._data["res"]

    def get_state(self):
        """Get the current state of the service proxy."""
        return self._data["state"]

    @staticmethod
    def _wait_for_service_impl(
        data: Dict,
        lock: Lock,
        abort: Event,
        claim_cb: Callable[[], None],
        unclaim_cb: Callable[[], None],
    ):
        """Background function that waits for a given service to be available.

        :param data: Data object beloning to the AsyncServiceProxy that contains all required data.
        :type data: Dict
        :param lock: Lock use to secure access to the data parameter.
        :type lock: Lock
        :param abort: Event inidicating whether to abort the call.
        :type abort: Event
        :param claim_cb: Fuction to claim an available ServiceProxy on the
        calling AsyncServiceProxy for the operation.
        :type claim_cb: Callable[[], None]
        :param unclaim_cb: Function to unclaim the claimed ServiceProxy.
        :type unclaim_cb: Callable[[], None]
        """

        claim_cb()
        try:
            data["proxy"].wait_for_service(data["timeout"])
            if abort.is_set():
                unclaim_cb()
                return
            with lock:
                data["timeout"] = None
                data["state"] = AsyncServiceProxy.SERVICE_AVAILABLE
        except AttributeError as exc:
            rospy.logerr(f"Service proxy is not present: {exc}")
            if abort.is_set():
                unclaim_cb()
                return
            with lock:
                data["state"] = AsyncServiceProxy.ERROR
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("wait_for_service aborted from other process")
        except rospy.exceptions.ROSException as e:
            rospy.logerr(str(e))
            if abort.is_set():
                unclaim_cb()
                return
            with lock:
                data["state"] = AsyncServiceProxy.TIMEOUT
        except Exception as e:
            rospy.logerr("Error waiting for service service: %s", str(e))
            if abort.is_set():
                unclaim_cb()
                return
            with lock:
                data["state"] = AsyncServiceProxy.ERROR
        unclaim_cb()

    @staticmethod
    def _call_service_impl(
        data: Dict,
        lock: Lock,
        abort: Event,
        claim_cb: Callable[[], None],
        unclaim_cb: Callable[[], None],
    ):
        """Function that calls a given service with the provided requests.

        :param data: Data object beloning to the AsyncServiceProxy that contains all required data.
        :type data: Dict
        :param lock: Lock use to secure access to the data parameter.
        :type lock: Lock
        :param abort: Event inidicating whether to abort the call.
        :type abort: Event
        :param claim_cb: Fuction to claim an available ServiceProxy on the
        calling AsyncServiceProxy for the operation.
        :type claim_cb: Callable[[], None]
        :param unclaim_cb: Function to unclaim the claimed ServiceProxy.
        :type unclaim_cb: Callable[[], None]
        """
        claim_cb()
        try:
            res = data["proxy"].call(data["req"])
            if abort.is_set():
                unclaim_cb()
                return
            with lock:
                data["res"] = res
                data["state"] = AsyncServiceProxy.RESPONSE_READY
        except AttributeError as exc:
            rospy.logerr(f"Service proxy is not present: {exc}")
            with lock:
                data["state"] = AsyncServiceProxy.ERROR
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("Service call aborted from other process")
        except Exception as e:
            rospy.logerr("Error calling service: %s", str(e))
            with lock:
                data["state"] = AsyncServiceProxy.ERROR
        unclaim_cb()


class LoggerLevel(object):
    """Data class containing a logging level."""

    def __init__(self, logger_level="info"):
        """Initialize a new LoggerLevel class."""
        self.logger_level = logger_level


class EnumValue(object):
    """Data class containing an enum value."""

    def __init__(self, enum_value=""):
        """Initialize a new EnumValue class."""
        self.enum_value = enum_value


def get_message_constant_fields(message_class):
    """Returns all constant fields of a message as a list"""
    if (
        inspect.isclass(message_class)
        and genpy.message.Message in message_class.__mro__
    ):
        msg = message_class()

        attributes = dir(message_class)
        methods = [
            method[0]
            for method in inspect.getmembers(message_class, predicate=inspect.ismethod)
        ]

        numpy_methods = ["deserialize_numpy", "serialize_numpy"]

        # filter out everything that is not a CONSTANT
        attributes_message = dir(genpy.Message)
        constants = [
            item
            for item in attributes
            if (
                not item.startswith("_")
                and item not in methods
                and item not in msg.__slots__
                and item not in attributes_message
                and item not in numpy_methods
            )
        ]
        return constants
    else:
        raise BehaviorTreeException(f"{message_class} is not a ROS Message")
