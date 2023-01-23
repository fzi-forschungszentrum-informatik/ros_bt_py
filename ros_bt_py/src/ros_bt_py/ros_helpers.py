#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
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
#    * Neither the name of the {copyright_holder} nor the names of its
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
#  -------- END LICENSE BLOCK --------
from multiprocessing import Process, Manager
import os
import signal

import inspect
from threading import Lock
from typing import Dict, Optional

import genpy

import rospy

from ros_bt_py.exceptions import BehaviorTreeException


class AsyncServiceProxy:
    """Implementation of an asynchronous service proxy for ROS services."""

    # define call states
    IDLE = 0
    RUNNING = 1
    RESPONSE_READY = 2
    ERROR = 3
    ABORTED = 4
    WAITING = 5
    TIMEOUT = 6
    SERVICE_AVAILABLE = 7

    class AsyncSerivceProxyInstance:
        def __init__(self, service_name: str, service_type: str):
            self._claimed = False
            self._process = None
            self._service_name = service_name
            self._service_proxy = rospy.ServiceProxy(service_name, service_type)

            self._manager = Manager()
            self._data = self._manager.dict()
            self._data["state"] = AsyncServiceProxy.IDLE
            self._data["req"] = None
            self._data["res"] = None
            self._data["proxy"] = self._service_proxy
            self._data["timeout"] = None

        @property
        def claimed(self) -> bool:
            return self._claimed

        @claimed.setter
        def claimed(self, new: bool):
            self._claimed = new

        def wait_for_service(self, timeout=None):
            """Async implementation of rospy.wait_for_service to be used in tick() methods"""

            if self._data["state"] == AsyncServiceProxy.WAITING:
                rospy.logwarn("Already waiting on %s", self._service_name)
                return
            if self._process is not None:
                # try to join the process, if that doesn't work, KILL IT WITH FIRE
                self._process.join(0)
                if self._process.is_alive():
                    self.stop_call()
                self._process = None
            if self._process is None:
                self._data["state"] = AsyncServiceProxy.WAITING
                self._data["timeout"] = timeout
                self._process = Process(
                    target=_wait_for_service_impl, args=(self._data,)
                )
                self._process.start()

        def shutdown(self):
            self.stop_call()
            if self._data["proxy"] is not None:
                self._data["proxy"].close()
            self._data["proxy"] = None

        def stop_call(self):
            if self._process is not None:
                # kill -9 the stuck process - not clean, but reliable
                # Fire...
                try:
                    os.kill(self._process.pid, signal.SIGKILL)
                except OSError:
                    pass
                # and forget!
                self._process = None
                self._data["state"] = AsyncServiceProxy.ABORTED
                self._data["timeout"] = None

        def call_service(self, req):
            if self._data["state"] == AsyncServiceProxy.RUNNING:
                rospy.logwarn("Aborting previous call to %s", self._service_name)
                self.stop_call()
            if self._process is not None:
                # try to join the process, if that doesn't work, KILL IT WITH FIRE
                self._process.join(0)
                if self._process.is_alive():
                    self.stop_call()
                self._process = None
            if self._process is None:
                self._data["req"] = req
                self._data["res"] = None
                self._data["state"] = AsyncServiceProxy.RUNNING

                self._process = Process(target=_call_service_impl, args=(self._data,))
                self._process.start()

        def get_response(self):
            if self._data["state"] == AsyncServiceProxy.RESPONSE_READY:
                self._data["state"] = AsyncServiceProxy.IDLE
            return self._data["res"]

        def get_state(self):
            return self._data["state"]

    _shared_state = {}

    def __new__(cls, *args, **kwargs):
        obj = super(AsyncServiceProxy, cls).__new__(cls)
        obj.__dict__ = cls._shared_state
        if not hasattr(obj, "service_proxies"):
            obj.service_proxies: Dict[
                str, Dict[int, cls.AsyncSerivceProxyInstance]
            ] = {}
        if not hasattr(obj, "singleton_lock"):
            obj.singleton_lock = Lock()
        if not hasattr(obj, "id_counter"):
            obj.id_counter = 0
        return obj

    def __init__(self, service_name, service_type):
        with self.singleton_lock:
            if service_name not in self.service_proxies:
                rospy.loginfo(
                    f"AsyncServiceProxy for service :'{service_name}' created!"
                )
                proxy_record = self.AsyncSerivceProxyInstance(
                    service_name=service_name, service_type=service_type
                )
                self.service_proxies[service_name] = {self.id_counter: proxy_record}
                self.id_counter += 1
            else:
                rospy.logdebug(
                    f"AsyncServiceProxy for service :'{service_name}' already exists!"
                )

        self._service_name: str = service_name
        self._service_type: str = service_type
        self._current_proxy_id: Optional[int] = None

    def _get_claimed_service_proxy(self) -> AsyncSerivceProxyInstance:
        """Get the currently claimed service proxy or claim a new service proxy.

        :return: Return a service proxy for this service that is only used by this instance.
        :rtype: AsyncSerivceProxyInstance
        """
        with self.singleton_lock:
            if self._current_proxy_id is None:
                try:
                    free_id: int = next(
                        filter(
                            lambda x: {
                                not self.service_proxies[self._service_name][x].claimed
                            },
                            self.service_proxies[self._service_name],
                        )
                    )
                    self._current_proxy_id = free_id
                except StopIteration:
                    rospy.logwarn("No free service handler found, allocating new one!")
                    proxy_record = self.AsyncSerivceProxyInstance(
                        service_name=self._service_name, service_type=self._service_type
                    )
                    proxy_record.claimed = True
                    self.service_proxies[self._service_name] = {
                        self.id_counter: proxy_record
                    }
                    self._current_proxy_id = self.id_counter
                    self.id_counter += 1
            self.service_proxies[self._service_name][
                self._current_proxy_id
            ].claimed = True
            return self.service_proxies[self._service_name][self._current_proxy_id]

    def _unclaim_service_proxy(self):
        """Unclaim the currently claimed service proxy."""
        if self._current_proxy_id is not None:
            with self.singleton_lock:
                self.service_proxies[self._service_name][
                    self._current_proxy_id
                ].claimed = False
            self._current_proxy_id = None

    def wait_for_service(self, timeout=None):
        """Async implementation of rospy.wait_for_service to be used in tick() methods."""
        proxy_record = self._get_claimed_service_proxy()
        proxy_record.wait_for_service(timeout=timeout)

    def shutdown(self):
        proxy_record = self._get_claimed_service_proxy()
        proxy_record.shutdown()
        self._unclaim_service_proxy()

    def stop_call(self):
        proxy_record = self._get_claimed_service_proxy()
        proxy_record.stop_call()
        self._unclaim_service_proxy()

    def call_service(self, req):
        proxy_record = self._get_claimed_service_proxy()
        proxy_record.call_service(req=req)

    def get_response(self):
        proxy_record = self._get_claimed_service_proxy()
        resp = proxy_record.get_response()
        if proxy_record.get_state() == self.IDLE:
            self._unclaim_service_proxy()
        return resp

    def get_state(self):
        proxy_record = self._get_claimed_service_proxy()
        return proxy_record.get_state()


def _wait_for_service_impl(data):
    try:
        data["proxy"].wait_for_service(data["timeout"])
        data["timeout"] = None
        data["state"] = AsyncServiceProxy.SERVICE_AVAILABLE
    except AttributeError:
        rospy.logerr("Service proxy is not present!")
        data["state"] = AsyncServiceProxy.ERROR
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn("wait_for_service aborted from other process")
    except rospy.exceptions.ROSException as e:
        rospy.logerr(str(e))
        data["state"] = AsyncServiceProxy.TIMEOUT
    except Exception as e:
        rospy.logerr("Error waiting for service service: %s", str(e))
        data["state"] = AsyncServiceProxy.ERROR


def _call_service_impl(data):
    try:
        res = data["proxy"].call(data["req"])
        data["res"] = res
        data["state"] = AsyncServiceProxy.RESPONSE_READY
    except AttributeError:
        rospy.logerr("Service proxy is not present!")
        data["state"] = AsyncServiceProxy.ERROR
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn("Service call aborted from other process")
    except Exception as e:
        rospy.logerr("Error calling service: %s", str(e))
        data["state"] = AsyncServiceProxy.ERROR


class LoggerLevel(object):
    def __init__(self, logger_level="info"):
        self.logger_level = logger_level


class EnumValue(object):
    def __init__(self, enum_value=""):
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
        raise BehaviorTreeException("%s is not a ROS Message" % (message_class))
