from multiprocessing import Process, Manager
import os
import signal

import inspect

import genpy

import rospy

from ros_bt_py.exceptions import BehaviorTreeException


class AsyncServiceProxy(object):
    # define call states
    IDLE = 0
    RUNNING = 1
    RESPONSE_READY = 2
    ERROR = 3
    ABORTED = 4
    WAITING = 5
    TIMEOUT = 6
    SERVICE_AVAILABLE = 7

    def __init__(self, service_name, service_type):
        self._process = None
        self._service_name = service_name
        self._service_proxy = rospy.ServiceProxy(service_name, service_type)

        self._manager = Manager()
        # TODO(nberg): Check if this is really safe to access without a Lock
        self._data = self._manager.dict()
        self._data['state'] = self.IDLE
        self._data['req'] = None
        self._data['res'] = None
        self._data['proxy'] = self._service_proxy
        self._data['timeout'] = None

    def wait_for_service(self, timeout=None):
        """Async implementation of rospy.wait_for_service to be used in tick() methods
        """
        if self._data['state'] == self.WAITING:
            rospy.logwarn('Already waiting on %s', self._service_name)
            return
        if self._process is not None:
            # try to join the process, if that doesn't work, KILL IT WITH FIRE
            self._process.join(0)
            if self._process.is_alive():
                self.stop_call()
            self._process = None
        if self._process is None:
            self._data['state'] = self.WAITING
            self._data['timeout'] = timeout
            self._process = Process(target=_wait_for_service_impl, args=(self._data,))
            self._process.start()

    def shutdown(self):
        self.stop_call()
        if self._data['proxy'] is not None:
            self._data['proxy'].close()
        self._data['proxy'] = None

    def stop_call(self):
        if self._process is not None:
            # kill -9 the stuck process - not clean, but reliable
            # Fire...
            try:
                os.kill(self._process.pid, signal.SIGKILL)
            except OSError as e:
                pass
            # and forget!
            self._process = None
            self._data['state'] = AsyncServiceProxy.ABORTED
            self._data['timeout'] = None

    def call_service(self, req):
        if self._data['state'] == self.RUNNING:
            rospy.logwarn('Aborting previous call to %s', self._service_name)
            self.stop_call()
        if self._process is not None:
            # try to join the process, if that doesn't work, KILL IT WITH FIRE
            self._process.join(0)
            if self._process.is_alive():
                self.stop_call()
            self._process = None
        if self._process is None:
            self._data['req'] = req
            self._data['res'] = None
            self._data['state'] = self.RUNNING

            self._process = Process(target=_call_service_impl, args=(self._data,))
            self._process.start()

    def get_response(self):
        if self._data['state'] == self.RESPONSE_READY:
            self._data['state'] = self.IDLE
        return self._data['res']

    def get_state(self):
        return self._data['state']


def _wait_for_service_impl(data):
    try:
        data['proxy'].wait_for_service(data['timeout'])
        data['timeout'] = None
        data['state'] = AsyncServiceProxy.SERVICE_AVAILABLE
    except AttributeError:
        rospy.logerr('Service proxy is not present!')
        data['state'] = AsyncServiceProxy.ERROR
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn('wait_for_service aborted from other process')
    except rospy.exceptions.ROSException as e:
        rospy.logerr(str(e))
        data['state'] = AsyncServiceProxy.TIMEOUT
    except Exception as e:
        rospy.logerr('Error waiting for service service: %s', str(e))
        data['state'] = AsyncServiceProxy.ERROR


def _call_service_impl(data):
    try:
        res = data['proxy'].call(data['req'])
        data['res'] = res
        data['state'] = AsyncServiceProxy.RESPONSE_READY
    except AttributeError:
        rospy.logerr('Service proxy is not present!')
        data['state'] = AsyncServiceProxy.ERROR
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn('Service call aborted from other process')
    except Exception as e:
        rospy.logerr('Error calling service: %s', str(e))
        data['state'] = AsyncServiceProxy.ERROR


class LoggerLevel(object):
    def __init__(self, logger_level='info'):
        self.logger_level = logger_level


class EnumValue(object):
    def __init__(self, enum_value=''):
        self.enum_value = enum_value


def get_message_constant_fields(message_class):
    """Returns all constant fields of a message as a list
    """
    if (inspect.isclass(message_class) and
            genpy.message.Message in message_class.__mro__):
        msg = message_class()

        attributes = dir(message_class)
        methods = [method[0]
                   for method in inspect.getmembers(
                       message_class, predicate=inspect.ismethod)]

        # filter out everything that is not a CONSTANT
        attributes_message = dir(genpy.Message)
        constants = [item
                     for item in attributes
                     if (not item.startswith('_') and
                         item not in methods and
                         item not in msg.__slots__ and
                         item not in attributes_message)]
        return constants
    else:
        raise BehaviorTreeException('%s is not a ROS Message' % (
            message_class
        ))
