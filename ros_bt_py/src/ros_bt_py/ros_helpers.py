from multiprocessing import Process, Manager
import os
import signal

import rospy


class AsyncServiceProxy(object):
    # define call states
    IDLE = 0
    RUNNING = 1
    RESPONSE_READY = 2
    ERROR = 3
    ABORTED = 4

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

    def shutdown(self):
        self.stop_call()
        self._data['proxy'].shutdown()
        self._data['proxy'] = None

    def stop_call(self):
        if self._process is not None and self._data['state'] == self.RUNNING:
            # kill -9 the stuck process - not clean, but reliable
            # Fire...
            os.kill(self._process.pid, signal.SIGKILL)
            # and forget!
            self._process = None
            self._data['state'] = AsyncServiceProxy.ABORTED

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
        self._data['state'] = self.IDLE
        return self._data['res']

    def get_state(self):
        return self._data['state']


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
    except Exception, e:
        rospy.logerr('Error calling service: %s', str(e))
        data['state'] = AsyncServiceProxy.ERROR
