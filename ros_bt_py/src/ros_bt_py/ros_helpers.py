from multiprocessing import Process, Manager

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
        self._data = self._manager.dict()
        self._data['state'] = self.IDLE
        self._data['req'] = None
        self._data['res'] = None
        self._data['proxy'] = self._service_proxy

    def stop_call(self):
        if self._process is not None and self._data['state'] == self.RUNNING:
            self._process.terminate()
            self._process.join()
            if self._process.is_alive():
                raise Exception('Failed to terminate process')
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
