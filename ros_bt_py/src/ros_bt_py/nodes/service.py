from threading import Lock
import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.ros_helpers import AsyncServiceProxy


@define_bt_node(NodeConfig(
    options={'service_type': type,
             'request_type': type,
             'response_type': type,
             'service_name': str,
             'wait_for_service_seconds': float,
             'wait_for_response_seconds': float},
    inputs={'request': OptionRef('request_type')},
    outputs={'response': OptionRef('response_type')},
    max_children=0))
class Service(Leaf):
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
        # throws if service is not available
        rospy.wait_for_service(self.options['service_name'],
                               self.options['wait_for_service_seconds'])

        self._service_proxy = AsyncServiceProxy(self.options['service_name'],
                                                self.options['service_type'])
        self._last_service_call_time = None
        self._last_request = None
        self._reported_result = False
        self.outputs['response'] = None

        return NodeMsg.IDLE

    def _do_reset(self):
        self._service_proxy.stop_call()
        self._last_service_call_time = None
        self._last_request = None
        self._reported_result = False
        self.outputs['response'] = None

        return NodeMsg.IDLE

    def _do_tick(self):
        # If theres' no service call in-flight, and we have already reported
        # the result (see below), start a new call and save the request
        proxy_state = self._service_proxy.get_state()
        if self._reported_result or \
          proxy_state == AsyncServiceProxy.IDLE or \
          proxy_state == AsyncServiceProxy.ABORTED:
            self._last_service_call_time = rospy.Time.now()
            self._last_request = self.inputs['request']
            self._reported_result = False
            self._service_proxy.call_service(self._last_request)

        if proxy_state == AsyncServiceProxy.RUNNING:
            # If the call takes longer than the specified timeout, abort the
            # call and return FAILED
            seconds_since_call = (rospy.Time.now() - self._last_service_call_time).to_sec()
            if seconds_since_call > self.options['wait_for_response_seconds']:
                self.logerr('Service call to %s with request %s timed out' % (
                    self.options['service_name'], self._last_request))
                self._service_proxy.stop_call()
                return NodeMsg.FAILED

            return NodeMsg.RUNNING
        else:
            new_state = NodeMsg.SUCCEEDED
            if proxy_state == AsyncServiceProxy.RESPONSE_READY:
                self.outputs['response'] = self._service_proxy.get_response()
            if proxy_state == AsyncServiceProxy.ERROR:
                # TODO(nberg): Leave old response or set to None?
                new_state = NodeMsg.FAILED

            self._reported_result = True
            return new_state

    def _do_untick(self):
        self._service_proxy.stop_call()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self._service_proxy.stop_call()
