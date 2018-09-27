from threading import Lock

from roslib.message import get_message_class
import rospy
import rosservice

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import RemoteSlotState, UtilityBounds
from ros_bt_py_msgs.srv import ControlTreeExecution, ControlTreeExecutionRequest

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.ros_helpers import AsyncServiceProxy


@define_bt_node(NodeConfig(
    options={'control_slot_service': str,
             'wait_for_service_seconds': float},
    inputs={},
    outputs={},
    max_children=0))
class RemoteSlot(Leaf):
    """This node controls a :class:`ros_bt_py.remote_tree_slot.RemoteTreeSlot`

    That is, if this node is ticked, unticked or reset, any tree
    that's currently loaded into the slot receives the same command.

    `control_slot_service` is the ROS name of a
    :class:`ros_bt_py_msgs.srv.ControlTreeExecution` service belonging
    to the :class:`ros_bt_py.remote_tree_slot.RemoteTreeSlot` we want
    to control.

    This will return `SUCCEEDED` when the
    :class:`ros_bt_py.remote_tree_slot.RemoteTreeSlot` reports that it
    has finished executing its tree, **regardless of the
    outcome**. The surrounding tree can use this information to allow
    a slot to execute fully before getting back to another task, but
    it can also stop the slot at any time, of course.

    """
    def _do_setup(self):
        rospy.wait_for_service(self.options['slot_namespace'],
                               self.options['wait_for_service_seconds'])

        self._service_proxy = AsyncServiceProxy(
            self.options['slot_namespace'] + '/control_slot_execution',
            ControlTreeExecution)

        self._lock = Lock()
        # We're only interested in the tree_finished member of
        # RemoteSlotState - this lets us return SUCCEEDED whenever the
        # slot finished execution, allowing the surrounding tree to
        # react.
        self._slot_finished = False
        self._slot_state_sub = rospy.Subscriber(
            self.options['slot_namespace'] + '/slot_state',
            RemoteSlotState,
            callback=self._slot_state_cb)

    def _slot_state_cb(self, msg):
        with self._lock:
            if msg.tree_finished:
                self._slot_finished = True

    def _do_tick(self):
        # If we received a RemoteSlotState message informing us the
        # slot is finished, stop everything, reset our service proxy
        # and return SUCCEEDED
        with self._lock:
            if self._slot_finished:
                self._do_reset()
                return NodeMsg.SUCCEEDED

        proxy_state = self._service_proxy.get_state()
        if proxy_state == AsyncServiceProxy.ERROR:
            return NodeMsg.FAILED

        if proxy_state == AsyncServiceProxy.RESPONSE_READY:
            res = self._service_proxy.get_response()
            if not res.success:
                return NodeMsg.FAILED
            # Get the proxy state after getting the response
            proxy_state = self._service_proxy.get_state()

        if proxy_state in [AsyncServiceProxy.IDLE,
                           AsyncServiceProxy.ABORTED]:
            self._service_proxy.call_service(ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.TICK_ONCE))

        return NodeMsg.RUNNING

    def _do_shutdown(self):
        if self._service_proxy.get_state() == AsyncServiceProxy.RUNNING:
            self._service_proxy.stop_call()

        self._service_proxy.call_service(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.SHUTDOWN))

    def _do_reset(self):
        with self._lock:
            self._slot_finished = False
        if self._service_proxy.get_state() == AsyncServiceProxy.RUNNING:
            self._service_proxy.stop_call()

        self._service_proxy.call_service(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.RESET))
        return NodeMsg.IDLE

    def _do_untick(self):
        if self._service_proxy.get_state() == AsyncServiceProxy.RUNNING:
            self._service_proxy.stop_call()

        self._service_proxy.call_service(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.STOP))

        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        resolved_service = rospy.resolve_name(self.options['control_slot_service'])

        try:
            service_type_name = rosservice.get_service_type(resolved_service)
        except rosservice.ROSServiceIOException as exc:
            # Defaults to no bounds set, dragging down the utility
            # score
            self.loginfo('Unable to check for service %s: %s' % (resolved_service, str(exc)))
            return UtilityBounds()

        if service_type_name:
            service_type = get_message_class(service_type_name)

            if service_type == ControlTreeExecution:
                self.loginfo(('Found service %s with correct type, returning '
                              'filled out UtilityBounds') % resolved_service)
                return UtilityBounds(has_lower_bound_success=True,
                                     has_upper_bound_success=True,
                                     has_lower_bound_failure=True,
                                     has_upper_bound_failure=True)

        self.loginfo('Service %s is unavailable or has wrong type.' % resolved_service)
        return UtilityBounds()
