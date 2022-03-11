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
from threading import Lock

from roslib.message import get_service_class
import rospy
import rosservice

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import RemoteSlotState, UtilityBounds
from ros_bt_py_msgs.srv import ControlTreeExecution, ControlTreeExecutionRequest

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.ros_helpers import AsyncServiceProxy


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'slot_namespace': str,
             'wait_for_service_seconds': float},
    inputs={},
    outputs={},
    max_children=0))
class RemoteSlot(Leaf):
    """This node controls a :class:`ros_bt_py.remote_tree_slot.RemoteTreeSlot`

    That is, if this node is ticked, unticked or reset, any tree
    that's currently loaded into the slot receives the same command.

    `slot_namespace` is a ROS namespace where a
    :class:`ros_bt_py_msgs.srv.ControlTreeExecution` service belonging
    to the :class:`ros_bt_py.remote_tree_slot.RemoteTreeSlot` we want
    to control lives.

    This will return `SUCCEEDED` when the
    :class:`ros_bt_py.remote_tree_slot.RemoteTreeSlot` reports that it
    has finished executing its tree, **regardless of the
    outcome**. The surrounding tree can use this information to allow
    a slot to execute fully before getting back to another task, but
    it can also stop the slot at any time, of course.

    """
    def _do_setup(self):
        rospy.wait_for_service(self.options['slot_namespace'] + '/control_slot_execution',
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
        self._tree_loaded = False
        self._run_command_sent = False
        self._slot_state_sub = rospy.Subscriber(
            self.options['slot_namespace'] + '/slot_state',
            RemoteSlotState,
            callback=self._slot_state_cb)

    def _slot_state_cb(self, msg):
        with self._lock:
            # tree_loaded is just a convenience rename
            self._tree_loaded = msg.tree_in_slot
            # run_command_sent causes the next tick to send a new
            # TICK_PERIODICALLY request
            if not msg.tree_in_slot or not msg.tree_running:
                self._run_command_sent = False
            # need to save slot_finished because tree_finished might
            # become False again before we get to report the finished
            # execution
            if msg.tree_finished:
                self._slot_finished = True

    def _do_tick(self):
        # If we received a RemoteSlotState message informing us the
        # slot is finished, stop everything, reset our service proxy
        # and return SUCCEEDED
        if self._slot_finished:
            self._do_reset()
            # self.loginfo('quitting early to report finished slot execution')
            return NodeMsg.SUCCEEDED

        # If no tree is loaded, just succeed and let our parent tree
        # get on with its business
        if not self._tree_loaded:
            # self.loginfo('no tree loaded, succeeding')
            return NodeMsg.SUCCEEDED

        proxy_state = self._service_proxy.get_state()
        if proxy_state == AsyncServiceProxy.ERROR:
            # self.loginfo('quitting early to report service error')
            return NodeMsg.FAILED

        if proxy_state == AsyncServiceProxy.RESPONSE_READY:
            res = self._service_proxy.get_response()
            if not res.success:
                # self.loginfo('quitting early to report failed service call')
                return NodeMsg.FAILED
            # Get the proxy state after getting the response
            proxy_state = self._service_proxy.get_state()

        if not self._run_command_sent and self._tree_loaded:
            if proxy_state in [AsyncServiceProxy.IDLE,
                               AsyncServiceProxy.RESPONSE_READY,
                               AsyncServiceProxy.ABORTED]:
                # Send a TICK_PERIODICALLY request without frequency,
                # so the slot can decide for itself
                self._service_proxy.call_service(ControlTreeExecutionRequest(
                    command=ControlTreeExecutionRequest.TICK_UNTIL_RESULT))
                self._run_command_sent = True
        # else:
        #     self.loginfo('Not sending request. command_sent: ' +
        #                  str(self._run_command_sent) +
        #                  ' tree_loaded: ' + str(self._tree_loaded) +
        #                  ' proxy_state: ' + str(proxy_state))

        return NodeMsg.RUNNING

    def _do_shutdown(self):
        if self._service_proxy.get_state() == AsyncServiceProxy.RUNNING:
            self._service_proxy.stop_call()
        self._run_command_sent = True

        self._service_proxy.call_service(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.SHUTDOWN))

    def _do_reset(self):
        with self._lock:
            self._slot_finished = False
            self._tree_loaded = False
        self._run_command_sent = False

        if self._service_proxy.get_state() == AsyncServiceProxy.RUNNING:
            self._service_proxy.stop_call()

        self._service_proxy.call_service(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.SHUTDOWN))
        return NodeMsg.IDLE

    def _do_untick(self):
        if self._service_proxy.get_state() == AsyncServiceProxy.RUNNING:
            self._service_proxy.stop_call()

        self._service_proxy.call_service(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.STOP))
        self._run_command_sent = False

        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        resolved_service = rospy.resolve_name(self.options['slot_namespace']
                                              + '/control_slot_execution')

        try:
            service_type_name = rosservice.get_service_type(resolved_service)
        except rosservice.ROSServiceIOException as exc:
            # Defaults to no bounds set, dragging down the utility
            # score
            self.loginfo('Unable to check for service %s: %s' % (resolved_service, str(exc)))
            return UtilityBounds()

        if service_type_name:
            service_type = get_service_class(service_type_name)

            if service_type == ControlTreeExecution:
                self.loginfo(('Found service %s with correct type, returning '
                              'filled out UtilityBounds') % resolved_service)
                return UtilityBounds(has_lower_bound_success=True,
                                     has_upper_bound_success=True,
                                     has_lower_bound_failure=True,
                                     has_upper_bound_failure=True)

        self.loginfo('Service %s is unavailable or has wrong type.' % resolved_service)
        return UtilityBounds()
