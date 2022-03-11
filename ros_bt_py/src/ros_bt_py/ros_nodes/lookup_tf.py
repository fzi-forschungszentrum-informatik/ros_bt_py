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
import rospy
import tf2_ros

from geometry_msgs.msg import Pose, Point
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={
        'parent_frame': str,
        'child_frame': str},
    inputs={},
    outputs={'transform_pose': Pose},
    max_children=0))
class LookupTFConst(Leaf):
    """Lookup the current tf between `parent_frame` and `child_frame`"""
    def _do_setup(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _do_tick(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.options['parent_frame'],
                                                    self.options['child_frame'],
                                                    rospy.Time())
            self.outputs['transform_pose'] = Pose(Point(trans.transform.translation.x,
                                                        trans.transform.translation.y,
                                                        trans.transform.translation.z),
                                                  trans.transform.rotation)
            return NodeMsg.SUCCEEDED
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return NodeMsg.FAILED

    def _do_shutdown(self):
        self.tf_listener.unregister()
        self.tf_buffer.clear()

    def _do_reset(self):
        self.tf_buffer.clear()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={},
    inputs={'parent_frame': str,
            'child_frame': str},
    outputs={'transform_pose': Pose},
    max_children=0))
class LookupTF(Leaf):
    """Lookup the current tf between `parent_frame` and `child_frame`"""
    def _do_setup(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _do_tick(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.inputs['parent_frame'],
                                                    self.inputs['child_frame'],
                                                    rospy.Time())
            self.outputs['transform_pose'] = Pose(Point(trans.transform.translation.x,
                                                        trans.transform.translation.y,
                                                        trans.transform.translation.z),
                                                  trans.transform.rotation)
            return NodeMsg.SUCCEEDED
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return NodeMsg.FAILED

    def _do_shutdown(self):
        self.tf_listener.unregister()
        self.tf_buffer.clear()

    def _do_reset(self):
        self.tf_buffer.clear()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
