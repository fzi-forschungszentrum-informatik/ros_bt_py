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

import rospy
import std_msgs

from std_msgs.msg import Header
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py_msgs.msg import Node as NodeMsg


@define_bt_node(
    NodeConfig(
        options={"frame_id": str},
        inputs={},
        outputs={"header": Header},
        version="1.0.0",
        max_children=0,
    )
)
class CreateStampedRosHeader(Leaf):
    seq = 0

    def _do_setup(self):
        self.seq = 0
        self.outputs["header"] = Header(
            seq=self.seq, stamp=rospy.Time.now(), frame_id=self.options["frame_id"]
        )

    def _do_tick(self):
        self.outputs["header"] = Header(
            seq=self.seq, stamp=rospy.Time.now(), frame_id=self.options["frame_id"]
        )
        self.seq += 1
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        options={"header_type": type},
        inputs={},
        outputs={"header": OptionRef("header_type")},
        max_children=0,
        version="0.9.0",
    )
)
class GetStdHeader(Leaf):
    """Get ROS-Time in Header format"""

    def _do_setup(self):
        self.header = std_msgs.msg.Header()
        pass

    def _do_tick(self):
        self.header.stamp = rospy.Time.now()
        if isinstance(self.header, self.options["header_type"]):
            self.outputs["header"] = self.header
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
