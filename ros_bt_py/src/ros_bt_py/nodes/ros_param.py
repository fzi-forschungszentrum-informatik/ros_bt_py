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

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'param_type': type, 'default_value': OptionRef('param_type'), 'param_name': str},
    inputs={},
    outputs={'param': OptionRef('param_type')},
    max_children=0))
class RosParamNoInput(Leaf):
    """Read a parameter from the ROS parameter server"""
    def _do_setup(self):
        pass

    def _do_tick(self):
        param = rospy.get_param(self.options['param_name'],
                                default=self.options['default_value'])
        if isinstance(param, self.options['param_type']):
            self.outputs['param'] = param
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        return UtilityBounds()


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'param_type': type, 'default_value': OptionRef('param_type')},
    inputs={'param_name': str},
    outputs={'param': OptionRef('param_type')},
    max_children=0))
class RosParamInput(Leaf):
    """Read a parameter from the ROS parameter server"""

    def _do_setup(self):
        pass

    def _do_tick(self):
        param = rospy.get_param(
            self.inputs["param_name"], default=self.options["default_value"]
        )
        if isinstance(param, self.options["param_type"]):
            self.outputs["param"] = param
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="1.0.0",
        options={"param_name": str, "param_type": type},
        inputs={"default_value": OptionRef("param_type")},
        outputs={"param": OptionRef("param_type")},
        max_children=0,
    )
)
class RosParamOption(Leaf):
    """Read a parameter from the ROS parameter server"""

    def _do_setup(self):
        pass

    def _do_tick(self):
        param = rospy.get_param(
            self.options["param_name"], default=self.inputs["default_value"]
        )
        if isinstance(param, self.options["param_type"]):
            self.outputs["param"] = param
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        resolved_param_name = rospy.resolve_name(self.options["param_name"])
        try:
            params = rospy.get_param_names()
        except rospy.ROSException:
            return UtilityBounds()
        if resolved_param_name in params and isinstance(
            rospy.get_param(resolved_param_name), self.options["param_type"]
        ):
            return UtilityBounds(
                can_execute=True,
                has_lower_bound_success=True,
                has_upper_bound_success=True,
                has_lower_bound_failure=True,
                has_upper_bound_failure=True,
            )

        self.loginfo(
            "Parameter %s is not available or has wrong type" % resolved_param_name
        )
        return UtilityBounds()
