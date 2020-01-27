import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


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
        param = rospy.get_param(self.inputs['param_name'],
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
    version='1.0.0',
    options={'param_name': str, 'param_type': type},
    inputs={'default_value': OptionRef('param_type')},
    outputs={'param': OptionRef('param_type')},
    max_children=0))
class RosParamOption(Leaf):
    """Read a parameter from the ROS parameter server"""
    def _do_setup(self):
        pass

    def _do_tick(self):
        param = rospy.get_param(self.options['param_name'],
                                default=self.inputs['default_value'])
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
        resolved_param_name = rospy.resolve_name(self.options['param_name'])
        try:
            params = rospy.get_param_names()
        except rospy.ROSException:
            return UtilityBounds()
        if (resolved_param_name in params and
                isinstance(rospy.get_param(resolved_param_name), self.options['param_type'])):
            return UtilityBounds(can_execute=True,
                                 has_lower_bound_success=True,
                                 has_upper_bound_success=True,
                                 has_lower_bound_failure=True,
                                 has_upper_bound_failure=True)

        self.loginfo('Parameter %s is not available or has wrong type' % resolved_param_name)
        return UtilityBounds()
