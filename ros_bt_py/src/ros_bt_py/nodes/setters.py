from copy import deepcopy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'list_type': type},
    inputs={'list': list,
            'value': OptionRef('list_type')},
    outputs={'new_list': list},
    max_children=0))
class AppendListItem(Leaf):
    """Appends `item` to the end of `list`"""
    def _do_setup(self):
        pass

    def _do_tick(self):
        if self.inputs.is_updated('list') or self.inputs.is_updated('value'):
            self.outputs['new_list'] = self.inputs['list'] + [self.inputs['value']]

        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    options={'object_type': type,
             'attr_name': str,
             'attr_type': type},
    inputs={'object': OptionRef('object_type'),
            'attr_value': OptionRef('attr_type')},
    outputs={'new_object': OptionRef('object_type')},
    max_children=0))
class SetAttr(Leaf):
    """Set the attribute named `attr` in `object`"""
    def _do_setup(self):
        pass

    def _do_tick(self):
        if self.inputs.is_updated('object') or self.inputs.is_updated('attr_value'):
            obj = deepcopy(self.inputs['object'])
            setattr(obj, self.options['attr_value'])
            self.outputs['new_object'] = obj
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
