from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig

from ros_bt_py.ros_helpers import EnumValue, get_message_constant_fields


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'ros_message_type': type,
             'constant_name': EnumValue},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[{'source': 'ros_message_type', 'target': 'constant_name'}]))
class Enum(Leaf):
    """Exposes a constant in a ROS message as an output
    """
    def __init__(self, options=None, debug_manager=None, name=None):
        super(Enum, self).__init__(options, debug_manager, name)

        node_outputs = {}

        constants = get_message_constant_fields(self.options['ros_message_type'])

        if not constants:
            raise BehaviorTreeException('%s has no constant fields' % (
                self.options['ros_message_type']
            ))

        if self.options['constant_name'].enum_value not in constants:
            raise BehaviorTreeException('%s has no field %s' % (
                self.options['ros_message_type'],
                self.options['constant_name'].enum_value
            ))

        self.msg = self.options['ros_message_type']()

        node_outputs['out'] = type(
            getattr(self.msg, self.options['constant_name'].enum_value))

        self.node_config.extend(NodeConfig(
            options={},
            inputs={},
            outputs=node_outputs,
            max_children=0))

        self._register_node_data(source_map=node_outputs,
                                 target_map=self.outputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        self.outputs['out'] = getattr(self.msg, self.options['constant_name'].enum_value)
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'ros_message_type': type},
    inputs={},
    outputs={},
    max_children=0))
class EnumFields(Leaf):
    """Exposes the constants in a ROS message as multiple output fields

    The outputs will be named after the fields in the ROS message
    """
    def __init__(self, options=None, debug_manager=None, name=None):
        super(EnumFields, self).__init__(options, debug_manager, name)

        node_outputs = {}

        constants = get_message_constant_fields(self.options['ros_message_type'])

        if not constants:
            raise BehaviorTreeException('%s has no constant fields' % (
                self.options['ros_message_type']
            ))

        self.msg = self.options['ros_message_type']()

        for field in constants:
            node_outputs[field] = type(getattr(self.msg, field))

        self.node_config.extend(NodeConfig(
            options={},
            inputs={},
            outputs=node_outputs,
            max_children=0))

        self._register_node_data(source_map=node_outputs,
                                 target_map=self.outputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        for field in self.outputs:
            self.outputs[field] = getattr(self.msg, field)
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE
