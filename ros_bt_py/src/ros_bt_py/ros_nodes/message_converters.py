import inspect

import genpy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'input_type': type},
    inputs={'in': OptionRef('input_type')},
    outputs={},
    max_children=0))
class MessageToFields(Leaf):
    """Takes a ROS message as input and splits it into multiple outputs based on the message fields

    The outputs will be named after the fields in the input message

    If the input is not as ROS message, it will be be passed through

    """
    def __init__(self, options=None, debug_manager=None, name=None):
        super(MessageToFields, self).__init__(options, debug_manager, name)

        node_outputs = {}

        self.passthrough = True

        if (inspect.isclass(self.options['input_type']) and
                genpy.message.Message in self.options['input_type'].__mro__):
            msg = self.options['input_type']()
            for field in str(msg).split("\n"):
                name = field.split(":")[0].strip()
                node_outputs[name] = type(getattr(msg, name))
            self.passthrough = False
        else:
            node_outputs['out'] = self.options['input_type']

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
        if self.passthrough:
            self.outputs['out'] = self.inputs['in']
        else:
            for field in self.outputs:
                self.outputs[field] = getattr(self.inputs['in'], field)
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    options={'output_type': type},
    inputs={},
    outputs={'out': OptionRef('output_type')},
    max_children=0))
class FieldsToMessage(Leaf):
    """Takes multiple fields as input and outputs a ROS message

    The inputs will be named after the fields in the output message

    If the output is not as ROS message, the input will be be passed through

    """
    def __init__(self, options=None, debug_manager=None, name=None):
        super(FieldsToMessage, self).__init__(options, debug_manager, name)

        node_inputs = {}

        self.passthrough = True

        if (inspect.isclass(self.options['output_type']) and
                genpy.message.Message in self.options['output_type'].__mro__):
            msg = self.options['output_type']()
            for field in str(msg).split("\n"):
                name = field.split(":")[0].strip()
                node_inputs[name] = type(getattr(msg, name))
            self.passthrough = False
        else:
            node_inputs['in'] = self.options['output_type']

        self.node_config.extend(NodeConfig(
            options={},
            inputs=node_inputs,
            outputs={},
            max_children=0))

        self._register_node_data(source_map=node_inputs,
                                 target_map=self.inputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.passthrough:
            self.outputs['out'] = self.inputs['in']
        else:
            try:
                msg = self.options['output_type']()
                fields = dict()
                for key in self.inputs:
                    fields[key] = self.inputs[key]
                genpy.message.fill_message_args(msg, [fields], keys={})
                self.outputs['out'] = msg
            except genpy.message.MessageException:
                return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE
