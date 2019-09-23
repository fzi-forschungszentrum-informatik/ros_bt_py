from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    options={},
    inputs={'a': str, 'b': str},
    outputs={'formatted_string': str},
    max_children=0))
class StringConcatenation(Leaf):
    """Concatenate strings a and b
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        self.outputs['formatted_string'] = self.inputs['a'] + self.inputs['b']
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['formatted_string'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    options={'format_string': str},
    inputs={'dict': dict},
    outputs={'formatted_string': str},
    max_children=0))
class FormatOptionNode(Leaf):
    """Accepts a dictionary as input and outputs a formatted string
    based on the format string set in the options.

    Example dict and format_string:
    dict: {'first': 'bar', 'second': 'not_printed'}
    format_string: 'foo {first}'

    results in the following output:
    formatted_string: 'foo bar'
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        try:
            self.outputs['formatted_string'] = self.options['format_string'].format(
                **self.inputs['dict'])
        except Exception:
            return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['formatted_string'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    options={},
    inputs={'dict': dict,
            'format_string': str},
    outputs={'formatted_string': str},
    max_children=0))
class FormatInputNode(Leaf):
    """Accepts a dictionary and a format string as input and outputs a formatted string
    based on the format string

    Example dict and format_string:
    dict: {'first': 'bar', 'second': 'not_printed'}
    format_string: 'foo {first}'

    results in the following output:
    formatted_string: 'foo bar'
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        try:
            self.outputs['formatted_string'] = self.inputs['format_string'].format(
                **self.inputs['dict'])
        except Exception:
            return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['formatted_string'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    options={'format_strings': list},
    inputs={'dict': dict},
    outputs={'formatted_strings': list},
    max_children=0))
class FormatOptionListNode(Leaf):
    """Accepts a dictionary as input and outputs a formatted strings in the list
    based on the format string set in the options.

    Example dict and format_string:
    dict: {'first': 'bar', 'second': 'not_printed'}
    format_strings: ['foo {first}', 'bar {first}']

    results in the following output:
    formatted_strings: ['foo bar', 'bar bar']
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        try:
            self.outputs['formatted_strings'] = [
                phrase.format(**self.inputs['dict'])
                for phrase in self.options['format_strings']
            ]
        except Exception:
            return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['formatted_strings'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    options={},
    inputs={'dict': dict,
            'format_strings': list},
    outputs={'formatted_strings': list},
    max_children=0))
class FormatInputListNode(Leaf):
    """Accepts a dictionary and a list of format strings as input and outputs a list of formatted strings
    based on the format string

    Example dict and format_string:
    dict: {'first': 'bar', 'second': 'not_printed'}
    format_strings: ['foo {first}', 'bar {first}']

    results in the following output:
    formatted_strings: ['foo bar', 'bar bar']
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        try:
            self.outputs['formatted_strings'] = [
                phrase.format(**self.inputs['dict'])
                for phrase in self.inputs['format_strings']
            ]
        except Exception:
            return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['formatted_strings'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE
