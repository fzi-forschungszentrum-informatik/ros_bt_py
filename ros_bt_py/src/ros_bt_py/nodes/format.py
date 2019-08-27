from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    options={'format_string': str},
    inputs={'dict': dict},
    outputs={'formatted_string': str},
    max_children=0))
class FormatOptionNode(Leaf):
    """Accepts a dictionary as input and outputs a formatted string
    based on the format string set in the options.
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
