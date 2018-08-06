from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'passthrough_type': type},
    inputs={'in': OptionRef('passthrough_type')},
    outputs={'out': OptionRef('passthrough_type')},
    max_children=0))
class PassthroughNode(Leaf):
    """Pass through a piece of data

    Only really useful for testing
    """
    def _do_setup(self):
        # Connect 'in' to 'out'
        self.inputs.subscribe('in', self.outputs.get_callback('out'), '.outputs[out]')
        return NodeMsg.IDLE

    def _do_tick(self):
        # Nothing to do here other than succeed
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.inputs['in'] = None
        self.inputs.reset_updated()
        self.outputs['out'] = None
        self.outputs.reset_updated()
        return NodeMsg.IDLE
