from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'passthrough_type': type},
    inputs={'in': OptionRef('passthrough_type')},
    outputs={'out': OptionRef('passthrough_type')},
    max_children=0))
class PassthroughNode(Node):
    """Pass through a piece of data

    Only really useful for testing
    """
    def setup(self):
        # Connect 'in' to 'out'
        self.inputs.subscribe('in', self.outputs.get_callback('out'), '.outputs[out]')
        return Node.States.IDLE

    def step(self):
        # Nothing to do here other than succeed
        return Node.States.SUCCEEDED

    def stop(self):
        return Node.States.IDLE

    def handle_reset(self):
        self.inputs['in'] = None
        self.inputs.reset_updated()
        self.outputs['out'] = None
        self.outputs.reset_updated()
        return Node.States.IDLE
