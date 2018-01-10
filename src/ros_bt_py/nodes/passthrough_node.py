from ros_bt_py.node import Node


class PassthroughNode(Node):
    """Pass through a piece of data

    Only really useful for testing
    """
    def __init__(self, passthrough_type):
        super(PassthroughNode, self).__init__()

        self._register_inputs({'in': passthrough_type})
        self._register_outputs({'out': passthrough_type})

        # Connect 'in' to 'out'
        self._wire_input('in', self.outputs.get_callback('out'))
        self.state = Node.States.IDLE


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
