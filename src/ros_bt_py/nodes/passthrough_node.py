from ros_bt_py.node import Node


class PassthroughNode(Node):
    """Pass through a piece of data

    Only really useful for testing
    """
    def __init__(self, passthrough_type):
        super(PassthroughNode, self).__init__()

        self._register_inputs({'in': passthrough_type})
        self._register_outputs({'out': passthrough_type})
        self.state = Node.States.IDLE

    def step(self):
        self.outputs['out'] = self.inputs['in']
        return Node.States.SUCCEEDED
