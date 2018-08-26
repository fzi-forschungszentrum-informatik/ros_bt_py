from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

@define_bt_node(NodeConfig(
    options={'test_type': type},
    inputs={
        'in': OptionRef('test_type'),
        'expected': OptionRef('test_type')
    },
    outputs={'result': bool},
    max_children=0))
class Test(Leaf):
    def _do_setup(self):
        # Initialize the output
        self.outputs['result'] = False
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.inputs.is_updated('in') or self.inputs.is_updated('expected'):
            self.outputs['result'] = self.inputs['in'] == self.inputs['expected']

        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        # Nothing to do
        return NodeMsg.IDLE

    def _do_reset(self):
        # Reset output to False, so we'll return False until we
        # receive a new input.
        self.outputs['result'] = False
        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Nothing to do
        pass
