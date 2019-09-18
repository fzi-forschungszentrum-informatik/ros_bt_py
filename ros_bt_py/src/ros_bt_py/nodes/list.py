from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={},
    inputs={'list': list},
    outputs={'length': int},
    max_children=0))
class ListLength(Leaf):
    """Compute list length"""
    def _do_setup(self):
        pass

    def _do_tick(self):
        self.outputs['length'] = len(self.inputs['list'])
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
