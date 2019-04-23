from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

from random import random

@define_bt_node(NodeConfig(
    options={'success': bool},
    inputs={},
    outputs={},
    max_children=0))
class DummyFailSuccess(Leaf):
    """Always success or fail

    Success if option is True, fail otherwise
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.options['success']:
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_untick(self):
        # Nothing to do
        return NodeMsg.IDLE

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Nothing to do
        pass

@define_bt_node(NodeConfig(
    options={'success_probability': float},
    inputs={},
    outputs={},
    max_children=0))
class ArmBandit(Leaf):
    """Like a slot machine, succeed randomly with given probability

    Success randomly with respect to given probability
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if random() < self.options['success_probability']:
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_untick(self):
        # Nothing to do
        return NodeMsg.IDLE

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Nothing to do
        pass
