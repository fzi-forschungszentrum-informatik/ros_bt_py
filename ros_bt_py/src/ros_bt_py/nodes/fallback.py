from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import FlowControl, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=None))
class Fallback(FlowControl):
    def _do_setup(self):
        for child in self.children:
            child.setup()

    def _do_tick(self):
        if not self.children:
            self.logwarn('Ticking without children. Is this really what you want?')
            return NodeMsg.FAILED
        # Tick children until one returns SUCCEEDED or RUNNING
        for index, child in enumerate(self.children):
            result = child.tick()
            if result == NodeMsg.SUCCEEDED or result == NodeMsg.RUNNING:
                # untick all children after the one that triggered this
                # condition
                for untick_child in self.children[index + 1:]:
                    untick_child.untick()
                return result
        # If all children failed, we too fail
        return NodeMsg.FAILED

    def _do_untick(self):
        for child in self.children:
            child.untick()
        return NodeMsg.IDLE

    def _do_reset(self):
        for child in self.children:
            child.reset()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        for child in self.children:
            child.shutdown()
