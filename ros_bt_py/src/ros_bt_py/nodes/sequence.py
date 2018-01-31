from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import FlowControl, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=None))
class Sequence(FlowControl):
    """Flow control node that succeeds when all children succeed.

    At every call of :meth:`Node.tick`, it ticks all of its children in
    order until one of three things happens:

    1. A node returns FAILED:

       In this case, the sequence also returns FAILED, and calls
       :meth:`Node.untick` on all remaining children.

    2. A node returns RUNNING:

       Just as with FAILED, the sequence will also return RUNNING and call
       :meth:`Node.untick` on all remaining children.

    3. All nodes return SUCCEEDED:

       The Sequence will return SUCCEEDED

    *Special case:*

    If a Sequence has no children, its :meth:`tick` method will always
    return FAILED.
    """
    def _do_setup(self):
        for child in self.children:
            child.setup()

    def _do_tick(self):
        if not self.children:
            self.logwarn('Ticking without children. Is this really what you want?')
            return NodeMsg.FAILED
        # Tick children until one returns FAILED or RUNNING
        for index, child in enumerate(self.children):
            result = child.tick()
            if result != NodeMsg.SUCCEEDED:
                # untick all children after the one that hasn't succeeded
                for untick_child in self.children[index + 1:]:
                    untick_child.untick()
                return result
        # If all children succeeded, we too succeed
        return NodeMsg.SUCCEEDED

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
