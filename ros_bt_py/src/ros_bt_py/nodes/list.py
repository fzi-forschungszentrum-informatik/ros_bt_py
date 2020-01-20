from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, Decorator, define_bt_node
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


@define_bt_node(NodeConfig(
    options={'compare_type': type,
             'list': list},
    inputs={
        'in': OptionRef('compare_type')
    },
    outputs={},
    max_children=0))
class IsInList(Leaf):
    """Check if `in` is in provided list

    Will succeed if `in` is in `list` and fail otherwise
    """
    def _do_setup(self):
        self._received_in = False
        return NodeMsg.IDLE

    def _do_tick(self):
        if not self._received_in:
            if self.inputs.is_updated('in'):
                self._received_in = True

        if self._received_in and self.inputs['in'] in self.options['list']:
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_untick(self):
        # Nothing to do
        return NodeMsg.IDLE

    def _do_reset(self):
        self.inputs.reset_updated()
        self._received_in = False

        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Nothing to do
        pass


@define_bt_node(NodeConfig(
    options={'item_type': type},
    inputs={'list': list},
    outputs={'list_item': OptionRef('item_type')},
    max_children=1))
class IterateList(Decorator):
    """
    Iterate through list, will succeed if iterated through entire list, returns running while iterating.
    To be used as a decorator, with one child.
    """
    def _do_setup(self):
        self.reset_counter()
        for child in self.children:
            child.setup()
        return NodeMsg.IDLE

    def reset_counter(self):
        self.counter = 0

    def _do_tick(self):
        if self.inputs.is_updated('list'):
            self.logwarn('Input list changed - resetting iterator')
            self.reset_counter()

        if self.counter < len(self.inputs['list']):
            self.outputs['list_item'] = self.inputs['list'][self.counter]

            if len(self.children) == 0:
                self.counter += 1
            else:
                for child in self.children:
                    result = child.tick()
                    if result != NodeMsg.RUNNING:
                        # we only increment the counter when the child succeeded or failed
                        self.counter += 1
            return NodeMsg.RUNNING
        else:
            self.reset_counter()
            return NodeMsg.SUCCEEDED

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return NodeMsg.IDLE

    def _do_reset(self):
        self.inputs.reset_updated()
        self.reset_counter()
        for child in self.children:
            return child.reset()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()
