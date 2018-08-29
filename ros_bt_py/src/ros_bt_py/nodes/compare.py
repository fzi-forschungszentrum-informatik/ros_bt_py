from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'compare_type': type},
    inputs={
        'a': OptionRef('compare_type'),
        'b': OptionRef('compare_type')
    },
    outputs={},
    max_children=0))
class Compare(Leaf):
    """Compares `a` and `b`

    Will succeed if `a == b` and fail otherwise
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        # if self.inputs.is_updated('in') or self.inputs.is_updated('expected'):
        if self.inputs['a'] == self.inputs['b']:
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_untick(self):
        # Nothing to do
        return NodeMsg.IDLE

    def _do_reset(self):
        # Reset output to False, so we'll return False until we
        # receive a new input.
        self.inputs['a'] = None
        self.inputs['b'] = None
        self.inputs.reset_updated()

        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Nothing to do
        pass


@define_bt_node(NodeConfig(
    options={'compare_type': type},
    inputs={
        'a': OptionRef('compare_type'),
        'b': OptionRef('compare_type')
    },
    outputs={},
    max_children=0))
class CompareNewOnly(Leaf):
    """Compares `a` and `b`, but only if at least one of them has been
updated this tick.

    Will succeed if `a == b` and fail otherwise
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.inputs.is_updated('a') or self.inputs.is_updated('b'):
            if self.inputs['a'] == self.inputs['b']:
                return NodeMsg.SUCCEEDED
            else:
                return NodeMsg.FAILED
        return NodeMsg.RUNNING

    def _do_untick(self):
        # Nothing to do
        return NodeMsg.IDLE

    def _do_reset(self):
        # Reset output to False, so we'll return False until we
        # receive a new input.
        self.inputs['a'] = None
        self.inputs['b'] = None
        self.inputs.reset_updated()

        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Nothing to do
        pass


@define_bt_node(NodeConfig(
    options={'compare_type': type,
             'expected': OptionRef('compare_type')},
    inputs={
        'in': OptionRef('compare_type')
    },
    outputs={},
    max_children=0))
class CompareConstant(Leaf):
    """Compares `expected` and `in`

    Will succeed if `expected == in` and fail otherwise
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        # if self.inputs.is_updated('in') or self.inputs.is_updated('expected'):
        if self.options['expected'] == self.inputs['in']:
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_untick(self):
        # Nothing to do
        return NodeMsg.IDLE

    def _do_reset(self):
        # Reset output to False, so we'll return False until we
        # receive a new input.
        self.inputs['in'] = None
        self.inputs.reset_updated()

        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Nothing to do
        pass


@define_bt_node(NodeConfig(
    options={},
    inputs={
        'a': float,
        'b': float
    },
    outputs={},
    max_children=0))
class ALessThanB(Leaf):
    """Compares `a` and `b`

    Will succeed if `a < b` and fail otherwise
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        # if self.inputs.is_updated('in') or self.inputs.is_updated('expected'):
        if self.inputs['a'] < self.inputs['b']:
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_untick(self):
        # Nothing to do
        return NodeMsg.IDLE

    def _do_reset(self):
        # Reset output to False, so we'll return False until we
        # receive a new input.
        self.inputs['a'] = None
        self.inputs['b'] = None
        self.inputs.reset_updated()

        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Nothing to do
        pass


@define_bt_node(NodeConfig(
    options={
        'target': float
    },
    inputs={
        'a': float
    },
    outputs={},
    max_children=0))
class LessThanConstant(Leaf):
    """Compares `a` and `b`

    Will succeed if `a < b` and fail otherwise
    """
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        # if self.inputs.is_updated('in') or self.inputs.is_updated('expected'):
        if self.inputs['a'] < self.options['target']:
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_untick(self):
        # Nothing to do
        return NodeMsg.IDLE

    def _do_reset(self):
        # Reset output to False, so we'll return False until we
        # receive a new input.
        self.inputs['a'] = None
        self.inputs.reset_updated()

        return NodeMsg.IDLE

    def _do_shutdown(self):
        # Nothing to do
        pass
