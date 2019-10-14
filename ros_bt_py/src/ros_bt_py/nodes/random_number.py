import random

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'min': int,
             'max': int},
    inputs={},
    outputs={'random_number': int},
    max_children=0,
    option_wirings=[]))
class RandomInt(Leaf):
    """Provides a pseudo-random integer in range min <= random_number < max
    """
    def _do_setup(self):
        validate_range(self.options['min'], self.options['max'])

    def _do_tick(self):
        validate_range(self.options['min'], self.options['max'])
        self.outputs['random_number'] = random.randrange(self.options['min'], self.options['max'])
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    # Uncomment this if your node provides a utility calculation
    #
    # def _do_calculate_utility(self):
    #     pass


@define_bt_node(NodeConfig(
    options={},
    inputs={'min': int,
            'max': int},
    outputs={'random_number': int},
    max_children=0,
    option_wirings=[]))
class RandomIntInputs(Leaf):
    """Provides a pseudo-random integer in range min <= random_number < max
    """
    def _do_setup(self):
        pass

    def _do_tick(self):
        validate_range(self.inputs['min'], self.inputs['max'])
        self.outputs['random_number'] = random.randrange(self.inputs['min'], self.inputs['max'])
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    # Uncomment this if your node provides a utility calculation
    #
    # def _do_calculate_utility(self):
    #     pass


def validate_range(minimum, maximum):
    """checks if `minimum` < `maximum` and raises a BehaviorTreeException if not"""
    if minimum == maximum:
        raise BehaviorTreeException(
            'minimum ({}) cannot be equal to maximum ({})'.format(minimum, maximum))
    if minimum > maximum:
        raise BehaviorTreeException(
            'minimum ({}) cannot be greater that maximum ({})'.format(minimum, maximum))
