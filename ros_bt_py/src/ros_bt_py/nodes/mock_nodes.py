from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.exceptions import NodeConfigError
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

@define_bt_node(NodeConfig(
    options={'output_type': type,
             'state_values': list,
             'output_values': list},
    inputs={},
    outputs={'out': OptionRef('output_type'),
             'current_index': int,
             'tick_count': int,
             'untick_count': int},
    max_children=0))
class MockLeaf(Leaf):
    def _do_setup(self):
        self.outputs['current_index'] = 0
        self.outputs['tick_count'] = 0
        self.outputs['untick_count'] = 0

        if len(self.options['state_values']) != len(self.options['output_values']):
            raise NodeConfigError('state_values and output_values must have the same length!')
        for value in self.options['output_values']:
            if not isinstance(value, self.options['output_type']):
                raise NodeConfigError('Provided output value "%s" is not of output type %s' %
                                      (str(value), self.options['output_type'].__name__))

    def _do_tick(self):
        self.outputs['out'] = self.options['output_values'][self.outputs['current_index']]
        new_state = self.options['state_values'][self.outputs['current_index']]
        # Increment index (and roll over if necessary
        self.outputs['current_index'] = ((self.outputs['current_index'] + 1) %
                                     len(self.options['state_values']))

        self.outputs['tick_count'] = self.outputs['tick_count'] + 1
        return new_state

    def _do_untick(self):
        # We leave current_index untouched, so paused is the most semantically
        # correct state
        self.outputs['untick_count'] = self.outputs['untick_count'] + 1
        return NodeMsg.PAUSED

    def _do_reset(self):
        self.outputs['current_index'] = 0
        self.outputs['tick_count'] = 0
        self.outputs['untick_count'] = 0

        return NodeMsg.IDLE

    def _do_shutdown(self):
        self.outputs['current_index'] = 0
