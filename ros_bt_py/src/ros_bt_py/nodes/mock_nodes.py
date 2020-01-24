from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.exceptions import NodeConfigError
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'output_type': type,
             'state_values': list,
             'output_values': list},
    inputs={},
    outputs={'out': OptionRef('output_type'),
             'current_index': int,
             'tick_count': int,
             'untick_count': int,
             'reset_count': int,
             'shutdown_count': int},
    max_children=0))
class MockLeaf(Leaf):
    def __init__(self, options=None, debug_manager=None, name=None):
        super(Leaf, self).__init__(options, debug_manager, name)

        self.setup_called = False
        self.tick_count = 0
        self.untick_count = 0
        self.reset_count = 0
        self.shutdown_count = 0

    def _do_setup(self):
        self.setup_called = True
        self.outputs['current_index'] = 0
        self.outputs['tick_count'] = self.tick_count
        self.outputs['untick_count'] = self.untick_count
        self.outputs['reset_count'] = self.reset_count
        self.outputs['shutdown_count'] = self.shutdown_count

        if len(self.options['state_values']) != len(self.options['output_values']):
            raise NodeConfigError('state_values and output_values must have the same length!')
        for value in self.options['output_values']:
            try:
                self.outputs['out'] = value
            except TypeError:
                raise NodeConfigError('Provided output value "%s (%s)" is not compatible with '
                                      'output type %s' %
                                      (str(value), type(value).__name__,
                                       self.options['output_type'].__name__))
            self.outputs['out'] = None

    def _do_tick(self):
        self.outputs['out'] = self.options['output_values'][self.outputs['current_index']]
        new_state = self.options['state_values'][self.outputs['current_index']]
        # Increment index (and roll over if necessary
        self.outputs['current_index'] = ((self.outputs['current_index'] + 1) %
                                         len(self.options['state_values']))

        self.tick_count += 1
        self.outputs['tick_count'] = self.tick_count
        return new_state

    def _do_untick(self):
        # We leave current_index untouched, so paused is the most semantically
        # correct state
        self.untick_count += 1
        self.outputs['untick_count'] = self.untick_count
        return NodeMsg.PAUSED

    def _do_reset(self):
        self.outputs['current_index'] = 0
        self.reset_count += 1
        self.outputs['reset_count'] = self.reset_count
        return NodeMsg.IDLE

    def _do_shutdown(self):
        self.outputs['current_index'] = 0
        self.shutdown_count += 1
        self.outputs['shutdown_count'] = self.shutdown_count

    def _do_calculate_utility(self):
        return UtilityBounds(can_execute=True)


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'can_execute': bool,
             'utility_lower_bound_success': float,
             'utility_upper_bound_success': float,
             'utility_lower_bound_failure': float,
             'utility_upper_bound_failure': float},
    inputs={},
    outputs={'calculate_utility_count': int},
    max_children=0))
class MockUtilityLeaf(Leaf):
    def _do_calculate_utility(self):
        if self.outputs['calculate_utility_count']:
            self.outputs['calculate_utility_count'] += 1
        else:
            self.outputs['calculate_utility_count'] = 1
        return UtilityBounds(
            can_execute=self.options['can_execute'],
            has_lower_bound_success=True,
            lower_bound_success=self.options['utility_lower_bound_success'],
            has_upper_bound_success=True,
            upper_bound_success=self.options['utility_upper_bound_success'],
            has_lower_bound_failure=True,
            lower_bound_failure=self.options['utility_lower_bound_failure'],
            has_upper_bound_failure=True,
            upper_bound_failure=self.options['utility_upper_bound_failure'])

    def _do_shutdown(self):
        self.shutdown_count += 1

    def _do_setup(self):
        self.setup_called = True
        self.tick_count = 0
        self.untick_count = 0
        self.reset_count = 0
        self.shutdown_count = 0

    def _do_tick(self):
        self.tick_count += 1
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        self.untick_count += 1
        return NodeMsg.IDLE

    def _do_reset(self):
        self.reset_count += 1
        return NodeMsg.IDLE
