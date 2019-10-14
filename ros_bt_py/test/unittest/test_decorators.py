from copy import deepcopy
import unittest
try:
    import unittest.mock as mock
except ImportError:
    import mock

from ros_bt_py_msgs.msg import Node, UtilityBounds

from ros_bt_py.nodes.mock_nodes import MockLeaf, MockUtilityLeaf
from ros_bt_py.nodes.decorators import IgnoreFailure, IgnoreSuccess, UntilSuccess, Retry, Optional
from ros_bt_py.nodes.decorators import Inverter, Repeat, RepeatAlways, RepeatUntilFail
from ros_bt_py.nodes.decorators import Throttle, ThrottleSuccess
from rospy import Time

class TestDecorators(unittest.TestCase):
    def setUp(self):
        self.succeeder = MockLeaf(name='succeeder',
                                  options={'output_type': int,
                                           'state_values': [Node.SUCCEEDED],
                                           'output_values': [1]})
        self.failer = MockLeaf(name='failer',
                               options={'output_type': int,
                                        'state_values': [Node.FAILED],
                                        'output_values': [1]})
        self.running = MockLeaf(name='running',
                                options={'output_type': int,
                                         'state_values': [Node.RUNNING],
                                         'output_values': [1]})
        self.fail_fail_succeed = MockLeaf(name='fail_fail_succeed',
                                          options={'output_type': int,
                                                   'state_values': [Node.FAILED, Node.FAILED,
                                                                    Node.SUCCEEDED],
                                                   'output_values': [1, 1, 1]})

        self.run_fail_succeed = MockLeaf(name='run_fail_succeed',
                                         options={'output_type': int,
                                                  'state_values': [Node.RUNNING, Node.FAILED,
                                                                   Node.SUCCEEDED],
                                                  'output_values': [1, 1, 1]})

        self.cannot_exec = MockUtilityLeaf(
            name='cannot_exec',
            options={
                'can_execute': False,
                'utility_lower_bound_success': 1.0,
                'utility_upper_bound_success': 2.0,
                'utility_lower_bound_failure': 5.0,
                'utility_upper_bound_failure': 10.0})

    def testIgnoreFailure(self):
        ignore_failure = IgnoreFailure()
        ignore_failure.setup()
        self.assertEqual(ignore_failure.tick(), Node.SUCCEEDED)
        self.assertEqual(ignore_failure.untick(), Node.IDLE)
        self.assertEqual(ignore_failure.reset(), Node.IDLE)
        self.assertEqual(ignore_failure.shutdown(), Node.SHUTDOWN)

        ignore_failure.add_child(self.fail_fail_succeed)
        ignore_failure.setup()

        self.assertEqual(ignore_failure.tick(), Node.SUCCEEDED)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(ignore_failure.untick(), Node.PAUSED)
        self.assertEqual(ignore_failure.reset(), Node.IDLE)
        self.assertEqual(ignore_failure.tick(), Node.SUCCEEDED)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(ignore_failure.tick(), Node.SUCCEEDED)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(ignore_failure.tick(), Node.SUCCEEDED)
        self.assertEqual(self.fail_fail_succeed.state, Node.SUCCEEDED)

        self.assertEqual(ignore_failure.shutdown(), Node.SHUTDOWN)

    def testIgnoreSuccess(self):
        ignore_success = IgnoreSuccess()
        ignore_success.setup()
        self.assertEqual(ignore_success.tick(), Node.FAILED)
        self.assertEqual(ignore_success.untick(), Node.IDLE)
        self.assertEqual(ignore_success.reset(), Node.IDLE)
        self.assertEqual(ignore_success.shutdown(), Node.SHUTDOWN)

        ignore_success.add_child(self.fail_fail_succeed)
        ignore_success.setup()

        self.assertEqual(ignore_success.tick(), Node.FAILED)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(ignore_success.untick(), Node.PAUSED)
        self.assertEqual(ignore_success.reset(), Node.IDLE)
        self.assertEqual(ignore_success.tick(), Node.FAILED)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(ignore_success.tick(), Node.FAILED)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(ignore_success.tick(), Node.FAILED)
        self.assertEqual(self.fail_fail_succeed.state, Node.SUCCEEDED)

        self.assertEqual(ignore_success.shutdown(), Node.SHUTDOWN)

    def testUntilSuccess(self):
        until_success = UntilSuccess()
        until_success.setup()
        self.assertEqual(until_success.tick(), Node.SUCCEEDED)
        self.assertEqual(until_success.untick(), Node.IDLE)
        self.assertEqual(until_success.reset(), Node.IDLE)
        self.assertEqual(until_success.shutdown(), Node.SHUTDOWN)

        until_success.add_child(self.fail_fail_succeed)
        until_success.setup()

        self.assertEqual(until_success.tick(), Node.RUNNING)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(until_success.untick(), Node.PAUSED)
        self.assertEqual(until_success.reset(), Node.IDLE)
        self.assertEqual(until_success.tick(), Node.RUNNING)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(until_success.tick(), Node.RUNNING)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(until_success.tick(), Node.SUCCEEDED)
        self.assertEqual(self.fail_fail_succeed.state, Node.SUCCEEDED)

        self.assertEqual(until_success.shutdown(), Node.SHUTDOWN)

    def testInverter(self):
        inverter = Inverter()
        inverter.setup()
        self.assertEqual(inverter.tick(), Node.SUCCEEDED)
        self.assertEqual(inverter.untick(), Node.IDLE)
        self.assertEqual(inverter.reset(), Node.IDLE)
        self.assertEqual(inverter.shutdown(), Node.SHUTDOWN)

        inverter.add_child(self.running)
        inverter.setup()
        self.assertEqual(inverter.tick(), Node.RUNNING)
        self.assertEqual(inverter.shutdown(), Node.SHUTDOWN)

        inverter.remove_child(child_name="running")

        inverter.add_child(self.fail_fail_succeed)
        inverter.setup()

        self.assertEqual(inverter.tick(), Node.SUCCEEDED)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(inverter.untick(), Node.PAUSED)
        self.assertEqual(inverter.reset(), Node.IDLE)
        self.assertEqual(inverter.tick(), Node.SUCCEEDED)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(inverter.tick(), Node.SUCCEEDED)
        self.assertEqual(self.fail_fail_succeed.state, Node.FAILED)
        self.assertEqual(inverter.tick(), Node.FAILED)
        self.assertEqual(self.fail_fail_succeed.state, Node.SUCCEEDED)

        self.assertEqual(inverter.shutdown(), Node.SHUTDOWN)

    def testRetry(self):
        retry = Retry(options={'num_retries': 1})
        retry.setup()
        self.assertEqual(retry.tick(), Node.SUCCEEDED)
        self.assertEqual(retry.untick(), Node.IDLE)
        self.assertEqual(retry.reset(), Node.IDLE)
        self.assertEqual(retry.shutdown(), Node.SHUTDOWN)

        retry.add_child(self.succeeder)
        retry.setup()
        self.assertEqual(retry.tick(), Node.SUCCEEDED)
        self.assertEqual(retry.untick(), Node.PAUSED)
        self.assertEqual(retry.reset(), Node.IDLE)
        self.assertEqual(retry.shutdown(), Node.SHUTDOWN)
        retry.remove_child(child_name="succeeder")

        retry.add_child(self.failer)
        retry.setup()

        self.assertEqual(retry.tick(), Node.RUNNING)
        self.assertEqual(self.failer.state, Node.IDLE)
        self.assertEqual(self.failer.tick_count, 1)
        self.assertEqual(self.failer.reset_count, 1)

        self.assertEqual(retry.untick(), Node.PAUSED)
        self.assertEqual(retry.reset(), Node.IDLE)

        self.assertEqual(retry.tick(), Node.RUNNING)
        self.assertEqual(self.failer.state, Node.IDLE)
        self.assertEqual(self.failer.tick_count, 2)
        self.assertEqual(self.failer.reset_count, 3)

        self.assertEqual(retry.tick(), Node.FAILED)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.failer.tick_count, 3)
        self.assertEqual(self.failer.reset_count, 3)

        self.assertEqual(retry.shutdown(), Node.SHUTDOWN)

    def testRepeat(self):
        repeat = Repeat(options={'num_repeats': 3})

        # check the no-children case
        repeat.setup()
        self.assertEqual(repeat.tick(), Node.SUCCEEDED)
        self.assertEqual(repeat.untick(), Node.IDLE)
        self.assertEqual(repeat.reset(), Node.IDLE)
        self.assertEqual(repeat.shutdown(), Node.SHUTDOWN)

        repeat.add_child(self.running)
        repeat.setup()
        self.assertEqual(repeat.reset(), Node.IDLE)
        self.assertEqual(repeat.tick(), Node.RUNNING)
        self.assertEqual(repeat.untick(), Node.PAUSED)
        self.assertEqual(repeat.shutdown(), Node.SHUTDOWN)

        repeat.remove_child(child_name="running")

        self.assertEqual(repeat.reset(), Node.IDLE)
        self.assertEqual(repeat.shutdown(), Node.SHUTDOWN)

        repeat.add_child(self.failer)
        repeat.setup()

        self.assertEqual(repeat.tick(), Node.FAILED)

        self.assertEqual(repeat.shutdown(), Node.SHUTDOWN)

        repeat.remove_child(child_name="failer")

        self.assertEqual(repeat.reset(), Node.IDLE)
        self.assertEqual(repeat.shutdown(), Node.SHUTDOWN)

        repeat.add_child(self.succeeder)
        repeat.setup()

        self.assertEqual(repeat.tick(), Node.RUNNING)
        self.assertEqual(repeat.tick(), Node.RUNNING)
        self.assertEqual(repeat.tick(), Node.RUNNING)
        self.assertEqual(repeat.tick(), Node.SUCCEEDED)

    def testRepeatAlways(self):
        repeat_always = RepeatAlways()

        # check the no-children case
        repeat_always.setup()
        self.assertEqual(repeat_always.tick(), Node.SUCCEEDED)
        self.assertEqual(repeat_always.untick(), Node.IDLE)
        self.assertEqual(repeat_always.reset(), Node.IDLE)
        self.assertEqual(repeat_always.shutdown(), Node.SHUTDOWN)

        repeat_always.add_child(self.succeeder)
        repeat_always.setup()

        self.assertEqual(repeat_always.tick(), Node.RUNNING)
        self.assertEqual(repeat_always.tick(), Node.RUNNING)
        self.assertEqual(repeat_always.tick(), Node.RUNNING)
        self.assertEqual(repeat_always.tick(), Node.RUNNING)

        self.assertEqual(repeat_always.untick(), Node.PAUSED)
        self.assertEqual(repeat_always.reset(), Node.IDLE)
        self.assertEqual(repeat_always.shutdown(), Node.SHUTDOWN)

    def testRepeatUntilFail(self):
        repeat_until_fail = RepeatUntilFail()

        # check the no-children case
        repeat_until_fail.setup()
        self.assertEqual(repeat_until_fail.tick(), Node.SUCCEEDED)
        self.assertEqual(repeat_until_fail.untick(), Node.IDLE)
        self.assertEqual(repeat_until_fail.reset(), Node.IDLE)
        self.assertEqual(repeat_until_fail.shutdown(), Node.SHUTDOWN)

        repeat_until_fail.add_child(self.run_fail_succeed)
        repeat_until_fail.setup()
        self.assertEqual(repeat_until_fail.tick(), Node.RUNNING)
        self.assertEqual(self.run_fail_succeed.state, Node.RUNNING)
        self.assertEqual(repeat_until_fail.tick(), Node.FAILED)
        self.assertEqual(self.run_fail_succeed.state, Node.FAILED)
        self.assertEqual(repeat_until_fail.tick(), Node.RUNNING)
        self.assertEqual(self.run_fail_succeed.state, Node.IDLE)
        self.assertEqual(repeat_until_fail.tick(), Node.RUNNING)
        self.assertEqual(self.run_fail_succeed.state, Node.RUNNING)
        self.assertEqual(repeat_until_fail.tick(), Node.FAILED)
        self.assertEqual(self.run_fail_succeed.state, Node.FAILED)

        self.assertEqual(repeat_until_fail.untick(), Node.PAUSED)
        self.assertEqual(repeat_until_fail.reset(), Node.IDLE)
        self.assertEqual(repeat_until_fail.shutdown(), Node.SHUTDOWN)

    def testOptional(self):
        optional = Optional()
        optional.setup()
        self.assertEqual(optional.reset(), Node.IDLE)
        self.assertEqual(optional.shutdown(), Node.SHUTDOWN)

        optional.add_child(self.failer)

        self.assertEqual(optional.calculate_utility(), self.failer.calculate_utility())

        optional.setup()
        # Since can_exec can execute, optional calls its setup method
        self.assertTrue(hasattr(self.failer, 'setup_called'))
        # optional should tick failer
        self.assertEqual(optional.tick(), Node.FAILED)
        self.assertEqual(self.failer.tick_count, 1)

        self.assertEqual(optional.untick(), Node.PAUSED)
        self.assertEqual(optional.shutdown(), Node.SHUTDOWN)
        self.assertEqual(optional.reset(), Node.IDLE)

        optional = Optional()\
            .add_child(self.cannot_exec)

        opt_bounds = optional.calculate_utility()
        self.assertNotEqual(opt_bounds, self.cannot_exec.calculate_utility())
        self.assertTrue(opt_bounds.can_execute)
        self.assertFalse(opt_bounds.has_lower_bound_success)
        self.assertFalse(opt_bounds.has_upper_bound_success)
        self.assertFalse(opt_bounds.has_lower_bound_failure)
        self.assertFalse(opt_bounds.has_upper_bound_failure)

        optional.setup()
        # cannot_exec can't execute, so its setup method is never called
        self.assertFalse(hasattr(self.cannot_exec, 'setup_called'))
        # but optional can still be successfully ticked!
        self.assertEqual(optional.tick(), Node.SUCCEEDED)
        # since setup() wasn't called, cannot_exec does not have
        # tick_count. this also means calling cannot_exec.tick() would
        # throw, but optional does not call it, so no exception is
        # raised
        self.assertFalse(hasattr(self.cannot_exec, 'tick_count'))

        self.assertEqual(optional.untick(), Node.IDLE)
        self.assertEqual(optional.shutdown(), Node.SHUTDOWN)
        self.assertEqual(optional.reset(), Node.IDLE)

    @mock.patch('ros_bt_py.nodes.decorators.rospy.Time.now')
    def testThrottle(self, mock_time_now):
        throttle = Throttle(options={'tick_interval': 100.})

        throttle.setup()
        self.assertEqual(throttle.tick(), Node.FAILED)
        self.assertEqual(throttle.untick(), Node.IDLE)
        self.assertEqual(throttle.reset(), Node.IDLE)
        self.assertEqual(throttle.shutdown(), Node.SHUTDOWN)

        magic_leaf = MockLeaf(name='magic_leaf',
                              options={'output_type': int,
                                       'state_values': [],
                                       'output_values': []})
        magic_leaf._do_tick = mock.MagicMock()
        throttle.add_child(magic_leaf)
        throttle.setup()

        # Should tick its child and return its state
        mock_time_now.return_value = Time.from_seconds(0.)
        magic_leaf._do_tick.return_value = Node.FAILED
        self.assertEqual(throttle.tick(), Node.FAILED)
        magic_leaf._do_tick.assert_called()
        magic_leaf._do_tick.reset_mock()

        # Should not tick its child multiple times within tick_interval
        mock_time_now.return_value = Time.from_seconds(50.)
        self.assertEqual(throttle.tick(), Node.FAILED)
        magic_leaf._do_tick.assert_not_called()
        magic_leaf._do_tick.reset_mock()

        # Should tick its child after tick interval
        mock_time_now.return_value = Time.from_seconds(105.)
        magic_leaf._do_tick.return_value = Node.SUCCEEDED
        self.assertEqual(throttle.tick(), Node.SUCCEEDED)
        magic_leaf._do_tick.assert_called()
        magic_leaf._do_tick.reset_mock()

        # Should not tick its child multiple times within tick_interval
        mock_time_now.return_value = Time.from_seconds(150.)
        magic_leaf._do_tick.return_value = Node.FAILED
        self.assertEqual(throttle.tick(), Node.SUCCEEDED)
        magic_leaf._do_tick.assert_not_called()
        magic_leaf._do_tick.reset_mock()

        self.assertEqual(throttle.shutdown(), Node.SHUTDOWN)

    @mock.patch('ros_bt_py.nodes.decorators.rospy.Time.now')
    def testThrottleSuccess(self, mock_time_now):
        throttle_success = ThrottleSuccess(options={'tick_interval': 100.})
        throttle_success.setup()
        self.assertEqual(throttle_success.tick(), Node.FAILED)
        self.assertEqual(throttle_success.untick(), Node.IDLE)
        self.assertEqual(throttle_success.reset(), Node.IDLE)
        self.assertEqual(throttle_success.shutdown(), Node.SHUTDOWN)

        magic_leaf = MockLeaf(name='magic_leaf_2',
                              options={'output_type': int,
                                       'state_values': [],
                                       'output_values': []})
        magic_leaf._do_tick = mock.MagicMock()
        throttle_success.add_child(magic_leaf)
        throttle_success.setup()

        # Should tick its child and return its state
        mock_time_now.return_value = Time.from_seconds(0.)
        magic_leaf._do_tick.return_value = Node.FAILED
        self.assertEqual(throttle_success.tick(), Node.FAILED)
        magic_leaf._do_tick.assert_called()
        magic_leaf._do_tick.reset_mock()

        # Should tick its child again as it did not succeed
        mock_time_now.return_value = Time.from_seconds(1.)
        magic_leaf._do_tick.return_value = Node.SUCCEEDED
        self.assertEqual(throttle_success.tick(), Node.SUCCEEDED)
        magic_leaf._do_tick.assert_called()
        magic_leaf._do_tick.reset_mock()

        # Should not tick its child multiple times within tick_interval and return FAILED
        mock_time_now.return_value = Time.from_seconds(50.)
        self.assertEqual(throttle_success.tick(), Node.FAILED)
        magic_leaf._do_tick.assert_not_called()
        magic_leaf._do_tick.reset_mock()

        # Should tick its child after tick interval
        mock_time_now.return_value = Time.from_seconds(105.)
        magic_leaf._do_tick.return_value = Node.SUCCEEDED
        self.assertEqual(throttle_success.tick(), Node.SUCCEEDED)
        magic_leaf._do_tick.assert_called()
        magic_leaf._do_tick.reset_mock()

        self.assertEqual(throttle_success.shutdown(), Node.SHUTDOWN)
