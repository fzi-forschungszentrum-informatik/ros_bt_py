from copy import deepcopy
import unittest

from ros_bt_py_msgs.msg import Node, UtilityBounds

from ros_bt_py.nodes.mock_nodes import MockLeaf, MockUtilityLeaf
from ros_bt_py.nodes.fallback import Fallback


class TestFallback(unittest.TestCase):
    def setUp(self):
        self.succeeder = MockLeaf(name='succeeder',
                                  options={'output_type': int,
                                           'state_values': [Node.SUCCEEDED],
                                           'output_values': [1]})
        self.failer = MockLeaf(name='failer',
                               options={'output_type': int,
                                        'state_values': [Node.FAILED],
                                        'output_values': [1]})
        self.runner = MockLeaf(name='runner',
                               options={'output_type': int,
                                        'state_values': [Node.RUNNING],
                                        'output_values': [1]})
        self.cheap_fail = MockUtilityLeaf(
            name='cheap_fail',
            options={
                'utility_lower_bound_success': 5.0,
                'utility_upper_bound_success': 10.0,
                'utility_lower_bound_failure': 1.0,
                'utility_upper_bound_failure': 2.0})
        self.cheap_success = MockUtilityLeaf(
            name='cheap_success',
            options={
                'utility_lower_bound_success': 1.0,
                'utility_upper_bound_success': 2.0,
                'utility_lower_bound_failure': 5.0,
                'utility_upper_bound_failure': 10.0})

        self.fallback = Fallback()

    def testEmptyTickFails(self):
        self.fallback.setup()
        self.fallback.tick()
        self.assertEqual(self.fallback.state, Node.FAILED)

    def testTickSuccess(self):
        self.fallback.add_child(self.succeeder)
        self.fallback.add_child(self.failer)
        self.fallback.setup()

        self.fallback.tick()

        # Fallback is done after the first success, so it should untick the
        # failer node.
        self.assertEqual(self.succeeder.outputs['tick_count'], 1)
        self.assertEqual(self.failer.outputs['untick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.SUCCEEDED)
        self.assertEqual(self.fallback.state, Node.SUCCEEDED)

    def testTickSuccessAfterFailure(self):
        self.fallback.add_child(self.failer)
        self.fallback.add_child(self.succeeder)
        self.fallback.setup()

        self.fallback.tick()

        # After the first child failed, the sequence should try the second one
        self.assertEqual(self.failer.outputs['tick_count'], 1)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.succeeder.outputs['tick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.SUCCEEDED)
        self.assertEqual(self.fallback.state, Node.SUCCEEDED)

    def testTickRunning(self):
        self.fallback.add_child(self.failer)
        self.fallback.add_child(self.runner)
        self.fallback.setup()

        self.fallback.tick()

        self.assertEqual(self.failer.outputs['tick_count'], 1)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.runner.outputs['tick_count'], 1)
        self.assertEqual(self.runner.state, Node.RUNNING)
        self.assertEqual(self.fallback.state, Node.RUNNING)

    def testTickFailing(self):
        self.fallback.add_child(self.failer)
        self.fallback.setup()

        self.fallback.tick()

        # There's nothing to fall back on after the only child failed, so the
        # Fallback fails.
        self.assertEqual(self.failer.outputs['tick_count'], 1)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.fallback.state, Node.FAILED)

    def testNested(self):
        inner_fallback = Fallback(name='inner')
        inner_fallback.add_child(self.succeeder)
        self.fallback.add_child(self.failer)
        self.fallback.add_child(inner_fallback)
        self.fallback.setup()

        self.fallback.tick()

        # The failer fails, and the inner Fallback succeeds, resulting in an
        # overall success.
        self.assertEqual(self.failer.outputs['tick_count'], 1)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.succeeder.outputs['tick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.SUCCEEDED)
        self.assertEqual(inner_fallback.state, Node.SUCCEEDED)
        self.assertEqual(self.fallback.state, Node.SUCCEEDED)

    def testCalculateUtility(self):
        self.fallback.add_child(self.cheap_fail)

        self.assertEqual(self.fallback.calculate_utility(),
                         self.cheap_fail.calculate_utility())

        cheap_fail_2 = deepcopy(self.cheap_fail)
        cheap_fail_2.name = 'cheap_fail_2'
        self.fallback.add_child(cheap_fail_2)
        cheap_fail_bounds = self.cheap_fail.calculate_utility()
        expected_bounds = UtilityBounds(has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)

        # Upper and lower bounds for failure should be the sum of the
        # children's bounds
        expected_bounds.lower_bound_failure = cheap_fail_bounds.lower_bound_failure * 2.0
        expected_bounds.upper_bound_failure = cheap_fail_bounds.upper_bound_failure * 2.0
        # Lower bound for success is the same as for a single node
        # (i.e. first child fails as cheaply as possible)
        expected_bounds.lower_bound_success = cheap_fail_bounds.lower_bound_success
        # Upper bound for success is sum of upper *failure* bounds for
        # all children but the last, plus upper *success* bound for
        # the last child. In our case, that's one failing and one
        # succeeding child.
        expected_bounds.upper_bound_success = (
            cheap_fail_bounds.upper_bound_failure +
            cheap_fail_bounds.upper_bound_success)

        self.assertEqual(self.fallback.calculate_utility(),
                         expected_bounds)

    def testUtilityWithDifferentChildren(self):
        self.fallback.add_child(self.cheap_fail)
        self.fallback.add_child(self.cheap_success)

        expected_bounds = UtilityBounds(has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)

        cheap_fail_bounds = self.cheap_fail.calculate_utility()
        cheap_success_bounds = self.cheap_success.calculate_utility()
        expected_bounds.lower_bound_success = (
            cheap_fail_bounds.lower_bound_failure +    # 1.0
            cheap_success_bounds.lower_bound_success)  # 1.0
        expected_bounds.upper_bound_success = (
            cheap_fail_bounds.upper_bound_success)     # 10.0
        expected_bounds.lower_bound_failure = (
            cheap_fail_bounds.lower_bound_failure +    # 1.0
            cheap_success_bounds.lower_bound_failure)  # 5.0
        expected_bounds.upper_bound_failure = (
            cheap_fail_bounds.upper_bound_failure +    # 1.0
            cheap_success_bounds.upper_bound_failure)  # 5.0

        self.assertEqual(self.fallback.calculate_utility(),
                         expected_bounds)
