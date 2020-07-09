from copy import deepcopy
import unittest

from ros_bt_py_msgs.msg import Node, UtilityBounds

from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.nodes.mock_nodes import MockLeaf, MockUtilityLeaf
from ros_bt_py.nodes.parallel import Parallel, ParallelFailureTolerance


def make_parallel(needed_successes):
    return Parallel(options={'needed_successes': needed_successes})


def make_parallel_failure_tolerance(needed_successes, tolerate_failures):
    return ParallelFailureTolerance(options={
        'needed_successes': needed_successes,
        'tolerate_failures': tolerate_failures
    })


class TestParallel(unittest.TestCase):
    def setUp(self):
        self.succeeder = MockLeaf(name='succeeder',
                                  options={'output_type': int,
                                           'state_values': [Node.SUCCEEDED],
                                           'output_values': [1]})
        self.failer = MockLeaf(name='failer',
                               options={'output_type': int,
                                        'state_values': [Node.FAILED],
                                        'output_values': [1]})
        self.run_then_succeed = MockLeaf(name='run_then_succeed',
                                         options={'output_type': int,
                                                  'state_values': [Node.RUNNING, Node.SUCCEEDED],
                                                  'output_values': [1, 1]})
        self.run_then_fail = MockLeaf(name='run_then_fail',
                                      options={'output_type': int,
                                               'state_values': [Node.RUNNING, Node.FAILED],
                                               'output_values': [1, 1]})
        self.runner = MockLeaf(name='runner',
                               options={'output_type': int,
                                        'state_values': [Node.RUNNING],
                                        'output_values': [1]})

        self.cheap_fail = MockUtilityLeaf(
            name='cheap_fail',
            options={
                'can_execute': True,
                'utility_lower_bound_success': 5.0,
                'utility_upper_bound_success': 10.0,
                'utility_lower_bound_failure': 1.0,
                'utility_upper_bound_failure': 2.0})
        self.cheap_success = MockUtilityLeaf(
            name='cheap_success',
            options={
                'can_execute': True,
                'utility_lower_bound_success': 1.0,
                'utility_upper_bound_success': 2.0,
                'utility_lower_bound_failure': 5.0,
                'utility_upper_bound_failure': 10.0})
        self.can_not_execute = MockUtilityLeaf(
            name='can_not_execute',
            options={
                'can_execute': False,
                'utility_lower_bound_success': 0.0,
                'utility_upper_bound_success': 0.0,
                'utility_lower_bound_failure': 0.0,
                'utility_upper_bound_failure': 0.0})

    def testSuccessesException(self):
        par = make_parallel(3)\
            .add_child(self.succeeder)\
            .add_child(self.run_then_succeed)

        self.assertRaises(BehaviorTreeException, par.setup)
        self.assertRaises(BehaviorTreeException, par.calculate_utility)

    def testWithRunningChildren(self):
        par = make_parallel(2)\
            .add_child(self.failer)\
            .add_child(self.runner)

        par.setup()
        self.assertEqual(par.tick(), Node.FAILED)
        self.assertEqual(par.untick(), Node.IDLE)
        self.assertEqual(par.reset(), Node.IDLE)
        self.assertEqual(par.shutdown(), Node.SHUTDOWN)

    def testBarrierSuccess(self):
        par = make_parallel(2)\
            .add_child(self.succeeder)\
            .add_child(self.run_then_succeed)

        par.setup()

        # run_then_succeed returns RUNNING on the first tick, so we
        # need a second tick
        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(par.tick(), Node.SUCCEEDED)

        # succeeder should not be ticked again as long as
        # run_then_succeed has not produced a result
        self.assertEqual(self.succeeder.tick_count, 1)
        self.assertEqual(self.run_then_succeed.tick_count, 2)

        self.assertEqual(par.tick(), Node.RUNNING)
        # The Parallel should reset its children after producing a
        # result
        self.assertEqual(self.succeeder.reset_count, 1)
        self.assertEqual(self.run_then_succeed.reset_count, 1)

        self.assertEqual(par.tick(), Node.SUCCEEDED)

        self.assertEqual(self.succeeder.tick_count, 2)
        self.assertEqual(self.run_then_succeed.tick_count, 4)

        par.shutdown()

    def testBarrierFailure(self):
        par = make_parallel(2)\
            .add_child(self.succeeder)\
            .add_child(self.run_then_fail)

        par.setup()

        # run_then_fail returns RUNNING on the first tick, so we
        # need a second tick
        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(par.tick(), Node.FAILED)

        # succeeder should not be ticked again as long as
        # run_then_fail has not produced a result
        self.assertEqual(self.succeeder.tick_count, 1)
        self.assertEqual(self.run_then_fail.tick_count, 2)

        self.assertEqual(par.tick(), Node.RUNNING)
        # The Parallel should reset its children after producing a
        # result
        self.assertEqual(self.succeeder.reset_count, 1)
        self.assertEqual(self.run_then_fail.reset_count, 1)

        self.assertEqual(par.tick(), Node.FAILED)

        self.assertEqual(self.succeeder.tick_count, 2)
        self.assertEqual(self.run_then_fail.tick_count, 4)

        par.shutdown()

    def testHeurekaSuccess(self):
        """The "Heureka" configuration returns SUCCEEDED with just a single succeeding child"""

        par = make_parallel(1)\
            .add_child(self.succeeder)\
            .add_child(self.run_then_fail)

        par.setup()

        # Because succeeder immediately succeeds, the Parallel
        # succeeds and resets run_then_fail before the second tick
        self.assertEqual(par.tick(), Node.SUCCEEDED)
        self.assertEqual(self.succeeder.tick_count, 1)
        self.assertEqual(self.run_then_fail.tick_count, 1)

        self.assertEqual(par.tick(), Node.SUCCEEDED)
        self.assertEqual(self.succeeder.reset_count, 1)
        self.assertEqual(self.run_then_fail.reset_count, 1)
        self.assertEqual(self.succeeder.tick_count, 2)
        self.assertEqual(self.run_then_fail.tick_count, 2)

        par.shutdown()

    def testHeurekaFailure(self):
        """The "Heureka" configuration returns FAILED only when all children fail"""

        par = make_parallel(1)\
            .add_child(self.failer)\
            .add_child(self.run_then_fail)

        par.setup()

        # Because succeeder immediately succeeds, the Parallel
        # succeeds and resets run_then_fail before the second tick
        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(self.failer.tick_count, 1)
        self.assertEqual(self.run_then_fail.tick_count, 1)

        self.assertEqual(par.tick(), Node.FAILED)
        # Again, failer should not be ticked again before run_then_fail produces a result
        self.assertEqual(self.failer.tick_count, 1)
        self.assertEqual(self.run_then_fail.tick_count, 2)

        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(self.failer.reset_count, 1)
        self.assertEqual(self.run_then_fail.reset_count, 1)

        self.assertEqual(par.tick(), Node.FAILED)
        self.assertEqual(self.failer.tick_count, 2)
        self.assertEqual(self.run_then_fail.tick_count, 4)

        par.shutdown()

    def testParallelUtilityCalculation(self):
        par = make_parallel(1)\
            .add_child(self.cheap_success)\
            .add_child(self.cheap_fail)

        expected_bounds = UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True)

        cheap_success_bounds = self.cheap_success.calculate_utility()
        cheap_fail_bounds = self.cheap_fail.calculate_utility()

        expected_bounds.lower_bound_success = cheap_success_bounds.lower_bound_success
        expected_bounds.upper_bound_success = cheap_fail_bounds.upper_bound_success

        expected_bounds.lower_bound_failure = (cheap_success_bounds.lower_bound_failure
                                               + cheap_fail_bounds.lower_bound_failure)
        expected_bounds.upper_bound_failure = (cheap_success_bounds.upper_bound_failure
                                               + cheap_fail_bounds.upper_bound_failure)
        self.assertEqual(par.calculate_utility(), expected_bounds)

        par = make_parallel(2)\
            .add_child(self.cheap_success)\
            .add_child(self.cheap_fail)

        # Now that we need two successes, success and failure are
        # basically swapped
        expected_bounds.lower_bound_success = (cheap_success_bounds.lower_bound_success
                                               + cheap_fail_bounds.lower_bound_success)
        expected_bounds.upper_bound_success = (cheap_success_bounds.upper_bound_success
                                               + cheap_fail_bounds.upper_bound_success)

        expected_bounds.lower_bound_failure = cheap_fail_bounds.lower_bound_failure
        expected_bounds.upper_bound_failure = cheap_success_bounds.upper_bound_failure
        self.assertEqual(par.calculate_utility(), expected_bounds)

    def testParallelUtilityCalculationCanNotExecute(self):
        par = make_parallel(1)\
            .add_child(self.can_not_execute)

        expected_bounds = UtilityBounds(
            can_execute=False,
            has_lower_bound_success=False,
            has_upper_bound_success=False,
            has_lower_bound_failure=False,
            has_upper_bound_failure=False)

        self.assertEqual(par.calculate_utility(), expected_bounds)


class TestParallelFailureTolerance(unittest.TestCase):
    def setUp(self):
        self.succeeder = MockLeaf(name='succeeder',
                                  options={'output_type': int,
                                           'state_values': [Node.SUCCEEDED],
                                           'output_values': [1]})
        self.failer = MockLeaf(name='failer',
                               options={'output_type': int,
                                        'state_values': [Node.FAILED],
                                        'output_values': [1]})
        self.run_then_succeed = MockLeaf(name='run_then_succeed',
                                         options={'output_type': int,
                                                  'state_values': [Node.RUNNING, Node.SUCCEEDED],
                                                  'output_values': [1, 1]})
        self.run_then_fail = MockLeaf(name='run_then_fail',
                                      options={'output_type': int,
                                               'state_values': [Node.RUNNING, Node.FAILED],
                                               'output_values': [1, 1]})
        self.runner = MockLeaf(name='runner',
                               options={'output_type': int,
                                        'state_values': [Node.RUNNING],
                                        'output_values': [1]})

        self.cheap_fail = MockUtilityLeaf(
            name='cheap_fail',
            options={
                'can_execute': True,
                'utility_lower_bound_success': 5.0,
                'utility_upper_bound_success': 10.0,
                'utility_lower_bound_failure': 1.0,
                'utility_upper_bound_failure': 2.0})
        self.cheap_success = MockUtilityLeaf(
            name='cheap_success',
            options={
                'can_execute': True,
                'utility_lower_bound_success': 1.0,
                'utility_upper_bound_success': 2.0,
                'utility_lower_bound_failure': 5.0,
                'utility_upper_bound_failure': 10.0})
        self.can_not_execute = MockUtilityLeaf(
            name='can_not_execute',
            options={
                'can_execute': False,
                'utility_lower_bound_success': 0.0,
                'utility_upper_bound_success': 0.0,
                'utility_lower_bound_failure': 0.0,
                'utility_upper_bound_failure': 0.0})

    def testSuccessesException(self):
        par = make_parallel_failure_tolerance(3, 3)\
            .add_child(self.succeeder)\
            .add_child(self.run_then_succeed)

        self.assertRaises(BehaviorTreeException, par.setup)
        self.assertRaises(BehaviorTreeException, par.calculate_utility)

    def testOverlyOptimistic(self):
        """Fail if two failures are received"""

        par = make_parallel_failure_tolerance(2, 2)\
            .add_child(self.failer)\
            .add_child(self.runner)

        par.setup()
        # the node tolerates 2 failures,
        # so it continues ticking the running child
        # even if it can get 2 successes anymore
        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(self.failer.tick_count, 1)
        self.assertEqual(self.runner.tick_count, 2)

        self.assertEqual(par.untick(), Node.IDLE)
        self.assertEqual(par.reset(), Node.IDLE)
        self.assertEqual(par.shutdown(), Node.SHUTDOWN)

    def testOverlyPessimistic(self):
        """Fail after first failure is received"""

        par = make_parallel_failure_tolerance(1, 0)\
            .add_child(self.failer)\
            .add_child(self.run_then_fail)

        par.setup()

        # the node tolerates 1 failure,
        # so it fails at the first tick,
        # even if it could still get 1 success from the other child
        self.assertEqual(par.tick(), Node.FAILED)
        self.assertEqual(self.failer.tick_count, 1)
        self.assertEqual(self.run_then_fail.tick_count, 1)

        par.shutdown()

    def testBarrierSuccess(self):
        par = make_parallel_failure_tolerance(2, 2)\
            .add_child(self.succeeder)\
            .add_child(self.run_then_succeed)

        par.setup()

        # run_then_succeed returns RUNNING on the first tick, so we
        # need a second tick
        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(par.tick(), Node.SUCCEEDED)

        # succeeder should not be ticked again as long as
        # run_then_succeed has not produced a result
        self.assertEqual(self.succeeder.tick_count, 1)
        self.assertEqual(self.run_then_succeed.tick_count, 2)

        self.assertEqual(par.tick(), Node.RUNNING)
        # The Parallel should reset its children after producing a
        # result
        self.assertEqual(self.succeeder.reset_count, 1)
        self.assertEqual(self.run_then_succeed.reset_count, 1)

        self.assertEqual(par.tick(), Node.SUCCEEDED)

        self.assertEqual(self.succeeder.tick_count, 2)
        self.assertEqual(self.run_then_succeed.tick_count, 4)

        par.shutdown()

    def testBarrierFailure(self):
        par = make_parallel_failure_tolerance(2, 0)\
            .add_child(self.succeeder)\
            .add_child(self.run_then_fail)

        par.setup()

        # run_then_fail returns RUNNING on the first tick, so we
        # need a second tick
        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(par.tick(), Node.FAILED)

        # succeeder should not be ticked again as long as
        # run_then_fail has not produced a result
        self.assertEqual(self.succeeder.tick_count, 1)
        self.assertEqual(self.run_then_fail.tick_count, 2)

        self.assertEqual(par.tick(), Node.RUNNING)
        # The Parallel should reset its children after producing a
        # result
        self.assertEqual(self.succeeder.reset_count, 1)
        self.assertEqual(self.run_then_fail.reset_count, 1)

        self.assertEqual(par.tick(), Node.FAILED)

        self.assertEqual(self.succeeder.tick_count, 2)
        self.assertEqual(self.run_then_fail.tick_count, 4)

        par.shutdown()

    def testHeurekaSuccess(self):
        """The "Heureka" configuration returns SUCCEEDED with just a single succeeding child"""

        par = make_parallel_failure_tolerance(1, 1)\
            .add_child(self.succeeder)\
            .add_child(self.run_then_fail)

        par.setup()

        # Because succeeder immediately succeeds, the Parallel
        # succeeds and resets run_then_fail before the second tick
        self.assertEqual(par.tick(), Node.SUCCEEDED)
        self.assertEqual(self.succeeder.tick_count, 1)
        self.assertEqual(self.run_then_fail.tick_count, 1)

        self.assertEqual(par.tick(), Node.SUCCEEDED)
        self.assertEqual(self.succeeder.reset_count, 1)
        self.assertEqual(self.run_then_fail.reset_count, 1)
        self.assertEqual(self.succeeder.tick_count, 2)
        self.assertEqual(self.run_then_fail.tick_count, 2)

        par.shutdown()

    def testHeurekaFailure(self):
        """The "Heureka" configuration returns FAILED only when all children fail"""

        par = make_parallel_failure_tolerance(1, 1)\
            .add_child(self.failer)\
            .add_child(self.run_then_fail)

        par.setup()

        # Because succeeder immediately succeeds, the Parallel
        # succeeds and resets run_then_fail before the second tick
        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(self.failer.tick_count, 1)
        self.assertEqual(self.run_then_fail.tick_count, 1)

        self.assertEqual(par.tick(), Node.FAILED)
        # Again, failer should not be ticked again before run_then_fail produces a result
        self.assertEqual(self.failer.tick_count, 1)
        self.assertEqual(self.run_then_fail.tick_count, 2)

        self.assertEqual(par.tick(), Node.RUNNING)
        self.assertEqual(self.failer.reset_count, 1)
        self.assertEqual(self.run_then_fail.reset_count, 1)

        self.assertEqual(par.tick(), Node.FAILED)
        self.assertEqual(self.failer.tick_count, 2)
        self.assertEqual(self.run_then_fail.tick_count, 4)

        par.shutdown()

    def testParallelUtilityCalculation(self):
        par = make_parallel_failure_tolerance(1, 1)\
            .add_child(self.cheap_success)\
            .add_child(self.cheap_fail)

        expected_bounds = UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True)

        cheap_success_bounds = self.cheap_success.calculate_utility()
        cheap_fail_bounds = self.cheap_fail.calculate_utility()

        expected_bounds.lower_bound_success = cheap_success_bounds.lower_bound_success
        expected_bounds.upper_bound_success = cheap_fail_bounds.upper_bound_success

        expected_bounds.lower_bound_failure = (cheap_success_bounds.lower_bound_failure
                                               + cheap_fail_bounds.lower_bound_failure)
        expected_bounds.upper_bound_failure = (cheap_success_bounds.upper_bound_failure
                                               + cheap_fail_bounds.upper_bound_failure)
        self.assertEqual(par.calculate_utility(), expected_bounds)

        par = make_parallel_failure_tolerance(2, 0)\
            .add_child(self.cheap_success)\
            .add_child(self.cheap_fail)

        # Now that we need two successes, success and failure are
        # basically swapped
        expected_bounds.lower_bound_success = (cheap_success_bounds.lower_bound_success
                                               + cheap_fail_bounds.lower_bound_success)
        expected_bounds.upper_bound_success = (cheap_success_bounds.upper_bound_success
                                               + cheap_fail_bounds.upper_bound_success)

        expected_bounds.lower_bound_failure = cheap_fail_bounds.lower_bound_failure
        expected_bounds.upper_bound_failure = cheap_success_bounds.upper_bound_failure
        self.assertEqual(par.calculate_utility(), expected_bounds)

    def testParallelUtilityCalculationCanNotExecute(self):
        par = make_parallel_failure_tolerance(1, 0)\
            .add_child(self.can_not_execute)

        expected_bounds = UtilityBounds(
            can_execute=False,
            has_lower_bound_success=False,
            has_upper_bound_success=False,
            has_lower_bound_failure=False,
            has_upper_bound_failure=False)

        self.assertEqual(par.calculate_utility(), expected_bounds)
