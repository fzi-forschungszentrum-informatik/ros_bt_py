# Copyright 2018-2023 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from copy import deepcopy
import unittest

try:
    import unittest.mock as mock
except ImportError:
    import mock

from ros_bt_py_msgs.msg import Node, UtilityBounds

from ros_bt_py.nodes.mock_nodes import MockLeaf, MockUtilityLeaf
from ros_bt_py.nodes.decorators import IgnoreFailure, IgnoreSuccess, UntilSuccess
from ros_bt_py.nodes.decorators import Retry, Optional, IgnoreRunning
from ros_bt_py.nodes.decorators import Inverter, Repeat, RepeatAlways, RepeatUntilFail
from ros_bt_py.nodes.decorators import RepeatIfFail, Throttle, ThrottleSuccess
from rospy import Time


class TestDecorators(unittest.TestCase):
    def setUp(self):
        self.succeeder = MockLeaf(
            name="succeeder",
            options={
                "output_type": int,
                "state_values": [Node.SUCCEEDED],
                "output_values": [1],
            },
        )
        self.failer = MockLeaf(
            name="failer",
            options={
                "output_type": int,
                "state_values": [Node.FAILED],
                "output_values": [1],
            },
        )
        self.running = MockLeaf(
            name="running",
            options={
                "output_type": int,
                "state_values": [Node.RUNNING],
                "output_values": [1],
            },
        )
        self.running_fail_running = MockLeaf(
            name="running_fail_running",
            options={
                "output_type": int,
                "state_values": [Node.RUNNING, Node.FAILED, Node.RUNNING],
                "output_values": [1, 1, 1],
            },
        )
        self.running_succeed_running = MockLeaf(
            name="running_succeed_running",
            options={
                "output_type": int,
                "state_values": [Node.RUNNING, Node.SUCCEEDED, Node.RUNNING],
                "output_values": [1, 1, 1],
            },
        )
        self.fail_fail_succeed = MockLeaf(
            name="fail_fail_succeed",
            options={
                "output_type": int,
                "state_values": [Node.FAILED, Node.FAILED, Node.SUCCEEDED],
                "output_values": [1, 1, 1],
            },
        )

        self.run_fail_succeed = MockLeaf(
            name="run_fail_succeed",
            options={
                "output_type": int,
                "state_values": [Node.RUNNING, Node.FAILED, Node.SUCCEEDED],
                "output_values": [1, 1, 1],
            },
        )

        self.cannot_exec = MockUtilityLeaf(
            name="cannot_exec",
            options={
                "can_execute": False,
                "utility_lower_bound_success": 1.0,
                "utility_upper_bound_success": 2.0,
                "utility_lower_bound_failure": 5.0,
                "utility_upper_bound_failure": 10.0,
            },
        )

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

    def testIgnoreRunning(self):
        ignore_running = IgnoreRunning(options={"running_is_success": True})
        ignore_running.setup()

        self.assertEqual(ignore_running.tick(), Node.FAILED)
        self.assertEqual(ignore_running.untick(), Node.IDLE)
        self.assertEqual(ignore_running.reset(), Node.IDLE)
        self.assertEqual(ignore_running.shutdown(), Node.SHUTDOWN)

        def test_for_running_option(running_is_success):
            if running_is_success:
                fail_or_success = Node.SUCCEEDED
            else:
                fail_or_success = Node.FAILED

            ignore_running = IgnoreRunning(
                options={"running_is_success": running_is_success}
            )

            ignore_running.add_child(self.running_succeed_running)
            ignore_running.setup()

            self.assertEqual(ignore_running.tick(), fail_or_success)
            self.assertEqual(ignore_running.untick(), Node.PAUSED)
            self.assertEqual(ignore_running.tick(), Node.SUCCEEDED)
            self.assertEqual(ignore_running.untick(), Node.PAUSED)
            self.assertEqual(ignore_running.tick(), fail_or_success)

            self.assertEqual(ignore_running.shutdown(), Node.SHUTDOWN)
            ignore_running.remove_child(child_name="running_succeed_running")
            ignore_running.add_child(self.running_fail_running)
            ignore_running.setup()

            self.assertEqual(ignore_running.tick(), fail_or_success)
            self.assertEqual(ignore_running.untick(), Node.PAUSED)
            self.assertEqual(ignore_running.tick(), Node.FAILED)
            self.assertEqual(ignore_running.untick(), Node.PAUSED)
            self.assertEqual(ignore_running.tick(), fail_or_success)

            self.assertEqual(ignore_running.shutdown(), Node.SHUTDOWN)

        test_for_running_option(False)
        test_for_running_option(True)

    def testIgnoreRunningWithChild(self):
        ignore_running = IgnoreRunning(options={"running_is_success": True})
        ignore_running.add_child(self.fail_fail_succeed)
        ignore_running.setup()

        self.assertEqual(ignore_running.tick(), Node.FAILED)
        self.assertEqual(ignore_running.untick(), Node.PAUSED)
        self.assertEqual(ignore_running.reset(), Node.IDLE)
        self.assertEqual(ignore_running.shutdown(), Node.SHUTDOWN)

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
        retry = Retry(options={"num_retries": 1})
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
        repeat = Repeat(options={"num_repeats": 3})

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

        # self.assertEqual(repeat.reset(), Node.IDLE)
        self.assertEqual(repeat.shutdown(), Node.SHUTDOWN)

        repeat.add_child(self.failer)
        repeat.setup()

        self.assertEqual(repeat.tick(), Node.FAILED)

        self.assertEqual(repeat.shutdown(), Node.SHUTDOWN)

        repeat.remove_child(child_name="failer")

        # self.assertEqual(repeat.reset(), Node.IDLE)
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

    def testRepeatIfFail(self):
        repeat_if_fail = RepeatIfFail()
        repeat_if_fail.setup()
        self.assertEqual(repeat_if_fail.tick(), Node.SUCCEEDED)
        self.assertEqual(repeat_if_fail.untick(), Node.IDLE)
        self.assertEqual(repeat_if_fail.reset(), Node.IDLE)
        self.assertEqual(repeat_if_fail.shutdown(), Node.SHUTDOWN)

        magic_leaf = MockLeaf(
            name="magic_leaf",
            options={"output_type": int, "state_values": [], "output_values": []},
        )
        magic_leaf._do_tick = mock.MagicMock()
        magic_leaf._do_tick.return_value = Node.FAILED

        # Should tick its child and return its state
        repeat_if_fail.add_child(magic_leaf)
        repeat_if_fail.setup()

        self.assertEqual(repeat_if_fail.tick(), Node.RUNNING)
        magic_leaf._do_tick.reset_mock()

        magic_leaf._do_tick.return_value = Node.SUCCEEDED

        self.assertEqual(repeat_if_fail.tick(), Node.SUCCEEDED)
        magic_leaf._do_tick.reset_mock()

        self.assertEqual(repeat_if_fail.untick(), Node.PAUSED)
        self.assertEqual(repeat_if_fail.reset(), Node.IDLE)
        self.assertEqual(repeat_if_fail.shutdown(), Node.SHUTDOWN)

    def testOptional(self):
        optional = Optional()
        optional.setup()
        self.assertEqual(optional.reset(), Node.IDLE)
        self.assertEqual(optional.shutdown(), Node.SHUTDOWN)

        optional.add_child(self.failer)

        self.assertEqual(optional.calculate_utility(), self.failer.calculate_utility())

        optional.setup()
        # Since can_exec can execute, optional calls its setup method
        self.assertTrue(hasattr(self.failer, "setup_called"))
        # optional should tick failer
        self.assertEqual(optional.tick(), Node.FAILED)
        self.assertEqual(self.failer.tick_count, 1)

        self.assertEqual(optional.untick(), Node.PAUSED)
        self.assertEqual(optional.shutdown(), Node.SHUTDOWN)
        # We cannot reset a shutdown node.
        # self.assertEqual(optional.reset(), Node.IDLE)

        optional = Optional().add_child(self.cannot_exec)

        opt_bounds = optional.calculate_utility()
        self.assertNotEqual(opt_bounds, self.cannot_exec.calculate_utility())
        self.assertTrue(opt_bounds.can_execute)
        self.assertFalse(opt_bounds.has_lower_bound_success)
        self.assertFalse(opt_bounds.has_upper_bound_success)
        self.assertFalse(opt_bounds.has_lower_bound_failure)
        self.assertFalse(opt_bounds.has_upper_bound_failure)

        optional.setup()
        # cannot_exec can't execute, so its setup method is never called
        self.assertFalse(hasattr(self.cannot_exec, "setup_called"))
        # but optional can still be successfully ticked!
        self.assertEqual(optional.tick(), Node.SUCCEEDED)
        # since setup() wasn't called, cannot_exec does not have
        # tick_count. this also means calling cannot_exec.tick() would
        # throw, but optional does not call it, so no exception is
        # raised
        self.assertFalse(hasattr(self.cannot_exec, "tick_count"))

        self.assertEqual(optional.untick(), Node.IDLE)
        self.assertEqual(optional.shutdown(), Node.SHUTDOWN)

    @mock.patch("ros_bt_py.nodes.decorators.rospy.Time.now")
    def testThrottle(self, mock_time_now):
        throttle = Throttle(options={"tick_interval": 100.0})

        throttle.setup()
        self.assertEqual(throttle.tick(), Node.FAILED)
        self.assertEqual(throttle.untick(), Node.IDLE)
        self.assertEqual(throttle.reset(), Node.IDLE)
        self.assertEqual(throttle.shutdown(), Node.SHUTDOWN)

        magic_leaf = MockLeaf(
            name="magic_leaf",
            options={"output_type": int, "state_values": [], "output_values": []},
        )
        magic_leaf._do_tick = mock.MagicMock()
        throttle.add_child(magic_leaf)
        throttle.setup()

        # Should tick its child and return its state
        mock_time_now.return_value = Time.from_seconds(0.0)
        magic_leaf._do_tick.return_value = Node.FAILED
        self.assertEqual(throttle.tick(), Node.FAILED)
        assert magic_leaf._do_tick.call_count > 0
        magic_leaf._do_tick.reset_mock()

        # Should not tick its child multiple times within tick_interval
        mock_time_now.return_value = Time.from_seconds(50.0)
        self.assertEqual(throttle.tick(), Node.FAILED)
        assert magic_leaf._do_tick.call_count == 0
        magic_leaf._do_tick.reset_mock()

        # Should tick its child after tick interval
        mock_time_now.return_value = Time.from_seconds(105.0)
        magic_leaf._do_tick.return_value = Node.SUCCEEDED
        self.assertEqual(throttle.tick(), Node.SUCCEEDED)
        assert magic_leaf._do_tick.call_count > 0
        magic_leaf._do_tick.reset_mock()

        # Should not tick its child multiple times within tick_interval
        mock_time_now.return_value = Time.from_seconds(150.0)
        magic_leaf._do_tick.return_value = Node.FAILED
        self.assertEqual(throttle.tick(), Node.SUCCEEDED)
        assert magic_leaf._do_tick.call_count == 0
        magic_leaf._do_tick.reset_mock()

        self.assertEqual(throttle.untick(), Node.PAUSED)
        self.assertEqual(throttle.reset(), Node.IDLE)
        self.assertEqual(throttle.shutdown(), Node.SHUTDOWN)

    @mock.patch("ros_bt_py.nodes.decorators.rospy.Time.now")
    def testThrottleWithRunningChild(self, mock_time_now):
        throttle = Throttle(options={"tick_interval": 100.0})

        throttle.setup()
        self.assertEqual(throttle.tick(), Node.FAILED)
        self.assertEqual(throttle.untick(), Node.IDLE)
        self.assertEqual(throttle.reset(), Node.IDLE)
        self.assertEqual(throttle.shutdown(), Node.SHUTDOWN)

        magic_leaf = MockLeaf(
            name="magic_leaf",
            options={
                "output_type": int,
                "state_values": [Node.RUNNING],
                "output_values": [1],
            },
        )
        magic_leaf._do_tick = mock.MagicMock()

        throttle.add_child(magic_leaf)
        throttle.setup()
        mock_time_now.return_value = Time.from_seconds(0.0)
        magic_leaf._do_tick.return_value = Node.RUNNING
        self.assertEqual(throttle.tick(), Node.RUNNING)
        assert magic_leaf._do_tick.call_count > 0
        magic_leaf._do_tick.reset_mock()

    @mock.patch("ros_bt_py.nodes.decorators.rospy.Time.now")
    def testThrottleSuccess(self, mock_time_now):
        throttle_success = ThrottleSuccess(options={"tick_interval": 100.0})
        throttle_success.setup()
        self.assertEqual(throttle_success.tick(), Node.FAILED)
        self.assertEqual(throttle_success.untick(), Node.IDLE)
        self.assertEqual(throttle_success.reset(), Node.IDLE)
        self.assertEqual(throttle_success.shutdown(), Node.SHUTDOWN)

        magic_leaf = MockLeaf(
            name="magic_leaf_2",
            options={"output_type": int, "state_values": [], "output_values": []},
        )
        magic_leaf._do_tick = mock.MagicMock()
        throttle_success.add_child(magic_leaf)
        throttle_success.setup()

        # Should tick its child and return its state
        mock_time_now.return_value = Time.from_seconds(0.0)
        magic_leaf._do_tick.return_value = Node.FAILED
        self.assertEqual(throttle_success.tick(), Node.FAILED)
        assert magic_leaf._do_tick.call_count > 0
        magic_leaf._do_tick.reset_mock()

        # Should tick its child again as it did not succeed
        mock_time_now.return_value = Time.from_seconds(1.0)
        magic_leaf._do_tick.return_value = Node.SUCCEEDED
        self.assertEqual(throttle_success.tick(), Node.SUCCEEDED)
        assert magic_leaf._do_tick.call_count > 0
        magic_leaf._do_tick.reset_mock()

        # Should not tick its child multiple times within tick_interval and return FAILED
        mock_time_now.return_value = Time.from_seconds(50.0)
        self.assertEqual(throttle_success.tick(), Node.FAILED)
        assert magic_leaf._do_tick.call_count == 0
        magic_leaf._do_tick.reset_mock()

        # Should tick its child after tick interval
        mock_time_now.return_value = Time.from_seconds(105.0)
        magic_leaf._do_tick.return_value = Node.SUCCEEDED
        self.assertEqual(throttle_success.tick(), Node.SUCCEEDED)
        assert magic_leaf._do_tick.call_count > 0
        magic_leaf._do_tick.reset_mock()

        self.assertEqual(throttle_success.untick(), Node.PAUSED)
        self.assertEqual(throttle_success.reset(), Node.IDLE)
        self.assertEqual(throttle_success.shutdown(), Node.SHUTDOWN)

    @mock.patch("ros_bt_py.nodes.decorators.rospy.Time.now")
    def testThrottleSuccessWithRunningChild(self, mock_time_now):
        throttle = ThrottleSuccess(options={"tick_interval": 100.0})

        throttle.setup()
        self.assertEqual(throttle.tick(), Node.FAILED)
        self.assertEqual(throttle.untick(), Node.IDLE)
        self.assertEqual(throttle.reset(), Node.IDLE)
        self.assertEqual(throttle.shutdown(), Node.SHUTDOWN)

        magic_leaf = MockLeaf(
            name="magic_leaf",
            options={
                "output_type": int,
                "state_values": [Node.RUNNING],
                "output_values": [1],
            },
        )
        magic_leaf._do_tick = mock.MagicMock()

        throttle.add_child(magic_leaf)
        throttle.setup()
        mock_time_now.return_value = Time.from_seconds(0.0)
        magic_leaf._do_tick.return_value = Node.RUNNING
        self.assertEqual(throttle.tick(), Node.RUNNING)
        assert magic_leaf._do_tick.call_count > 0
        magic_leaf._do_tick.reset_mock()
