#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022-2023 FZI Forschungszentrum Informatik
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
#    * Neither the name of the {copyright_holder} nor the names of its
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
#  -------- END LICENSE BLOCK --------
import unittest

from ros_bt_py.exceptions import NodeConfigError
from ros_bt_py_msgs.msg import Node

from ros_bt_py.nodes.mock_nodes import MockLeaf, MockUtilityLeaf


class TestMockLeaf(unittest.TestCase):
    def setUp(self):
        self.state_output_pairs = [
            (Node.RUNNING, 1),
            (Node.SUCCEEDED, 2),
            (Node.FAILED, 3),
        ]
        self.leaf = MockLeaf(
            options={
                "output_type": int,
                "state_values": [state for state, _ in self.state_output_pairs],
                "output_values": [output for _, output in self.state_output_pairs],
            }
        )
        self.leaf.setup()

    def testInvalidSetup(self):
        # Wrong type of outputs
        self.assertRaises(
            NodeConfigError,
            MockLeaf(
                options={
                    "output_type": str,
                    "state_values": [Node.IDLE, Node.SUCCEEDED, Node.FAILED],
                    "output_values": [1, 2, 3],
                }
            ).setup,
        )
        # Right type, but wrong number of outputs
        self.assertRaises(
            NodeConfigError,
            MockLeaf(
                options={
                    "output_type": int,
                    "state_values": [Node.IDLE, Node.SUCCEEDED, Node.FAILED],
                    "output_values": [1, 2],
                }
            ).setup,
        )

    def testTick(self):
        self.assertIsNone(self.leaf.outputs["out"])
        # Loop through pairs of states and outputs twice to ensure MockSelf.Leaf
        # properly wraps.
        for state, output in self.state_output_pairs * 2:
            self.leaf.tick()
            self.assertEqual(self.leaf.state, state)
            self.assertEqual(self.leaf.outputs["out"], output)

        self.assertEqual(
            self.leaf.outputs["tick_count"], len(self.state_output_pairs * 2)
        )

    def testUntick(self):
        for state, output in self.state_output_pairs:
            self.leaf.tick()
            self.assertEqual(self.leaf.state, state)
            self.assertEqual(self.leaf.outputs["out"], output)
            self.leaf.untick()
            self.assertEqual(self.leaf.state, Node.PAUSED)
        self.assertEqual(self.leaf.outputs["tick_count"], len(self.state_output_pairs))
        self.assertEqual(
            self.leaf.outputs["untick_count"], len(self.state_output_pairs)
        )

    def testReset(self):
        self.leaf.tick()
        self.assertEqual(self.leaf.state, self.state_output_pairs[0][0])
        self.assertEqual(self.leaf.outputs["out"], self.state_output_pairs[0][1])
        self.assertEqual(self.leaf.tick_count, 1)
        self.leaf.reset()
        self.assertEqual(self.leaf.tick_count, 1)
        self.assertEqual(self.leaf.reset_count, 1)
        self.assertEqual(self.leaf.state, Node.IDLE)

        # After resetting, we should get the first output and state again
        self.leaf.tick()
        self.assertEqual(self.leaf.state, self.state_output_pairs[0][0])
        self.assertEqual(self.leaf.outputs["out"], self.state_output_pairs[0][1])


class TestMockUtilityLeaf(unittest.TestCase):
    def setUp(self):
        self.state_output_pairs = [
            (Node.RUNNING, 1),
            (Node.SUCCEEDED, 2),
            (Node.FAILED, 3),
        ]
        self.leaf = MockUtilityLeaf(
            name="leaf",
            options={
                "can_execute": True,
                "utility_lower_bound_success": 5.0,
                "utility_upper_bound_success": 10.0,
                "utility_lower_bound_failure": 1.0,
                "utility_upper_bound_failure": 2.0,
            },
        )
        self.leaf.setup()

    def testLeaf(self):
        self.assertEqual(self.leaf.tick_count, 0)
        self.assertEqual(self.leaf.tick(), Node.SUCCEEDED)
        self.assertEqual(self.leaf.tick_count, 1)

        self.assertEqual(self.leaf.untick_count, 0)
        self.assertEqual(self.leaf.untick(), Node.IDLE)
        self.assertEqual(self.leaf.untick_count, 1)

        self.assertEqual(self.leaf.reset_count, 0)
        self.assertEqual(self.leaf.reset(), Node.IDLE)
        self.assertEqual(self.leaf.reset_count, 1)

        self.assertEqual(self.leaf.shutdown_count, 0)
        self.assertEqual(self.leaf.shutdown(), Node.SHUTDOWN)
        self.assertEqual(self.leaf.shutdown_count, 1)
