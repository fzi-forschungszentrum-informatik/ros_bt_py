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

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.compare import Compare, CompareNewOnly, CompareConstant
from ros_bt_py.nodes.compare import ALessThanB, LessThanConstant


class TestCompare(unittest.TestCase):
    def setUp(self):
        self.compare = Compare({"compare_type": int})
        self.compare.setup()
        self.compare.inputs["a"] = 42
        self.compare.inputs["b"] = 42

    def testResult(self):
        # Both inputs are set, we should have an output
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        # Another tick, same result
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.inputs["b"] = 41

        # 'in' changed, so we get a different result, FAILED
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

    def testReset(self):
        # Start as before, 42 == 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.reset()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)
        # Neither of the inputs have updated, so ticking fails
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

        # If we update the inputs, the result changes to SUCCEEDED
        self.compare.inputs["a"] = 42
        self.compare.inputs["b"] = 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

    def testUntick(self):
        self.compare.untick()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)

    def testShutdown(self):
        self.compare.shutdown()
        self.assertEqual(self.compare.state, NodeMsg.SHUTDOWN)


class TestCompareNewOnly(unittest.TestCase):
    def setUp(self):
        self.compare = CompareNewOnly({"compare_type": int})
        self.compare.setup()
        self.compare.inputs["a"] = 42
        self.compare.inputs["b"] = 42

    def testResult(self):
        # Both inputs are set, we should have an output
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        # Another tick, inputs were not updated => RUNNING
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.RUNNING)

        self.compare.inputs["b"] = 41
        # 'in' changed, so we get a different result, FAILED
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

    def testReset(self):
        # Start as before, 42 == 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.reset()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)
        # Neither of the inputs have updated, so the node stays
        # RUNNING
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.RUNNING)

        # If we update the inputs, the result changes to SUCCEEDED
        self.compare.inputs["a"] = 42
        self.compare.inputs["b"] = 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

    def testUntick(self):
        self.compare.untick()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)

    def testShutdown(self):
        self.compare.shutdown()
        self.assertEqual(self.compare.state, NodeMsg.SHUTDOWN)


class TestCompareConstant(unittest.TestCase):
    def setUp(self):
        self.compare = CompareConstant({"compare_type": int, "expected": 5})
        self.compare.setup()
        self.compare.inputs["in"] = 5

    def testResult(self):
        # Both inputs are set, we should have an output
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        # Another tick, same result
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.inputs["in"] = 41

        # 'in' changed, so we get a different result, FAILED
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

    def testReset(self):
        # Start as before, 42 == 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.reset()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)
        # The input hasn't updated yet, so ticking fails
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

        # If we update the input, the result changes to SUCCEEDED
        self.compare.inputs["in"] = 5
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

    def testUntick(self):
        self.compare.untick()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)

    def testShutdown(self):
        self.compare.shutdown()
        self.assertEqual(self.compare.state, NodeMsg.SHUTDOWN)


class TestLessThan(unittest.TestCase):
    def testALessThanB(self):
        less_than = ALessThanB()
        less_than.setup()

        with self.assertRaises(ValueError):
            less_than.tick()

        less_than.inputs["a"] = 42
        less_than.inputs["b"] = 42

        self.assertEqual(less_than.tick(), NodeMsg.FAILED)

        less_than.inputs["a"] = 100
        self.assertEqual(less_than.tick(), NodeMsg.FAILED)

        less_than.inputs["a"] = 1
        self.assertEqual(less_than.tick(), NodeMsg.SUCCEEDED)

        less_than.reset()
        self.assertEqual(less_than.state, NodeMsg.IDLE)
        # Neither of the inputs have updated, so ticking fails
        less_than.tick()
        self.assertEqual(less_than.state, NodeMsg.FAILED)

        # If we update the inputs, the result changes to SUCCEEDED
        less_than.inputs["a"] = 1
        less_than.inputs["b"] = 42
        less_than.tick()
        self.assertEqual(less_than.state, NodeMsg.SUCCEEDED)

        less_than.untick()
        self.assertEqual(less_than.state, NodeMsg.IDLE)

        less_than.shutdown()
        self.assertEqual(less_than.state, NodeMsg.SHUTDOWN)

    def testLessThanConstant(self):
        less_than = LessThanConstant({"target": 42})
        less_than.setup()

        with self.assertRaises(ValueError):
            less_than.tick()

        less_than.inputs["a"] = 42

        self.assertEqual(less_than.tick(), NodeMsg.FAILED)

        less_than.inputs["a"] = 100
        self.assertEqual(less_than.tick(), NodeMsg.FAILED)

        less_than.inputs["a"] = 1
        self.assertEqual(less_than.tick(), NodeMsg.SUCCEEDED)

        less_than.reset()
        self.assertEqual(less_than.state, NodeMsg.IDLE)
        # Neither of the inputs have updated, so ticking fails
        less_than.tick()
        self.assertEqual(less_than.state, NodeMsg.FAILED)

        # If we update the inputs, the result changes to SUCCEEDED
        less_than.inputs["a"] = 1
        less_than.tick()
        self.assertEqual(less_than.state, NodeMsg.SUCCEEDED)

        less_than.untick()
        self.assertEqual(less_than.state, NodeMsg.IDLE)

        less_than.shutdown()
        self.assertEqual(less_than.state, NodeMsg.SHUTDOWN)
