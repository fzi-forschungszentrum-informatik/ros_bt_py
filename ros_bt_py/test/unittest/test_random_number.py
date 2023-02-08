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
from ros_bt_py.nodes.random_number import RandomInt, RandomIntInputs

from ros_bt_py.exceptions import BehaviorTreeException


class TestRandomInt(unittest.TestCase):
    def testEqualNumbers(self):
        random_int = RandomInt({"min": 0, "max": 0})
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        self.assertRaises(BehaviorTreeException, random_int.setup)

    def testMinGreaterMax(self):
        random_int = RandomInt({"min": 1, "max": 0})
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        self.assertRaises(BehaviorTreeException, random_int.setup)

    def testMaxGreaterMin(self):
        random_int = RandomInt({"min": 0, "max": 1})
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        random_int.setup()

        self.assertEqual(random_int.state, NodeMsg.IDLE)
        self.assertEqual(random_int.tick(), NodeMsg.SUCCEEDED)
        self.assertAlmostEqual(random_int.outputs["random_number"], 0, delta=1)

        self.assertEqual(random_int.untick(), NodeMsg.IDLE)
        self.assertEqual(random_int.reset(), NodeMsg.IDLE)
        self.assertEqual(random_int.shutdown(), NodeMsg.SHUTDOWN)


class TestRandomIntInputs(unittest.TestCase):
    def testEqualNumbers(self):
        random_int = RandomIntInputs()
        random_int.inputs["min"] = 0
        random_int.inputs["max"] = 0

        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)

        random_int.setup()

        self.assertEqual(random_int.state, NodeMsg.IDLE)
        self.assertRaises(BehaviorTreeException, random_int.tick)

    def testMinGreaterMax(self):
        random_int = RandomIntInputs()
        random_int.inputs["min"] = 1
        random_int.inputs["max"] = 0
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        random_int.setup()

        self.assertEqual(random_int.state, NodeMsg.IDLE)
        self.assertRaises(BehaviorTreeException, random_int.tick)

    def testMaxGreaterMin(self):
        random_int = RandomIntInputs()
        random_int.inputs["min"] = 0
        random_int.inputs["max"] = 1
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        random_int.setup()

        self.assertEqual(random_int.state, NodeMsg.IDLE)
        self.assertEqual(random_int.tick(), NodeMsg.SUCCEEDED)
        self.assertAlmostEqual(random_int.outputs["random_number"], 0, delta=1)

        self.assertEqual(random_int.untick(), NodeMsg.IDLE)
        self.assertEqual(random_int.reset(), NodeMsg.IDLE)
        self.assertEqual(random_int.shutdown(), NodeMsg.SHUTDOWN)
