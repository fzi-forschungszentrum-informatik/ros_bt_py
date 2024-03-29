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


import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.maths import Convert, Operation, UnaryOperation
from ros_bt_py.helpers import MathUnaryOperator, MathBinaryOperator
from ros_bt_py.helpers import MathOperandType, MathUnaryOperandType


class TestConvert(unittest.TestCase):
    def testPassthrough(self):
        convert = Convert({"input_type": int, "output_type": int})

        convert.inputs["in"] = int(42)
        convert.setup()

        convert.tick()
        self.assertEqual(convert.state, NodeMsg.SUCCEEDED)
        self.assertEqual(convert.outputs["out"], 42)
        self.assertEqual(type(convert.outputs["out"]), int)

        convert.reset()
        self.assertEqual(convert.state, NodeMsg.IDLE)

        convert.untick()
        self.assertEqual(convert.state, NodeMsg.IDLE)

        convert.shutdown()
        self.assertEqual(convert.state, NodeMsg.SHUTDOWN)

    def testStr(self):
        convert = Convert({"input_type": int, "output_type": str})

        convert.inputs["in"] = int(42)
        convert.setup()

        convert.tick()
        self.assertEqual(convert.state, NodeMsg.SUCCEEDED)
        self.assertEqual(convert.outputs["out"], "42")
        self.assertEqual(type(convert.outputs["out"]), str)

    def testIntToFloat(self):
        convert = Convert({"input_type": int, "output_type": float})

        convert.inputs["in"] = int(42)
        convert.setup()

        convert.tick()
        self.assertEqual(convert.state, NodeMsg.SUCCEEDED)
        self.assertEqual(convert.outputs["out"], 42.0)
        self.assertEqual(type(convert.outputs["out"]), float)

    def testFloatToInt(self):
        convert = Convert({"input_type": float, "output_type": int})

        convert.inputs["in"] = float(42.0)
        convert.setup()

        convert.tick()
        self.assertEqual(convert.state, NodeMsg.SUCCEEDED)
        self.assertEqual(convert.outputs["out"], 42)
        self.assertEqual(type(convert.outputs["out"]), int)

    def testBoolToInt(self):
        convert = Convert({"input_type": bool, "output_type": int})

        convert.inputs["in"] = False
        convert.setup()

        convert.tick()
        self.assertEqual(convert.state, NodeMsg.SUCCEEDED)
        self.assertEqual(convert.outputs["out"], 0)
        self.assertEqual(type(convert.outputs["out"]), int)

        convert.inputs["in"] = True

        convert.tick()
        self.assertEqual(convert.state, NodeMsg.SUCCEEDED)
        self.assertEqual(convert.outputs["out"], 1)
        self.assertEqual(type(convert.outputs["out"]), int)

    def testIntToBool(self):
        convert = Convert({"input_type": int, "output_type": bool})

        convert.inputs["in"] = int(0)
        convert.setup()

        convert.tick()
        self.assertEqual(convert.state, NodeMsg.SUCCEEDED)
        self.assertEqual(convert.outputs["out"], False)
        self.assertEqual(type(convert.outputs["out"]), bool)

        convert.inputs["in"] = int(1)

        convert.tick()
        self.assertEqual(convert.state, NodeMsg.SUCCEEDED)
        self.assertEqual(convert.outputs["out"], True)
        self.assertEqual(type(convert.outputs["out"]), bool)

    def testNotImplemented(self):
        self.assertRaises(
            BehaviorTreeException, Convert, {"input_type": int, "output_type": NodeMsg}
        )


class TestOperation(unittest.TestCase):
    def testAdd(self):
        operation = Operation(
            {
                "operand_type": MathOperandType("int"),
                "operator": MathBinaryOperator("+"),
            }
        )

        operation.inputs["a"] = int(42)
        operation.inputs["b"] = int(23)
        operation.setup()

        operation.tick()
        self.assertEqual(operation.state, NodeMsg.SUCCEEDED)
        self.assertEqual(operation.outputs["result"], 65)
        self.assertEqual(type(operation.outputs["result"]), int)

        operation.reset()
        self.assertEqual(operation.state, NodeMsg.IDLE)

        operation.untick()
        self.assertEqual(operation.state, NodeMsg.IDLE)

        operation.shutdown()
        self.assertEqual(operation.state, NodeMsg.SHUTDOWN)

    def testAddFloat(self):
        operation = Operation(
            {
                "operand_type": MathOperandType("float"),
                "operator": MathBinaryOperator("+"),
            }
        )

        operation.inputs["a"] = float(0)
        operation.inputs["b"] = float(0)
        operation.setup()

        operation.tick()
        self.assertEqual(operation.state, NodeMsg.SUCCEEDED)
        self.assertEqual(operation.outputs["result"], 0)
        self.assertEqual(type(operation.outputs["result"]), float)

    def testAndBool(self):
        operation = Operation(
            {
                "operand_type": MathOperandType("bool"),
                "operator": MathBinaryOperator("&"),
            }
        )

        operation.inputs["a"] = True
        operation.inputs["b"] = False
        operation.setup()

        operation.tick()
        self.assertEqual(operation.state, NodeMsg.SUCCEEDED)
        self.assertEqual(operation.outputs["result"], False)
        self.assertEqual(type(operation.outputs["result"]), bool)

        operation.inputs["a"] = True
        operation.inputs["b"] = True

        operation.tick()
        self.assertEqual(operation.state, NodeMsg.SUCCEEDED)
        self.assertEqual(operation.outputs["result"], True)
        self.assertEqual(type(operation.outputs["result"]), bool)

    def testWrongOperator(self):
        self.assertRaises(
            BehaviorTreeException,
            Operation,
            {
                "operand_type": MathOperandType("bool"),
                "operator": MathBinaryOperator("does_not_exist"),
            },
        )


class TestUnaryOperation(unittest.TestCase):
    def testNeg(self):
        operation = UnaryOperation(
            {
                "operand_type": MathUnaryOperandType("int"),
                "operator": MathUnaryOperator("neg"),
            }
        )

        operation.inputs["in"] = int(42)
        operation.setup()

        operation.tick()
        self.assertEqual(operation.state, NodeMsg.SUCCEEDED)
        self.assertEqual(operation.outputs["result"], -42)
        self.assertEqual(type(operation.outputs["result"]), int)

        operation.reset()
        self.assertEqual(operation.state, NodeMsg.IDLE)

        operation.untick()
        self.assertEqual(operation.state, NodeMsg.IDLE)

        operation.shutdown()
        self.assertEqual(operation.state, NodeMsg.SHUTDOWN)

    def testNegFloat(self):
        operation = UnaryOperation(
            {
                "operand_type": MathUnaryOperandType("float"),
                "operator": MathUnaryOperator("neg"),
            }
        )

        operation.inputs["in"] = float(42)
        operation.setup()

        operation.tick()
        self.assertEqual(operation.state, NodeMsg.SUCCEEDED)
        self.assertEqual(operation.outputs["result"], float(-42))
        self.assertEqual(type(operation.outputs["result"]), float)

    def testWrongOperator(self):
        self.assertRaises(
            BehaviorTreeException,
            UnaryOperation,
            {
                "operand_type": MathUnaryOperandType("int"),
                "operator": MathUnaryOperator("does_not_exist"),
            },
        )
