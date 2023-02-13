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
from ros_bt_py.node_data import NodeData, from_string
from std_msgs.msg import Time

import rospy
import unittest


class TestNodeData(unittest.TestCase):
    def testTypechecking(self):
        values = [
            (int, 1, [1.1, "wrong", {"very": "wrong"}]),
            (float, 1.5, ["nope", {"extremely": "nope"}]),
            (str, "good", [1, 1.5, [1, 2, 3]]),
            (dict, {"a": 1, "b": 2}, [["a", "b", "c"], 1.5, "wrong"]),
            (Time, Time(), [123, rospy.Duration(1.0), "wrong"]),
        ]

        for value in values:
            data = NodeData(data_type=value[0], initial_value=value[1])
            for bad_value in value[2]:
                self.assertRaises(Exception, data.set, bad_value)

    def testStringTypes(self):
        values = ["hi", "hi"]
        data = NodeData(data_type=str)
        for value in values:
            # should not throw
            data.set(value)

    def testGet(self):
        data = NodeData(data_type=int)
        data.set(1)
        self.assertEqual(data.get(), 1)

    def testRepr(self):
        data = NodeData(data_type=int)
        data.set(1)
        self.assertEqual(repr(data), "1 (int) [#]")

    def testUpdated(self):
        data = NodeData(data_type=int, initial_value=0)

        # A fresh NodeData object with initial_data is updated
        self.assertTrue(data.updated)
        data.set(1)
        self.assertTrue(data.updated)
        data.reset_updated()
        self.assertFalse(data.updated)
        # Should not be able to assign a float to int NodeData
        self.assertRaises(TypeError, data.set, 1.5)
        # Failed assignment should not set updated
        self.assertFalse(data.updated)
        # Old value should still be there
        self.assertEqual(data.get(), 1)

        data2 = NodeData(data_type=str)
        self.assertFalse(data2.updated)

    def testWriteToStatic(self):
        data = NodeData(data_type=str, initial_value="Hello", static=True)

        self.assertRaises(Exception, data.set, "World")

    def testEmptyStatic(self):
        # Static with no initial_value should only let us write to it once
        data = NodeData(data_type=float, static=True)
        self.assertFalse(data.updated)
        data.set(4.2)
        self.assertTrue(data.updated)
        self.assertRaises(Exception, data.set, 1.5)

    def testFromString(self):
        string_data = from_string(int, "42")

        self.assertEqual(string_data.get(), 42)

    def testAppendToList(self):
        # This test exists mostly to document this behavior.
        # Beware, this can break staticness!
        data = NodeData(data_type=list, initial_value=[], static=False)
        data.get().append(1)

        self.assertEqual([1], data.get())

    def testEqualNotEqual(self):
        first = NodeData(data_type=int, initial_value=42)
        second = NodeData(data_type=int, initial_value=23)

        self.assertEqual(first, first)
        self.assertNotEqual(first, second)

    def testTakes(self):
        static_data = NodeData(data_type=int, initial_value=42, static=True)
        # static data can not take a new value:
        self.assertEqual(static_data.takes(23), False)

        # float can take an int
        data = NodeData(data_type=float, initial_value=42.0)
        self.assertEqual(data.takes(23), True)

    def testFailedJsonpickleDecode(self):
        data = NodeData(data_type=int)
        self.assertRaises(TypeError, data.set, {"py/type": "__builtin__.dict"})

    def testSerializedValueNone(self):
        data = NodeData(data_type=None)
        self.assertNotEqual(data.get_serialized(), None)
        self.assertEqual(data.get_serialized(), "null")

        # this should always return null, even after setting the member variable to None
        data._serialized_value = None
        self.assertEqual(data.get_serialized(), "null")
