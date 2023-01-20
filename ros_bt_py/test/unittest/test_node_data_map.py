#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
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
import sys

from ros_bt_py.node_data import NodeData, NodeDataMap


class TestNodeDataMap(unittest.TestCase):
    def setUp(self):
        self.map = NodeDataMap()

    def testAddValue(self):
        # key is string, but value is not NodeData
        self.assertRaises(TypeError, self.map.add, key="test", value="foo")
        # value is NodeData, but key is not a string
        self.assertRaises(TypeError, self.map.add, key=1, value=NodeData(data_type=int))

        # These should both work
        self.map.add(key="test", value=NodeData(data_type=int))
        self.map.add(key="unicodetest", value=NodeData(data_type=int))

        # Overwriting data should be impossible
        self.assertRaises(
            KeyError, self.map.add, key="test", value=NodeData(data_type=str)
        )
        # Unicode and regular versions of a key should be equivalent - thus,
        # this too should fail.
        self.assertRaises(
            KeyError, self.map.add, key="test", value=NodeData(data_type=str)
        )

    def testDictness(self):
        """Test whether NodeDataMap behaves like a dict"""
        self.map.add(key="one", value=NodeData(data_type=int, initial_value=1))
        self.assertEqual(len(self.map), 1)
        self.assertTrue("one" in self.map)
        self.assertEqual(self.map["one"], 1)

        # It should not be possible to delete values
        def try_delete():
            del self.map["one"]

        self.assertRaises(AttributeError, try_delete)
        self.assertEqual(len(self.map), 1)

    def testUpdated(self):
        self.assertRaises(KeyError, self.map.is_updated, key="integer")

        self.map.add(key="integer", value=NodeData(data_type=int))

        self.assertEqual(self.map["integer"], None)
        self.assertFalse(self.map.is_updated("integer"))

        self.map["integer"] = 42

        self.assertEqual(self.map["integer"], 42)
        self.assertTrue(self.map.is_updated("integer"))

        self.map.reset_updated()

        self.assertFalse(self.map.is_updated("integer"))

        self.map.set_updated("integer")

        self.assertTrue(self.map.is_updated("integer"))

        self.assertRaises(KeyError, self.map.set_updated, "does_not_exist")

        self.assertEqual(self.map.get_serialized("integer"), "42")
        self.assertRaises(KeyError, self.map.get_serialized, "does_not_exist")

        self.assertEqual(
            self.map.get_serialized_type("integer"), '{"py/type": "__builtin__.int"}'
        )
        self.assertEqual(self.map.get_type("integer"), int)
        self.assertRaises(KeyError, self.map.get_serialized_type, "does_not_exist")

        self.assertEqual(self.map.get_type("integer"), int)
        self.assertRaises(KeyError, self.map.get_type, "does_not_exist")
        self.assertRaises(KeyError, self.map.__getitem__, "does_not_exist")
        self.assertRaises(KeyError, self.map.__setitem__, "does_not_exist", 23)

    def testGetCallback(self):
        self.map.add(key="hello", value=NodeData(data_type=str))

        self.assertRaises(KeyError, self.map.get_callback, key="foo")

        callback = self.map.get_callback(key="hello")

        hello_string = "Hello!"
        callback(hello_string)

        self.assertEqual(self.map["hello"], hello_string)
        self.assertTrue(self.map.is_updated("hello"))

    def testSubscription(self):
        self.map.add(key="hello", value=NodeData(data_type=str))

        self.map.unsubscribe("hello")
        self.assertEqual(len(self.map.callbacks["hello"]), 0)

        self.called = False

        def callback(_):
            self.called = True

        self.map.subscribe("hello", callback)

        self.assertIn("hello", self.map.callbacks)
        self.assertEqual(len(self.map.callbacks["hello"]), 1)

        self.map.subscribe("hello", callback)
        self.map.subscribe("hello", callback)
        self.map.subscribe("hello", callback)

        # Subscribing multiple times with the same callback should have no
        # further effect.
        self.assertEqual(len(self.map.callbacks["hello"]), 1)

        self.map.handle_subscriptions()

        # 'hello' hasn't been changed yet, so the callback is not called
        self.assertFalse(self.called)

        self.map["hello"] = "Hello, world!"
        self.map.handle_subscriptions()

        self.assertTrue(self.called)

        self.called = False

        self.map.unsubscribe("hello", callback)
        self.assertEqual(len(self.map.callbacks["hello"]), 0)
        self.map["hello"] = "Hello once more!"
        self.map.handle_subscriptions()
        self.assertFalse(self.called)

        # Unsubscribe with no callback -> unsubscribe all
        self.map.subscribe("hello", callback)
        self.assertEqual(len(self.map.callbacks["hello"]), 1)
        self.map.unsubscribe("hello")
        self.assertEqual(len(self.map.callbacks["hello"]), 0)
        self.map.unsubscribe("hello", "unknown_callback")
        self.assertEqual(len(self.map.callbacks["hello"]), 0)

        # (un)subscribing non existing key raises KeyError
        self.assertRaises(KeyError, self.map.subscribe, "does_not_exist", None)
        self.assertRaises(KeyError, self.map.unsubscribe, "does_not_exist")

    def testNotEqual(self):
        other_map = NodeDataMap()
        other_map.add(key="some_key", value=NodeData(data_type=int))

        self.assertNotEqual(self.map, other_map)
