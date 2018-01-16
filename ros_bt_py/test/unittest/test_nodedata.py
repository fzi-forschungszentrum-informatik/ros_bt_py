from ros_bt_py.node_data import NodeData, from_string
from std_msgs.msg import Time

import rospy
import unittest


class TestNodeData(unittest.TestCase):
    def testTypechecking(self):
        values = [
            (int, 1, [1.1, 'wrong', {'very': 'wrong'}]),
            (float, 1.5, [1, 'nope', {'extremely': 'nope'}]),
            (str, 'good', [1, 1.5, [1, 2, 3]]),
            (dict, {'a': 1, 'b': 2}, [['a', 'b', 'c'], 1.5, 'wrong']),
            (Time, Time(), [123, rospy.Duration(1.0), 'wrong'])]

        for value in values:
            data = NodeData(data_type=value[0],
                            initial_value=value[1])
            for bad_value in value[2]:
                self.assertRaises(Exception, data.set, bad_value)

    def testStringTypes(self):
        values = ['hi', u'hi']
        data = NodeData(data_type=str)
        for value in values:
            # should not throw
            data.set(value)

    def testGet(self):
        data = NodeData(data_type=int)
        data.set(1)
        self.assertEqual(data.get(), 1)

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
        string_data = from_string(int, '42')

        self.assertEqual(string_data.get(), 42)