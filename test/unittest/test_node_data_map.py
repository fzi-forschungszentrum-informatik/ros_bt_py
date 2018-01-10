import unittest

from ros_bt_py.node_data import NodeData, NodeDataMap


class TestNodeDataMap(unittest.TestCase):
    def setUp(self):
        self.map = NodeDataMap()
    def testAddValue(self):
        # key is string, but value is not NodeData
        self.assertRaises(TypeError, self.map.add, key='test', value='foo')
        # value is NodeData, but key is not a string
        self.assertRaises(TypeError, self.map.add, key=1, value=NodeData(data_type=int))

        # These should both work
        self.map.add(key='test', value=NodeData(data_type=int))
        self.map.add(key=u'unicodetest', value=NodeData(data_type=int))

        # Overwriting data should be impossible
        self.assertRaises(KeyError, self.map.add, key='test', value=NodeData(data_type=str))
        # Unicode and regular versions of a key should be equivalent - thus,
        # this too should fail.
        self.assertRaises(KeyError, self.map.add, key=u'test', value=NodeData(data_type=str))

    def testDictness(self):
        """Test whether NodeDataMap behaves like a dict"""
        self.map.add(key='one', value=NodeData(data_type=int,
                                               initial_value=1))
        self.assertEqual(len(self.map), 1)
        self.assertTrue('one' in self.map)
        self.assertEqual(self.map['one'], 1)

        # It should not be possible to delete values
        def try_delete():
            del self.map['one']
        self.assertRaises(AttributeError, try_delete)
        self.assertEqual(len(self.map), 1)

    def testUpdated(self):
        self.assertRaises(KeyError, self.map.is_updated, key='integer')

        self.map.add(key='integer', value=NodeData(data_type=int))

        self.assertEqual(self.map['integer'], None)
        self.assertFalse(self.map.is_updated('integer'))

        self.map['integer'] = 42

        self.assertEqual(self.map['integer'], 42)
        self.assertTrue(self.map.is_updated('integer'))

        self.map.reset_updated()

        self.assertFalse(self.map.is_updated('integer'))

    def testGetCallback(self):
        self.map.add(key='hello', value=NodeData(data_type=str))

        self.assertRaises(KeyError, self.map.get_callback, key='foo')

        callback = self.map.get_callback(key='hello')

        hello_string = 'Hello!'
        callback(hello_string)

        self.assertEqual(self.map['hello'], hello_string)
        self.assertTrue(self.map.is_updated('hello'))
