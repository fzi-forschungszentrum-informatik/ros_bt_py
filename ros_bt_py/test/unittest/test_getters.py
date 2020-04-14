import jsonpickle
import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData, NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.getters import GetConstListItem, GetListItem, GetDictItem, \
    GetDictItemFromKey, GetAttr, GetMultipleDictItems


class TestListGetter(unittest.TestCase):
    def testConstListGetter(self):
        getter = GetConstListItem({'list_type': int,
                                   'index': 3,
                                   'succeed_on_stale_data': False})
        getter.setup()
        self.assertEqual(NodeMsg.IDLE, getter.state)

        getter.inputs['list'] = [0]
        self.assertTrue(getter.inputs.is_updated('list'))

        # Index 3 is out of bounds in a single element array
        self.assertEqual(NodeMsg.FAILED, getter.tick())

        getter.inputs['list'] = [1, 2, 3, 4]

        # Now index 3 points to 4
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['item'], 4)

        # Tick should return RUNNING on stale data
        self.assertEqual(NodeMsg.RUNNING, getter.tick())

        getter = GetConstListItem({'list_type': int,
                                   'index': 3,
                                   'succeed_on_stale_data': True})
        getter.setup()
        getter.inputs['list'] = [1, 2, 3, 4]
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

        # This tick should succeed on stale data
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

        self.assertEqual(NodeMsg.IDLE, getter.untick())
        self.assertEqual(NodeMsg.IDLE, getter.reset())
        self.assertEqual(NodeMsg.SHUTDOWN, getter.shutdown())

    def testConstListGetterAsDecorator(self):
        getter = GetConstListItem({'list_type': int,
                                   'index': 3,
                                   'succeed_on_stale_data': True})
        list_provider = MockLeaf(
            options={'output_type': list,
                     'state_values': [NodeMsg.SUCCEEDED],
                     'output_values': [[1, 2, 3, 4]]})

        getter.add_child(list_provider)
        list_provider.outputs.subscribe('out', getter.inputs.get_callback('list'))

        getter.setup()

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['item'], 4)

    def testGetListItem(self):
        getter = GetListItem({'list_type': int,
                              'succeed_on_stale_data': True})

        self.assertEqual(NodeMsg.UNINITIALIZED, getter.state)

        getter.inputs['list'] = [1, 2, 3, 4]
        getter.inputs['index'] = 0

        getter.setup()

        self.assertEqual(NodeMsg.IDLE, getter.state)

        getter.inputs['index'] = 0
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(NodeMsg.SUCCEEDED, getter.state)
        self.assertEqual(getter.outputs['item'], 1)

        getter.inputs['index'] = 1
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(NodeMsg.SUCCEEDED, getter.state)
        self.assertEqual(getter.outputs['item'], 2)

        getter.inputs['index'] = 2
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(NodeMsg.SUCCEEDED, getter.state)
        self.assertEqual(getter.outputs['item'], 3)

        getter.inputs['index'] = 3
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(NodeMsg.SUCCEEDED, getter.state)
        self.assertEqual(getter.outputs['item'], 4)

        getter.inputs['index'] = 4
        self.assertEqual(NodeMsg.FAILED, getter.tick())
        self.assertEqual(NodeMsg.FAILED, getter.state)

        self.assertEqual(NodeMsg.IDLE, getter.untick())
        self.assertEqual(NodeMsg.IDLE, getter.reset())
        self.assertEqual(getter.outputs['item'], None)

        self.assertEqual(NodeMsg.SHUTDOWN, getter.shutdown())

    def testGetListItemStaleData(self):
        getter = GetListItem({'list_type': int,
                              'succeed_on_stale_data': True})

        self.assertEqual(NodeMsg.UNINITIALIZED, getter.state)

        getter.inputs['list'] = [1, 2, 3, 4]
        getter.inputs['index'] = 0

        getter.setup()

        self.assertEqual(NodeMsg.IDLE, getter.state)

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(NodeMsg.SUCCEEDED, getter.state)
        self.assertEqual(getter.outputs['item'], 1)

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(NodeMsg.SUCCEEDED, getter.state)
        self.assertEqual(getter.outputs['item'], 1)

        getter = GetListItem({'list_type': int,
                              'succeed_on_stale_data': False})

        self.assertEqual(NodeMsg.UNINITIALIZED, getter.state)

        getter.inputs['list'] = [1, 2, 3, 4]
        getter.inputs['index'] = 0

        getter.setup()

        self.assertEqual(NodeMsg.IDLE, getter.state)

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(NodeMsg.SUCCEEDED, getter.state)
        self.assertEqual(getter.outputs['item'], 1)

        self.assertEqual(NodeMsg.RUNNING, getter.tick())
        self.assertEqual(NodeMsg.RUNNING, getter.state)
        self.assertEqual(getter.outputs['item'], 1)

    def testGetListItemAsDecorator(self):
        getter = GetListItem({'list_type': int,
                              'succeed_on_stale_data': True})
        list_provider = MockLeaf(
            options={'output_type': list,
                     'state_values': [NodeMsg.SUCCEEDED],
                     'output_values': [[1, 2, 3, 4]]})

        getter.add_child(list_provider)
        list_provider.outputs.subscribe('out', getter.inputs.get_callback('list'))

        getter.setup()

        getter.inputs['index'] = 3
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['item'], 4)


class TestDictGetter(unittest.TestCase):
    def testDictGetter(self):
        getter = GetDictItem({'value_type': int,
                              'key': 'frob',
                              'succeed_on_stale_data': False})
        getter.setup()
        self.assertEqual(NodeMsg.IDLE, getter.state)

        getter.inputs['dict'] = {'foo': 5}
        self.assertTrue(getter.inputs.is_updated('dict'))

        # The required key 'frob' is not in the dict
        self.assertEqual(NodeMsg.FAILED, getter.tick())

        getter.inputs['dict'] = {
            'foo': 5,
            'bar': 'hello',
            'frob': 42
        }

        # Now the key 'frob' has the value 42
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['value'], 42)

        # Tick should return RUNNING on stale data
        self.assertEqual(NodeMsg.RUNNING, getter.tick())

        getter = GetDictItem({'value_type': int,
                              'key': 'frob',
                              'succeed_on_stale_data': True})
        getter.setup()
        getter.inputs['dict'] = {
            'foo': 5,
            'bar': 'hello',
            'frob': 42
        }
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

        # This tick should succeed on stale data
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

        self.assertEqual(NodeMsg.IDLE, getter.untick())
        self.assertEqual(NodeMsg.IDLE, getter.reset())
        self.assertEqual(getter.outputs['value'], None)

        self.assertEqual(NodeMsg.SHUTDOWN, getter.shutdown())

    def testDictGetterAsDecorator(self):
        getter = GetDictItem({'value_type': int,
                              'key': 'frob',
                              'succeed_on_stale_data': True})
        dict_provider = MockLeaf(
            options={
                'output_type': dict,
                'state_values': [NodeMsg.SUCCEEDED],
                'output_values': [{
                    'foo': 5,
                    'bar': 'hello',
                    'frob': 42
                }]
            })

        getter.add_child(dict_provider)
        dict_provider.outputs.subscribe('out', getter.inputs.get_callback('dict'))

        getter.setup()

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['value'], 42)

        # Succeed on fail is true, so this should work too
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['value'], 42)


class TestMultipleDictGetter(unittest.TestCase):
    def testMultipleDictGetter(self):
        getter = GetMultipleDictItems({'keys': ['frob', 'toto'],
                                      'succeed_on_stale_data': False})
        getter.setup()
        self.assertEqual(NodeMsg.IDLE, getter.state)

        getter.inputs['dict'] = {'foo': 5}
        self.assertTrue(getter.inputs.is_updated('dict'))

        # The required key 'frob' is not in the dict
        self.assertEqual(NodeMsg.FAILED, getter.tick())

        getter.inputs['dict'] = {
            'foo': 5,
            'bar': 'hello',
            'frob': 42
        }

        # The required key 'toto' is not in the dict
        self.assertEqual(NodeMsg.FAILED, getter.tick())

        getter.inputs['dict'] = {
            'foo': 5,
            'bar': 'hello',
            'frob': 42,
            'toto': 55
        }

        # Now the keys 'frob' and 'toto' are in the dict
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(len(getter.outputs['values']), 2)
        self.assertEqual(getter.outputs['values'][0], 42)
        self.assertEqual(getter.outputs['values'][1], 55)

        # Tick should return RUNNING on stale data
        self.assertEqual(NodeMsg.RUNNING, getter.tick())

        getter = GetMultipleDictItems({'keys': ['frob', 'toto'],
                                       'succeed_on_stale_data': True})
        getter.setup()
        getter.inputs['dict'] = {
            'foo': 5,
            'bar': 'hello',
            'frob': 42,
            'toto': 55
        }

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

        # This tick should succeed on stale data
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

        self.assertEqual(NodeMsg.IDLE, getter.untick())
        self.assertEqual(NodeMsg.IDLE, getter.reset())
        self.assertEqual(getter.outputs['values'], [])

        self.assertEqual(NodeMsg.SHUTDOWN, getter.shutdown())

    def testDictGetterAsDecorator(self):
        getter = GetMultipleDictItems({'keys': ['frob', 'toto'],
                                      'succeed_on_stale_data': True})
        dict_provider = MockLeaf(
            options={
                'output_type': dict,
                'state_values': [NodeMsg.SUCCEEDED],
                'output_values': [{
                    'foo': 5,
                    'bar': 'hello',
                    'frob': 42,
                    'toto': 55
                }]
            })

        getter.add_child(dict_provider)
        dict_provider.outputs.subscribe('out', getter.inputs.get_callback('dict'))

        getter.setup()

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(len(getter.outputs['values']), 2)
        self.assertEqual(getter.outputs['values'][0], 42)
        self.assertEqual(getter.outputs['values'][1], 55)

        # Succeed on fail is true, so this should work too
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(len(getter.outputs['values']), 2)
        self.assertEqual(getter.outputs['values'][0], 42)
        self.assertEqual(getter.outputs['values'][1], 55)


class TestDictGetterFromKey(unittest.TestCase):
    def testDictGetter(self):
        getter = GetDictItemFromKey({'value_type': int,
                                     'dict': {'foo': 5, 'bar': 42},
                                     'succeed_on_stale_data': False})
        getter.setup()
        self.assertEqual(NodeMsg.IDLE, getter.state)

        getter.inputs['key'] = 'not_in_dict'
        self.assertTrue(getter.inputs.is_updated('key'))
        self.assertEqual(NodeMsg.FAILED, getter.tick())

        getter.inputs['key'] = 'foo'
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['value'], 5)

        # Tick should return RUNNING on stale data
        self.assertEqual(NodeMsg.RUNNING, getter.tick())

        getter = GetDictItemFromKey({'value_type': int,
                                     'dict': {'foo': 5, 'bar': 42},
                                     'succeed_on_stale_data': True})

        getter.setup()
        getter.inputs['key'] = 'bar'
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['value'], 42)

        # This tick should succeed on stale data
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['value'], 42)

        self.assertEqual(NodeMsg.IDLE, getter.untick())
        self.assertEqual(NodeMsg.IDLE, getter.reset())
        self.assertEqual(getter.outputs['value'], None)

        self.assertEqual(NodeMsg.SHUTDOWN, getter.shutdown())

    def testDictGetterAsDecorator(self):
        getter = GetDictItemFromKey({'value_type': int,
                                     'dict': {'foo': 5, 'bar': 42},
                                     'succeed_on_stale_data': True})
        key_provider = MockLeaf(
            options={
                'output_type': str,
                'state_values': [NodeMsg.SUCCEEDED],
                'output_values': ['foo']
            })

        getter.add_child(key_provider)
        key_provider.outputs.subscribe('out', getter.inputs.get_callback('key'))

        getter.setup()

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['value'], 5)

        # Succeed on fail is true, so this should work too
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['value'], 5)


class TestAttrGetter(unittest.TestCase):
    def testAttrGetter(self):
        getter = GetAttr({'attr_type': str,
                          'attr_name': 'node_name',
                          'succeed_on_stale_data': False})
        getter.setup()
        self.assertEqual(NodeMsg.IDLE, getter.state)

        getter.inputs['object'] = NodeData(key='baz', serialized_value='')
        self.assertTrue(getter.inputs.is_updated('object'))

        # NodeData doesn't have a 'node_name' attribute
        self.assertEqual(NodeMsg.FAILED, getter.tick())

        getter.inputs['object'] = NodeDataLocation(node_name='foo')

        # NodeDataLocation *does* have node_name, and it's foo
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['attr'], 'foo')

        # Tick should return RUNNING on stale data
        self.assertEqual(NodeMsg.RUNNING, getter.tick())

        getter = GetAttr({'attr_type': str,
                          'attr_name': 'node_name',
                          'succeed_on_stale_data': True})
        getter.setup()
        getter.inputs['object'] = NodeDataLocation(node_name='foo')
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

        # This tick should succeed on stale data
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

        self.assertEqual(NodeMsg.IDLE, getter.untick())
        self.assertEqual(NodeMsg.IDLE, getter.reset())
        self.assertEqual(getter.outputs['attr'], None)

        self.assertEqual(NodeMsg.SHUTDOWN, getter.shutdown())

    def testAttrGetterNestedObject(self):
        class Inner(object):
            def __init__(self, name='inner_object'):
                self.name = name

        class Outer(object):
            def __init__(self):
                self.inner = Inner()

        getter = GetAttr({'attr_type': str,
                          'attr_name': 'inner.name',
                          'succeed_on_stale_data': False})
        getter.setup()
        self.assertEqual(NodeMsg.IDLE, getter.state)

        getter.inputs['object'] = Outer()
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['attr'], 'inner_object')

    def testAttrGetterAsDecorator(self):
        getter = GetAttr({'attr_type': str,
                          'attr_name': 'node_name',
                          'succeed_on_stale_data': False})
        location_provider = MockLeaf(
            options={'output_type': NodeDataLocation,
                     'state_values': [NodeMsg.SUCCEEDED],
                     'output_values': [NodeDataLocation(node_name='foo')]})

        getter.add_child(location_provider)
        location_provider.outputs.subscribe('out', getter.inputs.get_callback('object'))

        getter.setup()

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['attr'], 'foo')

        # Succeed on fail is true, so this should work too
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['attr'], 'foo')
