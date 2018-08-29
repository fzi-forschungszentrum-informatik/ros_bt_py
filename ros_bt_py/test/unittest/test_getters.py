import jsonpickle
import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData, NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.getters import GetListItem, GetDictItem, GetAttr


class TestListGetter(unittest.TestCase):
    def testListGetter(self):
        getter = GetListItem({'list_type': int,
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

        getter = GetListItem({'list_type': int,
                              'index': 3,
                              'succeed_on_stale_data': True})
        getter.setup()
        getter.inputs['list'] = [1, 2, 3, 4]
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

        # This tick should succeed on stale data
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())

    def testListGetterAsDecorator(self):
        getter = GetListItem({'list_type': int,
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

    def testDictGetterAsDecorator(self):
        getter = GetDictItem({'value_type': int,
                              'key': 'frob',
                              'succeed_on_stale_data': True})
        dict_provider = MockLeaf(
            options={'output_type': dict,
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

    def testAttrGetterAsDecorator(self):
        getter = GetAttr({'attr_type': str,
                          'attr_name': 'node_name',
                          'succeed_on_stale_data': False})
        location_provider = MockLeaf(
            options={'output_type': NodeDataLocation,
                     'state_values': [NodeMsg.SUCCEEDED],
                     'output_values': [NodeDataLocation(node_name='foo')]
            })

        getter.add_child(location_provider)
        location_provider.outputs.subscribe('out', getter.inputs.get_callback('object'))

        getter.setup()

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['attr'], 'foo')

        # Succeed on fail is true, so this should work too
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['attr'], 'foo')
