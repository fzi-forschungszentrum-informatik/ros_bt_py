import jsonpickle
import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData, NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.getters import GetListItem


class TestGetters(unittest.TestCase):
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

        # Tick should fail on stale data
        self.assertEqual(NodeMsg.FAILED, getter.tick())

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
                     'output_values': [[1,2,3,4]]})

        getter.add_child(list_provider)
        list_provider.outputs.subscribe('out', getter.inputs.get_callback('list'))

        getter.setup()

        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['item'], 4)
