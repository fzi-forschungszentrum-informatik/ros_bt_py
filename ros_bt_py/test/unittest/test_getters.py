import jsonpickle
import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData, NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError

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

        getter.inputs['list'] = [1,2,3,4]

        # Now index 3 points to 4
        self.assertEqual(NodeMsg.SUCCEEDED, getter.tick())
        self.assertEqual(getter.outputs['item'], 4)
