import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeDataWiring
from ros_bt_py_msgs.msg import NodeDataLocation

from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.nodes.test import Test

from ros_bt_py.exceptions import BehaviorTreeException

class testWiring(unittest.TestCase):
    def setUp(self):
        self.root = Sequence()

        self.first = MockLeaf({
            'state_values': [NodeMsg.SUCCEEDED],
            'output_type': str,
            'output_values': ['Yay']
        })
        self.root.add_child(self.first)

        self.second = Test({
            'test_type': str
        })
        self.root.add_child(self.second)

        self.root.setup()

    def testWiringFromMissingNode(self):
        with self.assertRaises(BehaviorTreeException):
            self.second.wire_data(NodeDataWiring(
                source=NodeDataLocation(
                    node_name='fake_node',
                    data_kind=NodeDataLocation.OUTPUT_DATA,
                    data_key='out'
                ),
                target=NodeDataLocation(
                    node_name=self.second.name,
                    data_kind=NodeDataLocation.INPUT_DATA,
                    data_key='in'
                )))

    def testWiringFromMissingKey(self):
        with self.assertRaises(KeyError):
            # Target is valid, but source key should be 'out', not
            # 'output'
            self.second.wire_data(NodeDataWiring(
                source=NodeDataLocation(
                    node_name=self.first.name,
                    data_kind=NodeDataLocation.OUTPUT_DATA,
                    data_key='output'
                ),
                target=NodeDataLocation(
                    node_name=self.second.name,
                    data_kind=NodeDataLocation.INPUT_DATA,
                    data_key='in'
                )))

    def testWiringToMissingKey(self):
        with self.assertRaises(KeyError):
            # Source is valid, but target key should be 'in', not
            # 'input'
            self.second.wire_data(NodeDataWiring(
                source=NodeDataLocation(
                    node_name=self.first.name,
                    data_kind=NodeDataLocation.OUTPUT_DATA,
                    data_key='out'
                ),
                target=NodeDataLocation(
                    node_name=self.second.name,
                    data_kind=NodeDataLocation.INPUT_DATA,
                    data_key='input'
                )))

    def testWiringToOtherNode(self):
        with self.assertRaises(BehaviorTreeException):
            # Source is valid, but target is not the node we call
            # wire_data() on
            self.second.wire_data(NodeDataWiring(
                source=NodeDataLocation(
                    node_name=self.first.name,
                    data_kind=NodeDataLocation.OUTPUT_DATA,
                    data_key='out'
                ),
                target=NodeDataLocation(
                    node_name='other',
                    data_kind=NodeDataLocation.INPUT_DATA,
                    data_key='in'
                )))

    def testCorrectWiring(self):
        self.second.wire_data(NodeDataWiring(
            source=NodeDataLocation(
                node_name=self.first.name,
                data_kind=NodeDataLocation.OUTPUT_DATA,
                data_key='out'
            ),
            target=NodeDataLocation(
                node_name=self.second.name,
                data_kind=NodeDataLocation.INPUT_DATA,
                data_key='in'
            )))

        self.assertEqual(len(self.first.subscribers), 1)
        self.assertEqual(len(self.first.subscriptions), 0)
        self.assertEqual(len(self.second.subscribers), 0)
        self.assertEqual(len(self.second.subscriptions), 1)
