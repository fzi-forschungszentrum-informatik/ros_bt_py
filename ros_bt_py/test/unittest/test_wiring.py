import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeDataWiring
from ros_bt_py_msgs.msg import NodeDataLocation

from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.nodes.compare import Compare

from ros_bt_py.exceptions import BehaviorTreeException


class TestWiring(unittest.TestCase):
    def setUp(self):
        self.root = Sequence()

        self.first = MockLeaf({
            'state_values': [NodeMsg.SUCCEEDED],
            'output_type': str,
            'output_values': ['Yay']
        })
        self.root.add_child(self.first)

        self.second = Compare({
            'compare_type': str
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
                    data_key='a'
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
                    data_key='a'
                )))

    def testWiringToMissingKey(self):
        with self.assertRaises(KeyError):
            # Source is valid, but target key should be 'a', not
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
                    data_key='a'
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
                data_key='a'
            )))

        self.assertEqual(len(self.first.subscribers), 1)
        self.assertEqual(len(self.first.subscriptions), 0)
        self.assertEqual(len(self.second.subscribers), 0)
        self.assertEqual(len(self.second.subscriptions), 1)

    def testMultipleWirings(self):
        self.second.inputs['a'] = 'asdf'
        self.second.wire_data(NodeDataWiring(
            source=NodeDataLocation(
                node_name=self.first.name,
                data_kind=NodeDataLocation.OUTPUT_DATA,
                data_key='out'
            ),
            target=NodeDataLocation(
                node_name=self.second.name,
                data_kind=NodeDataLocation.INPUT_DATA,
                data_key='a'
            )))

        self.second.wire_data(NodeDataWiring(
            source=NodeDataLocation(
                node_name=self.first.name,
                data_kind=NodeDataLocation.OUTPUT_DATA,
                data_key='out'
            ),
            target=NodeDataLocation(
                node_name=self.second.name,
                data_kind=NodeDataLocation.INPUT_DATA,
                data_key='b'
            )))

        self.assertEqual(len(self.first.subscribers), 2)
        self.assertEqual(len(self.first.subscriptions), 0)
        self.assertEqual(len(self.second.subscribers), 0)
        self.assertEqual(len(self.second.subscriptions), 2)

        self.root.tick()
