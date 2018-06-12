import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeDataWiring, NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.node import Node, load_node_module, increment_name
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.passthrough_node import PassthroughNode


class TestSubtree(unittest.TestCase):
    def setUp(self):
        mock_options = {'output_type': int,
                        'state_values': [NodeMsg.SUCCEEDED],
                        'output_values': [1]}
        self.root = Sequence(name='outer_seq')
        self.outer_leaf_1 = MockLeaf(name='outer_leaf_1',
                                     options=mock_options)
        self.outer_leaf_2 = MockLeaf(name='outer_leaf_2',
                                     options=mock_options)

        self.inner_leaf_1 = MockLeaf(name='inner_leaf_1',
                                     options=mock_options)
        self.inner_leaf_2 = MockLeaf(name='inner_leaf_2',
                                     options=mock_options)

        self.passthrough = PassthroughNode(name='passthrough',
                                           options={'passthrough_type': int})

        self.root.add_child(self.outer_leaf_1)\
                 .add_child(Sequence(name='inner_seq')
                            .add_child(self.inner_leaf_1)
                            .add_child(self.inner_leaf_2)
                            .add_child(self.passthrough))\
                 .add_child(self.outer_leaf_2)

    def testSubtree(self):
        subtree, external_connections = self.root.find_node('inner_seq').get_subtree_msg()
        self.assertEqual(len(subtree.nodes), 4)
        self.assertEqual(len(external_connections), 0)

    def testSubtreeWiring(self):
        wiring = NodeDataWiring()
        wiring.source.node_name = 'outer_leaf_1'
        wiring.source.data_kind = NodeDataLocation.OUTPUT_DATA
        wiring.source.data_key = 'out'

        wiring.target.node_name = 'passthrough'
        wiring.target.data_kind = NodeDataLocation.INPUT_DATA
        wiring.target.data_key = 'in'

        self.passthrough.wire_data(wiring)

        subtree, external_connections = self.root.find_node('inner_seq').get_subtree_msg()
        self.assertEqual(len(subtree.nodes), 4)
        self.assertEqual(len(subtree.public_node_data), 1)
        self.assertEqual(len(external_connections), 1)

