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

        self.inner_passthrough = PassthroughNode(name='inner_passthrough',
                                                 options={'passthrough_type': int})
        self.outer_passthrough = PassthroughNode(name='outer_passthrough',
                                                 options={'passthrough_type': int})

        self.root.add_child(self.outer_leaf_1)\
                 .add_child(Sequence(name='inner_seq')
                            .add_child(self.inner_leaf_1)
                            .add_child(self.inner_leaf_2)
                            .add_child(self.inner_passthrough))\
                 .add_child(self.outer_leaf_2)\
                 .add_child(self.outer_passthrough)

    def testSubtree(self):
        subtree, incoming_connections, outgoing_connections = \
            self.root.find_node('inner_seq').get_subtree_msg()
        self.assertEqual(len(subtree.nodes), 4)
        self.assertEqual(len(incoming_connections), 0)
        self.assertEqual(len(outgoing_connections), 0)

    def testSubtreeWiring(self):
        # This wiring is between two nodes in the subtree, so it shouldn't be
        # exposed externally
        internal_wiring = NodeDataWiring()
        internal_wiring.source.node_name = 'inner_leaf_1'
        internal_wiring.source.data_kind = NodeDataLocation.OUTPUT_DATA
        internal_wiring.source.data_key = 'out'

        internal_wiring.target.node_name = 'inner_passthrough'
        internal_wiring.target.data_kind = NodeDataLocation.INPUT_DATA
        internal_wiring.target.data_key = 'in'

        self.inner_passthrough.wire_data(internal_wiring)

        # This wiring goes from outside the subtree to inside, so it must
        # appear in external_collections / public_node_data
        incoming_wiring = NodeDataWiring()
        incoming_wiring.source.node_name = 'outer_leaf_1'
        incoming_wiring.source.data_kind = NodeDataLocation.OUTPUT_DATA
        incoming_wiring.source.data_key = 'out'

        incoming_wiring.target.node_name = 'inner_passthrough'
        incoming_wiring.target.data_kind = NodeDataLocation.INPUT_DATA
        incoming_wiring.target.data_key = 'in'

        self.inner_passthrough.wire_data(incoming_wiring)

        # This wiring goes from inside the subtree to outside, so it must
        # appear in external_collections / public_node_data
        outgoing_wiring = NodeDataWiring()
        outgoing_wiring.source.node_name = 'inner_leaf_1'
        outgoing_wiring.source.data_kind = NodeDataLocation.OUTPUT_DATA
        outgoing_wiring.source.data_key = 'out'

        outgoing_wiring.target.node_name = 'outer_passthrough'
        outgoing_wiring.target.data_kind = NodeDataLocation.INPUT_DATA
        outgoing_wiring.target.data_key = 'in'

        self.outer_passthrough.wire_data(outgoing_wiring)

        subtree, incoming_connections, outgoing_connections = \
            self.root.find_node('inner_seq').get_subtree_msg()
        self.assertEqual(len(subtree.nodes), 4)
        self.assertEqual(len(subtree.public_node_data), 2)
        self.assertEqual(len([d for d in subtree.public_node_data
                              if d.data_kind == NodeDataLocation.INPUT_DATA]), 1)
        self.assertEqual(len([d for d in subtree.public_node_data
                              if d.data_kind == NodeDataLocation.OUTPUT_DATA]), 1)
        self.assertEqual(len(incoming_connections), 1)
        self.assertEqual(len(outgoing_connections), 1)

    def testUnconnectedInputsArePublic(self):
        smol_tree = Sequence(name='Sequence')\
            .add_child(self.inner_passthrough)\
            .add_child(self.outer_passthrough)

        passthrough_wiring = NodeDataWiring()
        passthrough_wiring.source.node_name = 'inner_passthrough'
        passthrough_wiring.source.data_kind = NodeDataLocation.OUTPUT_DATA
        passthrough_wiring.source.data_key = 'out'
        passthrough_wiring.target.node_name = 'outer_passthrough'
        passthrough_wiring.target.data_kind = NodeDataLocation.INPUT_DATA
        passthrough_wiring.target.data_key = 'in'

        self.outer_passthrough.wire_data(passthrough_wiring)
        subtree, incoming_conns, outgoing_conns = smol_tree.get_subtree_msg()

        self.assertEqual(len(subtree.nodes), 3)
        self.assertEqual(len(incoming_conns), 0)
        self.assertEqual(len(outgoing_conns), 0)
        self.assertEqual(len(subtree.public_node_data), 1)
