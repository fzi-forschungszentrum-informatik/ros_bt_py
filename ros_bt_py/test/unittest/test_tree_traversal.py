import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeDataWiring, NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.node import Node, load_node_module, increment_name
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.passthrough_node import PassthroughNode


class TestTreeTraversal(unittest.TestCase):
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

    def testFindNode(self):
        self.assertEqual(self.outer_leaf_1.find_node('outer_leaf_1'),
                         self.outer_leaf_1,
                         msg='Failed to find self')
        self.assertEqual(self.inner_leaf_1.find_node('outer_leaf_1'),
                         self.outer_leaf_1,
                         msg='Failed to find outer_leaf_1 from inner_leaf_1')
        self.assertEqual(self.inner_leaf_1.find_node('outer_leaf_2'),
                         self.outer_leaf_2,
                         msg='Failed to find outer_leaf_2 from inner_leaf_1')

        self.assertEqual(self.outer_leaf_1.find_node('outer_leaf_2'),
                         self.outer_leaf_2,
                         msg='Failed to find outer_leaf_2 from outer_leaf_1')

        self.assertEqual(self.outer_leaf_1.find_node('inner_leaf_2'),
                         self.inner_leaf_2,
                         msg='Failed to find inner_leaf_2 from outer_leaf_1')

        self.assertEqual(self.outer_leaf_2.find_node('outer_leaf_1'),
                         self.outer_leaf_1,
                         msg='Failed to find outer_leaf_1 from outer_leaf_2')

        self.assertEqual(self.outer_leaf_2.find_node('inner_leaf_1'),
                         self.inner_leaf_1,
                         msg='Failed to find inner_leaf_1 from outer_leaf_2')

    def testSubtree(self):
        subtree = self.root.find_node('inner_seq').get_subtree_msg()
        self.assertEqual(len(subtree.nodes), 4)

    def testNodeToNodeSub(self):
        wiring = NodeDataWiring()
        wiring.source.node_name = 'inner_leaf_1'
        wiring.source.data_kind = NodeDataLocation.OUTPUT_DATA
        wiring.source.data_key = 'out'

        wiring.target.node_name = 'passthrough'
        wiring.target.data_kind = NodeDataLocation.INPUT_DATA
        wiring.target.data_key = 'in'
        self.passthrough.wire_data(wiring)

        self.root.setup()
        self.root.tick()

        self.assertEqual(self.passthrough.inputs['in'], self.inner_leaf_1.outputs['out'])
        self.assertEqual(len(self.passthrough.subscriptions), 1)
        self.assertEqual(len(self.passthrough.subscribers), 0)

        self.assertEqual(len(self.inner_leaf_1.subscriptions), 0)
        self.assertEqual(len(self.inner_leaf_1.subscribers), 1)

        # Test some combinations that should raise errors

        # wire_data(wiring) must be called on the *target* of the wiring
        self.assertRaises(KeyError, self.inner_leaf_1.wire_data, wiring)
        # wiring the same connection twice is an error
        self.assertRaises(BehaviorTreeException, self.passthrough.wire_data, wiring)
        # if either input or output key doesn't exist, wiring should fail
        wiring.source.data_key = 'foo'
        self.assertRaises(KeyError, self.passthrough.wire_data, wiring)
        wiring.source.data_key = 'out'
        wiring.target.data_key = 'bar'
        self.assertRaises(KeyError, self.passthrough.wire_data, wiring)
        wiring.target.data_key = 'in'

        # Same for the source node - if it doesn't exist, this should throw
        wiring.source.node_name = 'baz'
        self.assertRaises(BehaviorTreeException, self.passthrough.wire_data, wiring)
        wiring.source.node_name = 'inner_leaf_1'

        # Should *not* throw, even if called repeatedly
        self.passthrough.unwire_data(wiring)
        self.assertEqual(len(self.passthrough.subscriptions), 0)
        self.assertEqual(len(self.passthrough.subscribers), 0)

        self.assertEqual(len(self.inner_leaf_1.subscriptions), 0)
        self.assertEqual(len(self.inner_leaf_1.subscribers), 0)

        self.passthrough.unwire_data(wiring)

        # don't need to setup again, but reset instead
        self.root.reset()

        # since the input of the passthrough isn't connected to anything, this fails
        self.assertRaises(ValueError, self.root.tick)

        # unwire_data is more tolerant than wire_data - if a wiring didn't
        # exist before calling it, it does nothing and returns, since the
        # post-condition (wiring does not exist) is true
        wiring.source.data_key = 'foo'
        self.passthrough.unwire_data(wiring)
        wiring.source.data_key = 'out'
        wiring.target.data_key = 'bar'
        self.passthrough.unwire_data(wiring)
        wiring.target.data_key = 'in'

        # It still throws for a wrong source or target node name, though:
        wiring.source.node_name = 'baz'
        self.assertRaises(BehaviorTreeException, self.passthrough.unwire_data, wiring)
        wiring.source.node_name = 'inner_leaf_1'

        wiring.target.node_name = 'foobar'
        self.assertRaises(KeyError, self.passthrough.unwire_data, wiring)
