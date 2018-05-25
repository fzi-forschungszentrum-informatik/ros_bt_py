import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.node import Node, load_node_module, increment_name
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.nodes.mock_nodes import MockLeaf


class TestLoadModule(unittest.TestCase):
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

        self.root.add_child(self.outer_leaf_1)\
                 .add_child(Sequence(name='inner_seq')
                            .add_child(self.inner_leaf_1)
                            .add_child(self.inner_leaf_2))\
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
        self.assertEqual(len(subtree.nodes), 3)
