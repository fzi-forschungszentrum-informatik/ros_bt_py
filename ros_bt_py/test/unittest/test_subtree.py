import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeDataWiring, NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.nodes.subtree import Subtree


class TestSubtree(unittest.TestCase):
    def setUp(self):
        self.subtree_options = {
            'subtree_path': 'package://ros_bt_py/etc/trees/test.yaml',
            'use_io_nodes': False
        }

    def testSubtreeLoad(self):
        subtree = Subtree(options=self.subtree_options)

        self.assertTrue(subtree.outputs['load_success'])
        subtree.setup()
        subtree.tick()

        self.assertEqual(subtree.state, NodeMsg.SUCCEEDED)
        self.assertIn('succeeder.out', subtree.outputs)
        self.assertEqual(subtree.outputs['succeeder.out'], 'Yay!')

        subtree.untick()
        self.assertEqual(subtree.state, NodeMsg.IDLE)

    def testSubtreeUntick(self):
        subtree = Subtree(options=self.subtree_options)

        self.assertTrue(subtree.outputs['load_success'])
        subtree.setup()
        subtree.tick()
        subtree.shutdown()
        self.assertEqual(subtree.state, NodeMsg.SHUTDOWN)
        subtree.setup()
        subtree.untick()

        self.assertTrue(subtree.state == NodeMsg.IDLE or subtree.state == NodeMsg.PAUSED,
                        'subtree node is neither IDLE nor PAUSED after unticking!')

    def testIOSubtree(self):
        self.subtree_options['subtree_path'] = 'package://ros_bt_py/etc/trees/io_test.yaml'
        subtree = Subtree(options=self.subtree_options)

        # Should have one input for the subtree's one public input
        self.assertEqual(len(subtree.inputs), 1)
        # And 3 outputs (load_success, load_error_msg, plus one for the public output)
        self.assertEqual(len(subtree.outputs), 3)

        self.assertTrue(subtree.outputs['load_success'], subtree.outputs['load_error_msg'])
        subtree.setup()

        self.assertRaises(ValueError, subtree.tick)

        subtree.inputs['passthrough.in'] = 'Hewwo'

        subtree.tick()
        self.assertEqual(subtree.state, NodeMsg.SUCCEEDED)
        self.assertEqual(subtree.outputs['passthrough.out'], 'Hewwo')

        subtree.inputs['passthrough.in'] = 'owo'

        subtree.tick()
        self.assertEqual(subtree.state, NodeMsg.SUCCEEDED)
        self.assertEqual(subtree.outputs['passthrough.out'], 'owo')
