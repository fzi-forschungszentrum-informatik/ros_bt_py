import sys
import unittest

import jsonpickle

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData

from ros_bt_py.node import Node
from ros_bt_py.tree_manager import TreeManager


class TestTreeManager(unittest.TestCase):
    def setUp(self):
        # Empty this before each test, because it's global and will persist
        # otherwise.
        Node.node_classes = {}
        # Ensure that PassthroughNode gets loaded in each test (if needed), so
        # that the decorator code is executed and registers the node class in
        # Node.node_classes
        if 'ros_bt_py.nodes.passthrough_node' in sys.modules:
            del sys.modules['ros_bt_py.nodes.passthrough_node']

    def testLoadNode(self):
        self.assertEqual(len(Node.node_classes), 0)

        manager = TreeManager()
        msg = NodeMsg(
            is_subtree=False,
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])

        _ = manager.instantiate_node_from_msg(msg)
        _ = manager.instantiate_node_from_msg(msg)
        msg.name = 'Test Node'
        _ = manager.instantiate_node_from_msg(msg)

        self.assertIn('PassthroughNode', manager.nodes)
        self.assertIn('PassthroughNode_2', manager.nodes)
        self.assertIn('Test Node', manager.nodes)
