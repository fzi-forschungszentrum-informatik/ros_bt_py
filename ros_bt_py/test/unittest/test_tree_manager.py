import unittest

import jsonpickle

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData

from ros_bt_py.node import Node
from ros_bt_py.tree_manager import TreeManager


class TestTreeManager(unittest.TestCase):
    def testLoadNode(self):
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
