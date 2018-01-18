import random
import sys
import unittest

import jsonpickle

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData

from ros_bt_py.node import Node
from ros_bt_py.tree_manager import increment_name, load_node_module, TreeManager

class TestTreeManager(unittest.TestCase):
    def setUp(self):
        # Empty this before each test, because it's global and will persist
        # otherwise.
        Node.node_classes = {}
        if 'ros_bt_py.nodes.passthrough_node' in sys.modules:
            del sys.modules['ros_bt_py.nodes.passthrough_node']

    def testIncrementName(self):
        numbers = [random.randint(0, 20) for _ in xrange(20)]
        for number in numbers:
            name = 'foo'
            # Special case for 0 - if number is zero, add no suffix and expect
            # _2 in incremented name
            if number > 0:
                name += '_%d' % number
            expected = '_%d' % (number + 1 if number > 0 else 2)
            self.assertRegexpMatches(increment_name(name),
                                     expected)

        self.assertRegexpMatches(increment_name('i_like_underscores___'),
                                 '_2')

    def testLoadNode(self):
        self.assertEqual(len(Node.node_classes), 0)

        manager = TreeManager()
        msg = NodeMsg(
            is_subtree=False,
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            options=[NodeData(key='passthrough_type',
                              value_serialized=jsonpickle.encode(int))])
        instance = manager.instantiate_node_from_msg(msg)

        node_class = type(instance)
        self.assertTrue(node_class.__module__ in Node.node_classes)
        self.assertTrue(node_class.__name__ in Node.node_classes[node_class.__module__])
        self.assertEqual(node_class,
                         Node.node_classes[node_class.__module__][node_class.__name__])

        self.assertEqual(instance.options['passthrough_type'], int)

    def testLoadModule(self):
        self.assertEqual(load_node_module('i.do.not.exist'), None)

        module = load_node_module('ros_bt_py.nodes.passthrough_node')
        self.assertEqual(len(Node.node_classes), 1, msg=str(Node.node_classes))
        self.assertEqual(module.__name__, Node.node_classes.keys()[0])
