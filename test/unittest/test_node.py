import unittest

from ros_bt_py.node import Node
from ros_bt_py.nodes.passthrough_node import PassthroughNode


class TestNode(unittest.TestCase):
    def testNewNodeIsUninitialized(self):
        node = Node()
        self.assertEqual(node.state, Node.States.UNINITIALIZED)

    def testNewNodeCannotTick(self):
        node = Node()
        self.assertRaises(Exception, node.tick)
        node.state = Node.States.IDLE
        self.assertRaises(NotImplementedError, node.tick)

    def testPassthroughNode(self):
        passthrough = PassthroughNode(passthrough_type=int)

        self.assertEqual(passthrough.state, Node.States.IDLE)
        self.assertEqual(passthrough.inputs['in'].get(), None)
        self.assertEqual(passthrough.outputs['out'].get(), None)
        self.assertRaises(ValueError, passthrough.tick)

        passthrough.inputs['in'].set(42)
        passthrough.tick()
        self.assertEqual(passthrough.outputs['out'].get(), 42)
