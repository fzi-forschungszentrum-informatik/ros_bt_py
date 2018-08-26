import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.nodes.test import Test


class TestTest(unittest.TestCase):
    def setUp(self):
        self.test_node = Test({'test_type' : int})
        self.test_node.setup()
        self.test_node.inputs['expected'] = 42
        self.test_node.inputs['in'] = 42

    def testResult(self):
        # Both inputs are set, we should have an output
        self.test_node.tick()
        self.assertEqual(self.test_node.state, NodeMsg.SUCCEEDED)
        self.assertTrue(self.test_node.outputs.is_updated('result'))
        self.assertTrue(self.test_node.outputs['result'])

        # Another tick, but none of the input values changed, so
        # 'result' should not be updated
        self.test_node.tick()
        self.assertEqual(self.test_node.state, NodeMsg.SUCCEEDED)
        self.assertFalse(self.test_node.outputs.is_updated('result'))
        self.assertTrue(self.test_node.outputs['result'])

        self.test_node.inputs['in'] = 41

        # 'in' changed, so we get a new 'result'
        self.test_node.tick()
        self.assertEqual(self.test_node.state, NodeMsg.SUCCEEDED)
        self.assertTrue(self.test_node.outputs.is_updated('result'))
        self.assertFalse(self.test_node.outputs['result'])

        self.test_node.inputs['in'] = 40

        # 'in' changed, so we get a new 'result' - even though the
        # *value* is the same, 'result' should still be marked as
        # updated
        self.test_node.tick()
        self.assertEqual(self.test_node.state, NodeMsg.SUCCEEDED)
        self.assertTrue(self.test_node.outputs.is_updated('result'))
        self.assertFalse(self.test_node.outputs['result'])

    def testReset(self):
        # Start as before, 42 == 42
        self.test_node.tick()
        self.assertEqual(self.test_node.state, NodeMsg.SUCCEEDED)
        self.assertTrue(self.test_node.outputs.is_updated('result'))
        self.assertTrue(self.test_node.outputs['result'])

        # Resetting the node updates the result value to False
        self.test_node.reset()
        self.assertEqual(self.test_node.state, NodeMsg.IDLE)
        self.assertTrue(self.test_node.outputs.is_updated('result'))
        self.assertFalse(self.test_node.outputs['result'])

        # Neither of the inputs have updated, so the result doesn't either
        self.test_node.tick()
        self.assertEqual(self.test_node.state, NodeMsg.SUCCEEDED)
        self.assertFalse(self.test_node.outputs.is_updated('result'))
        self.assertFalse(self.test_node.outputs['result'])

        # If we update an input, the result changes to True
        self.test_node.inputs['in'] = 42
        self.test_node.tick()
        self.assertEqual(self.test_node.state, NodeMsg.SUCCEEDED)
        self.assertTrue(self.test_node.outputs.is_updated('result'))
        self.assertTrue(self.test_node.outputs['result'])
