import unittest

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.node import Node
from ros_bt_py.nodes.passthrough_node import PassthroughNode


class TestNode(unittest.TestCase):
    def TestNodeInitFails(self):
        self.assertRaises(ValueError, Node)

    def testMissingOption(self):
        self.assertRaises(ValueError, PassthroughNode)

    def testPassthroughNode(self):
        passthrough = PassthroughNode({'passthrough_type': int}, debug_manager=DebugManager())

        self.assertEqual(passthrough.state, Node.States.IDLE)
        self.assertEqual(passthrough.inputs['in'], None)
        self.assertEqual(passthrough.outputs['out'], None)
        self.assertRaises(ValueError, passthrough.tick)

        passthrough.inputs['in'] = 42
        passthrough.tick()
        self.assertEqual(passthrough.state, Node.States.SUCCEEDED)
        self.assertTrue(passthrough.outputs.is_updated('out'))
        self.assertEqual(passthrough.outputs['out'], 42)

        # Ensure that changing the input does not affect the output
        passthrough.inputs['in'] = 0
        self.assertEqual(passthrough.outputs['out'], 42)

        # Calling reset() should bring the node back into the same state if was
        # in when it was freshly created:
        passthrough.reset()

        fresh_passthrough = PassthroughNode({'passthrough_type': int},
                                            debug_manager=DebugManager())

        self.assertEqual(passthrough.inputs['in'], fresh_passthrough.inputs['in'])
        self.assertEqual(passthrough.outputs['out'], fresh_passthrough.outputs['out'])
        self.assertEqual(passthrough.inputs.is_updated('in'),
                         fresh_passthrough.inputs.is_updated('in'))
        self.assertEqual(passthrough.outputs.is_updated('out'),
                         fresh_passthrough.outputs.is_updated('out'))

    def testPassthroughNodeUntick(self):
        passthrough = PassthroughNode({'passthrough_type': float},
                                      debug_manager=DebugManager())

        passthrough.inputs['in'] = 1.5
        self.assertEqual(passthrough.state, Node.States.IDLE)

        passthrough.tick()

        self.assertEqual(passthrough.state, Node.States.SUCCEEDED)
        self.assertTrue(passthrough.outputs.is_updated('out'))

        passthrough.untick()

        self.assertEqual(passthrough.state, Node.States.IDLE)
        self.assertFalse(passthrough.outputs.is_updated('out'))
