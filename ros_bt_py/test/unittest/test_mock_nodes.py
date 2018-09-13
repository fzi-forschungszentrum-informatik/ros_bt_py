import unittest

from ros_bt_py.exceptions import NodeConfigError
from ros_bt_py_msgs.msg import Node

from ros_bt_py.nodes.mock_nodes import MockLeaf


class TestMockLeaf(unittest.TestCase):
    def setUp(self):
        self.state_output_pairs = [(Node.RUNNING, 1),
                                   (Node.SUCCEEDED, 2),
                                   (Node.FAILED, 3)]
        self.leaf = MockLeaf(
            options={'output_type': int,
                     'state_values': [state for state, _ in self.state_output_pairs],
                     'output_values': [output for _, output in self.state_output_pairs]})
        self.leaf.setup()

    def testInvalidSetup(self):
        # Wrong type of outputs
        self.assertRaises(NodeConfigError,
                          MockLeaf(options={'output_type': str,
                                            'state_values': [Node.IDLE,
                                                             Node.SUCCEEDED,
                                                             Node.FAILED],
                                            'output_values': [1, 2, 3]}).setup)
        # Right type, but wrong number of outputs
        self.assertRaises(NodeConfigError,
                          MockLeaf(options={'output_type': int,
                                            'state_values': [Node.IDLE,
                                                             Node.SUCCEEDED,
                                                             Node.FAILED],
                                            'output_values': [1, 2]}).setup)

    def testTick(self):
        self.assertIsNone(self.leaf.outputs['out'])
        # Loop through pairs of states and outputs twice to ensure MockSelf.Leaf
        # properly wraps.
        for state, output in self.state_output_pairs * 2:
            self.leaf.tick()
            self.assertEqual(self.leaf.state, state)
            self.assertEqual(self.leaf.outputs['out'], output)

        self.assertEqual(self.leaf.outputs['tick_count'], len(self.state_output_pairs * 2))

    def testUntick(self):
        for state, output in self.state_output_pairs:
            self.leaf.tick()
            self.assertEqual(self.leaf.state, state)
            self.assertEqual(self.leaf.outputs['out'], output)
            self.leaf.untick()
            self.assertEqual(self.leaf.state, Node.PAUSED)
        self.assertEqual(self.leaf.outputs['tick_count'], len(self.state_output_pairs))
        self.assertEqual(self.leaf.outputs['untick_count'], len(self.state_output_pairs))

    def testReset(self):
        self.leaf.tick()
        self.assertEqual(self.leaf.state, self.state_output_pairs[0][0])
        self.assertEqual(self.leaf.outputs['out'], self.state_output_pairs[0][1])
        self.assertEqual(self.leaf.tick_count, 1)
        self.leaf.reset()
        self.assertEqual(self.leaf.tick_count, 1)
        self.assertEqual(self.leaf.reset_count, 1)
        self.assertEqual(self.leaf.state, Node.IDLE)

        # After resetting, we should get the first output and state again
        self.leaf.tick()
        self.assertEqual(self.leaf.state, self.state_output_pairs[0][0])
        self.assertEqual(self.leaf.outputs['out'], self.state_output_pairs[0][1])
