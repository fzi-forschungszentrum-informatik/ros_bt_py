import unittest

from ros_bt_py_msgs.msg import Node

from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.sequence import Sequence


class TestSequence(unittest.TestCase):
    def setUp(self):
        self.succeeder = MockLeaf(name='succeeder',
                                  options={'output_type': int,
                                           'state_values': [Node.SUCCEEDED],
                                           'output_values': [1]})
        self.failer = MockLeaf(name='failer',
                               options={'output_type': int,
                                        'state_values': [Node.FAILED],
                                        'output_values': [1]})
        self.runner = MockLeaf(name='runner',
                               options={'output_type': int,
                                        'state_values': [Node.RUNNING],
                                        'output_values': [1]})
        self.sequence = Sequence()

    def testEmptyTickFails(self):
        self.sequence.setup()
        self.sequence.tick()
        self.assertEqual(self.sequence.state, Node.FAILED)

    def testTickSuccess(self):
        self.sequence.add_child(self.succeeder)
        self.sequence.setup()
        self.sequence.tick()
        self.assertEqual(self.succeeder.outputs['tick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.SUCCEEDED)
        self.assertEqual(self.sequence.state, Node.SUCCEEDED)

    def testTickRunning(self):
        self.sequence.add_child(self.runner)
        self.sequence.add_child(self.succeeder)
        self.sequence.setup()
        self.sequence.tick()
        self.assertEqual(self.runner.outputs['tick_count'], 1)
        self.assertEqual(self.runner.state, Node.RUNNING)
        self.assertEqual(self.succeeder.outputs['tick_count'], 0)
        self.assertEqual(self.succeeder.outputs['untick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.PAUSED)
        self.assertEqual(self.sequence.state, Node.RUNNING)

    def testTickFailing(self):
        self.sequence.add_child(self.failer)
        self.sequence.add_child(self.succeeder)
        self.sequence.setup()
        self.sequence.tick()
        self.assertEqual(self.failer.outputs['tick_count'], 1)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.succeeder.outputs['tick_count'], 0)
        self.assertEqual(self.succeeder.outputs['untick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.PAUSED)
        self.assertEqual(self.sequence.state, Node.FAILED)

    def testNested(self):
        inner_sequence = Sequence(name='inner')
        inner_sequence.add_child(self.succeeder)
        self.sequence.add_child(inner_sequence)
        self.sequence.setup()

        self.sequence.tick()
        self.assertEqual(self.succeeder.outputs['tick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.SUCCEEDED)
        self.assertEqual(inner_sequence.state, Node.SUCCEEDED)
        self.assertEqual(self.sequence.state, Node.SUCCEEDED)
