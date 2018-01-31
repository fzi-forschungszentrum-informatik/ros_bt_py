import unittest

from ros_bt_py_msgs.msg import Node

from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.fallback import Fallback


class TestFallback(unittest.TestCase):
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
        self.fallback = Fallback()

    def testEmptyTickFails(self):
        self.fallback.setup()
        self.fallback.tick()
        self.assertEqual(self.fallback.state, Node.FAILED)

    def testTickSuccess(self):
        self.fallback.add_child(self.succeeder)
        self.fallback.add_child(self.failer)
        self.fallback.setup()

        self.fallback.tick()

        # Fallback is done after the first success, so it should untick the
        # failer node.
        self.assertEqual(self.succeeder.outputs['tick_count'], 1)
        self.assertEqual(self.failer.outputs['untick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.SUCCEEDED)
        self.assertEqual(self.fallback.state, Node.SUCCEEDED)

    def testTickSuccessAfterFailure(self):
        self.fallback.add_child(self.failer)
        self.fallback.add_child(self.succeeder)
        self.fallback.setup()

        self.fallback.tick()

        # After the first child failed, the sequence should try the second one
        self.assertEqual(self.failer.outputs['tick_count'], 1)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.succeeder.outputs['tick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.SUCCEEDED)
        self.assertEqual(self.fallback.state, Node.SUCCEEDED)

    def testTickRunning(self):
        self.fallback.add_child(self.failer)
        self.fallback.add_child(self.runner)
        self.fallback.setup()

        self.fallback.tick()

        self.assertEqual(self.failer.outputs['tick_count'], 1)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.runner.outputs['tick_count'], 1)
        self.assertEqual(self.runner.state, Node.RUNNING)
        self.assertEqual(self.fallback.state, Node.RUNNING)

    def testTickFailing(self):
        self.fallback.add_child(self.failer)
        self.fallback.setup()

        self.fallback.tick()

        # There's nothing to fall back on after the only child failed, so the
        # Fallback fails.
        self.assertEqual(self.failer.outputs['tick_count'], 1)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.fallback.state, Node.FAILED)

    def testNested(self):
        inner_fallback = Fallback(name='inner')
        inner_fallback.add_child(self.succeeder)
        self.fallback.add_child(self.failer)
        self.fallback.add_child(inner_fallback)
        self.fallback.setup()

        self.fallback.tick()

        # The failer fails, and the inner Fallback succeeds, resulting in an
        # overall success.
        self.assertEqual(self.failer.outputs['tick_count'], 1)
        self.assertEqual(self.failer.state, Node.FAILED)
        self.assertEqual(self.succeeder.outputs['tick_count'], 1)
        self.assertEqual(self.succeeder.state, Node.SUCCEEDED)
        self.assertEqual(inner_fallback.state, Node.SUCCEEDED)
        self.assertEqual(self.fallback.state, Node.SUCCEEDED)
