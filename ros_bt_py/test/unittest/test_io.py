import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg, Tree

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.nodes.io import IOInputOption, IOInput, IOOutputOption, IOOutput


class TestIOInputOption(unittest.TestCase):
    def setUp(self):
        self.default_tree = Tree(name='default_tree')
        self.input_tree = Tree(name='input_tree')

    def testWithoutInput(self):
        io = IOInputOption({'io_type': int,
                            'default': 42})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithIntInput(self):
        io = IOInputOption({'io_type': int,
                            'default': 42})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['in'] = 23

        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithMessageInput(self):
        io = IOInputOption({'io_type': Tree,
                            'default': self.default_tree})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.default_tree)

        io.inputs['in'] = self.input_tree
        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)


class TestIOInput(unittest.TestCase):
    def setUp(self):
        self.default_tree = Tree(name='default_tree')
        self.input_tree = Tree(name='input_tree')

    def testWithoutInputs(self):
        io = IOInput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)
        self.assertRaises(ValueError, io.tick)

    def testWithoutInputButWithDefault(self):
        io = IOInput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = 42

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithIntInput(self):
        io = IOInput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = 42
        io.inputs['in'] = 23

        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithMessageInput(self):
        io = IOInput({'io_type': Tree})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = self.default_tree

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.default_tree)

        io.inputs['in'] = self.input_tree
        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)


class TestIOOutputOption(unittest.TestCase):
    def setUp(self):
        self.default_tree = Tree(name='default_tree')
        self.input_tree = Tree(name='input_tree')

    def testWithoutInput(self):
        io = IOOutputOption({'io_type': int,
                             'default': 42})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithIntInput(self):
        io = IOOutputOption({'io_type': int,
                             'default': 42})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['in'] = 23

        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithMessageInput(self):
        io = IOOutputOption({'io_type': Tree,
                             'default': self.default_tree})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.default_tree)

        io.inputs['in'] = self.input_tree
        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)


class TestIOOutput(unittest.TestCase):
    def setUp(self):
        self.default_tree = Tree(name='default_tree')
        self.input_tree = Tree(name='input_tree')

    def testWithoutInputs(self):
        io = IOOutput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)
        self.assertRaises(ValueError, io.tick)

    def testWithoutInputButWithDefault(self):
        io = IOOutput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = 42

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 42)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithIntInput(self):
        io = IOOutput({'io_type': int})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = 42
        io.inputs['in'] = 23

        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], 23)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)

    def testWithMessageInput(self):
        io = IOOutput({'io_type': Tree})
        io.setup()
        self.assertEqual(io.state, NodeMsg.IDLE)

        io.inputs['default'] = self.default_tree

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.default_tree)

        io.inputs['in'] = self.input_tree
        self.assertTrue(io.inputs.is_updated('in'))

        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        # stale data
        self.assertFalse(io.inputs.is_updated('in'))
        self.assertEqual(io.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(io.outputs['out'], self.input_tree)

        self.assertEqual(io.untick(), NodeMsg.IDLE)
        self.assertEqual(io.reset(), NodeMsg.IDLE)
        self.assertEqual(io.shutdown(), NodeMsg.SHUTDOWN)
