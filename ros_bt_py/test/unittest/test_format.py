import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.nodes.format import FormatOptionNode, FormatInputNode

from ros_bt_py.exceptions import BehaviorTreeException


class TestFormatOptionNode(unittest.TestCase):
    def testSimpleFormatString(self):
        format_option = FormatOptionNode({'format_string': 'foo {first}'})
        format_option.inputs['dict'] = {'first': 'bar', 'second': 'not_printed'}
        self.assertEqual(format_option.state, NodeMsg.UNINITIALIZED)

        format_option.setup()

        self.assertEqual(format_option.state, NodeMsg.IDLE)
        self.assertEqual(format_option.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(format_option.outputs['formatted_string'], 'foo bar')

        self.assertEqual(format_option.untick(), NodeMsg.IDLE)
        self.assertEqual(format_option.reset(), NodeMsg.IDLE)
        self.assertEqual(format_option.shutdown(), NodeMsg.SHUTDOWN)

    def testFormatStringWrongKey(self):
        format_option = FormatOptionNode({'format_string': 'foo {missing}'})
        format_option.inputs['dict'] = {'first': 'bar', 'second': 'not_printed'}
        self.assertEqual(format_option.state, NodeMsg.UNINITIALIZED)

        format_option.setup()

        self.assertEqual(format_option.state, NodeMsg.IDLE)
        self.assertEqual(format_option.tick(), NodeMsg.FAILED)


class TestFormatInputNode(unittest.TestCase):
    def testSimpleFormatString(self):
        format_input = FormatInputNode()
        format_input.inputs['format_string'] = 'foo {first}'
        format_input.inputs['dict'] = {'first': 'bar', 'second': 'not_printed'}
        self.assertEqual(format_input.state, NodeMsg.UNINITIALIZED)

        format_input.setup()

        self.assertEqual(format_input.state, NodeMsg.IDLE)
        self.assertEqual(format_input.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(format_input.outputs['formatted_string'], 'foo bar')

        self.assertEqual(format_input.untick(), NodeMsg.IDLE)
        self.assertEqual(format_input.reset(), NodeMsg.IDLE)
        self.assertEqual(format_input.shutdown(), NodeMsg.SHUTDOWN)

    def testFormatStringWrongKey(self):
        format_input = FormatInputNode()
        format_input.inputs['format_string'] = 'foo {missing}'
        format_input.inputs['dict'] = {'first': 'bar', 'second': 'not_printed'}
        self.assertEqual(format_input.state, NodeMsg.UNINITIALIZED)

        format_input.setup()

        self.assertEqual(format_input.state, NodeMsg.IDLE)
        self.assertEqual(format_input.tick(), NodeMsg.FAILED)
