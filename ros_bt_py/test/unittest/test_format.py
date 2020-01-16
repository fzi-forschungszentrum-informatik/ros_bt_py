import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.nodes.format import FormatOptionNode, FormatInputNode, \
    FormatOptionListNode, FormatInputListNode, \
    StringConcatenation

from ros_bt_py.exceptions import BehaviorTreeException


class TestFormatOptionNode(unittest.TestCase):
    def testSimpleFormatString(self):
        format_option = FormatOptionNode({'format_string': 'foo {first} {third!u} {third!l} {third!c}'})
        format_option.inputs['dict'] = {'first': 'bar', 'second': 'not_printed', 'third': 'ToTo'}
        self.assertEqual(format_option.state, NodeMsg.UNINITIALIZED)

        format_option.setup()

        self.assertEqual(format_option.state, NodeMsg.IDLE)
        self.assertEqual(format_option.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(format_option.outputs['formatted_string'], 'foo bar TOTO toto Toto')

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

class TestFormatOptionListNode(unittest.TestCase):
    def testSimpleFormatListString(self):
        format_option = FormatOptionListNode({'format_strings': ['foo {first}',
                                                                 'woot {first} {third!l}']
                                              })
        format_option.inputs['dict'] = {'first': 'bar', 'second': 'not_printed', 'third': 'ToTo'}
        self.assertEqual(format_option.state, NodeMsg.UNINITIALIZED)

        format_option.setup()

        self.assertEqual(format_option.state, NodeMsg.IDLE)
        self.assertEqual(format_option.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(format_option.outputs['formatted_strings'][0], 'foo bar')
        self.assertEqual(format_option.outputs['formatted_strings'][1], 'woot bar toto')

        self.assertEqual(format_option.untick(), NodeMsg.IDLE)
        self.assertEqual(format_option.reset(), NodeMsg.IDLE)
        self.assertEqual(format_option.shutdown(), NodeMsg.SHUTDOWN)

    def testFormatStringWrongKey(self):
        format_option = FormatOptionListNode({'format_strings': ['foo {missing}']})
        format_option.inputs['dict'] = {'first': 'bar', 'second': 'not_printed'}
        self.assertEqual(format_option.state, NodeMsg.UNINITIALIZED)

        format_option.setup()

        self.assertEqual(format_option.state, NodeMsg.IDLE)
        self.assertEqual(format_option.tick(), NodeMsg.FAILED)


class TestFormatInputNode(unittest.TestCase):
    def testSimpleFormatString(self):
        format_input = FormatInputNode()
        format_input.inputs['format_string'] = 'foo {first} {third!u} {third!l} {third!c}'
        format_input.inputs['dict'] = {'first': 'bar', 'second': 'not_printed', 'third': 'ToTo'}
        self.assertEqual(format_input.state, NodeMsg.UNINITIALIZED)

        format_input.setup()

        self.assertEqual(format_input.state, NodeMsg.IDLE)
        self.assertEqual(format_input.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(format_input.outputs['formatted_string'], 'foo bar TOTO toto Toto')

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


class TestFormatInputListNode(unittest.TestCase):
    def testSimpleFormatString(self):
        format_input = FormatInputListNode()
        format_input.inputs['format_strings'] = ['foo {first}',
                                                 'woot {first} {third!l}']
        format_input.inputs['dict'] = {'first': 'bar', 'second': 'not_printed', 'third': 'ToTo'}
        self.assertEqual(format_input.state, NodeMsg.UNINITIALIZED)

        format_input.setup()

        self.assertEqual(format_input.state, NodeMsg.IDLE)
        self.assertEqual(format_input.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(format_input.outputs['formatted_strings'][0], 'foo bar')
        self.assertEqual(format_input.outputs['formatted_strings'][1], 'woot bar toto')

        self.assertEqual(format_input.untick(), NodeMsg.IDLE)
        self.assertEqual(format_input.reset(), NodeMsg.IDLE)
        self.assertEqual(format_input.shutdown(), NodeMsg.SHUTDOWN)

    def testFormatStringWrongKey(self):
        format_input = FormatInputListNode()
        format_input.inputs['format_strings'] = ['foo {missing}']
        format_input.inputs['dict'] = {'first': 'bar', 'second': 'not_printed'}
        self.assertEqual(format_input.state, NodeMsg.UNINITIALIZED)

        format_input.setup()

        self.assertEqual(format_input.state, NodeMsg.IDLE)
        self.assertEqual(format_input.tick(), NodeMsg.FAILED)


class TestStringConcatenation(unittest.TestCase):
    def testConcatenation(self):
        concat = StringConcatenation()
        concat.inputs['a'] = 'toto'
        concat.inputs['b'] = 'foobar'
        self.assertEqual(concat.state, NodeMsg.UNINITIALIZED)
        concat.setup()

        self.assertEqual(concat.state, NodeMsg.IDLE)
        self.assertEqual(concat.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(concat.outputs['formatted_string'], 'totofoobar')

        self.assertEqual(concat.untick(), NodeMsg.IDLE)
        self.assertEqual(concat.reset(), NodeMsg.IDLE)
        self.assertEqual(concat.shutdown(), NodeMsg.SHUTDOWN)
