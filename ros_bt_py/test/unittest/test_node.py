import jsonpickle
import random
import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData

from ros_bt_py.node import Node, load_node_module, increment_name
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.nodes.passthrough_node import PassthroughNode


class TestLoadModule(unittest.TestCase):
    def testLoadModule(self):
        self.assertEqual(load_node_module('i.do.not.exist'), None)

        module = load_node_module('ros_bt_py.nodes.passthrough_node')
        self.assertEqual(len(Node.node_classes), 1, msg=str(Node.node_classes))
        self.assertEqual(module.__name__, Node.node_classes.keys()[0])


class TestIncrementName(unittest.TestCase):
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

        self.assertRegexpMatches(increment_name(''),
                                 '_2')


class TestNode(unittest.TestCase):
    def TestNodeHasNoConfig(self):
        self.assertEqual(Node.node_config, None)

    def TestNodeInitFails(self):
        self.assertRaises(ValueError, Node)

    def testMissingOption(self):
        self.assertRaises(ValueError, PassthroughNode)


class TestPassthroughNode(unittest.TestCase):
    def testPassthroughNodeRegistered(self):
        self.assertTrue(PassthroughNode.__module__ in Node.node_classes)
        self.assertTrue(PassthroughNode.__name__ in Node.node_classes[PassthroughNode.__module__])
        self.assertEqual(PassthroughNode,
                         Node.node_classes[PassthroughNode.__module__][PassthroughNode.__name__])

    def testPassthroughNodeConfig(self):
        self.assertEqual(PassthroughNode.node_config,
                         NodeConfig(
                             options={'passthrough_type': type},
                             inputs={'in': OptionRef('passthrough_type')},
                             outputs={'out': OptionRef('passthrough_type')},
                             max_children=0))

    def testPassthroughNodeIO(self):
        passthrough = PassthroughNode({'passthrough_type': int})
        self.assertEqual(passthrough.inputs.name, 'inputs')
        self.assertEqual(passthrough.outputs.name, 'outputs')
        self.assertEqual(passthrough.options.name, 'options')

    def testPassthroughNode(self):
        passthrough = PassthroughNode({'passthrough_type': int})
        self.assertEqual(passthrough.name, 'PassthroughNode')
        self.assertEqual(passthrough.state, NodeMsg.UNINITIALIZED)

        passthrough.setup()

        self.assertEqual(passthrough.state, NodeMsg.IDLE)
        self.assertEqual(passthrough.inputs['in'], None)
        self.assertEqual(passthrough.outputs['out'], None)
        self.assertRaises(ValueError, passthrough.tick)

        passthrough.inputs['in'] = 42
        passthrough.tick()
        self.assertEqual(passthrough.state, NodeMsg.SUCCEEDED)
        self.assertTrue(passthrough.outputs.is_updated('out'))
        self.assertEqual(passthrough.outputs['out'], 42)

        # Ensure that changing the input does not affect the output
        passthrough.inputs['in'] = 0
        self.assertEqual(passthrough.outputs['out'], 42)

        # Calling reset() should bring the node back into the same state if was
        # in when it was freshly created:
        passthrough.reset()

        fresh_passthrough = PassthroughNode({'passthrough_type': int})

        self.assertEqual(passthrough.inputs['in'], fresh_passthrough.inputs['in'])
        self.assertEqual(passthrough.outputs['out'], fresh_passthrough.outputs['out'])
        self.assertEqual(passthrough.inputs.is_updated('in'),
                         fresh_passthrough.inputs.is_updated('in'))
        self.assertEqual(passthrough.outputs.is_updated('out'),
                         fresh_passthrough.outputs.is_updated('out'))

    def testPassthroughNodeUntick(self):
        passthrough = PassthroughNode({'passthrough_type': float})
        passthrough.setup()

        passthrough.inputs['in'] = 1.5
        self.assertEqual(passthrough.state, NodeMsg.IDLE)

        passthrough.tick()

        self.assertEqual(passthrough.state, NodeMsg.SUCCEEDED)
        self.assertTrue(passthrough.outputs.is_updated('out'))

        passthrough.untick()

        self.assertEqual(passthrough.state, NodeMsg.IDLE)
        self.assertFalse(passthrough.outputs.is_updated('out'))

    def testNodeFromMsg(self):
        msg = NodeMsg(
            is_subtree=False,
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])
        instance = Node.from_msg(msg)

        node_class = type(instance)
        self.assertTrue(node_class.__module__ in Node.node_classes)
        self.assertTrue(node_class.__name__ in Node.node_classes[node_class.__module__])
        self.assertEqual(node_class,
                         Node.node_classes[node_class.__module__][node_class.__name__])

        self.assertEqual(instance.options['passthrough_type'], int)
        # Node names should default to their class name
        self.assertEqual(instance.name, 'PassthroughNode')

        # Check that the second node gets an incremented name if we're using
        # node_dict
        self.assertEqual(Node.from_msg(msg, node_dict={'PassthroughNode': 1}).name,
                         'PassthroughNode_2')

        # Explicitly set name in message
        msg.name = 'Test Node'
        self.assertEqual(Node.from_msg(msg).name, 'Test Node')

        msg.state = 'Something'
        # Nodes created from messages do **not** get their state, since they're
        # always uninitialized!
        self.assertEqual(Node.from_msg(msg).state, NodeMsg.UNINITIALIZED)

    def testNodeToMsg(self):
        node = PassthroughNode(options={'passthrough_type': int})

        msg = Node.to_msg(node)

        self.assertFalse(msg.is_subtree)
        self.assertEqual(msg.module, PassthroughNode.__module__)
        self.assertEqual(msg.node_class, PassthroughNode.__name__)
        self.assertEqual(len(msg.options), len(node.options))
        self.assertEqual(msg.options[0].key, 'passthrough_type')
        self.assertEqual(jsonpickle.decode(msg.options[0].serialized_value), int)
        self.assertEqual(len(msg.current_inputs), len(node.inputs))
        self.assertEqual(len(msg.current_outputs), len(node.outputs))
        self.assertEqual(msg.state, node.state)

    def testNodeToMsgRoundtrip(self):
        node = PassthroughNode(options={'passthrough_type': int})

        msg = Node.to_msg(node)

        self.assertEqual(Node.from_msg(msg), node)

    def testMsgToNodeRoundtrip(self):
        msg = NodeMsg(
            name='Test Node',
            is_subtree=False,
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])
        instance = Node.from_msg(msg)
        self.assertEqual(instance.state, NodeMsg.UNINITIALIZED)

        # Not exactly equal, since to_msg picks up the empty input and output,
        # plus the UNINITIALIZED state.
        roundtrip_msg = Node.to_msg(instance)
        self.assertNotEqual(roundtrip_msg, msg)

        # Should have an empty input and output
        self.assertEqual(len(roundtrip_msg.current_inputs), 1)
        self.assertEqual(len(roundtrip_msg.current_outputs), 1)

        roundtrip_msg.current_inputs = []
        roundtrip_msg.current_outputs = []
        roundtrip_msg.state = ''

        self.assertEqual(roundtrip_msg, msg)
