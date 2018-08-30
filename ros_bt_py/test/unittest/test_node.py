from copy import deepcopy
import jsonpickle
import random
import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData, NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.node import Node, load_node_module, increment_name
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.nodes.passthrough_node import PassthroughNode


class TestLoadModule(unittest.TestCase):
    def testLoadModule(self):
        self.assertEqual(load_node_module('i.do.not.exist'), None)

        load_node_module('ros_bt_py.nodes.passthrough_node')
        self.assertIn('ros_bt_py.nodes.passthrough_node', Node.node_classes)


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
    def testNodeHasNoConfig(self):
        self.assertEqual(Node._node_config, None)

    def testNodeInitFails(self):
        self.assertRaises(NodeConfigError, Node)

    def testMissingOption(self):
        self.assertRaises(ValueError, PassthroughNode)

    def testExtraOption(self):
        with self.assertRaises(ValueError):
            PassthroughNode({'passthrough_type': int,
                             'foo': 42})


class TestNodeConfig(unittest.TestCase):
    def setUp(self):
        self.conf = NodeConfig(
            options={'float_option': float},
            inputs={'int_input': int},
            outputs={'str_output': str},
            max_children=42)

    def testDetectDuplicates(self):
        duplicate_option = NodeConfig(options=deepcopy(self.conf.options),
                                      inputs={},
                                      outputs={},
                                      max_children=42)
        duplicate_input = NodeConfig(options={},
                                     inputs=deepcopy(self.conf.inputs),
                                     outputs={},
                                     max_children=42)
        duplicate_output = NodeConfig(options={},
                                      inputs={},
                                      outputs=deepcopy(self.conf.outputs),
                                      max_children=42)

        wrong_num_children = NodeConfig(options={'new_option': int},
                                        inputs={},
                                        outputs={},
                                        max_children=23)

        conf_original = deepcopy(self.conf)
        self.assertRaises(KeyError, self.conf.extend, duplicate_option)
        self.assertEqual(self.conf, conf_original)

        self.assertRaises(KeyError, self.conf.extend, duplicate_input)
        self.assertEqual(self.conf, conf_original)

        self.assertRaises(KeyError, self.conf.extend, duplicate_output)
        self.assertEqual(self.conf, conf_original)

        self.assertRaises(ValueError, self.conf.extend, wrong_num_children)
        self.assertEqual(self.conf, conf_original)


class TestPassthroughNode(unittest.TestCase):
    def testPassthroughNodeRegistered(self):
        self.assertTrue(PassthroughNode.__module__ in Node.node_classes)
        self.assertTrue(PassthroughNode.__name__ in Node.node_classes[PassthroughNode.__module__])
        self.assertEqual(PassthroughNode,
                         Node.node_classes[PassthroughNode.__module__][PassthroughNode.__name__])

    def testPassthroughNodeConfig(self):
        self.assertEqual(PassthroughNode._node_config,
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

    def testInitWithInvalidParams(self):
        self.assertRaises(TypeError, PassthroughNode, {'passthrough_type': 1})

    def testPassthroughNode(self):
        passthrough = PassthroughNode({'passthrough_type': int})
        self.assertEqual(passthrough.name, 'PassthroughNode')
        self.assertEqual(passthrough.state, NodeMsg.UNINITIALIZED)

        passthrough.setup()

        self.assertEqual(passthrough.state, NodeMsg.IDLE)
        self.assertEqual(passthrough.inputs['in'], None)
        self.assertEqual(passthrough.outputs['out'], None)
        self.assertRaises(ValueError, passthrough.tick)

        self.assertRaises(Exception, passthrough.setup)

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

    def testPassthroughNodeSetupAfterShutdown(self):
        passthrough = PassthroughNode({'passthrough_type': int})
        self.assertEqual(passthrough.name, 'PassthroughNode')
        self.assertEqual(passthrough.state, NodeMsg.UNINITIALIZED)

        passthrough.setup()
        self.assertEqual(passthrough.state, NodeMsg.IDLE)

        passthrough.shutdown()
        self.assertEqual(passthrough.state, NodeMsg.SHUTDOWN)

        passthrough.setup()
        self.assertEqual(passthrough.state, NodeMsg.IDLE)

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

    def testGetdataMap(self):
        passthrough = PassthroughNode({'passthrough_type': float})

        self.assertEqual(passthrough.get_data_map(NodeDataLocation.INPUT_DATA).name, 'inputs')
        self.assertEqual(passthrough.get_data_map(NodeDataLocation.OUTPUT_DATA).name, 'outputs')
        self.assertEqual(passthrough.get_data_map(NodeDataLocation.OPTION_DATA).name, 'options')
        self.assertRaises(KeyError, passthrough.get_data_map, '')
        self.assertRaises(KeyError, passthrough.get_data_map, 'INPUT_DATA')

    def testAddChild(self):
        """It should be impossible to add children to a PassthroughNode"""
        passthrough = PassthroughNode({'passthrough_type': float})

        self.assertRaises(BehaviorTreeException, passthrough.add_child,
                          PassthroughNode({'passthrough_type': float}))

    def testRemoveChild(self):
        """Removing a child from PassthroughNode should fail"""
        passthrough = PassthroughNode({'passthrough_type': float})

        self.assertRaises(KeyError, passthrough.remove_child,
                          'foo')

    def testNodeFromMsg(self):
        msg = NodeMsg(
            is_subtree=False,
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            inputs=[NodeData(key="in",
                             serialized_value=jsonpickle.encode(42))],
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
        self.assertEqual(instance.inputs['in'], 42)

        # Explicitly set name in message
        msg.name = 'Test Node'
        self.assertEqual(Node.from_msg(msg).name, 'Test Node')

        msg.state = 'Something'
        # Nodes created from messages do **not** get their state, since they're
        # always uninitialized!
        self.assertEqual(Node.from_msg(msg).state, NodeMsg.UNINITIALIZED)

    def testNodeFromMsgInvalidOptions(self):
        msg = NodeMsg(
            is_subtree=False,
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            options=[NodeData(key='passthrough_type',
                              serialized_value='definitely_not_a_type')])
        self.assertRaises(BehaviorTreeException, Node.from_msg, msg)

    def testNodeFromMsgInvalidInput(self):
        msg = NodeMsg(
            is_subtree=False,
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            inputs=[NodeData(key="in", serialized_value='nope')],
            options=[NodeData(key="passthrough_type",
                              serialized_value=jsonpickle.encode(int))])
        self.assertRaises(BehaviorTreeException, Node.from_msg, msg)

    def testNodeToMsg(self):
        node = PassthroughNode(options={'passthrough_type': int})

        msg = node.to_msg()

        self.assertFalse(msg.is_subtree)
        self.assertEqual(msg.module, PassthroughNode.__module__)
        self.assertEqual(msg.node_class, PassthroughNode.__name__)
        self.assertEqual(len(msg.options), len(node.options))
        self.assertEqual(msg.options[0].key, 'passthrough_type')
        self.assertEqual(jsonpickle.decode(msg.options[0].serialized_value), int)
        self.assertEqual(len(msg.inputs), len(node.inputs))
        self.assertEqual(len(msg.outputs), len(node.outputs))
        self.assertEqual(msg.state, node.state)

    def testNodeToMsgRoundtrip(self):
        node = PassthroughNode(options={'passthrough_type': int})

        msg = node.to_msg()

        self.assertEqual(Node.from_msg(msg), node)

    def testMsgToNodeRoundtrip(self):
        msg = NodeMsg(
            name='Test Node',
            is_subtree=False,
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int),
                              serialized_type=jsonpickle.encode(type))])
        instance = Node.from_msg(msg)
        self.assertEqual(instance.state, NodeMsg.UNINITIALIZED)

        # Not exactly equal, since to_msg picks up the empty input and output,
        # plus the UNINITIALIZED state.
        roundtrip_msg = instance.to_msg()
        self.assertNotEqual(roundtrip_msg, msg)

        # Should have an empty input and output
        self.assertEqual(len(roundtrip_msg.inputs), 1)
        self.assertEqual(len(roundtrip_msg.outputs), 1)

        roundtrip_msg.inputs = []
        roundtrip_msg.outputs = []
        roundtrip_msg.state = ''

        self.assertEqual(roundtrip_msg, msg)
