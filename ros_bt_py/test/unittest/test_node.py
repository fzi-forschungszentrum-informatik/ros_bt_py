#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
from copy import deepcopy
import random
import unittest
import types
import sys

from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds
from ros_bt_py_msgs.msg import NodeData, NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError, NodeStateError
from ros_bt_py.node import Node, Leaf, load_node_module, increment_name, define_bt_node
from ros_bt_py.node import FlowControl, Decorator
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.nodes.passthrough_node import PassthroughNode
from ros_bt_py.nodes.mock_nodes import MockUtilityLeaf
from ros_bt_py.node_data import NodeDataMap, NodeData as NodeDataObj
from ros_bt_py.helpers import json_encode, json_decode

try:
    range = xrange
except NameError:
    pass


class TestLoadModule(unittest.TestCase):
    def testLoadModule(self):
        self.assertEqual(load_node_module('i.do.not.exist'), None)

        load_node_module('ros_bt_py.nodes.passthrough_node')
        self.assertIn('ros_bt_py.nodes.passthrough_node', Node.node_classes)


class TestIncrementName(unittest.TestCase):
    def testIncrementName(self):
        numbers = [random.randint(0, 20) for _ in range(20)]
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

    def testNotImplementedEverything(self):
        @define_bt_node(NodeConfig(
            options={},
            inputs={},
            outputs={},
            max_children=0))
        class NotImplementedNode(Leaf):
            pass

        node = NotImplementedNode()
        self.assertRaises(NotImplementedError, node.setup)
        self.assertRaises(BehaviorTreeException, node.tick)

    def testNotImplementedTickUntickResetShutdown(self):
        @define_bt_node(NodeConfig(
            options={},
            inputs={},
            outputs={},
            max_children=0))
        class NotImplementedNode(Leaf):
            def _do_setup(self):
                pass

        node = NotImplementedNode()
        node.setup()
        self.assertEqual(node.state, NodeMsg.IDLE)
        self.assertRaises(NotImplementedError, node.tick)
        self.assertRaises(NotImplementedError, node.untick)
        self.assertRaises(NotImplementedError, node.reset)
        self.assertRaises(NotImplementedError, node.shutdown)

    def testUnsetOption(self):
        @define_bt_node(NodeConfig(
            options={'missing': type},
            inputs={},
            outputs={},
            max_children=0))
        class MissingOptionsNode(Leaf):
            def _do_setup(self):
                pass

            def _do_tick(self):
                return NodeMsg.SUCCEEDED

            def _do_shutdown(self):
                pass

            def _do_reset(self):
                return NodeMsg.IDLE

            def _do_untick(self):
                return NodeMsg.IDLE

        node = MissingOptionsNode({'missing': int})
        node.setup()
        node.options.reset_updated()
        self.assertRaises(BehaviorTreeException, node.tick)

    def testRegisterNodeData(self):
        @define_bt_node(NodeConfig(
            options={},
            inputs={},
            outputs={},
            max_children=0))
        class MinimalNode(Leaf):
            def _do_setup(self):
                pass

        source = {'test': int}

        target = NodeDataMap(name='options')
        target.add(key='test', value=NodeDataObj(data_type=int))

        minimal_node = MinimalNode()
        with self.assertRaises(NodeConfigError):
            minimal_node._register_node_data(source_map=source,
                                             target_map=target)

    def testRegisterNodeDataDuplicateOptionRef(self):
        @define_bt_node(NodeConfig(
            options={},
            inputs={},
            outputs={},
            max_children=0))
        class MinimalNode(Leaf):
            def _do_setup(self):
                pass

        source = {'test': OptionRef('test_target')}

        target = NodeDataMap(name='options')
        target.add(key='test_target', value=NodeDataObj(data_type=int))
        target.add(key='test', value=NodeDataObj(data_type=OptionRef('test_target')))

        minimal_node = MinimalNode()
        with self.assertRaises(NodeConfigError):
            minimal_node._register_node_data(source_map=source,
                                             target_map=target)

    def testRegisterNodeDataOptionRefInvalidOptionKey(self):
        @define_bt_node(NodeConfig(
            options={'invalid': OptionRef('missing')},
            inputs={},
            outputs={},
            max_children=0))
        class MinimalNode(Leaf):
            def _do_setup(self):
                pass

        with self.assertRaises(NodeConfigError):
            minimal_node = MinimalNode()

    def testRegisterNodeDataOptionRefUnwrittenOptionKey(self):
        @define_bt_node(NodeConfig(
            options={'unwritten': type,
                     'invalid': OptionRef('unwritten')},
            inputs={},
            outputs={},
            max_children=0))
        class MinimalNode(Leaf):
            def _do_setup(self):
                pass

        with self.assertRaises(NodeConfigError):
            minimal_node = MinimalNode()

    def testRegisterNodeDataOptionRefToNoType(self):
        def func():
            pass

        @define_bt_node(NodeConfig(
            options={'function': types.FunctionType,
                     'invalid': OptionRef('function')},
            inputs={},
            outputs={},
            max_children=0))
        class MinimalNode(Leaf):
            def _do_setup(self):
                pass

        with self.assertRaises(NodeConfigError):
            minimal_node = MinimalNode({'function': func})

    def testNodeNotEqual(self):
        @define_bt_node(NodeConfig(
            options={},
            inputs={},
            outputs={},
            max_children=0))
        class MinimalNode(Leaf):
            def _do_setup(self):
                pass

        first = MinimalNode(name='first')
        second = MinimalNode(name='second')

        self.assertNotEqual(first, second)

    def testNodeImplementationWithInvalidState(self):
        @define_bt_node(NodeConfig(
            options={},
            inputs={},
            outputs={},
            max_children=0))
        class InvalidStateNode(Leaf):
            def _do_setup(self):
                pass

            def _do_tick(self):
                return NodeMsg.IDLE  # this is wrong

            def _do_shutdown(self):
                pass

            def _do_reset(self):
                return NodeMsg.IDLE

            def _do_untick(self):
                return NodeMsg.IDLE

        node = InvalidStateNode()
        node.setup()
        self.assertRaises(NodeStateError, node.tick)

    def testNodeImplementationMultipleChildrenWithSameName(self):
        @define_bt_node(NodeConfig(
            options={},
            inputs={},
            outputs={},
            max_children=None))
        class FlowControlNode(FlowControl):
            def _do_setup(self):
                pass

            def _do_tick(self):
                return NodeMsg.SUCCEEDED

            def _do_shutdown(self):
                pass

            def _do_reset(self):
                return NodeMsg.IDLE

            def _do_untick(self):
                return NodeMsg.IDLE

        node = FlowControlNode()
        node.add_child(FlowControlNode(name='foo'))
        with self.assertRaises(KeyError):
            node.add_child(FlowControlNode(name='foo'))

    def testDecoratorWithoutChild(self):
        decorator = Decorator()
        expected_bounds = UtilityBounds(can_execute=True,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)

        self.assertEqual(decorator.calculate_utility(), expected_bounds)

    def testDecoratorWithChild(self):
        decorator = Decorator()
        cannot_exec = MockUtilityLeaf(
            name='cannot_exec',
            options={
                'can_execute': False,
                'utility_lower_bound_success': 1.0,
                'utility_upper_bound_success': 2.0,
                'utility_lower_bound_failure': 5.0,
                'utility_upper_bound_failure': 10.0})
        decorator.add_child(cannot_exec)
        expected_bounds = UtilityBounds(can_execute=False,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True,
                                        lower_bound_success=1.0,
                                        upper_bound_success=2.0,
                                        lower_bound_failure=5.0,
                                        upper_bound_failure=10.0,)

        self.assertEqual(decorator.calculate_utility(), expected_bounds)

    def testNodeImplementsLoggingMethods(self):
        @define_bt_node(NodeConfig(
            options={},
            inputs={},
            outputs={},
            max_children=0))
        class MinimalNode(Leaf):
            def _do_setup(self):
                pass

        node = MinimalNode()
        node.logdebug('')
        node.loginfo('')
        node.logwarn('')
        node.logerr('')
        node.logfatal('')


class TestDefineBTNodeDecoratorOnNonSubclass(unittest.TestCase):
    def testDecorator(self):
        with self.assertRaises(TypeError):
            @define_bt_node(NodeConfig(
                options={},
                inputs={},
                outputs={},
                max_children=0))
            class ShouldNotWork(object):
                pass


class TestOptionRef(unittest.TestCase):
    def testOptionRefImplementation(self):
        first = OptionRef('first')
        second = OptionRef('second')

        self.assertEqual(first, first)
        self.assertNotEqual(first, second)

        self.assertEqual("OptionRef(option_key='first')", repr(first))
        self.assertEqual("OptionRef(option_key='first')", first.__name__())


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

    def testExtend(self):
        additional_option = NodeConfig(options={'new_option': int},
                                       inputs={},
                                       outputs={},
                                       max_children=42)

        conf_original = deepcopy(self.conf)
        self.conf.extend(additional_option)
        self.assertNotEqual(self.conf, conf_original)

    def testRepr(self):
        if sys.version_info[0] == 2:
            expected = "NodeConfig(inputs={'int_input': <type 'int'>}," \
                       " outputs={'str_output': <type 'str'>}," \
                       " options={'float_option': <type 'float'>}," \
                       " max_children=42, optional_options=[], version=)"
        else:
            expected = "NodeConfig(inputs={'int_input': <class 'int'>}," \
                       " outputs={'str_output': <class 'str'>}," \
                       " options={'float_option': <class 'float'>}," \
                       " max_children=42, optional_options=[], version=)"
        self.assertEqual(expected, repr(self.conf))


class TestPassthroughNode(unittest.TestCase):
    def testPassthroughNodeRegistered(self):
        self.assertTrue(PassthroughNode.__module__ in Node.node_classes)
        self.assertTrue(PassthroughNode.__name__ in Node.node_classes[PassthroughNode.__module__])
        self.assertEqual(PassthroughNode,
                         Node.node_classes[PassthroughNode.__module__][PassthroughNode.__name__])

    def testPassthroughNodeConfig(self):
        self.assertEqual(PassthroughNode._node_config,
                         NodeConfig(
                             version='0.9.0',
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

        self.assertRaises(BehaviorTreeException, passthrough.untick)
        self.assertRaises(BehaviorTreeException, passthrough.reset)

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

        # Calling reset() should bring the node back into the same
        # state if was in when it was freshly created, save for the
        # input values (the updated state should be the same though)
        passthrough.reset()

        fresh_passthrough = PassthroughNode({'passthrough_type': int})

        self.assertEqual(passthrough.outputs['out'], fresh_passthrough.outputs['out'])
        self.assertEqual(passthrough.inputs.is_updated('in'),
                         fresh_passthrough.inputs.is_updated('in'))
        self.assertEqual(passthrough.outputs.is_updated('out'),
                         fresh_passthrough.outputs.is_updated('out'))

        expected_bounds = UtilityBounds(can_execute=True,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)

        self.assertEqual(fresh_passthrough.calculate_utility(), expected_bounds)

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
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            inputs=[NodeData(key="in",
                             serialized_value=json_encode(42))],
            options=[NodeData(key='passthrough_type',
                              serialized_value=json_encode(int))])
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
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            options=[NodeData(key='passthrough_type',
                              serialized_value='definitely_not_a_type')])
        self.assertRaises(BehaviorTreeException, Node.from_msg, msg)

    def testNodeFromMsgInvalidInput(self):
        msg = NodeMsg(
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            inputs=[NodeData(key="in", serialized_value='nope')],
            options=[NodeData(key="passthrough_type",
                              serialized_value=json_encode(int))])
        self.assertRaises(BehaviorTreeException, Node.from_msg, msg)

    def testNodeFromMsgInvalidOutput(self):
        msg = NodeMsg(
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            inputs=[NodeData(key="in",
                             serialized_value=json_encode(42))],
            outputs=[NodeData(key="out", serialized_value='nope')],
            options=[NodeData(key="passthrough_type",
                              serialized_value=json_encode(int))])
        self.assertRaises(BehaviorTreeException, Node.from_msg, msg)

    def testNodeToMsg(self):
        node = PassthroughNode(options={'passthrough_type': int})

        msg = node.to_msg()

        self.assertEqual(msg.module, PassthroughNode.__module__)
        self.assertEqual(msg.node_class, PassthroughNode.__name__)
        self.assertEqual(len(msg.options), len(node.options))
        self.assertEqual(msg.options[0].key, 'passthrough_type')
        self.assertEqual(json_decode(msg.options[0].serialized_value), int)
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
            module="ros_bt_py.nodes.passthrough_node",
            node_class="PassthroughNode",
            version="0.9.0",
            options=[NodeData(key='passthrough_type',
                              serialized_value=json_encode(int),
                              serialized_type=json_encode(type))])
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
