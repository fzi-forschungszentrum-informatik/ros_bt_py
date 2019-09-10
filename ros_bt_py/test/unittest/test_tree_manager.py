import unittest

import jsonpickle
import sys
import time

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData, NodeDataWiring, NodeDataLocation, Tree
from ros_bt_py_msgs.srv import (WireNodeDataRequest, AddNodeRequest, RemoveNodeRequest,
                                ControlTreeExecutionRequest, GetAvailableNodesRequest,
                                SetExecutionModeRequest, SetOptionsRequest, ContinueRequest,
                                LoadTreeRequest, MoveNodeRequest, ReplaceNodeRequest,
                                ClearTreeRequest, LoadTreeFromPathRequest,
                                SetExecutionModeResponse, ModifyBreakpointsRequest,
                                GetSubtreeRequest)

from ros_bt_py.node import Node, Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.exceptions import BehaviorTreeException, MissingParentError, TreeTopologyError
from ros_bt_py.tree_manager import TreeManager
from ros_bt_py.tree_manager import (get_success as tm_get_success,
                                    get_error_message as tm_get_error_message)


class TestTreeManager(unittest.TestCase):
    def setUp(self):
        self.tree_msg = None
        self.debug_info_msg = None

        def set_tree_msg(msg):
            self.tree_msg = msg

        def set_debug_info_msg(msg):
            self.debug_info_msg = msg

        self.manager = TreeManager(publish_tree_callback=set_tree_msg,
                                   publish_debug_info_callback=set_debug_info_msg)
        self.node_msg = NodeMsg(
            module='ros_bt_py.nodes.passthrough_node',
            node_class='PassthroughNode',
            inputs=[NodeData(key='in',
                             serialized_value=jsonpickle.encode(42))],
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])

        self.constant_msg = NodeMsg(
            module='ros_bt_py.nodes.constant',
            node_class='Constant',
            options=[NodeData(key='constant_type',
                              serialized_value=jsonpickle.encode(int)),
                     NodeData(key='constant_value',
                              serialized_value=jsonpickle.encode(42))])

        self.sequence_msg = NodeMsg(
            module='ros_bt_py.nodes.sequence',
            node_class='Sequence')

        self.succeeder_msg = NodeMsg(
            module='ros_bt_py.nodes.mock_nodes',
            node_class='MockLeaf',
            options=[NodeData(key='output_type',
                              serialized_value=jsonpickle.encode(str)),
                     NodeData(key='state_values',
                              serialized_value=jsonpickle.encode([NodeMsg.SUCCEEDED])),
                     NodeData(key='output_values',
                              serialized_value=jsonpickle.encode(['Yay!']))])

    def testEnsureTickFrequencyGreaterZero(self):
        manager = TreeManager(tick_frequency_hz=0)
        self.assertNotEquals(manager.tree_msg.tick_frequency_hz, 0)

    def testTickFrequencyTooHigh(self):
        tick_frequency_hz = 10000000000000.0
        sleep_duration_sec = (1.0 / tick_frequency_hz)
        manager = TreeManager(tick_frequency_hz=tick_frequency_hz)
        add_request = AddNodeRequest(node=self.node_msg,
                                     allow_rename=True)
        self.assertTrue(manager.add_node(add_request).success)

        execution_request = ControlTreeExecutionRequest()
        execution_request.command = ControlTreeExecutionRequest.TICK_PERIODICALLY

        start_time = time.time()
        self.assertTrue(get_success(manager.control_execution(execution_request)))
        tick_duration = time.time() - start_time
        self.assertGreater(tick_duration, sleep_duration_sec)

        time.sleep(0.1)

        manager.tree_msg.state = Tree.STOP_REQUESTED
        manager._tick_thread.join(0.1)
        self.assertFalse(manager._tick_thread.is_alive())

    def testLoadNodeModule(self):
        manager = TreeManager(module_list=['ros_bt_py.nodes.sequence'])
        self.assertIn('ros_bt_py.nodes.sequence', sys.modules)

    def testCycle(self):
        node = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        self.manager.nodes[node.name].parent = node.name

        self.assertRaises(TreeTopologyError, self.manager.find_root)

    def testOrphan(self):
        node = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        node2 = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        self.manager.nodes[node.name].parent = node2
        self.manager.remove_node(RemoveNodeRequest(node_name=node2.name,
                                                   remove_children=False))

        self.assertRaises(MissingParentError, self.manager.tick, True)

    def testNoNodes(self):
        self.manager.tick(once=True)
        self.assertEqual(self.manager.tree_msg.state, Tree.EDITABLE)

    def testGetSuccessErrorMessageDict(self):
        message = {'success': False,
                   'error_message': 'error'}
        self.assertFalse(tm_get_success(message))
        self.assertEqual(tm_get_error_message(message), 'error')

    def testGetSubtreeService(self):
        add_request = AddNodeRequest(node=self.sequence_msg,
                                     allow_rename=True)

        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 1)
        self.assertTrue(get_success(response))

        add_request = AddNodeRequest(node=self.sequence_msg,
                                     allow_rename=True,
                                     parent_name=response.actual_node_name)
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 2)
        self.assertTrue(get_success(response))

        seq_2_name = response.actual_node_name

        add_request = AddNodeRequest(node=self.succeeder_msg,
                                     allow_rename=True,
                                     parent_name=seq_2_name)
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 3)
        self.assertTrue(get_success(response))

        add_request = AddNodeRequest(node=self.succeeder_msg,
                                     allow_rename=True,
                                     parent_name=seq_2_name)
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 4)
        self.assertTrue(get_success(response))

        subtree_request = GetSubtreeRequest(subtree_root_name=seq_2_name)
        subtree_response = self.manager.get_subtree(subtree_request)

        self.assertTrue(get_success(subtree_response))
        self.assertEqual(len(subtree_response.subtree.nodes), 3)

        subtree_request = GetSubtreeRequest(subtree_root_name='no_in_tree')
        subtree_response = self.manager.get_subtree(subtree_request)

        self.assertFalse(get_success(subtree_response))

    def testTickExceptionHandling(self):
        @define_bt_node(NodeConfig(
            options={},
            inputs={},
            outputs={},
            max_children=0))
        class ExceptionNode(Leaf):
            def _do_setup(self):
                pass

            def _do_tick(self):
                raise BehaviorTreeException

            def _do_shutdown(self):
                pass

            def _do_reset(self):
                return NodeMsg.IDLE

            def _do_untick(self):
                return NodeMsg.IDLE

        node = ExceptionNode()
        manager = TreeManager(show_traceback_on_exception=False)
        manager.nodes[node.name] = node
        self.assertEqual(manager.tree_msg.state, Tree.EDITABLE)
        manager.tick_report_exceptions()
        self.assertEqual(manager.tree_msg.state, Tree.ERROR)

        manager = TreeManager(show_traceback_on_exception=True)
        manager.nodes[node.name] = node
        self.assertEqual(manager.tree_msg.state, Tree.EDITABLE)
        manager.tick_report_exceptions()
        self.assertEqual(manager.tree_msg.state, Tree.ERROR)

    def testLoadNode(self):
        _ = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        _ = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        self.node_msg.name = 'Test Node'
        _ = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)

        self.assertIn('PassthroughNode', self.manager.nodes)
        self.assertIn('PassthroughNode_2', self.manager.nodes)
        self.assertIn('Test Node', self.manager.nodes)

    def testWireData(self):
        root = self.manager.instantiate_node_from_msg(
            NodeMsg(
                module='ros_bt_py.nodes.sequence',
                node_class='Sequence',
                name='root'),
            allow_rename=False)

        self.node_msg.name = 'source_node'
        source = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        root.add_child(source)
        self.node_msg.name = 'target_node'
        target = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        root.add_child(target)

        self.assertIn('source_node', self.manager.nodes)
        self.assertIn('target_node', self.manager.nodes)

        valid_request = WireNodeDataRequest()
        valid_request.wirings.append(NodeDataWiring(
            source=NodeDataLocation(node_name='source_node',
                                    data_key='out',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name='target_node',
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA)))

        response = self.manager.wire_data(valid_request)
        self.assertTrue(get_success(response), get_error_message(response))
        self.assertEqual(len(self.manager.nodes['source_node'].outputs.callbacks), 1)
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 1)

    def testWireWithInvalidKey(self):
        self.node_msg.name = 'source_node'
        self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        self.node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)

        self.assertIn('source_node', self.manager.nodes)
        self.assertIn('target_node', self.manager.nodes)

        invalid_key_request = WireNodeDataRequest()
        invalid_key_request.wirings.append(NodeDataWiring(
            source=NodeDataLocation(node_name='source_node',
                                    # PassthroghNode does not have this key!
                                    data_key='wrong',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name='target_node',
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA)))

        response = self.manager.wire_data(invalid_key_request)
        self.assertFalse(get_success(response))

    def testWireWithInvalidNodeName(self):
        self.node_msg.name = 'source_node'
        self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        self.node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)

        self.assertIn('source_node', self.manager.nodes)
        self.assertIn('target_node', self.manager.nodes)

        invalid_node_request = WireNodeDataRequest()
        invalid_node_request.wirings.append(NodeDataWiring(
            # Wrong node name for source node
            source=NodeDataLocation(node_name='fantasy_node',
                                    data_key='out',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name='target_node',
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA)))

        response = self.manager.wire_data(invalid_node_request)
        self.assertFalse(get_success(response))

    def testMultiWireWithOneInvalid(self):
        """WireNodeData supports wiring multiple pairs of NodeData at once.

        If there's an error while handling any pair, none of the wirings must be applied!
        """
        self.node_msg.name = 'source_node'
        self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        self.node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)

        self.assertIn('source_node', self.manager.nodes)
        self.assertIn('target_node', self.manager.nodes)

        invalid_multi_request = WireNodeDataRequest()
        # This is fine and should work
        invalid_multi_request.wirings.append(NodeDataWiring(
            source=NodeDataLocation(node_name='source_node',
                                    data_key='out',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name='target_node',
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA)))
        invalid_multi_request.wirings.append(NodeDataWiring(
            # Wrong node name for source node
            source=NodeDataLocation(node_name='fantasy_node',
                                    data_key='out',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name='target_node',
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA)))

        response = self.manager.wire_data(invalid_multi_request)
        self.assertFalse(get_success(response))
        # The first half should not have been applied -> no callbacks for
        # source_node
        self.assertEqual(len(self.manager.nodes['source_node'].outputs.callbacks), 0)

    def testUnwire(self):
        root = self.manager.instantiate_node_from_msg(
            NodeMsg(
                module='ros_bt_py.nodes.sequence',
                node_class='Sequence',
                name='root'),
            allow_rename=False)

        wire_request = WireNodeDataRequest()
        wire_request.wirings.append(NodeDataWiring(
            # Wrong node name for source node
            source=NodeDataLocation(node_name='source_node',
                                    data_key='out',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name='target_node',
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA)))

        response = self.manager.unwire_data(wire_request)
        # Our manager has no nodes at all, so unwiring anything won't work
        self.assertFalse(get_success(response))

        self.node_msg.name = 'source_node'
        source = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        root.add_child(source)
        self.node_msg.name = 'target_node'
        target = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)
        root.add_child(target)

        response = self.manager.unwire_data(wire_request)
        # The nodes and keys exist. There aren't any callbacks to remove, but
        # the unwire operation still succeeds (after running it, the two data
        # values are unconnected).
        self.assertTrue(
            get_success(response),
            get_error_message(response) + "\n" + str(self.manager.nodes))

        response = self.manager.wire_data(wire_request)
        self.assertTrue(get_success(response))
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 1)

        response = self.manager.unwire_data(wire_request)
        self.assertTrue(get_success(response))
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 0)

    def testClearTree(self):
        # Adding a node to the tree and ticking it once
        add_request = AddNodeRequest(node=self.succeeder_msg)

        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 1)
        self.assertTrue(get_success(response))

        self.manager.nodes['MockLeaf'].state = NodeMsg.RUNNING

        # Clear will fail until the tree is shutdown
        clear_request = ClearTreeRequest()
        response = self.manager.clear(clear_request)
        self.assertFalse(get_success(response))
        self.assertEqual(len(self.manager.nodes), 1)

        execution_request = ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.TICK_ONCE)
        execution_request.command = ControlTreeExecutionRequest.SHUTDOWN
        self.assertTrue(self.manager.control_execution(execution_request).success)

        # after shutdown clear works again
        response = self.manager.clear(clear_request)
        self.assertTrue(get_success(response))
        self.assertEqual(len(self.manager.nodes), 0)

        # even a tree with multiple nodes (and no root) is cleared
        add_request = AddNodeRequest(node=self.succeeder_msg, allow_rename=True)
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 1)
        self.assertTrue(get_success(response))
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 2)
        self.assertTrue(get_success(response))

        response = self.manager.clear(clear_request)
        self.assertTrue(get_success(response))
        self.assertEqual(len(self.manager.nodes), 0)

    def testAddNode(self):
        add_request = AddNodeRequest(node=self.node_msg)

        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 1)
        self.assertTrue(get_success(response))

        broken_add = AddNodeRequest(node=NodeMsg(module='asdf',
                                                 node_class='foo'))

        response = self.manager.add_node(broken_add)
        self.assertFalse(get_success(response))

    def testAddWithMissingParent(self):
        self.assertFalse(self.manager.add_node(AddNodeRequest(node=self.node_msg,
                                                              parent_name='foo')).success)

    def testAddMultiple(self):
        add_request = AddNodeRequest(node=self.sequence_msg)
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 1)
        self.assertTrue(get_success(response))

        add_request = AddNodeRequest(node=self.node_msg,
                                     parent_name=response.actual_node_name)
        response = self.manager.add_node(add_request)
        self.assertEqual(len(self.manager.nodes), 2)
        self.assertTrue(get_success(response))

    def testAddRenaming(self):
        add_request = AddNodeRequest(node=self.sequence_msg)
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 1)
        self.assertTrue(get_success(response))

        # Add the same node again - since allow_rename should default
        # to false, this will fail.
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 1)
        self.assertFalse(get_success(response))

        # Same with allow_rename set to False explicitly
        add_request.allow_rename = False
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 1)
        self.assertFalse(get_success(response))

        # But it should work if we set allow_rename to True
        add_request.allow_rename = True
        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 2)
        self.assertTrue(get_success(response))

    def testAddWithChild(self):
        add_request = AddNodeRequest(node=self.sequence_msg)
        response = self.manager.add_node(add_request)

        self.assertTrue(get_success(response))
        self.assertEqual(len(self.manager.nodes), 1)
        self.sequence_msg.child_names.append(response.actual_node_name)

        add_request = AddNodeRequest(node=self.sequence_msg,
                                     allow_rename=True)
        response = self.manager.add_node(add_request)

        self.assertTrue(get_success(response))
        self.assertEqual(len(self.manager.nodes), 2)
        root = self.manager.find_root()
        # The newly inserted second node should be the root of the tree, since
        # the other one is its child
        self.assertEqual(response.actual_node_name, root.name)
        self.assertEqual(len(root.children), 1)

    def testAddWithMissingChild(self):
        self.sequence_msg.child_names.append('imaginary_node')
        add_request = AddNodeRequest(node=self.sequence_msg)
        response = self.manager.add_node(add_request)

        # Don't add nodes with missing children to the tree
        self.assertFalse(get_success(response))
        self.assertEqual(len(self.manager.nodes), 0)

    def testAddWithInvalidOption(self):
        self.node_msg.options = [
            NodeData(key='passthrough_type',
                     # passthrough_type must be a type, not an int
                     serialized_value=jsonpickle.encode(42))]
        add_request = AddNodeRequest(node=self.node_msg)
        response = self.manager.add_node(add_request)

        self.assertFalse(get_success(response))

    def testBuildCycle(self):
        add_request = AddNodeRequest(node=self.sequence_msg)
        response = self.manager.add_node(add_request)

        self.assertTrue(get_success(response))
        self.assertEqual(len(self.manager.nodes), 1)
        self.sequence_msg.child_names.append(response.actual_node_name)

        add_request = AddNodeRequest(parent_name=response.actual_node_name,
                                     node=self.sequence_msg,
                                     allow_rename=True)
        response = self.manager.add_node(add_request)

        self.assertFalse(get_success(response))
        self.assertEqual(len(self.manager.nodes), 1)

    def testRemoveNode(self):
        instance = self.manager.instantiate_node_from_msg(self.node_msg, allow_rename=True)

        remove_request = RemoveNodeRequest(node_name=instance.name)

        response = self.manager.remove_node(remove_request)
        self.assertTrue(get_success(response))

        # Second remove will fail, there's nothing left to remove.
        response = self.manager.remove_node(remove_request)
        self.assertFalse(get_success(response))

    def testRemoveParent(self):
        add_response = self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))

        self.manager.add_node(
            AddNodeRequest(node=self.node_msg,
                           parent_name=add_response.actual_node_name))

        self.assertEqual(len(self.manager.nodes), 2)

        remove_response = self.manager.remove_node(
            RemoveNodeRequest(node_name=add_response.actual_node_name,
                              remove_children=False))

        self.assertTrue(get_success(remove_response))
        self.assertEqual(len(self.manager.nodes), 1)

        self.manager.tick(once=True)
        root_node = [node for node in self.tree_msg.nodes
                     if node.name == self.tree_msg.root_name][0]
        self.assertEqual(root_node.state, NodeMsg.SUCCEEDED)

    def testRemoveParentAndChildren(self):
        add_response = self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))

        self.manager.add_node(
            AddNodeRequest(node=self.node_msg,
                           parent_name=add_response.actual_node_name))

        self.assertEqual(len(self.manager.nodes), 2)

        remove_response = self.manager.remove_node(
            RemoveNodeRequest(node_name=add_response.actual_node_name,
                              remove_children=True))

        self.assertTrue(get_success(remove_response), get_error_message(remove_response))
        self.assertEqual(len(self.manager.nodes), 0)

    def testMoveNode(self):
        self.sequence_msg.name = 'outer_seq'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))

        self.sequence_msg.name = "inner_seq"
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg,
                           parent_name='outer_seq'))))

        self.succeeder_msg.name = 'A'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='outer_seq'))))

        self.succeeder_msg.name = 'B'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='inner_seq'))))

        self.assertEqual(len(self.manager.nodes), 4)

        # Should fail, since "A" is a MockLeaf, which can't have children
        self.assertFalse(get_success(self.manager.move_node(
            MoveNodeRequest(
                node_name='B',
                new_parent_name='A',
                new_child_index=0))))

        # Should fail, since "asdf" is not in the tree
        self.assertFalse(get_success(self.manager.move_node(
            MoveNodeRequest(
                node_name='B',
                new_parent_name='asdf',
                new_child_index=0))))

        # Should fail, since "asdf" is not in the tree
        self.assertFalse(get_success(self.manager.move_node(
            MoveNodeRequest(
                node_name='asdf',
                new_parent_name='outer_seq',
                new_child_index=0))))

        # Should succeed and put "A" after "B" (-1 means
        # "first from the back")
        self.assertTrue(get_success(self.manager.move_node(
            MoveNodeRequest(
                node_name='A',
                new_parent_name='inner_seq',
                new_child_index=-1))))

        self.assertIn('inner_seq', [node.name for node in self.tree_msg.nodes])
        for node in self.tree_msg.nodes:
            if node.name == 'outer_seq':
                # After moving A into inner_seq, outer_seq has only
                # one child
                self.assertEqual(len(node.child_names), 1)
            if node.name == 'inner_seq':
                A_index = None
                B_index = None
                for index, name in enumerate(node.child_names):
                    if name == "A":
                        A_index = index
                    if name == "B":
                        B_index = index
                self.assertIsNotNone(A_index, 'Node A is not a child of inner_seq!')
                self.assertIsNotNone(B_index, 'Node B is not a child of inner_seq!')
                # As mentioned above, A should appear *after* B in the
                # list of inner_seq's children!
                self.assertGreater(A_index, B_index)

    def testMoveToNoParent(self):
        self.sequence_msg.name = 'seq'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))

        self.succeeder_msg.name = 'A'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='seq'))))
        self.assertEqual(len(self.tree_msg.nodes), 2)

        self.assertTrue(get_success(self.manager.move_node(
            MoveNodeRequest(
                node_name='A',
                new_parent_name=''))))

        # With A removed from seq's children, no node should have any
        # children!
        self.assertTrue(all([not node.child_names for node in self.tree_msg.nodes]))

    def testMoveWithinSameParent(self):
        self.sequence_msg.name = 'seq'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))

        self.succeeder_msg.name = 'A'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='seq'))))

        self.succeeder_msg.name = 'B'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='seq'))))
        self.succeeder_msg.name = 'C'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='seq'))))

        self.assertEqual(len(self.tree_msg.nodes), 4)

        # Confirm the positions of all three succeeders
        seq_msg = None
        for node in self.tree_msg.nodes:
            if node.name == 'seq':
                seq_msg = node
                break
        self.assertIsNotNone(seq_msg, 'Failed to find sequence in tree message')
        self.assertEqual(seq_msg.child_names, ['A', 'B', 'C'])

        self.assertTrue(get_success(self.manager.move_node(
            MoveNodeRequest(
                node_name='A',
                new_parent_name='seq',
                new_child_index=1
            ))))

        seq_msg = None
        for node in self.tree_msg.nodes:
            if node.name == 'seq':
                seq_msg = node
                break
        self.assertIsNotNone(seq_msg, 'Failed to find sequence in tree message')
        self.assertEqual(seq_msg.child_names, ['B', 'A', 'C'])

    def testMoveToOwnChild(self):
        self.sequence_msg.name = 'seq'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))

        self.sequence_msg.name = 'seq_2'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg,
                           parent_name='seq'))))

        # This should be impossible, since it leads to a
        # circular graph!
        self.assertFalse(get_success(self.manager.move_node(
            MoveNodeRequest(
                node_name='seq',
                new_parent_name='seq_2',
                new_child_index=0
            ))))

    def testReplaceNode(self):
        self.sequence_msg.name = 'seq'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))

        self.succeeder_msg.name = 'A'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='seq'))))

        self.succeeder_msg.name = 'B'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='seq'))))

        self.assertEqual(len(self.tree_msg.nodes), 3)

        self.assertFalse(get_success(self.manager.replace_node(
            ReplaceNodeRequest(
                old_node_name="asdf",
                new_node_name="A"))))

        self.assertFalse(get_success(self.manager.replace_node(
            ReplaceNodeRequest(
                old_node_name="B",
                new_node_name="asdf"))))

        self.assertTrue(get_success(self.manager.replace_node(
            ReplaceNodeRequest(
                old_node_name="B",
                new_node_name="A"))))

        self.assertEqual(len(self.tree_msg.nodes), 2)
        # B was overwritten by A
        self.assertNotIn("B", [node.name for node in self.tree_msg.nodes])

        self.sequence_msg.name = 'new_seq'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))

        self.assertEqual(len(self.tree_msg.nodes), 3)

        self.assertTrue(get_success(self.manager.replace_node(
            ReplaceNodeRequest(
                old_node_name="seq",
                new_node_name="new_seq"))))

        self.assertEqual(len(self.tree_msg.nodes), 2)
        # seq should be overwritten by new_seq
        self.assertNotIn("seq", [node.name for node in self.tree_msg.nodes])
        self.assertIn("new_seq", [node.name for node in self.tree_msg.nodes])

        for node in self.tree_msg.nodes:
            if node.name == 'new_seq':
                self.assertIn("A", node.child_names)

    def testReplaceParent(self):
        self.sequence_msg.name = 'seq'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))

        self.sequence_msg.name = 'seq_2'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg,
                           parent_name='seq'))))

        # This should succeed, but to avoid cycles, seq_2 cannot
        # inherit all of seq's children (which would include itself)
        self.assertTrue(get_success(self.manager.replace_node(
            ReplaceNodeRequest(
                old_node_name='seq',
                new_node_name='seq_2'
            ))))

        self.assertEqual(len(self.tree_msg.nodes), 1)
        # seq only had seq_2 as its child, so seq_2 should have 0
        # children now
        self.assertEqual(len(self.tree_msg.nodes[0].child_names), 0)

    def testReplaceOrder(self):
        self.sequence_msg.name = 'seq'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))

        self.succeeder_msg.name = 'A'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='seq'))))

        self.succeeder_msg.name = 'B'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='seq'))))
        self.succeeder_msg.name = 'C'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.succeeder_msg,
                           parent_name='seq'))))

        self.assertEqual(len(self.tree_msg.nodes), 4)

        # Confirm the positions of all three succeeders
        seq_msg = None
        for node in self.tree_msg.nodes:
            if node.name == 'seq':
                seq_msg = node
                break
        self.assertIsNotNone(seq_msg, 'Failed to find sequence in tree message')
        self.assertEqual(seq_msg.child_names, ['A', 'B', 'C'])

        self.assertTrue(get_success(self.manager.replace_node(
            ReplaceNodeRequest(
                new_node_name='A',
                old_node_name='B'
            ))))

        seq_msg = None
        for node in self.tree_msg.nodes:
            if node.name == 'seq':
                seq_msg = node
                break
        self.assertIsNotNone(seq_msg, 'Failed to find sequence in tree message')
        self.assertEqual(seq_msg.child_names, ['A', 'C'])

    def testTick(self):
        add_request = AddNodeRequest(node=self.node_msg)
        add_request.node.inputs.append(NodeData(key='in',
                                                serialized_value=jsonpickle.encode(42)))

        response = self.manager.add_node(add_request)
        self.assertTrue(get_success(response))

        self.manager.tick(once=True)
        self.assertEqual(self.manager.nodes[response.actual_node_name].outputs['out'], 42)
        # After finishing the tick, the TreeManager should have called the tree
        # and debug info callbacks, setting these values.
        self.assertIsNotNone(self.tree_msg)
        self.assertIsNotNone(self.debug_info_msg)
        self.assertIn(response.actual_node_name,
                      [node.name for
                       node in self.tree_msg.nodes])
        node_msg = next((node for
                         node in self.tree_msg.nodes if node.name == response.actual_node_name))
        self.assertEqual(jsonpickle.decode(node_msg.inputs[0].serialized_value), 42)
        self.assertEqual(jsonpickle.decode(node_msg.outputs[0].serialized_value), 42)

    def testControlTree(self):
        add_request = AddNodeRequest(node=self.node_msg)
        add_request.node.name = 'passthrough'
        add_request.node.inputs.append(NodeData(key='in',
                                                serialized_value=jsonpickle.encode(42)))

        self.assertTrue(self.manager.add_node(add_request).success)
        self.assertEqual(self.manager.nodes['passthrough'].inputs['in'], 42)
        self.assertIsNone(self.manager.nodes['passthrough'].outputs['out'])

        execution_request = ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.TICK_ONCE)

        response = self.manager.control_execution(execution_request)
        self.assertTrue(get_success(response))
        self.assertEqual(response.tree_state, Tree.WAITING_FOR_TICK)

        self.assertEqual(self.manager.nodes['passthrough'].outputs['out'], 42)

        # Start, then stop, continuous execution
        execution_request.command = ControlTreeExecutionRequest.TICK_PERIODICALLY
        execution_request.tick_frequency_hz = 2
        response = self.manager.control_execution(execution_request)
        self.assertTrue(get_success(response))
        self.assertEqual(response.tree_state, Tree.TICKING)

        # Trying to start ticking while the tree already is ticking should fail
        self.assertFalse(self.manager.control_execution(execution_request).success)
        execution_request.command = ControlTreeExecutionRequest.TICK_ONCE
        self.assertFalse(self.manager.control_execution(execution_request).success)

        # Stopping should put the tree back in the IDLE state
        execution_request.command = ControlTreeExecutionRequest.STOP
        response = self.manager.control_execution(execution_request)
        self.assertTrue(get_success(response))
        self.assertEqual(response.tree_state, Tree.IDLE)

        # stopping a stopped tree is fine
        self.assertTrue(self.manager.control_execution(execution_request).success)

        # After resetting, output should be None again
        execution_request.command = ControlTreeExecutionRequest.RESET
        self.assertIsNotNone(self.manager.nodes['passthrough'].outputs['out'])
        self.assertTrue(self.manager.control_execution(execution_request).success)
        self.assertIsNone(self.manager.nodes['passthrough'].outputs['out'])

        execution_request.command = ControlTreeExecutionRequest.SHUTDOWN
        self.assertTrue(self.manager.control_execution(execution_request).success)
        self.assertEqual(self.manager.nodes['passthrough'].state, NodeMsg.SHUTDOWN)

        # test DO_NOTHING and an unknown command
        execution_request.command = ControlTreeExecutionRequest.DO_NOTHING
        self.assertTrue(self.manager.control_execution(execution_request).success)

        execution_request.command = 42
        self.assertFalse(self.manager.control_execution(execution_request).success)

    def testControlBrokenTree(self):
        add_request = AddNodeRequest(node=self.node_msg,
                                     allow_rename=True)
        # Add two nodes, so there's no one root node
        self.assertTrue(self.manager.add_node(add_request).success)
        self.assertTrue(self.manager.add_node(add_request).success)

        execution_request = ControlTreeExecutionRequest()

        # All of these should fail, since the manager cannot find a root node
        # to tick (or reset)
        execution_request.command = ControlTreeExecutionRequest.TICK_ONCE
        self.assertFalse(get_success(self.manager.control_execution(execution_request)))
        execution_request.command = ControlTreeExecutionRequest.TICK_PERIODICALLY
        self.assertFalse(get_success(self.manager.control_execution(execution_request)))
        execution_request.command = ControlTreeExecutionRequest.RESET
        self.assertFalse(get_success(self.manager.control_execution(execution_request)))

    def testControlTreeWithUnsetInputNode(self):
        load_request = LoadTreeRequest(tree=Tree(
            name='from_file',
            path='package://ros_bt_py/test/testdata/trees/subtree_compare.yaml'))
        self.assertTrue(get_success(self.manager.load_tree(load_request)))

        execution_request = ControlTreeExecutionRequest()
        execution_request.command = ControlTreeExecutionRequest.TICK_ONCE
        self.assertFalse(get_success(self.manager.control_execution(execution_request)))
        self.assertFalse(get_success(self.manager.control_execution(execution_request)))

    def testControlSetupAndShutdown(self):
        add_request = AddNodeRequest(node=self.node_msg,
                                     allow_rename=True)
        self.assertTrue(self.manager.add_node(add_request).success)

        execution_request = ControlTreeExecutionRequest()

        # SETUP_AND_SHUTDOWN does not work when ticking
        execution_request.command = ControlTreeExecutionRequest.TICK_PERIODICALLY
        self.assertTrue(get_success(self.manager.control_execution(execution_request)))

        execution_request.command = ControlTreeExecutionRequest.SETUP_AND_SHUTDOWN
        self.assertFalse(get_success(self.manager.control_execution(execution_request)))

        execution_request.command = ControlTreeExecutionRequest.SHUTDOWN
        self.assertTrue(get_success(self.manager.control_execution(execution_request)))

        execution_request.command = ControlTreeExecutionRequest.SETUP_AND_SHUTDOWN
        self.assertTrue(get_success(self.manager.control_execution(execution_request)))

        # SETUP fails on a TreeTopologyError
        self.assertTrue(self.manager.add_node(add_request).success)

        execution_request.command = ControlTreeExecutionRequest.SETUP_AND_SHUTDOWN
        self.assertFalse(get_success(self.manager.control_execution(execution_request)))

    def testControlSetupAndShutdownFails(self):
        random_int_msg = NodeMsg(
            module='ros_bt_py.nodes.random_number',
            node_class='RandomInt',
            options=[NodeData(key='min',
                              serialized_value=jsonpickle.encode(1)),
                     NodeData(key='max',
                              serialized_value=jsonpickle.encode(0))])
        add_request = AddNodeRequest(node=random_int_msg,
                                     allow_rename=True)
        self.assertTrue(self.manager.add_node(add_request).success)

        execution_request = ControlTreeExecutionRequest()

        # Fails because of the nodes BehaviorTreeException
        execution_request.command = ControlTreeExecutionRequest.SETUP_AND_SHUTDOWN
        self.assertFalse(get_success(self.manager.control_execution(execution_request)))

    def testGetAvailableNodes(self):
        request = GetAvailableNodesRequest(node_modules=['ros_bt_py.nodes.passthrough_node'])

        response = self.manager.get_available_nodes(request)
        self.assertTrue(get_success(response), get_error_message(response))
        self.assertGreaterEqual(len(response.available_nodes), 1)

        self.assertIn("PassthroughNode", [node.node_class for node in response.available_nodes])

    def testSetOptions(self):
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.node_msg))))
        # There's only one node...
        node = self.tree_msg.nodes[0]
        # and it only has one option
        self.assertEqual(node.options[0].serialized_value, jsonpickle.encode(int))

        # a node that is not in the tree should fail
        self.assertFalse(get_success(self.manager.set_options(
            SetOptionsRequest(node_name='not_in_tree',
                              rename_node=False,
                              options=[NodeData(key='passthrough_type',
                                                serialized_value=jsonpickle.encode(str))]))))

        # unparseable values should fail
        self.assertFalse(get_success(self.manager.set_options(
            SetOptionsRequest(node_name='PassthroughNode',
                              rename_node=False,
                              options=[NodeData(key='passthrough_type',
                                                serialized_value='invalid_value')]))))

        # assigning values to invalid keys should fail too
        self.assertFalse(get_success(self.manager.set_options(
            SetOptionsRequest(node_name='PassthroughNode',
                              rename_node=False,
                              options=[NodeData(key='invalid_key',
                                                serialized_value=jsonpickle.encode(str))]))))

        # assigning values of the wrong type should also fail
        self.assertFalse(get_success(self.manager.set_options(
            SetOptionsRequest(node_name='PassthroughNode',
                              rename_node=False,
                              options=[NodeData(key='passthrough_type',
                                                serialized_value=jsonpickle.encode(
                                                    'I am not a type, but a string!'))]))))

        # finally, this is valid :)
        self.assertTrue(get_success(self.manager.set_options(
            SetOptionsRequest(node_name='PassthroughNode',
                              rename_node=False,
                              options=[NodeData(key='passthrough_type',
                                                serialized_value=jsonpickle.encode(str))]))))
        node = self.tree_msg.nodes[0]
        self.assertEqual(node.options[0].serialized_value, jsonpickle.encode(str))

    def testSetSomeOptions(self):
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.constant_msg))))
        # We expect the old value of constant_type (int) to be
        # preserved - if it weren't Node.__init__() would raise an
        # error!
        self.assertTrue(get_success(self.manager.set_options(
            SetOptionsRequest(node_name='Constant',
                              rename_node=False,
                              options=[NodeData(key='constant_value',
                                                serialized_value=jsonpickle.encode(23))]))))

    def testRename(self):
        self.sequence_msg.name = 'foo'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))

        self.constant_msg.name = 'const'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(parent_name=self.sequence_msg.name,
                           node=self.constant_msg))))

        set_options_response = self.manager.set_options(
            SetOptionsRequest(node_name=self.constant_msg.name,
                              rename_node=True,
                              new_name='bar'))

        self.assertTrue(get_success(set_options_response),
                        get_error_message(set_options_response))

        self.assertIn('bar', (node.name for node in self.tree_msg.nodes))
        self.assertNotIn('const', (node.name for node in self.tree_msg.nodes))

        set_options_response = self.manager.set_options(
            SetOptionsRequest(node_name='bar',
                              rename_node=True,
                              new_name='foo'))

        # 'foo' is already taken, so this shouldn't succeed
        self.assertFalse(get_success(set_options_response))

    def testSetOptionsWithWirings(self):
        # Add a Sequence with two children
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))))
        self.node_msg.name = 'child1'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(parent_name='Sequence',
                           node=self.node_msg))))
        self.node_msg.name = 'child2'
        self.assertTrue(get_success(self.manager.add_node(
            AddNodeRequest(parent_name='Sequence',
                           node=self.node_msg))))
        wire_request = WireNodeDataRequest()
        wire_request.wirings.append(NodeDataWiring(
            source=NodeDataLocation(
                node_name='child1',
                data_kind=NodeDataLocation.OUTPUT_DATA,
                data_key='out'),
            target=NodeDataLocation(
                node_name='child2',
                data_kind=NodeDataLocation.INPUT_DATA,
                data_key='in')))

        self.assertTrue(get_success(self.manager.wire_data(wire_request)))

        # Should work - the new value is the same as the old one, so
        # it definitely works
        self.assertTrue(get_success(self.manager.set_options(SetOptionsRequest(
            node_name='child1',
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))]))))

        # Should fail because the wiring cannot be re-established
        # (child1.out is now a str, but child2.in still expects an
        # int)
        failed_res = self.manager.set_options(SetOptionsRequest(
            node_name='child1',
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(str))]))
        self.assertFalse(get_success(failed_res))

        # The failed attempt should reset everything to the way it was
        # before, so this must still work
        retry_res = self.manager.set_options(SetOptionsRequest(
            node_name='child1',
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))]))
        self.assertTrue(get_success(retry_res), get_error_message(retry_res))

    def testEnforceEditable(self):
        add_request = AddNodeRequest(node=self.node_msg)
        add_request.node.name = 'first'

        self.assertEqual(self.tree_msg.state, "EDITABLE")
        self.assertTrue(get_success(self.manager.add_node(add_request)))

        self.assertTrue(get_success(self.manager.control_execution(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.TICK_ONCE))))

        add_request.node.name = 'second'
        # The tree is not editable after ticking once
        self.assertNotEqual(self.tree_msg.state, "EDITABLE")
        # Neither by adding...
        self.assertFalse(get_success(self.manager.add_node(add_request)))
        # Nor deleting a node
        self.assertFalse(get_success(self.manager.remove_node(
            RemoveNodeRequest(node_name='first',
                              remove_children=False))))
        self.assertFalse(get_success(self.manager.set_options(
            SetOptionsRequest(node_name='first', options=[]))))
        # TODO(nberg): test other editing services here as they're implemented

        # But after shutting it down, we can edit it again
        self.assertTrue(get_success(self.manager.control_execution(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.SHUTDOWN))))

        self.assertEqual(self.tree_msg.state, "EDITABLE")
        self.assertTrue(get_success(self.manager.add_node(add_request)))
        self.assertTrue(get_success(self.manager.remove_node(
            RemoveNodeRequest(node_name='first',
                              remove_children=False))))

    def testLoadTreeFromPath(self):
        load_request = LoadTreeFromPathRequest(
            path='package://ros_bt_py/test/testdata/trees/subtree_constant.yaml')
        self.assertTrue(get_success(self.manager.load_tree_from_path(load_request)))

    def testLoadFromInvalidFiles(self):
        load_request = LoadTreeRequest(tree=Tree(
            name='from_file',
            path='/notareal.file'))
        self.assertFalse(get_success(self.manager.load_tree(load_request)))

        load_request = LoadTreeRequest(tree=Tree(
            name='from_file',
            path='file://'))
        self.assertFalse(get_success(self.manager.load_tree(load_request)))

        load_request = LoadTreeRequest(tree=Tree(
            name='from_file',
            path='package://ros_bt_py/etc/trees/notareal.file'))
        self.assertFalse(get_success(self.manager.load_tree(load_request)))

        load_request = LoadTreeRequest(tree=Tree(
            name='from_file',
            path='package://ros_bt_py/etc/trees/two_trees.yaml'))
        self.assertFalse(get_success(self.manager.load_tree(load_request)))

        load_request = LoadTreeRequest(tree=Tree(
            name='from_file',
            path='package://ros_bt_py/etc/trees/empty.yaml'))
        self.assertFalse(get_success(self.manager.load_tree(load_request)))

        load_request = LoadTreeRequest(tree=Tree(
            name='from_file',
            path='package://ros_bt_py/test/testdata/trees/broken_node_with_child.yaml'))
        self.assertFalse(get_success(self.manager.load_tree(load_request)))

        load_request = LoadTreeRequest(tree=Tree(
            name='from_file',
            path='package://ros_bt_py/test/testdata/trees/broken_node_with_missing_child.yaml'))
        self.assertFalse(get_success(self.manager.load_tree(load_request)))

        load_request = LoadTreeRequest(tree=Tree(
            name='from_file',
            path='package://ros_bt_py/test/testdata/trees/broken_wiring.yaml'))
        self.assertFalse(get_success(self.manager.load_tree(load_request)))

    def testLoadFromValidFile(self):
        load_request = LoadTreeRequest(tree=Tree(name='from_file',
                                                 path='package://ros_bt_py/etc/trees/test.yaml'))
        response = self.manager.load_tree(load_request)

        self.assertTrue(get_success(response), get_error_message(response))
        # test.yaml contains a sequence, two succeeders, a fallback and a failer
        self.assertEqual(len(self.manager.nodes), 5)

        self.assertTrue(get_success(self.manager.control_execution(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.TICK_ONCE))))

    def testLoadFromValidFileWithEmptyObject(self):
        """Load a tree from a rostopic echo file that has "---" at the end"""
        load_request = LoadTreeRequest(
            tree=Tree(name='from_file',
                      path='package://ros_bt_py/etc/trees/test_extra_empty.yaml'))
        response = self.manager.load_tree(load_request)

        self.assertTrue(get_success(response), get_error_message(response))
        # test.yaml contains a sequence, two succeeders, a fallback and a failer
        self.assertEqual(len(self.manager.nodes), 5)

        self.assertTrue(get_success(self.manager.control_execution(ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.TICK_ONCE))))

    def testLoadWithoutNodesAndWithoutPath(self):
        request = self.manager.load_tree(
            LoadTreeRequest(tree=Tree(name='broken')))
        self.assertFalse(get_success(request))

    def testLoadFromFileWithIndirection(self):
        request = self.manager.load_tree(
            LoadTreeRequest(tree=Tree(name='from_file',
                                      path='package://ros_bt_py/etc/trees/indirection.yaml')))
        # Indirection should work as well (this yaml file refers to test.yaml)
        self.assertTrue(get_success(request), get_error_message(request))

    def testLoadSubtree(self):
        load_request = LoadTreeRequest(tree=Tree(name='from_file',
                                                 path='package://ros_bt_py/etc/trees/test.yaml'))
        response = self.manager.load_tree(load_request)
        self.assertTrue(get_success(response), get_error_message(response))

        # Fallback is an inner node with 2 children
        fallback = self.manager.find_root().find_node('fallback')
        self.assertIsNotNone(fallback)

        subtree, _, _ = fallback.get_subtree_msg()

        # Now load the subtree
        load_request = LoadTreeRequest(tree=subtree)
        response = self.manager.load_tree(load_request)
        self.assertTrue(get_success(response), get_error_message(response))

    def testSetExecutionMode(self):
        request = SetExecutionModeRequest(single_step=False,
                                          collect_performance_data=False, publish_subtrees=True)
        self.assertEqual(self.manager.set_execution_mode(request), SetExecutionModeResponse())
        self.assertEqual(self.manager.get_state(), Tree.EDITABLE)

        request = SetExecutionModeRequest(single_step=False,
                                          collect_performance_data=False, publish_subtrees=False)
        self.assertEqual(self.manager.set_execution_mode(request), SetExecutionModeResponse())

    def testDebugStep(self):
        request = ContinueRequest()
        self.assertTrue(self.manager.debug_step(request).success)

    def testModifyBreakpoints(self):
        breakpoints = ["first", "second", "third", "fourth"]
        request = ModifyBreakpointsRequest(add=breakpoints)
        self.assertEqual(self.manager.modify_breakpoints(request).current_breakpoints,
                         breakpoints)


class TestWiringServices(unittest.TestCase):
    def setUp(self):
        self.tree_msg = None
        self.debug_info_msg = None

        def set_tree_msg(msg):
            self.tree_msg = msg

        def set_debug_info_msg(msg):
            self.debug_info_msg = msg

        self.manager = TreeManager(publish_tree_callback=set_tree_msg,
                                   publish_debug_info_callback=set_debug_info_msg)
        node_msg = NodeMsg(
            module='ros_bt_py.nodes.passthrough_node',
            node_class='PassthroughNode',
            inputs=[NodeData(key='in',
                             serialized_value=jsonpickle.encode(42))],
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])
        self.sequence_msg = NodeMsg(
            module='ros_bt_py.nodes.sequence',
            node_class='Sequence')
        response = self.manager.add_node(
            AddNodeRequest(node=self.sequence_msg))
        # Add three passthrough nodes that we can wire between
        self.node_1_name = 'passthrough_1'
        node_msg.name = self.node_1_name
        self.manager.add_node(
            AddNodeRequest(parent_name=response.actual_node_name,
                           node=node_msg))
        self.node_2_name = 'passthrough_2'
        node_msg.name = self.node_2_name
        self.manager.add_node(
            AddNodeRequest(parent_name=response.actual_node_name,
                           node=node_msg))
        self.node_3_name = 'passthrough_3'
        node_msg.name = self.node_3_name
        self.manager.add_node(
            AddNodeRequest(parent_name=response.actual_node_name,
                           node=node_msg))

    def wiring(self, from_name, to_name):
        return NodeDataWiring(
            source=NodeDataLocation(node_name=from_name,
                                    data_key='out',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name=to_name,
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA))

    def testWireMultiple(self):
        wire_request = WireNodeDataRequest()
        wire_request.wirings.append(self.wiring(self.node_1_name, self.node_2_name))
        wire_request.wirings.append(self.wiring(self.node_2_name, self.node_3_name))

        response = self.manager.wire_data(wire_request)
        self.assertTrue(get_success(response), get_error_message(response))
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 2)

        response = self.manager.unwire_data(wire_request)
        self.assertTrue(get_success(response), get_error_message(response))
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 0)

    def testUndoWiringOnError(self):
        wire_request = WireNodeDataRequest()
        wire_request.wirings.append(self.wiring(self.node_1_name, self.node_2_name))
        wire_request.wirings.append(
            NodeDataWiring(
                source=NodeDataLocation(node_name=self.node_2_name,
                                        data_key='fake',
                                        data_kind=NodeDataLocation.OUTPUT_DATA),
                target=NodeDataLocation(node_name=self.node_3_name,
                                        data_key='invalid',
                                        data_kind=NodeDataLocation.INPUT_DATA)))

        response = self.manager.wire_data(wire_request)
        self.assertFalse(get_success(response))
        # Even though the first wiring was valid, it should be undone if
        # another in the same request is invalid
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 0)

    def testRewireAfterUnwire(self):
        wire_request = WireNodeDataRequest()
        wire_request.wirings.append(self.wiring(self.node_1_name, self.node_2_name))

        response = self.manager.wire_data(wire_request)
        self.assertTrue(get_success(response), get_error_message(response))
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 1)

        response = self.manager.unwire_data(wire_request)
        self.assertTrue(get_success(response), get_error_message(response))
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 0)

        response = self.manager.wire_data(wire_request)
        self.assertTrue(get_success(response), get_error_message(response))
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 1)

    def testRedoWiringOnError(self):
        wire_request = WireNodeDataRequest()
        wire_request.wirings.append(self.wiring(self.node_1_name, self.node_2_name))
        wire_request.wirings.append(self.wiring(self.node_2_name, self.node_3_name))

        unwire_request = WireNodeDataRequest()
        unwire_request.wirings.append(self.wiring(self.node_1_name, self.node_2_name))
        unwire_request.wirings.append(
            NodeDataWiring(
                source=NodeDataLocation(node_name=self.node_2_name,
                                        data_key='fake',
                                        data_kind=NodeDataLocation.OUTPUT_DATA),
                target=NodeDataLocation(node_name=self.node_3_name,
                                        data_key='invalid',
                                        data_kind=NodeDataLocation.INPUT_DATA)))

        wire_response = self.manager.wire_data(wire_request)
        self.assertTrue(get_success(wire_response), get_error_message(wire_response))
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 2)

        # Number of wirings should stay the same, since the unwire operation failed
        unwire_response = self.manager.wire_data(unwire_request)
        self.assertFalse(get_success(unwire_response))
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 2)


def get_success(response):
    if isinstance(response, dict):
        return response['success']

    return response.success


def get_error_message(response):
    if isinstance(response, dict):
        return response['error_message']

    return response.error_message
