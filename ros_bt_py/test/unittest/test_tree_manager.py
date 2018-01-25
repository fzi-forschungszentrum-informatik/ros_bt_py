import unittest

import jsonpickle

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData, NodeDataWiring, NodeDataLocation
from ros_bt_py_msgs.srv import WireNodeDataRequest, AddNodeRequest, RemoveNodeRequest

from ros_bt_py.node import Node
from ros_bt_py.tree_manager import TreeManager


class TestTreeManager(unittest.TestCase):
    def setUp(self):
        self.tree_msg = None

        def set_tree_msg(msg):
            self.tree_msg = msg
        self.debug_info_msg = None

        def set_debug_info_msg(msg):
            self.debug_info_msg = msg

        self.manager = TreeManager(publish_tree_callback=set_tree_msg,
                                   publish_debug_info_callback=set_debug_info_msg)
        self.node_msg = NodeMsg(
            is_subtree=False,
            module='ros_bt_py.nodes.passthrough_node',
            node_class='PassthroughNode',
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])

    def testLoadNode(self):
        _ = self.manager.instantiate_node_from_msg(self.node_msg)
        _ = self.manager.instantiate_node_from_msg(self.node_msg)
        self.node_msg.name = 'Test Node'
        _ = self.manager.instantiate_node_from_msg(self.node_msg)

        self.assertIn('PassthroughNode', self.manager.nodes)
        self.assertIn('PassthroughNode_2', self.manager.nodes)
        self.assertIn('Test Node', self.manager.nodes)

    def testWireData(self):
        self.node_msg.name = 'source_node'
        self.manager.instantiate_node_from_msg(self.node_msg)
        self.node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(self.node_msg)

        self.assertIn('source_node', self.manager.nodes)
        self.assertIn('target_node', self.manager.nodes)

        valid_request = WireNodeDataRequest(tree_name='')
        valid_request.wirings.append(NodeDataWiring(
            source=NodeDataLocation(node_name='source_node',
                                    data_key='out',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name='target_node',
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA)))

        response = self.manager.wire_data(valid_request)
        self.assertTrue(response.success)
        self.assertEqual(len(self.manager.nodes['source_node'].outputs.callbacks), 1)
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 1)

    def testWireWithInvalidKey(self):
        self.node_msg.name = 'source_node'
        self.manager.instantiate_node_from_msg(self.node_msg)
        self.node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(self.node_msg)

        self.assertIn('source_node', self.manager.nodes)
        self.assertIn('target_node', self.manager.nodes)

        invalid_key_request = WireNodeDataRequest(tree_name='')
        invalid_key_request.wirings.append(NodeDataWiring(
            source=NodeDataLocation(node_name='source_node',
                                    # PassthroghNode does not have this key!
                                    data_key='wrong',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name='target_node',
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA)))

        response = self.manager.wire_data(invalid_key_request)
        self.assertFalse(response.success)

    def testWireWithInvalidNodeName(self):
        self.node_msg.name = 'source_node'
        self.manager.instantiate_node_from_msg(self.node_msg)
        self.node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(self.node_msg)

        self.assertIn('source_node', self.manager.nodes)
        self.assertIn('target_node', self.manager.nodes)

        invalid_node_request = WireNodeDataRequest(tree_name='')
        invalid_node_request.wirings.append(NodeDataWiring(
            # Wrong node name for source node
            source=NodeDataLocation(node_name='fantasy_node',
                                    data_key='out',
                                    data_kind=NodeDataLocation.OUTPUT_DATA),
            target=NodeDataLocation(node_name='target_node',
                                    data_key='in',
                                    data_kind=NodeDataLocation.INPUT_DATA)))

        response = self.manager.wire_data(invalid_node_request)
        self.assertFalse(response.success)

    def testMultiWireWithOneInvalid(self):
        """WireNodeData supports wiring multiple pairs of NodeData at once.

        If there's an error while handling any pair, none of the wirings must be applied!
        """
        self.node_msg.name = 'source_node'
        self.manager.instantiate_node_from_msg(self.node_msg)
        self.node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(self.node_msg)

        self.assertIn('source_node', self.manager.nodes)
        self.assertIn('target_node', self.manager.nodes)

        invalid_multi_request = WireNodeDataRequest(tree_name='')
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
        self.assertFalse(response.success)
        # The first half should not have been applied -> no callbacks for
        # source_node
        self.assertEqual(len(self.manager.nodes['source_node'].outputs.callbacks), 0)

    def testUnwire(self):
        wire_request = WireNodeDataRequest(tree_name='')
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
        self.assertFalse(response.success)

        self.node_msg.name = 'source_node'
        self.manager.instantiate_node_from_msg(self.node_msg)
        self.node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(self.node_msg)

        response = self.manager.unwire_data(wire_request)
        # The nodes and keys exist. There aren't any callbacks to remove, but
        # the unwire operation still succeeds (after running it, the two data
        # values are unconnected).
        self.assertTrue(response.success)

        response = self.manager.wire_data(wire_request)
        self.assertTrue(response.success)
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 1)

        response = self.manager.unwire_data(wire_request)
        self.assertTrue(response.success)
        self.assertEqual(len(self.manager.tree_msg.data_wirings), 0)

    def testAddNode(self):
        add_request = AddNodeRequest(tree_name='',
                                     node=self.node_msg)

        response = self.manager.add_node(add_request)

        self.assertEqual(len(self.manager.nodes), 1)
        self.assertTrue(response.success)

        broken_add = AddNodeRequest(tree_name='',
                                    node=NodeMsg(module='asdf',
                                                 node_class='foo'))

        response = self.manager.add_node(broken_add)
        self.assertFalse(response.success)

    def testRemoveNode(self):
        instance = self.manager.instantiate_node_from_msg(self.node_msg)

        remove_request = RemoveNodeRequest(node_name=instance.name)

        response = self.manager.remove_node(remove_request)
        self.assertTrue(response.success)

        # Second remove will fail, there's nothing left to remove.
        response = self.manager.remove_node(remove_request)
        self.assertFalse(response.success)

    def testTick(self):
        add_request = AddNodeRequest(tree_name='',
                                     node=self.node_msg)
        add_request.node.current_inputs.append(NodeData(key='in',
                                                        serialized_value=jsonpickle.encode(42)))

        response = self.manager.add_node(add_request)

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
        self.assertEqual(jsonpickle.decode(node_msg.current_inputs[0].serialized_value), 42)
        self.assertEqual(jsonpickle.decode(node_msg.current_outputs[0].serialized_value), 42)
