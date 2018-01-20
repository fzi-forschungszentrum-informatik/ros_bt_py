import unittest

import jsonpickle

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData, NodeDataWiring, NodeDataLocation
from ros_bt_py_msgs.srv import WireNodeDataRequest

from ros_bt_py.node import Node
from ros_bt_py.tree_manager import TreeManager


class TestTreeManager(unittest.TestCase):
    def setUp(self):
        self.manager = TreeManager()

    def testLoadNode(self):
        msg = NodeMsg(
            is_subtree=False,
            module='ros_bt_py.nodes.passthrough_node',
            node_class='PassthroughNode',
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])

        _ = self.manager.instantiate_node_from_msg(msg)
        _ = self.manager.instantiate_node_from_msg(msg)
        msg.name = 'Test Node'
        _ = self.manager.instantiate_node_from_msg(msg)

        self.assertIn('PassthroughNode', self.manager.nodes)
        self.assertIn('PassthroughNode_2', self.manager.nodes)
        self.assertIn('Test Node', self.manager.nodes)

    def testWireData(self):
        node_msg = NodeMsg(
            is_subtree=False,
            name='source_node',
            module='ros_bt_py.nodes.passthrough_node',
            node_class='PassthroughNode',
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])

        self.manager.instantiate_node_from_msg(node_msg)
        node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(node_msg)

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

    def testWireWithInvalidKey(self):
        node_msg = NodeMsg(
            is_subtree=False,
            name='source_node',
            module='ros_bt_py.nodes.passthrough_node',
            node_class='PassthroughNode',
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])

        self.manager.instantiate_node_from_msg(node_msg)
        node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(node_msg)

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
        node_msg = NodeMsg(
            is_subtree=False,
            name='source_node',
            module='ros_bt_py.nodes.passthrough_node',
            node_class='PassthroughNode',
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])

        self.manager.instantiate_node_from_msg(node_msg)
        node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(node_msg)

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
        node_msg = NodeMsg(
            is_subtree=False,
            name='source_node',
            module='ros_bt_py.nodes.passthrough_node',
            node_class='PassthroughNode',
            options=[NodeData(key='passthrough_type',
                              serialized_value=jsonpickle.encode(int))])

        self.manager.instantiate_node_from_msg(node_msg)
        node_msg.name = 'target_node'
        self.manager.instantiate_node_from_msg(node_msg)

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
