import importlib
import re

import jsonpickle

import rospy

from ros_bt_py_msgs.srv import WireNodeDataResponse
from ros_bt_py_msgs.msg import NodeDataLocation

from ros_bt_py.node import Node
from ros_bt_py.debug_manager import DebugManager


class TreeManager(object):
    """Provide methods to manage a Behavior Tree

     These methods are suited (intended, even) for use as ROS service handlers.
    """

    def __init__(self, module_list=None, debug_manager=None):
        self.debug_manager = debug_manager
        if not self.debug_manager:
            rospy.loginfo('Tree manager instantiated without explicit debug manager '
                          '- building our own with default parameters')
            self.debug_manager = DebugManager()
        self.nodes = {}

        # Skip if module_list is empty or None
        if module_list:
            for module_name in module_list:
                load_node_module(module_name)

    def load_tree(self, tree_path):
        pass

    def load_tree_from_string(self, tree_string):
        # fill initial list of nodes with all the nodes that have no parent

        # for each node in list

        # add its children to list

        # reverse list

        # create nodes, adding children as needed
        pass

    def add_node(self, parent_name, child_node):
        # return the name the child was added under
        pass

    def insert_node(self, parent_name, previous_child_name, child_node):
        """Insert `child_node` in between two nodes

        The two nodes are identified by `parent_name` and
        `previous_child_name`
        """
        pass

    def remove_node(self, node_name, remove_children):
        """Remove the node identified by `node_name` from the tree.

        If the parent of the node removed supports enough children to
        take on all of the removed node's children, it will. Otherwise,
        children will be orphaned.
        """
        pass

    def move_child(self, node_name, child_name, new_index):
        """Move the named child of a node to the given index.

        Will fail if `new_index` is invalid.

        For moving a node to a different parent, see
        :meth:TreeManager.move_node
        """
        pass

    def move_node(self, node_name, new_parent_name, child_index=None):
        """Move the named node to a different parent and insert it at the given index.

        """
        pass

    def replace_node(self, old_node_name, new_node):
        """Replace the named node with `new_node`.

        Will also move all children of the old node to the new one, but
        only if `new_node` supports that number of children. Otherwise,
        this will return an error.
        """

    def wire_data(self, request):
        response = WireNodeDataResponse(success=True)
        # TODO(nberg): Check request.tree_name to see if the request concerns
        # this tree or a subtree.
        subscription_data = []
        for wiring in request.wirings:
            if wiring.source.node_name not in self.nodes:
                response.success = False
                response.error_message += 'Unable to find node %s in tree %s' % (
                    wiring.source.node_name, request.tree_name)
                break
            if wiring.target.node_name not in self.nodes:
                response.success = False
                response.error_message += 'Unable to find node %s in tree %s' % (
                    wiring.target.node_name, request.tree_name)
                break
            source_node = self.nodes[wiring.source.node_name]
            target_node = self.nodes[wiring.target.node_name]
            try:
                source_map = source_node.get_data_map(wiring.source.data_kind)
                target_map = target_node.get_data_map(wiring.target.data_kind)
            except KeyError as exception:
                response.success = False
                response.error_message = str(exception)
                break
            if not wiring.source.data_key in source_map:
                response.success = False
                response.error_message = 'Source key %s.%s[%s] does not exist!' % (
                    source_node.name,
                    wiring.source.data_kind,
                    wiring.source.data_key)
                break
            if not wiring.target.data_key in target_map:
                response.success = False
                response.error_message = 'Target key %s.%s[%s] does not exist!' % (
                    target_node.name,
                    wiring.target.data_kind,
                    wiring.target.data_key)
                break
            if not issubclass(source_map.get_type(wiring.source.data_key),
                              target_map.get_type(wiring.target.data_key)):
                response.success = False
                response.error_message = (
                    'Type of %s.%s[%s] (%s) is not compatible with '
                    'Type of %s.%s[%s] (%s)!' % (
                        source_node.name,
                        wiring.source.data_kind,
                        wiring.source.data_key,
                        source_map.get_type(wiring.source.data_key).__name__,
                        target_node.name,
                        wiring.target.data_kind,
                        wiring.target.data_key,
                        target_map.get_type(wiring.target.data_key).__name__))
                break
            subscription_data.append((source_map,
                                      wiring.source.data_key,
                                      target_map.get_callback(wiring.target.data_key),
                                      target_node.name))

        # only actually wire any data if there were no errors
        if response.success:
            for source_map, key, callback, name in subscription_data:
                source_map.subscribe(key, callback, name)

        return response

    def instantiate_node_from_msg(self, node_msg):
        # TODO(nberg): Subtree handling
        if node_msg.is_subtree:
            raise NotImplementedError('Subtree nodes are not supported yet!')
        else:
            node_instance = Node.from_msg(node_msg, self.nodes)
            # Set DebugManager
            node_instance.debug_manager = self.debug_manager

            self.nodes[node_instance.name] = node_instance

            return node_instance
