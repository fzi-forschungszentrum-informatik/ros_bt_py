import importlib
import re

import jsonpickle

import rospy

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

    def wire_data(self, source_node_name, source_node):
        pass

    def instantiate_node_from_msg(self, node_msg):
        # TODO(nberg): Subtree handling
        if node_msg.is_subtree:
            raise NotImplementedError('Subtree nodes are not supported yet!')
        else:
            node_instance = Node.from_msg(node_msg)
            # Set DebugManager
            node_instance.debug_manager = self.debug_manager
            # Ensure that name is unique
            while node_instance.name in self.nodes:
                node_instance.name = increment_name(node_instance.name)

            self.nodes[node_instance.name] = node_instance

            return node_instance




def increment_name(name):
    """If `name` does not already end in a number, add "_2" to it.

    Otherwise, increase the number after the underscore.
    """
    match = re.search('_([0-9]+)$', name)
    prev_number = 1
    if match:
        prev_number = int(match.group(1))
        # remove the entire _$number part from the name
        name = name[:len(name) - len(match.group(0))]

    name += '_%d' % (prev_number + 1)
    return name
