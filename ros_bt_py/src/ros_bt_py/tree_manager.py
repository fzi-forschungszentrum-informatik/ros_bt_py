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
            if (node_msg.module not in Node.node_classes or
                    node_msg.node_class not in Node.node_classes[node_msg.module]):
                # If the node class was not available, try to load it
                load_node_module(node_msg.module)

            # If loading didn't work, abort
            if (node_msg.module not in Node.node_classes or
                    node_msg.node_class not in Node.node_classes[node_msg.module]):
                rospy.logerr('Failed to instantiate node from message - node class '
                             'not available. Original message:\n%s', str(node_msg))
                return None

            node_class = Node.node_classes[node_msg.module][node_msg.node_class]

            # Got the class, now ensure the name is unique
            while node_msg.name in self.nodes:
                node_msg.name = increment_name(node_msg.name)

            # Populate options dict
            options_dict = {}
            for option in node_msg.options:
                options_dict[option.key] = jsonpickle.decode(option.value_serialized)

            # Instantiate node - this shouldn't do anything yet, since we don't
            # call setup()
            node_instance = node_class(options=options_dict,
                                       debug_manager=self.debug_manager)
            self.nodes[node_msg.name] = node_instance

            # Set inputs
            for input_msg in node_msg.current_inputs:
                node_instance.inputs[input_msg.key] = jsonpickle.decode(
                    input_msg.value_serialized)

            # Set outputs
            for output_msg in node_msg.current_outputs:
                node_instance.outputs[output_msg.key] = jsonpickle.decode(
                    output_msg.value_serialized)

            return node_instance


def load_node_module(package_name):
    """Import the named module at run-time.

    If the module contains any (properly decorated) node classes,
    they will be registered and available to load via the other
    commands in this class.
    """
    try:
        return importlib.import_module(package_name)
    except ImportError:
        return None


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
