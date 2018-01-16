import importlib

import rospy


class TreeManager(object):
    """Provide methods to manage a Behavior Tree

     These methods are suited (intended, even) for use as ROS service handlers.
    """

    def __init__(self, package_list=None):
        self.available_nodes = self.get_nodes_from_package_names(package_list)

    def get_nodes_from_package_names(self, package_list):
        if not package_list:
            return []
        return []

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

    def wire_data(self, source_node_name, source_node
