import importlib
import re

import jsonpickle

import rospy

from ros_bt_py_msgs.srv import WireNodeDataResponse
from ros_bt_py_msgs.msg import NodeDataLocation, Tree

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Node, load_node_module
from ros_bt_py.debug_manager import DebugManager


class TreeManager(object):
    """Provide methods to manage a Behavior Tree

     These methods are suited (intended, even) for use as ROS service handlers.
    """

    def __init__(self, name=None, module_list=None, debug_manager=None):
        self.debug_manager = debug_manager
        if not self.debug_manager:
            rospy.loginfo('Tree manager instantiated without explicit debug manager '
                          '- building our own with default parameters')
            self.debug_manager = DebugManager()
        self.nodes = {}
        self.tree_msg = Tree()
        self.tree_msg.name = name if name else ''
        self.tree_msg.state = Tree.IDLE

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
        """Connect the given pairs of node data to one another.

        :param ros_bt_py_msgs.srv.WireNodeDataRequest request:

        Contains a list of :class: ros_bt_py_msgs.msg.NodeDataWiring
        objects that model connections

        :returns: :class:ros_bt_py_msgs.src.WireNodeDataResponse or `None`
        """
        response = WireNodeDataResponse(success=True)
        # TODO(nberg): Check request.tree_name to see if the request concerns
        # this tree or a subtree.
        try:
            subscription_data = self._extract_subscription_data(request)
        except BehaviorTreeException as exc:
            response.success = False
            response.error_message = str(exc)

        # only actually wire any data if there were no errors
        if response.success:
            for source_map, key, callback, name in subscription_data:
                source_map.subscribe(key, callback, name)

        # We made it here, so all the Wirings should be valid. Time to save
        # them.
        self.tree_msg.data_wirings.extend(request.wirings)

        return response

    def unwire_data(self, request):
        """Disconnect the given pairs of node data.

        :param ros_bt_py_msgs.srv.WireNodeDataRequest request:

        Contains a list of :class: ros_bt_py_msgs.msg.NodeDataWiring
        objects that model connections

        :returns: :class:ros_bt_py_msgs.src.WireNodeDataResponse or `None`
        """
        response = WireNodeDataResponse(success=True)
        # TODO(nberg): Check request.tree_name to see if the request concerns
        # this tree or a subtree.
        try:
            subscription_data = self._extract_subscription_data(request)
        except BehaviorTreeException as exc:
            response.success = False
            response.error_message = str(exc)

        # only actually unsubscribe any callbacks if there were no errors
        if response.success:
            for source_map, key, callback, _ in subscription_data:
                source_map.unsubscribe(key, callback)

        # We've removed these NodeDataWirings, so remove them from tree_msg as
        # well.
        for wiring in request.wirings:
            if wiring in self.tree_msg.data_wirings:
                self.tree_msg.data_wirings.remove(wiring)

        return response

    def _extract_subscription_data(self, request):
        """Extract the data necessary to act on a `WireNodeDataRequest`

        :returns: A list of tuples of the form
            `(source_map, source_data_key, callback, callback_name)`
        """
        subscription_data = []
        for wiring in request.wirings:
            if wiring.source.node_name not in self.nodes:
                raise BehaviorTreeException('Unable to find node %s in tree %s' % (
                    wiring.source.node_name, request.tree_name))
            if wiring.target.node_name not in self.nodes:
                raise BehaviorTreeException('Unable to find node %s in tree %s' % (
                    wiring.target.node_name, request.tree_name))
            source_node = self.nodes[wiring.source.node_name]
            target_node = self.nodes[wiring.target.node_name]
            try:
                source_map = source_node.get_data_map(wiring.source.data_kind)
                target_map = target_node.get_data_map(wiring.target.data_kind)
            except KeyError as exception:
                raise BehaviorTreeException(str(exception))
            if wiring.source.data_key not in source_map:
                raise BehaviorTreeException('Source key %s.%s[%s] does not exist!' % (
                    source_node.name,
                    wiring.source.data_kind,
                    wiring.source.data_key))
            if wiring.target.data_key not in target_map:
                raise BehaviorTreeException('Target key %s.%s[%s] does not exist!' % (
                    target_node.name,
                    wiring.target.data_kind,
                    wiring.target.data_key))
            if not issubclass(source_map.get_type(wiring.source.data_key),
                              target_map.get_type(wiring.target.data_key)):
                raise BehaviorTreeException((
                    'Type of %s.%s[%s] (%s) is not compatible with '
                    'Type of %s.%s[%s] (%s)!' % (
                        source_node.name,
                        wiring.source.data_kind,
                        wiring.source.data_key,
                        source_map.get_type(wiring.source.data_key).__name__,
                        target_node.name,
                        wiring.target.data_kind,
                        wiring.target.data_key,
                        target_map.get_type(wiring.target.data_key).__name__)))
            subscription_data.append((source_map,
                                      wiring.source.data_key,
                                      target_map.get_callback(wiring.target.data_key),
                                      '%s.%s[%s]' % (target_node.name,
                                                     wiring.target.data_kind,
                                                     wiring.target.data_key)))

        return subscription_data

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

    def to_msg(self):
        tree_msg = Tree(tree_name=self.name,
                        nodes=[node.to_msg() for node in self.nodes.values()],
                            tree_state=self.state)
        return tree_msg
