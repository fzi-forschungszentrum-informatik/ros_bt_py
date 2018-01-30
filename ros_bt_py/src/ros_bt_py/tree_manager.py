from threading import Thread, Lock
import time

import rospy

from ros_bt_py_msgs.srv import RemoveNodeRequest
from ros_bt_py_msgs.srv import WireNodeDataResponse, AddNodeResponse, RemoveNodeResponse
from ros_bt_py_msgs.srv import ControlTreeExecutionRequest, ControlTreeExecutionResponse
from ros_bt_py_msgs.msg import Tree
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Node, load_node_module
from ros_bt_py.debug_manager import DebugManager


class TreeManager(object):
    """Provide methods to manage a Behavior Tree

     These methods are suited (intended, even) for use as ROS service handlers.
    """

    def __init__(self,
                 name=None,
                 module_list=None,
                 debug_manager=None,
                 tick_frequency_hz=20.0,
                 publish_tree_callback=None,
                 publish_debug_info_callback=None):
        self.debug_manager = debug_manager
        if not self.debug_manager:
            rospy.loginfo('Tree manager instantiated without explicit debug manager '
                          '- building our own with default parameters')
            self.debug_manager = DebugManager()

        self.debug_manager.set_tree_name(name)
        self.debug_manager.set_tick_frequency(tick_frequency_hz)
        self.debug_manager.publish_debug_info = self.publish_info

        self.publish_tree = publish_tree_callback
        if self.publish_tree is None:
            rospy.loginfo('No callback for publishing tree data provided.')
        self.publish_debug_info = publish_debug_info_callback
        if self.publish_debug_info is None:
            rospy.loginfo('No callback for publishing debug data provided.')

        self.nodes = {}

        self._state_lock = Lock()
        self._once = False

        self.tree_msg = Tree()
        self.tree_msg.name = name if name else ''
        self.tree_msg.tick_frequency_hz = tick_frequency_hz

        with self._state_lock:
            self.tree_msg.state = Tree.IDLE

        self._tick_thread = None

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

    def publish_info(self, debug_info_msg=None):
        if self.publish_tree:
            self.publish_tree(self.to_msg())
        if debug_info_msg and self.publish_debug_info:
            self.publish_debug_info(debug_info_msg)

    def find_root(self):
        """Find the root node of the tree

        :raises: `BehaviorTreeException` if either no root or multiple roots are found
        """
        # Find root node
        possible_roots = [node for node in self.nodes.itervalues() if node.parent_name == '']

        if len(possible_roots) > 1:
            raise BehaviorTreeException('Tree "%s" has multiple nodes without parents. '
                                        'Cannot tick!' % self.tree_msg.name)
        if not possible_roots:
            raise BehaviorTreeException('All nodes in tree "%s" have parents. You have '
                                        'made a cycle, which makes the tree impossible to run!' %
                                        self.tree_msg.name)
        return possible_roots[0]

    def tick(self, once=None):
        if once is not None:
            self._once = once

        root = self.find_root()
        if root.state == NodeMsg.UNINITIALIZED:
            root.setup()
        sleep_duration_sec = (1.0/self.tree_msg.tick_frequency_hz)

        while True:
            start_time = time.time()
            with self._state_lock:
                if self.tree_msg.state == Tree.STOP_REQUESTED:
                    break
            root.tick()
            self.publish_info(self.debug_manager.get_debug_info_msg())
            if self._once:
                self._once = False
                break
            tick_duration = time.time() - start_time
            if tick_duration <= 0:
                rospy.logwarn('Tick took longer than set period, cannot tick at %f.2 Hz',
                              self.tree_msg.tick_frequency_hz)
            else:
                rospy.sleep(sleep_duration_sec - tick_duration)

        with self._state_lock:
            self.tree_msg.state = Tree.IDLE

    def find_nodes_in_cycles(self):
        """Return a list of all nodes in the tree that are part of cycles."""
        safe_node_names = []
        nodes_in_cycles = []
        # Follow the chain of parent nodes for each node name in self.nodes
        for starting_name in self.nodes:
            cycle_candidates = [starting_name]
            current_node = self.nodes[starting_name]
            while current_node.parent_name != '':
                current_node = self.nodes[current_node.parent_name]
                cycle_candidates.append(current_node.name)
                if current_node.parent_name == starting_name:
                    nodes_in_cycles.extend(cycle_candidates)
                    break
                if current_node.parent_name in safe_node_names:
                    # We've already checked for cycles from the parent node, no
                    # need to do that again.
                    safe_node_names.extend(cycle_candidates)
                    break
            if current_node.parent_name == '':
                safe_node_names.extend(cycle_candidates)

        return nodes_in_cycles

    def control_execution(self, request):
        """Control tree execution.

        :param ros_bt_py_msgs.srv.ControlTreeExecutionRequest request:

        Can request a tick, periodic ticking, or stopping.
        """
        response = ControlTreeExecutionResponse(success=False)
        if self._tick_thread is not None:
            with self._state_lock:
                is_idle = self.tree_msg.state == Tree.IDLE
            if is_idle and self._tick_thread.is_alive():
                self._tick_thread.join(0.5)
                if self._tick_thread.is_alive():
                    raise BehaviorTreeException('Tried to join tick thread with Tree state '
                                                'IDLE, but failed!')

        # Make a new tick thread if there isn't one or the old one has been
        # successfully joined.
        if self._tick_thread is None or not self._tick_thread.is_alive():
            self._tick_thread = Thread(target=self.tick)

        if request.command == ControlTreeExecutionRequest.STOP:
            with self._state_lock:
                is_ticking = self.tree_msg.state == Tree.TICKING
            if is_ticking:
                with self._state_lock:
                    self.tree_msg.state = Tree.STOP_REQUESTED
                # Two seconds should be plenty of time to finish the current
                # tick, if the tree has not stopped by then we're in deep
                # trouble.
                if self._tick_thread.is_alive():
                    self._tick_thread.join((1.0 / self.tree_msg.tick_frequency_hz) * 4.0)
                    if self._tick_thread.is_alive():
                        raise BehaviorTreeException('Tried to join tick thread after requesting '
                                                    'stop, but failed!')
                if self.tree_msg.state == Tree.IDLE:
                    response.tree_state = Tree.IDLE
                    response.success = True
                else:
                    response.error_message = ('Successfully stopped ticking, but tree state is '
                                              '%s, not IDLE' % self.tree_msg.state)
                    response.success = False
                    rospy.logerr(response.error_message)
            else:
                rospy.loginfo('Received stop command, but tree was not running')
                response.success = True
                response.tree_state = self.tree_msg.state
        elif request.command == ControlTreeExecutionRequest.TICK_ONCE:
            if self._tick_thread.is_alive() or self.tree_msg.state == Tree.TICKING:
                response.success = False
                response.error_message = 'Tried to tick when tree is already running, aborting'
                rospy.logwarn(response.error_message)
            else:
                try:
                    self.find_root()
                    self._once = True
                    with self._state_lock:
                        self.tree_msg.state = Tree.TICKING
                    self._tick_thread.start()
                    # Give the tick thread much more time than it should take
                    self._tick_thread.join((1.0 / self.tree_msg.tick_frequency_hz) * 4.0)
                    if self._tick_thread.is_alive():
                        raise BehaviorTreeException('Tried to join tick thread after requesting '
                                                    'stop, but failed!')
                    else:
                        response.success = True
                        response.tree_state = Tree.IDLE
                except BehaviorTreeException, e:
                    response.success = False
                    response.error_message = str(e)
                    response.tree_state = self.tree_msg.state
        elif request.command == ControlTreeExecutionRequest.TICK_PERIODICALLY:
            if self._tick_thread.is_alive() or self.tree_msg.state == Tree.TICKING:
                response.success = False
                response.error_message = ('Tried to start periodic ticking when tree is '
                                          'already running, aborting')
                response.tree_state = self.tree_msg.state
                rospy.logwarn(response.error_message)
            else:
                try:
                    self.find_root()
                    with self._state_lock:
                        self.tree_msg.state = Tree.TICKING
                    self._once = False
                    # Use provided tick frequency, if any
                    if request.tick_frequency_hz != 0:
                        self.tree_msg.tick_frequency_hz = request.tick_frequency_hz
                    self._tick_thread.start()
                    response.success = True
                    response.tree_state = Tree.TICKING
                except BehaviorTreeException, e:
                    response.success = False
                    response.error_message = str(e)
                    response.tree_state = self.tree_msg.state
        else:
            response.error_message = 'Received unknown command %d' % request.command
            rospy.logerr(response.error_message)
            response.success = False

        return response

    def add_node(self, request):
        """Add the node in this request to the tree.

        :param ros_bt_py_msgs.srv.AddNodeRequest request:
          A request describing the node to add.
        """
        # TODO(nberg): handle subtrees
        response = AddNodeResponse()
        try:
            instance = self.instantiate_node_from_msg(request.node)
            response.success = True
            response.actual_node_name = instance.name
        except (BehaviorTreeException, KeyError) as exc:
            response.success = False
            response.error_message = str(exc)

        nodes_in_cycles = self.find_nodes_in_cycles()
        if nodes_in_cycles:
            response.success = False
            response.error_message = ('Found cycles in tree %s after inserting node %s as %s.'
                                      ' Nodes in cycles: %s' %
                                      (self.tree_msg.name,
                                       request.node.name,
                                       response.actual_node_name,
                                       str(nodes_in_cycles)))
            # Remove node from tree
            self.remove_node(RemoveNodeRequest(node_name=instance.name, remove_children=True))
        self.publish_info()
        return response

    def insert_node(self, parent_name, previous_child_name, child_node):
        """Insert `child_node` in between two nodes

        The two nodes are identified by `parent_name` and
        `previous_child_name`
        """
        pass

    def remove_node(self, request):
        """Remove the node identified by `request.node_name` from the tree.

        If the parent of the node removed supports enough children to
        take on all of the removed node's children, it will. Otherwise,
        children will be orphaned.
        """
        # TODO(nberg): handle subtrees
        response = RemoveNodeResponse()

        if request.node_name not in self.nodes:
            response.success = False
            response.error_message = 'No node with name %s in tree %s' % (
                request.node_name, self.tree_msg.name)
            return response

        # Shutdown node - this should also shutdown all children, but you
        # never know, so check later.
        self.nodes[request.node_name].shutdown()
        names_to_remove = {request.node_name}
        if request.remove_children:
            add_children_of = [request.node_name]
            children_added = set()
            while add_children_of:
                name = add_children_of.pop()
                if name not in children_added:
                    names_to_remove |= {child_name for child_name
                                        in self.nodes[name].children.iterkeys()}
                    add_children_of.append(child_name for child_name
                                           in self.nodes[name].children.iterkeys())
        for name in names_to_remove:
            # Check if node is already in shutdown state. If not, call
            # shutdown, but warn, because the parent node should have
            # done that!
            if self.nodes[name].state != NodeMsg.SHUTDOWN:
                parent_name = self.nodes[name].parent_name
                rospy.logwarn('Node %s was not shut down. Check parent node %s (%s) '
                              'for proper implementation of do_shutdown()',
                              name,
                              parent_name,
                              type(self.nodes[parent_name]).__name__)
                self.nodes[name].shutdown()
            del self.nodes[name]

        # Keep tree_msg up-to-date
        self.tree_msg.nodes = [node for node in self.tree_msg.nodes
                               if node.name in names_to_remove]
        self.tree_msg.data_wirings = [
            wiring for wiring in self.tree_msg.data_wirings
            if (wiring.source.node_name in names_to_remove or
                wiring.target.node_name in names_to_remove)]
        self.tree_msg.public_node_data = [data for data in self.tree_msg.public_node_data
                                          if data.node_name in names_to_remove]

        response.success = True
        self.publish_info()
        return response

    def move_child(self, node_name, child_name, new_index):
        """Move the named child of a node to the given index.

        Will fail if `new_index` is invalid.

        For moving a node to a different parent, see
        :meth:`move_node`
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

        Contains a list of :class: `ros_bt_py_msgs.msg.NodeDataWiring`
        objects that model connections

        :returns: :class:`ros_bt_py_msgs.src.WireNodeDataResponse` or `None`
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

        self.publish_info()
        return response

    def unwire_data(self, request):
        """Disconnect the given pairs of node data.

        :param ros_bt_py_msgs.srv.WireNodeDataRequest request:

        Contains a list of :class:`ros_bt_py_msgs.msg.NodeDataWiring`
        objects that model connections

        :returns: :class:`ros_bt_py_msgs.src.WireNodeDataResponse` or `None`
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

        self.publish_info()
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
        self.tree_msg.nodes = [node.to_msg() for node in self.nodes.values()]
        return self.tree_msg
