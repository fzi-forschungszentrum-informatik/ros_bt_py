from threading import Thread, Lock, RLock
import time
import jsonpickle
import yaml

import genpy
import rospy
import rospkg

from ros_bt_py_msgs.srv import LoadTreeResponse
from ros_bt_py_msgs.srv import MoveNodeRequest, RemoveNodeRequest, ReplaceNodeRequest, WireNodeDataRequest
from ros_bt_py_msgs.srv import MoveNodeResponse, ReplaceNodeResponse
from ros_bt_py_msgs.srv import WireNodeDataResponse, AddNodeResponse, RemoveNodeResponse
from ros_bt_py_msgs.srv import ContinueResponse
from ros_bt_py_msgs.srv import ControlTreeExecutionRequest, ControlTreeExecutionResponse
from ros_bt_py_msgs.srv import GetAvailableNodesResponse
from ros_bt_py_msgs.srv import SetExecutionModeResponse
from ros_bt_py_msgs.srv import SetOptionsResponse
from ros_bt_py_msgs.srv import ModifyBreakpointsResponse
from ros_bt_py_msgs.msg import Tree
from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData

from ros_bt_py.exceptions import BehaviorTreeException, MissingParentError, TreeTopologyError
from ros_bt_py.node import Node, load_node_module, increment_name
from ros_bt_py.debug_manager import DebugManager


def is_edit_service(func):
    """Decorator for all tree editing service handlers.

    This allows the common behavior of responding with a response that
    has success=False and an error_message if the tree is not
    currently editable, relying on all editing service responses to
    have at least those two members.

    It also ensures that all edits are atomic, i.e. external service
    calls cannot interweave. The lock used to ensure this is a
    `threading.RLock`, which means the service handlers *can* call
    each other if need be.

    """
    def service_handler(self, request):
        with self._state_lock:
            if self.tree_msg.state != Tree.EDITABLE:
                return {
                    'success': False,
                    'error_message': ('Cannot edit tree in state %s. You need to '
                                      'shut down the tree to enable editing.'
                                      % self.tree_msg.state)
                    }
        with self._edit_lock:
            return func(self, request)
    return service_handler


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
                 publish_debug_info_callback=None,
                 publish_debug_settings_callback=None):
        self.publish_tree = publish_tree_callback
        if self.publish_tree is None:
            rospy.loginfo('No callback for publishing tree data provided.')

        self.publish_debug_info = publish_debug_info_callback
        if self.publish_debug_info is None:
            rospy.loginfo('No callback for publishing debug data provided.')

        self.publish_debug_settings = publish_debug_settings_callback
        if self.publish_debug_settings is None:
            rospy.loginfo('No callback for publishing debug data provided.')

        self.debug_manager = debug_manager
        if not self.debug_manager:
            rospy.loginfo('Tree manager instantiated without explicit debug manager '
                          '- building our own with default parameters')
            self.debug_manager = DebugManager()

        self.debug_manager.set_tree_name(name)
        self.debug_manager.set_tick_frequency(tick_frequency_hz)
        self.debug_manager.publish_debug_info = self.publish_info
        self.debug_manager.publish_debug_settings = self.publish_debug_settings

        self.nodes = {}

        self._state_lock = Lock()
        self._edit_lock = RLock()

        # Stop the tick thread after a single tick
        self._once = False
        # Stop the tick thread after the tree returns something other than
        # RUNNING for the first time
        self._stop_after_result = False

        self.tree_msg = Tree()
        self.tree_msg.name = name if name else ''
        self.tree_msg.tick_frequency_hz = tick_frequency_hz

        self._last_error = None
        with self._state_lock:
            self.tree_msg.state = Tree.EDITABLE

        self._tick_thread = None

        # Skip if module_list is empty or None
        if module_list:
            for module_name in module_list:
                load_node_module(module_name)

        self.publish_info(self.debug_manager.get_debug_info_msg())

    def publish_info(self, debug_info_msg=None):
        """Publish the current tree state using the callback supplied to the constructor

        In most cases, you'll want that callback to publish to a ROS
        topic.

        If debugging is enabled, also publish debug info.
        """
        if self.publish_tree:
            self.publish_tree(self.to_msg())
        if debug_info_msg and self.publish_debug_info:
            self.publish_debug_info(debug_info_msg)

    def find_root(self):
        """Find the root node of the tree

        :raises: `TreeTopologyError`

        if nodes exist, but either no root or multiple roots are
        found.
        """
        if not self.nodes:
            return None
        # Find root node
        possible_roots = [node for node in self.nodes.itervalues() if not node.parent]

        if len(possible_roots) > 1:
            raise TreeTopologyError('Tree "%s" has multiple nodes without parents. '
                                    % self.tree_msg.name)
        if not possible_roots:
            raise TreeTopologyError('All nodes in tree "%s" have parents. You have '
                                    'made a cycle, which makes the tree impossible to run!' %
                                    self.tree_msg.name)
        return possible_roots[0]

    def tick_report_exceptions(self):
        """Wrap :meth:`TreeManager.tick()` and catch *all* errors"""
        try:
            self.tick()
        except Exception, e:
            # TODO(nberg): don't catch the ROSException that is raised on shutdown
            rospy.logerr('Encountered error while ticking tree: %s', e)
            with self._state_lock:
                self._last_error = e
                self.tree_msg.state = Tree.ERROR

    def tick(self, once=None):
        """Execute a tick, starting from the tree's root.

        This behaves differently based on the current configuration of
        the `TreeManager` - it can tick once, continuously, until the
        tree reports a result (either SUCCEEDED or FAILED).

        Each of those can be done in debug mode (i.e. requiring a call
        of :meth:`TreeManager.debug_step()` after each node's tick)
        too.

        This method should *NOT* be called directly, but rather
        triggered via :meth:`TreeManager.control_execution()`!
        """
        if once is not None:
            self._once = once

        # First check for nodes with missing parents
        orphans = ['"%s"(parent: "%s")' % (node.name, node.parent.name if node.parent else '')
                   for node in self.nodes.itervalues()
                   if node.parent and node.parent.name not in self.nodes]
        if orphans:
            raise MissingParentError('The following nodes\' parents are missing: %s'
                                     % ', '.join(orphans))
        root = self.find_root()
        if not root:
            rospy.loginfo('No nodes in tree, tick will not do anything')
            return
        with self._state_lock:
            self.tree_msg.root_name = root.name
        if root.state == NodeMsg.UNINITIALIZED or root.state == NodeMsg.SHUTDOWN:
            root.setup()
        sleep_duration_sec = (1.0/self.tree_msg.tick_frequency_hz)

        while True:
            start_time = time.time()
            with self._state_lock:
                if self.tree_msg.state == Tree.STOP_REQUESTED:
                    break
            root.tick()
            self.publish_info(self.debug_manager.get_debug_info_msg())

            if self._stop_after_result:
                if root.state != NodeMsg.RUNNING:
                    break

            if self._once:
                # Return immediately, not unticking anything
                self._once = False
                with self._state_lock:
                    self.tree_msg.state = Tree.WAITING_FOR_TICK
                return

            tick_duration = time.time() - start_time
            if tick_duration <= 0:
                rospy.logwarn('Tick took longer than set period, cannot tick at %.2f Hz',
                              self.tree_msg.tick_frequency_hz)
            else:
                rospy.sleep(sleep_duration_sec - tick_duration)

        # Ensure all nodes are stopped and not doing anything in
        # the background.
        root.untick()
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
            while current_node.parent:
                current_node = self.nodes[current_node.parent.name]
                cycle_candidates.append(current_node.name)
                if current_node.name == starting_name:
                    nodes_in_cycles.extend(cycle_candidates)
                    break
                if current_node.name in safe_node_names:
                    # We've already checked for cycles from the parent node, no
                    # need to do that again.
                    safe_node_names.extend(cycle_candidates)
                    break
            if not current_node.parent:
                safe_node_names.extend(cycle_candidates)

        return nodes_in_cycles

    ####################
    # Service Handlers #
    ####################

    @is_edit_service
    def clear(self, _):
        root = self.find_root()
        if not root:
            # No root, no problems
            return
        if not (root.state == NodeMsg.UNINITIALIZED or root.state == NodeMsg.SHUTDOWN):
            rospy.logerr('Please shut down the tree before clearing it')
            return

        self.nodes = {}
        with self._state_lock:
            self.tree_msg = Tree(name='',
                                 state=Tree.EDITABLE)
        self.publish_info(self.debug_manager.get_debug_info_msg())

    @is_edit_service
    def load_tree(self, request):
        """Load a tree from the given message (which may point to a file)

        `request.tree` describes the tree to be loaded, including
        nodes, wirings and public node data.

        If the `Tree` message itself isn't populated, but contains a
        `path` to load a tree from, we open the file it points to and
        load that.

        """
        response = LoadTreeResponse()
        tree = request.tree
        rospack = rospkg.RosPack()
        while not tree.nodes:
            # TODO(nberg): Save visited file names to find loops

            # as long as we don't have any nodes, the tree message is
            # just a pointer to a file containing the actual tree, so
            # load that file.
            file_path = ''
            if not tree.path:
                response.success = False
                response.error_message = ('Trying to load tree, but found no nodes and no path '
                                          'to read from: %s') % str(tree)
                return response
            if tree.path.startswith('file://'):
                file_path = tree.path[len('file://'):]
            elif tree.path.startswith('package://'):
                package_name = tree.path[len('package://'):].split('/', 1)[0]
                package_path = rospack.get_path(package_name)
                file_path = package_path + tree.path[len('package://') + len(package_name):]
            else:
                response.success = False
                response.error_message = ('Tree path "%s" is malformed. It needs to start with '
                                          'either "file://" or "package://"') % tree.path
                return response

            # load tree file and parse yaml, then convert to Tree message
            try:
                tree_file = open(file_path, 'r')
            except IOError, e:
                response.success = False
                response.error_message = ('Error opening file %s: %s' % (file_path, str(e)))
                return response
            with tree_file:
                data = yaml.load_all(tree_file)
                read_data = False
                for d in data:
                    if not read_data:
                        tree = Tree()
                        genpy.message.fill_message_args(tree, d, keys={})
                        read_data = True
                    else:
                        response.success = False
                        response.error_message = ('Tree YAML file must contain '
                                                  'exactly one YAML object!')
                        return response
                if not read_data:
                    response.success = False
                    response.error_message = ('No data in YAML file %s!' % file_path)

                    return response

        # we should have a tree message with all the info we need now

        # Clear existing tree, then replace it with the message's contents
        self.clear(None)
        # add nodes whose children exist already, until all nodes are there
        while len(self.nodes) != len(tree.nodes):
            added = 0
            # find nodes whose children are all in the tree already, then add them
            for node in (node for node in tree.nodes
                         if (node.name not in self.nodes and
                             all((name in self.nodes for name in node.child_names)))):
                try:
                    instance = self.instantiate_node_from_msg(node, allow_rename=False)
                    for child_name in node.child_names:
                        instance.add_child(self.nodes[child_name])
                    self.nodes[node.name] = instance
                    added += 1
                except BehaviorTreeException as exc:
                    response.success = False
                    response.error_message = str(exc)
                    return response
            if added == 0:
                response.success = False
                response.error_message = 'Unable to add all nodes to tree.'
                return response

        # All nodes are added, now do the wiring
        wire_response = self.wire_data(WireNodeDataRequest(tree_name=tree.name,
                                                           wirings=tree.data_wirings))
        if not _get_success(wire_response):
            response.success = False
            response.error_message = _get_error_message(wire_response)
            return response

        self.tree_msg = tree
        # Ensure Tree is editable after loading
        with self._state_lock:
            self.tree_msg.state = Tree.EDITABLE

        response.success = True
        self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    def set_execution_mode(self, request):
        """Set the parameters of our :class:`DebugManager`

        :param request ros_bt_msgs.srv.SetExecutionModeRequest:
        """
        self.debug_manager.set_execution_mode(
            request.single_step,
            request.collect_performance_data)
        return SetExecutionModeResponse()

    def debug_step(self, request):
        """Continue execution

        If single step mode is enabled, advance a single step. If we're
        currently stopped at a breakpoint, continue to the next, if
        any. If we're not stopped at all, do nothing.

        :param request ros_bt_msgs.srv.ContinueRequest:
        """
        # TODO(nberg): Add more logic to at least warn if we're not stopped
        self.debug_manager.continue_debug()
        return ContinueResponse(success=True)

    def modify_breakpoints(self, request):
        return ModifyBreakpointsResponse(
            current_breakpoints=self.debug_manager.modify_breakpoints(
                add=request.add,
                remove=request.remove,
                remove_all=request.remove_all))

    def control_execution(self, request):
        """Control tree execution.

        :param ros_bt_py_msgs.srv.ControlTreeExecutionRequest request:

        Can request a tick, periodic ticking, periodic ticking until
        the root node reports a result (SUCCEEDED or FAILED), or to
        stop or reset the entire tree.

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
            self._tick_thread = Thread(target=self.tick_report_exceptions)

        if (request.command == ControlTreeExecutionRequest.STOP or
                request.command == ControlTreeExecutionRequest.SHUTDOWN):
            with self._state_lock:
                tree_state = self.tree_msg.state
            if tree_state == Tree.TICKING:
                with self._state_lock:
                    self.tree_msg.state = Tree.STOP_REQUESTED
                # Four times the allowed period should be plenty of time to
                # finish the current tick, if the tree has not stopped by then
                # we're in deep trouble.
                if self._tick_thread.is_alive():
                    # Give the tick thread some time to finish
                    self._tick_thread.join((1.0 / self.tree_msg.tick_frequency_hz) * 4.0)
                    # If we're debugging (and ROS is not shutting down), keep
                    # sleepin until the thread finishes
                    while (self._tick_thread.is_alive() and
                           not rospy.is_shutdown() and
                           self.debug_manager.is_debugging()):
                        self._tick_thread.join((1.0 / self.tree_msg.tick_frequency_hz) * 4.0)
                    if self._tick_thread.is_alive():
                        raise BehaviorTreeException('Tried to join tick thread after requesting '
                                                    'stop, but failed!')
                if self.tree_msg.state == Tree.IDLE:
                    response.tree_state = Tree.IDLE
                    response.success = True
                elif self.tree_msg.state == Tree.ERROR:
                    response.error_message = ('Error stopping tick: %s' % str(self._last_error))
                    response.success = False
                    rospy.logerr(response.error_message)
                else:
                    response.error_message = ('Successfully stopped ticking, but tree state is '
                                              '%s, not IDLE' % self.tree_msg.state)
                    response.success = False
                    rospy.logerr(response.error_message)
            elif tree_state == Tree.WAITING_FOR_TICK:
                try:
                    root = self.find_root()
                    if root:
                        root.untick()
                        state = root.state
                        if state == NodeMsg.IDLE or state == NodeMsg.PAUSED:
                            response.tree_state = Tree.IDLE
                            response.success = True
                        else:
                            response.tree_state = Tree.ERROR
                            response.success = False
                            rospy.logerr(
                                ('Root node (%s) state after unticking is neither '
                                 'IDLE nor PAUSED, but %s'),
                                         str(root),
                                         state)
                            response.error_message = 'Failed to untick root node.'
                    else:
                        rospy.loginfo('Unticking a tree with no nodes.')
                        response.tree_state = Tree.IDLE
                        response.success = True
                except TreeTopologyError as e:
                    response.success = False
                    response.error_message = str(e)

                self.publish_info(self.debug_manager.get_debug_info_msg())
            else:
                rospy.loginfo('Received stop command, but tree was not running')
                response.success = True
                response.tree_state = self.tree_msg.state

            # actually shut down the tree
            if request.command == ControlTreeExecutionRequest.SHUTDOWN:
                try:
                    root = self.find_root()
                    if root:
                        root.shutdown()
                    else:
                        rospy.loginfo('Shutting down a tree with no nodes.')
                except TreeTopologyError, e:
                    response.success = False
                    response.error_message = str(e)

                # Now the tree is editable again - all nodes are in a state
                # where they must be initialized.
                with self._state_lock:
                    self.tree_msg.state = Tree.EDITABLE

            self.publish_info(self.debug_manager.get_debug_info_msg())

        elif request.command == ControlTreeExecutionRequest.TICK_ONCE:
            if self._tick_thread.is_alive() or self.tree_msg.state == Tree.TICKING:
                response.success = False
                response.error_message = 'Tried to tick when tree is already running, aborting'
                rospy.logwarn(response.error_message)
            else:
                try:
                    self.find_root()
                    self._once = True
                    self._stop_after_result = False
                    with self._state_lock:
                        self.tree_msg.state = Tree.TICKING
                    self._tick_thread.start()
                    # Give the tick thread some time to finish
                    self._tick_thread.join((1.0 / self.tree_msg.tick_frequency_hz) * 4.0)
                    # If we're debugging (and ROS is not shutting down), keep
                    # sleepin until the thread finishes
                    while (self._tick_thread.is_alive() and
                           not rospy.is_shutdown() and
                           self.debug_manager.is_debugging()):
                        self._tick_thread.join((1.0 / self.tree_msg.tick_frequency_hz) * 4.0)
                    if self._tick_thread.is_alive():
                        raise BehaviorTreeException('Tried to join tick thread after requesting '
                                                    'stop, but failed!')
                    if self.tree_msg.state == Tree.IDLE:
                        response.tree_state = Tree.IDLE
                        response.success = True
                    if self.tree_msg.state == Tree.WAITING_FOR_TICK:
                        response.tree_state = Tree.WAITING_FOR_TICK
                        response.success = True
                    elif self.tree_msg.state == Tree.ERROR:
                        response.error_message = ('Error during single tick: %s'
                                                  % str(self._last_error))
                        response.success = False
                        rospy.logerr(response.error_message)
                    else:
                        response.error_message = ('Successfully stopped ticking, but tree state '
                                                  'is %s, not IDLE' % self.tree_msg.state)
                        response.success = False
                        rospy.logerr(response.error_message)
                except TreeTopologyError, e:
                    response.success = False
                    response.error_message = str(e)
                    response.tree_state = self.tree_msg.state

        elif (request.command == ControlTreeExecutionRequest.TICK_PERIODICALLY or
              request.command == ControlTreeExecutionRequest.TICK_UNTIL_RESULT):
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
                    if request.command == ControlTreeExecutionRequest.TICK_UNTIL_RESULT:
                        self._stop_after_result = True
                    # Use provided tick frequency, if any
                    if request.tick_frequency_hz != 0:
                        self.tree_msg.tick_frequency_hz = request.tick_frequency_hz
                    if self.tree_msg.tick_frequency_hz == 0.0:
                        rospy.logwarn('Loaded tree had frequency 0Hz. Defaulting to 20Hz')
                        self.tree_msg.tick_frequency_hz = 20.0
                    self._tick_thread.start()
                    response.success = True
                    response.tree_state = Tree.TICKING
                except TreeTopologyError, e:
                    response.success = False
                    response.error_message = str(e)
                    response.tree_state = self.tree_msg.state

        elif request.command == ControlTreeExecutionRequest.RESET:
            if self._tick_thread.is_alive() or self.tree_msg.state == Tree.TICKING:
                response.success = False
                response.error_message = ('Tried to reset tree while it is running, aborting')
                response.tree_state = self.tree_msg.state
                rospy.logwarn(response.error_message)
            else:
                try:
                    root = self.find_root()
                    if root:
                        root.reset()
                    else:
                        rospy.loginfo('Resetting a tree with no root.')
                    with self._state_lock:
                        self.tree_msg.state = Tree.IDLE
                    response.success = True
                    response.tree_state = self.tree_msg.state
                except TreeTopologyError, e:
                    response.success = False
                    response.error_message = str(e)
                    response.tree_state = self.tree_msg.state

            self.publish_info(self.debug_manager.get_debug_info_msg())
        else:
            response.error_message = 'Received unknown command %d' % request.command
            rospy.logerr(response.error_message)
            response.success = False

        return response

    @is_edit_service
    def add_node(self, request):
        """Add the node in this request to the tree.

        :param ros_bt_py_msgs.srv.AddNodeRequest request:
          A request describing the node to add.
        """
        response = AddNodeResponse()

        try:
            instance = self.instantiate_node_from_msg(request.node, request.allow_rename)
            response.success = True
            response.actual_node_name = instance.name
        except BehaviorTreeException as exc:
            response.success = False
            response.error_message = str(exc)
            return response

        # Add node as child of the named parent, if any
        if request.parent_name:
            if request.parent_name not in self.nodes:
                response.success = False
                response.error_message = ('Parent %s of node %s does not exist!' %
                                          (request.parent_name, instance.name))
                # Remove node from tree
                self.remove_node(RemoveNodeRequest(node_name=instance.name,
                                                   remove_children=False))
                return response
            else:
                self.nodes[request.parent_name].add_child(instance)

        # Add children from msg to node
        missing_children = []
        for child_name in request.node.child_names:
            if child_name in self.nodes:
                instance.add_child(self.nodes[child_name])
            else:
                missing_children.append(child_name)
        if missing_children:
            response.success = False
            response.error_message = ('Children for node %s are not in tree: %s' %
                                      (instance.name, str(missing_children)))
            # Remove node from tree to restore state before insertion attempt
            self.remove_node(RemoveNodeRequest(node_name=instance.name,
                                               remove_children=False))

        nodes_in_cycles = self.find_nodes_in_cycles()
        if nodes_in_cycles:
            response.success = False
            response.error_message = ('Found cycles in tree %s after inserting node %s as %s.'
                                      ' Nodes in cycles: %s' %
                                      (self.tree_msg.name,
                                       request.node.name,
                                       response.actual_node_name,
                                       str(nodes_in_cycles)))
            # First, remove all of the node's children to avoid infinite
            # recursion in remove_node()
            for child_name in [c.name for c in instance.children]:
                instance.remove_child(child_name)

            # Then remove the node from the tree
            self.remove_node(RemoveNodeRequest(node_name=instance.name,
                                               remove_children=False))
            return response
        self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    @is_edit_service
    def insert_node(self, parent_name, previous_child_name, child_node):
        """Insert `child_node` in between two nodes

        The two nodes are identified by `parent_name` and
        `previous_child_name`
        """
        pass

    @is_edit_service
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
                if name not in self.nodes:
                    response.success = False
                    response.error_message = ('Error while removing children of node %s: '
                                              'No node with name %s in tree %s' %
                                              (request.node_name, name, self.tree_msg.name))
                    return response
                if name not in children_added:
                    names_to_remove |= {child.name for child
                                        in self.nodes[name].children}
                    add_children_of.extend([child.name for child
                                            in self.nodes[name].children])
        else:
            # If we're not removing the children, at least set their parent to None
            for child in self.nodes[request.node_name].children:
                child.parent = None
        for name in names_to_remove:
            # Check if node is already in shutdown state. If not, call
            # shutdown, but warn, because the parent node should have
            # done that!
            if self.nodes[name].state != NodeMsg.SHUTDOWN:
                # It's reasonable to expect parent to not be None here, since
                # the node is one of a list of children
                parent_name = self.nodes[name].parent.name
                rospy.logwarn('Node %s was not shut down. Check parent node %s (%s) '
                              'for proper implementation of _do_shutdown()',
                              name,
                              parent_name,
                              type(self.nodes[parent_name]).__name__)
                self.nodes[name].shutdown()

            # If we have a parent, remove the node from that parent
            if self.nodes[name].parent and self.nodes[name].parent.name in self.nodes:
                self.nodes[self.nodes[name].parent.name].remove_child(name)
            del self.nodes[name]

        # Unwire wirings that have removed nodes as source or target
        unwire_resp = self.unwire_data(WireNodeDataRequest(
            tree_name=self.tree_msg.name,
            wirings=[wiring for wiring in self.tree_msg.data_wirings
                     if (wiring.source.node_name in names_to_remove or
                         wiring.target.node_name in names_to_remove)]))

        # Keep tree_msg up-to-date
        self.tree_msg.data_wirings = [
            wiring for wiring in self.tree_msg.data_wirings
            if (wiring.source.node_name not in names_to_remove and
                wiring.target.node_name not in names_to_remove)]
        self.tree_msg.public_node_data = [data for data in self.tree_msg.public_node_data
                                          if data.node_name not in names_to_remove]

        response.success = True
        self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    @is_edit_service
    def set_options(self, request):
        """Set the option values of a given node.

        This is an "edit service", i.e. it can only be used when the
        tree has not yet been initialized or has been shut down.
        """
        if request.node_name not in self.nodes:
            return SetOptionsResponse(
                success=False,
                error_message='Unable to find node %s in tree %s' % (
                    request.node_name,
                    request.tree_name))

        node = self.nodes[request.node_name]
        unknown_options = []
        incompatible_options = []
        try:
            deserialized_options = dict((option.key, jsonpickle.decode(option.serialized_value))
                                        for option in request.options)
        except ValueError, e:
            return SetOptionsResponse(
                success=False,
                error_message='Failed to deserialize option value: %s' % str(e))
        # Find any options values that
        # a) the node does not expect
        # b) have the wrong type
        for key, value in deserialized_options.iteritems():
            if key not in node.options:
                unknown_options.append(key)
                continue

            required_type = node.options.get_type(key)
            if not isinstance(value, required_type):
                incompatible_options.append((key, required_type.__name__))

        error_strings = []
        if unknown_options:
            error_strings.append('Unknown option keys: %s' % str(unknown_options))
        if incompatible_options:
            error_strings.append('Incompatible option keys:\n' + '\n'.join(
                ['Key %s has type %s, should be %s' % (
                    key,
                    type(deserialized_options[key]).__name__,
                    required_type_name) for key, required_type_name in incompatible_options]))
        if error_strings:
            return SetOptionsResponse(
                success=False,
                error_message='\n'.join(error_strings))

        # Because options are used at construction time, we need to
        # construct a new node with the new options.

        # First, we need to add the option values that didn't change
        # to our dict:
        for key in node.options:
            if key not in deserialized_options:
                deserialized_options[key] = node.options[key]

        # Now we can construct the new node - no need to call setup,
        # since we're guaranteed to be in the edit state
        # (i.e. `root.setup()` will be called before anything that
        # needs the node to be set up)
        new_node = node.__class__(options=deserialized_options,
                                  name=node.name,
                                  debug_manager=node.debug_manager)

        # Use this request to unwire any data connections the existing
        # node has - if we didn't do this, the node wouldn't ever be
        # garbage collected, among other problems.
        #
        # We'll use the same request to re-wire the connections to the
        # new node (or the old one, if anything goes wrong).
        wire_request = WireNodeDataRequest(
            tree_name=self.tree_msg.name,
            wirings=[wiring for wiring in self.tree_msg.data_wirings
                     if (wiring.source.node_name == node.name or
                         wiring.target.node_name == node.name)])

        unwire_resp = self.unwire_data(wire_request)
        if not unwire_resp.success:
            return SetOptionsResponse(
                success=False,
                error_message='Failed to unwire data for node %s' % node.name)

        parent = None
        if node.parent:
            parent = node.parent
            # Remember the old index so we can insert the new instance at
            # the same position
            old_child_index = parent.get_child_index(node.name)

            if old_child_index is None:
                return SetOptionsResponse(
                    success=False,
                    error_message=('Parent of node %s claims to have no child with that name?!' %
                                   node.name))

            try:
                parent.remove_child(node.name)
            except KeyError as e:
                error_message = ('Failed to remove old instance of node %s: %s' %
                                 (node.name, str(e)))
                rewire_resp = self.wire_data(wire_request)
                if not rewire_resp.success:
                    error_message += '\nAlso failed to restore data wirings!'

                return SetOptionsResponse(
                    success=False,
                    error_message=error_message)

            try:
                parent.add_child(new_node, at_index=old_child_index)
            except (KeyError, BehaviorTreeException) as e:
                error_message = ('Failed to add new instance of node %s: %s' %
                                 (node.name, str(e)))

                try:
                    parent.add_child(node, at_index=old_child_index)
                    rewire_resp = self.wire_data(wire_request)
                    if not rewire_resp.success:
                        error_message += '\nAlso failed to restore data wirings!'
                except (KeyError, BehaviorTreeException) as e:
                    error_message += '\n Also failed to restore old node.'

                return SetOptionsResponse(
                    success=False,
                    error_message=error_message)

        # Add the new node to self.nodes
        self.nodes[node.name] = new_node

        # Re-wire all the data, just as it was before
        rewire_resp = self.wire_data(wire_request)
        if not rewire_resp.success:
            error_message = 'Failed to re-wire data to new node %s' % new_node.name
            # Try to undo everything
            self.nodes[node.name] = node
            if parent is not None:
                try:
                    parent.remove_child(new_node.name)
                    parent.add_child(node)
                except (KeyError, BehaviorTreeException) as e:
                    error_message += '\nError restoring old node: %s' % str(e)
            recovery_wire_response = self.wire_data(wire_request)
            if not rewire_resp.success:
                error_message += 'Failed to re-wire data to restored node %s' % new_node.name

            return SetOptionsResponse(
                success=False,
                error_message=error_message)

        # We made it!
        self.publish_info(self.debug_manager.get_debug_info_msg())
        return SetOptionsResponse(success=True)

    @is_edit_service
    def move_child(self, node_name, child_name, new_index):
        """Move the named child of a node to the given index.

        Will fail if `new_index` is invalid.

        For moving a node to a different parent, see
        :meth:`move_node`
        """
        pass

    @is_edit_service
    def move_node(self, request):
        """Move the named node to a different parent and insert it at the given index.

        """
        if request.node_name not in self.nodes:
            return MoveNodeResponse(
                success=False,
                error_message="Node to be moved (\"%s\") is not in tree." % request.node_name)

        # Empty parent name -> just remove node from parent
        if request.new_parent_name == '':
            node = self.nodes[request.node_name]
            if node.parent is not None:
                node.parent.remove_child(node.name)
            self.publish_info(self.debug_manager.get_debug_info_msg())
            return MoveNodeResponse(success=True)

        if request.new_parent_name not in self.nodes:
            return MoveNodeResponse(
                success=False,
                error_message="New parent (\"%s\") is not in tree." % request.new_parent_name)

        new_parent_max_children = self.nodes[request.new_parent_name].node_config.max_children
        if (new_parent_max_children is not None and
              len(self.nodes[request.new_parent_name].children) == new_parent_max_children):
            return MoveNodeResponse(
                success=False,
                error_message="Cannot move node %s to new parent node %s. Parent node already has the maximum number of children (%d)." % (
                    request.node_name,
                    request.new_parent_name,
                    new_parent_max_children))

        # Remove node from old parent, if any
        old_parent = self.nodes[request.node_name].parent
        if old_parent is not None:
            old_parent.remove_child(request.node_name)

        # Add node to new parent
        self.nodes[request.new_parent_name].add_child(
            child=self.nodes[request.node_name],
            at_index=request.new_child_index)

        self.publish_info(self.debug_manager.get_debug_info_msg())
        return MoveNodeResponse(success=True)


    @is_edit_service
    def replace_node(self, request):
        """Replace the named node with `new_node`.

        Will also move all children of the old node to the new one, but
        only if `new_node` supports that number of children. Otherwise,
        this will return an error and leave the tree unchanged.
        """
        if request.old_node_name not in self.nodes:
            return ReplaceNodeResponse(
                success=False,
                error_message="Node to be replaced (\"%s\") is not in tree." % request.old_node_name)
        if request.new_node_name not in self.nodes:
            return ReplaceNodeResponse(
                success=False,
                error_message="Replacement node (\"%s\") is not in tree." % request.new_node_name)

        old_node = self.nodes[request.old_node_name]

        new_node_max_children = self.nodes[request.new_node_name].node_config.max_children
        # We're *replacing* one of the children, so there should only
        # be an issue if there were too many children before. Which
        # shouldn't happen. But you know, better safe than sorry!
        if (new_node_max_children is not None and
              len(old_node.children) > new_node_max_children):
            return ReplaceNodeResponse(
                success=False,
                error_message="Replacement node (\"%s\") does not support the number of \
                               children required (%s has %d children, %s supports %d." % (
                    request.new_node_name,
                    request.old_node_name,
                    len(old_node.children),
                    request.new_node_name,
                    new_node_max_children))

        # TODO(nberg): Actually implement this

        # If the new node has inputs/outputs with the same name and
        # type as the old one,wire them the same way the old node was
        # wired

        # Note the old node's position in its parent's children array
        old_node_parent = old_node.parent
        # Initialize to 0 just to be sure. We *should* be guaranteed
        # to find the old node in its parent's children array, but
        # better safe than sorry.
        old_node_child_index = 0
        if old_node_parent is not None:
            for index, child in enumerate(old_node_parent.children):
                if child.name == request.old_node_name:
                    old_node_child_index = index
                    break

        # Move the children from old to new
        new_node = self.nodes[request.new_node_name]
        for child_name in [child.name for child in old_node.children]:
            new_node.add_child(old_node.remove_child(child_name))
        # Remove the old node (we just moved the children, so we can
        # set remove_children to True)
        res = self.remove_node(RemoveNodeRequest(
            node_name=request.old_node_name,
            remove_children=True))

        if not res.success:
            with self._state_lock:
                self.tree_msg.state = Tree.ERROR
            self.publish_info(self.debug_manager.get_debug_info_msg())
            return ReplaceNodeResponse(
                success=False,
                error_message="Could not remove old node: \"%s\"" % res.error_message)

        # Move the new node to the old node's parent (if it had one)
        if old_node_parent is not None:
            self.move_node(MoveNodeRequest(
                node_name=request.new_node_name,
                new_parent_name=old_node_parent.name,
                new_child_index=old_node_child_index))

        self.publish_info(self.debug_manager.get_debug_info_msg())
        return ReplaceNodeResponse(
            success=True)

    @is_edit_service
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
            root = self.find_root()
            if not root:
                raise TreeTopologyError('No nodes in tree')
        except TreeTopologyError, e:
            response.success = False
            response.error_message = 'Unable to find root node: %s' % str(e)
            return response

        successful_wirings = []
        for wiring in request.wirings:
            target_node = root.find_node(wiring.target.node_name)
            if not target_node:
                response.success = False
                response.error_message = 'Target node %s does not exist' % wiring.target.node_name
                break
            try:
                target_node.wire_data(wiring)
                successful_wirings.append(wiring)
            except (KeyError, BehaviorTreeException), e:
                response.success = False
                response.error_message = 'Failed to execute wiring "%s": %s' % (wiring, str(e))
                break

        if not response.success:
            # Undo the successful wirings
            for wiring in successful_wirings:
                target_node = root.find_node(wiring.target.node_name)
                try:
                    target_node.unwire_data(wiring)
                except (KeyError, BehaviorTreeException), e:
                    response.success = False
                    response.error_message = (
                        'Failed to undo wiring "%s": %s\nPrevious error: %s'
                        % (wiring, str(e), response.error_message))
                    rospy.logerr('Failed to undo successful wiring after error. '
                                 'Tree is in undefined state!')
                    with self._state_lock:
                        self.tree_msg.state = Tree.ERROR
                    break

        # only actually wire any data if there were no errors
        if response.success:
            # We made it here, so all the Wirings should be valid. Time to save
            # them.
            self.tree_msg.data_wirings.extend(request.wirings)

            self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    @is_edit_service
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
            root = self.find_root()
            if not root:
                raise TreeTopologyError('No nodes in tree')
        except TreeTopologyError, e:
            response.success = False
            response.error_message = 'Unable to find root node: %s' % str(e)
            return response

        successful_unwirings = []
        for wiring in request.wirings:
            target_node = root.find_node(wiring.target.node_name)
            if not target_node:
                response.success = False
                response.error_message = 'Target node %s does not exist' % wiring.target.node_name
                break
            try:
                target_node.unwire_data(wiring)
                successful_unwirings.append(wiring)
            except (KeyError, BehaviorTreeException), e:
                response.success = False
                response.error_message = 'Failed to remove wiring "%s": %s' % (wiring, str(e))
                break

        if not response.success:
            # Re-Wire the successful unwirings
            for wiring in successful_unwirings:
                target_node = root.find_node(wiring.target.node_name)
                try:
                    target_node.wire_data(wiring)
                except (KeyError, BehaviorTreeException), e:
                    response.success = False
                    response.error_message = (
                        'Failed to redo wiring "%s": %s\nPrevious error: %s'
                        % (wiring, str(e), response.error_message))
                    rospy.logerr('Failed to rewire successful unwiring after error. '
                                 'Tree is in undefined state!')
                    with self._state_lock:
                        self.tree_msg.state = Tree.ERROR
                    break

        # only actually wire any data if there were no errors
        if response.success:
            # We've removed these NodeDataWirings, so remove them from tree_msg as
            # well.
            for wiring in request.wirings:
                if wiring in self.tree_msg.data_wirings:
                    self.tree_msg.data_wirings.remove(wiring)

            self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    def get_available_nodes(self, request):
        """List the types of nodes that are currently known

        This includes all nodes from modules that were passed to our
        constructor in `module_list`, ones from modules that nodes have
        been successfully loaded from since launch, and ones from
        modules explicitly asked for in `request.node_modules`

        :param ros_bt_py_msgs.srv.GetAvailableNodesRequest request:

        If `request.node_modules` is not empty, try to load those
        modules before responding.

        :returns: :class:`ros_bt_py_msgs.src.GetAvailableNodesResponse` or `None`
        """
        response = GetAvailableNodesResponse()
        for module_name in request.node_modules:
            if module_name and load_node_module(module_name) is None:
                response.success = False
                response.error_message = 'Failed to import module %s' % module_name
                return response

        def to_node_data(data_map):
            return [NodeData(key=name,
                             serialized_value=jsonpickle.encode(type_or_ref))
                    for (name, type_or_ref) in data_map.iteritems()]

        for (module, nodes) in Node.node_classes.iteritems():
            for (class_name, node_class) in nodes.iteritems():
                max_children = node_class._node_config.max_children
                max_children = -1 if max_children is None else max_children
                response.available_nodes.append(NodeMsg(
                    module=module,
                    node_class=class_name,
                    max_children=max_children,
                    name=class_name,
                    options=to_node_data(node_class._node_config.options),
                    inputs=to_node_data(node_class._node_config.inputs),
                    outputs=to_node_data(node_class._node_config.outputs)
                    ))

        response.success = True
        return response

    #########################
    # Service Handlers Done #
    #########################

    def instantiate_node_from_msg(self, node_msg, allow_rename):
        node_instance = Node.from_msg(node_msg)
        if node_instance.name in self.nodes:
            if allow_rename:
                node_instance.name = self.make_name_unique(node_instance.name)
            else:
                raise BehaviorTreeException('Node with name "%s" exists already'
                                            % node_instance.name)

        # Set DebugManager
        node_instance.debug_manager = self.debug_manager

        self.nodes[node_instance.name] = node_instance

        return node_instance

    def make_name_unique(self, name):
        while name in self.nodes:
            name = increment_name(name)
        return name

    def to_msg(self):
        self.tree_msg.nodes = [node.to_msg() for node in self.nodes.itervalues()]
        return self.tree_msg


def _get_success(response):
    if isinstance(response, dict):
        return response['success']

    return response.success


def _get_error_message(response):
    if isinstance(response, dict):
        return response['error_message']

    return response.error_message
