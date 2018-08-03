import jsonpickle
from threading import Lock

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds, Tree, NodeData, NodeDataLocation
from ros_bt_py_msgs.srv import LoadTreeRequest, ControlTreeExecutionRequest, SetOptionsRequest

from ros_bt_py.tree_manager import TreeManager, _get_success, _get_error_message
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    options={'subtree_path': str},
    inputs={},
    outputs={'load_success': bool,
             'load_error_msg': str},
    max_children=0))
class Subtree(Leaf):
    """Loads a subtree from the location pointed to by `subtree_uri`

    This is the only node that modifies its `node_config` member - it
    will populate its inputs and outputs when its constructor is
    called, based on the public inputs and outputs of the subtree.
    """
    def __init__(self, options=None, debug_manager=None, name=None):
        """Create the tree manager, load the subtree and call `super.__init__()`"""
        super(Subtree, self).__init__(options, debug_manager, name)

        self.tree_msg = None
        self.tree_msg_lock = Lock()

        # TODO(nberg): Maybe don't use the parent debug_manager? Could
        # break with duplicate node names?
        self.manager = TreeManager(
            name=name,
            publish_tree_callback=self._update_tree_msg,
            debug_manager=debug_manager)

        response = self.manager.load_tree(LoadTreeRequest(tree=Tree(
            path=self.options['subtree_path'])))

        # TODO(nberg):
        #
        # Check self.options for keys that aren't in the original
        # NodeConfig. Those option keys must be public options of the
        # subtree.
        #
        # Then tell the TreeManager to set those options.

        if not _get_success(response):
            self.outputs['load_success'] = False
            self.outputs['load_error_msg'] = _get_error_message(response)
            return

        self.outputs['load_success'] = True

        # If we loaded the tree successfully, change node_config to
        # include the public inputs and outputs
        subtree_inputs = {}
        subtree_outputs = {}
        for node_data in self.manager.to_msg().public_node_data:
            if node_data.data_kind == NodeDataLocation.INPUT_DATA:
                subtree_inputs['%s.%s' % (node_data.node_name, node_data.data_key)] = \
                    self.manager.nodes[node_data.node_name].inputs.get_type(node_data.data_key)
            elif node_data.data_kind == NodeDataLocation.OUTPUT_DATA:
                subtree_outputs['%s.%s' % (node_data.node_name, node_data.data_key)] = \
                    self.manager.nodes[node_data.node_name].outputs.get_type(node_data.data_key)
            elif node_data.data_kind == NodeDataLocation.OPTION_DATA:
                raise BehaviorTreeException('Option values cannot be public!')

        # merge subtree input and option dicts, so we can receive
        # option updates between ticks
        self.node_config.extend(NodeConfig(
            options={},
            inputs=subtree_inputs,
            outputs=subtree_outputs,
            max_children=0))

        # Register the input and output values from the subtree - they
        # don't contain any OptionRefs, so don't allow references,
        # just to be sure
        self._register_node_data(source_map=subtree_inputs,
                                 target_map=self.inputs,
                                 allow_ref=False)
        self._register_node_data(source_map=subtree_outputs,
                                 target_map=self.outputs,
                                 allow_ref=False)

        # Handle forwarding inputs and outputs using the subscribe mechanics:
        for node_data in self.manager.to_msg().public_node_data:
            if node_data.data_kind == NodeDataLocation.INPUT_DATA:
                self.inputs.subscribe(
                    key='%s.%s' % (node_data.node_name, node_data.data_key),
                    callback=self.manager.nodes[node_data.node_name].inputs.get_callback(
                        node_data.data_key))
            elif node_data.data_kind == NodeDataLocation.OUTPUT_DATA:
                self.manager.nodes[node_data.node_name].outputs.subscribe(
                    key=node_data.data_key,
                    callback=self.outputs.get_callback(
                        '%s.%s' % (node_data.node_name, node_data.data_key)))

    def _update_tree_msg(self, new_msg):
        with self.tree_msg_lock:
            self.tree_msg = new_msg

    def _do_setup(self):
        """Nothing to do - the tree manager will call `root.setup()` on tick if necessary."""
        pass

    def _do_tick(self):
        new_state = self._send_command(ControlTreeExecutionRequest.TICK_ONCE)

        return new_state

    def _do_untick(self):
        return self._send_command(ControlTreeExecutionRequest.STOP)

    def _do_reset(self):
        return self._send_command(ControlTreeExecutionRequest.RESET)

    def _do_shutdown(self):
        return self._send_command(ControlTreeExecutionRequest.SHUTDOWN)

    def _send_command(self, command):
        """Send the given command to the TreeManager

        Returns the state of the subtree's root node, if any, and
        `ros_bt_py_msgs.msg.Node.FAILED` otherwise.
        """
        response = self.manager.control_execution(ControlTreeExecutionRequest(
            command=command))

        if not _get_success(response):
            self.logerr(_get_error_message(response))
            return NodeMsg.FAILED

        return self._get_root_state()

    def _get_root_state(self):
        with self.tree_msg_lock:
            if self.tree_msg:
                for node in self.tree_msg.nodes:
                    if node.name == self.tree_msg.root_name:
                        return node.state
        return NodeMsg.FAILED
