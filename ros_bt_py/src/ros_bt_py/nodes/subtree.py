from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds, Tree, NodeDataLocation
from ros_bt_py_msgs.srv import LoadTreeRequest

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

        # TODO(nberg): Maybe don't use the parent debug_manager? Could
        # break with duplicate node names?
        self.manager = TreeManager(
            name=name,
            debug_manager=debug_manager)

        response = self.manager.load_tree(LoadTreeRequest(tree=Tree(
            path=self.options['subtree_path'])))

        if not _get_success(response):
            self.outputs['load_success'] = False
            self.outputs['load_error_msg'] = _get_error_message(response)
            return

        self.outputs['load_success'] = True

        # If we loaded the tree successfully, change node_config to
        # include the public inputs and outputs
        new_node_config = NodeConfig()
        for node_data in self.manager.to_msg().public_node_data:
            if node_data.data_kind == NodeDataLocation.INPUT_DATA:
                new_node_config.inputs['%s.%s' % (node_data.node_name, node_data.data_key)] = \
                    self.manager.nodes[node_data.node_name].inputs.get_type(node_data.data_key)
            elif node_data.data_kind == NodeDataLocation.OUTPUT_DATA:
                new_node_config.outputs['%s.%s' % (node_data.node_name, node_data.data_key)] = \
                    self.manager.nodes[node_data.node_name].outputs.get_type(node_data.data_key)
            elif node_data.data_kind == NodeDataLocation.OPTION_DATA:
                new_node_config.options['%s.%s' % (node_data.node_name, node_data.data_key)] = \
                    self.manager.nodes[node_data.node_name].options.get_type(node_data.data_key)

        self.node_config.extend(new_node_config)
