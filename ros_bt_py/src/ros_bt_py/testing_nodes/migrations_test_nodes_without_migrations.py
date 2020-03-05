from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    version='',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithoutVersionAndWithoutMigrationsModule(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithVersionAndWithoutMigrationsModule(Leaf):
    def _do_setup(self):
        pass

    def _do_tick(self):
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
