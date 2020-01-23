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
class NodeWithoutVersion(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithVersionButWithoutMigration(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithoutMigrationFunction(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithoutMigrationToFirstVersion(Leaf):
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
    version='',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithoutVersionAndWithFirstMigration(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithBrokenMigrationPath(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithIdenticalVersions(Leaf):
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
    version='1.0.0',
    options={'valid_option': int,
             'type_option': type},
    inputs={},
    outputs={'valid_output': int},
    max_children=0,
    option_wirings=[]))
class NodeWithWorkingMigrations(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithWorkingMigrationsThatChangesTree(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithGetOptionException(Leaf):
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
    version='1.0.0',
    options={'valid_option': int},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithAddOptionException(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithAddInputException(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={'valid_output': int},
    max_children=0,
    option_wirings=[]))
class NodeWithAddOutputException(Leaf):
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
    version='1.0.0',
    options={},
    inputs={},
    outputs={},
    max_children=0,
    option_wirings=[]))
class NodeWithOptionrefException(Leaf):
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
