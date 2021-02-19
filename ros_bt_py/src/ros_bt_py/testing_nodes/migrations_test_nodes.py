from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    version='',
    options={},
    inputs={},
    outputs={},
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
    options={'change_type': str},
    inputs={'change_type': str},
    outputs={'change_type': str},
    max_children=0))
class NodeWithWorkingMigrationsChangeType(Leaf):
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
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
    max_children=0))
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
