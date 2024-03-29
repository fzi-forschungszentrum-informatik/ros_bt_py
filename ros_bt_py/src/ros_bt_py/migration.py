# Copyright 2018-2023 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Module providing the migration system, allowing nodes to be updated before runtime.

This includes a versioning system and automated patches.
"""
import importlib
import inspect
from copy import deepcopy
from typing import Any

try:  # pragma: no cover
    from itertools import izip
except ImportError:  # pragma: no cover
    izip = zip
import os
import sys
import traceback

import rospkg
import rospy

from ros_bt_py.tree_manager import get_available_nodes, TreeManager, load_tree_from_file
from ros_bt_py.exceptions import MigrationException
from ros_bt_py.helpers import get_default_value, json_decode
from ros_bt_py.node import increment_name, load_node_module
from ros_bt_py.node_config import OptionRef
from ros_bt_py.node_data import NodeData

from ros_bt_py_msgs.msg import NodeData as NodeDataMsg, Tree, Node
from ros_bt_py_msgs.srv import (
    GetAvailableNodesRequest,
    MigrateTreeResponse,
    MigrateTreeRequest,
)


def load_migration_module(package_name):
    """Import the named module at run-time."""
    try:
        return importlib.import_module(package_name)
    except (ImportError, ValueError):
        return None


def check_node_versions(request: MigrateTreeRequest) -> MigrateTreeResponse:
    """Check the provided tree against the current version loaded from disk.

    The function loads the tree from disk and compares all nodes against the nodes
    loaded with the BT library.

    :param request: Request to load a tree from the disk.
    :type request: LoadTreeRequest
    :return: Response if the nodes in the tree were migrated.
    :rtype: MigrateTreeResponse
    """
    load_response = load_tree_from_file(request)

    if not load_response.success:
        return load_response

    tree = load_response.tree

    perform_migration = False

    for msg in tree.nodes:
        m = load_node_module(msg.module)
        c = getattr(m, msg.node_class, None)

        if c is None:
            perform_migration = True
        else:
            rospy.loginfo(
                'version in tree: "%s", loaded: "%s"',
                msg.version,
                c._node_config.version,
                logger_name="migration",
            )

            if msg.version != c._node_config.version:
                perform_migration = True

    return MigrateTreeResponse(migrated=perform_migration, success=True)


class MigrationManager(object):
    """Checks the loaded node modules for missing version tags, incorrect migrations, etc."""

    def __init__(self, tree_manager: TreeManager):
        """Create a new MigrationManager for a specific TreeManager and its tree."""
        rospy.loginfo("initializing MigrationManager", logger_name="migration")
        self.rospack = rospkg.RosPack()
        self.migrations_classes = {}
        self.tree_manager = tree_manager
        response = get_available_nodes(request=GetAvailableNodesRequest())
        available_nodes = response.available_nodes
        # sort the list by module then name:
        available_nodes.sort(key=lambda node: (node.module, node.node_class))
        for node in available_nodes:
            self.check_for_available_migration(node)

    def check_for_available_migration(self, node, old_node=False):
        """Check if any migration for the provided node is available.

        This node will check for availbale migrations and return an error if none or incomplete
        ones are found.

        :param node: Behavior tree node to check for migrations.
        :type node: Node
        :param old_node: If the node is a old node no version info is expected, defaults to False.
        :type old_node: bool, optional
        """
        # check if a migration for this node exists
        index = node.module.rindex(".")
        migrations_module = f"{node.module[:index]}.migrations{node.module[index:]}"
        migrations_module_name = f"{migrations_module}.{node.node_class}"
        node_module_migrations = load_migration_module(migrations_module)
        module_name = f"{node.module}.{node.node_class}"

        if node.module.find("capability") != -1:
            rospy.loginfo_throttle(10, "Migrations for capabilities are disabled!")
            return

        if node_module_migrations is None:
            if node.version == "":
                rospy.logwarn_throttle(
                    30,
                    f"{module_name} has no version information, please add a version and"
                    " a migration to that version!",
                    logger_name="migration",
                )
                Migration.migration_info.setdefault(
                    migrations_module_name, MigrationInfoCollection(valid=True)
                )
            else:
                rospy.logwarn_throttle(
                    30,
                    "%s has a version (%s) but no migration, "
                    'please add the missing migration "%s"!'
                    % (module_name, node.version, migrations_module),
                    logger_name="migration",
                )
        else:
            # check if a migrations class is available
            migrations_class = getattr(node_module_migrations, node.node_class, None)
            if migrations_class is None and not old_node:
                if node.version == "":
                    rospy.logwarn(
                        "%s has no version information, please add a version "
                        "and a migration to that version!" % (module_name),
                        logger_name="migration",
                    )
                else:
                    rospy.logwarn(
                        "%s has a version (%s) but no migration, "
                        'please add the missing migration "%s"!'
                        % (module_name, node.version, migrations_module),
                        logger_name="migration",
                    )
            else:
                try:
                    m = migrations_class(node, Tree())
                    self.migrations_classes[migrations_module_name] = m
                    if (
                        not m.migration_info
                        or migrations_module_name not in m.migration_info
                    ):
                        rospy.logwarn(
                            "%s has a migration class (%s) but no migration functions, "
                            "please fix this!" % (module_name, migrations_module_name),
                            logger_name="migration",
                        )
                    else:
                        module_migration_info = m.migration_info[
                            migrations_module_name
                        ].info
                        valid = True
                        # check if the first migration is not from "" to some version number
                        if module_migration_info[0].from_version != "":
                            rospy.logwarn(
                                '%s has no migration from "" to the first version, '
                                'consider adding one. The first migration is "%s" => "%s".'
                                % (
                                    module_name,
                                    module_migration_info[0].from_version,
                                    module_migration_info[0].to_version,
                                ),
                                logger_name="migration",
                            )
                            valid = False
                        else:
                            if node.version == "" and not old_node:
                                rospy.logwarn(
                                    '%s has no version information, but a migration "%s" '
                                    'to a first version ("%s" => "%s") please add a version '
                                    "and a migration to that version!"
                                    % (
                                        module_name,
                                        module_migration_info[0].function,
                                        module_migration_info[0].from_version,
                                        module_migration_info[0].to_version,
                                    ),
                                    logger_name="migration",
                                )
                                valid = False
                        migration_path = []
                        for migration_info in module_migration_info:
                            # error if the from_version and to_version are identical
                            # this most certainly is a bug
                            if migration_info.from_version == migration_info.to_version:
                                rospy.logerr(
                                    '%s has a broken migration "%s": '
                                    'from_version: "%s" AND to_version: "%s" '
                                    "ARE IDENTICAL! This is a bug, please fix it!"
                                    % (
                                        module_name,
                                        migration_info.function,
                                        migration_info.from_version,
                                        migration_info.to_version,
                                    ),
                                    logger_name="migration",
                                )
                                valid = False
                            migration_path.append(migration_info.from_version)
                            migration_path.append(migration_info.to_version)

                        # check if a complete migration path exists
                        current_version = migration_path[0]
                        paths = iter(migration_path)
                        for first, second in izip(paths, paths):
                            if first == current_version:
                                current_version = second
                            else:
                                rospy.logerr(
                                    "%s has a broken migration! "
                                    '"%s" => "%s" is NOT AVAILABLE! '
                                    "This is a bug, please fix it!"
                                    % (module_name, current_version, first),
                                    logger_name="migration",
                                )
                                valid = False
                                break

                        # check if the last migration leads to the current version of the node
                        if len(migration_path) >= 2:
                            if migration_path[-1] != node.version and not old_node:
                                rospy.logerr(
                                    '%s has a broken migration "%s": '
                                    'last migration ("%s" => "%s") DOES NOT lead to '
                                    'current node version "%s"! '
                                    "This is a bug, please fix it!"
                                    % (
                                        module_name,
                                        module_migration_info[-1].function,
                                        module_migration_info[-1].from_version,
                                        module_migration_info[-1].to_version,
                                        node.version,
                                    ),
                                    logger_name="migration",
                                )
                                valid = False
                        m.migration_info[migrations_module_name].valid = valid
                except TypeError as e:
                    rospy.logwarn(
                        f"{module_name}: no migration available, the exception was: "
                        f"{e}",
                        logger_name="migration",
                    )

    def migrate_tree(self, request: MigrateTreeRequest) -> MigrateTreeResponse:
        """Migrate a tree by applying all available migrations to nodes.

        :param request: The request containg the tree to be migrated.
        :type request: MigrateTreeRequest
        :return: Response if the migration was successfull.
        :rtype: MigrateTreeResponse
        """
        load_response = load_tree_from_file(request)

        if not load_response.success:
            return load_response

        tree = load_response.tree

        migration_performed = False
        modified_tree = deepcopy(tree)

        for msg in tree.nodes:
            index = msg.module.rindex(".")
            migrations_module = f"{msg.module[:index]}.migrations{msg.module[index:]}"
            migrations_module_name = f"{migrations_module}.{msg.node_class}"

            node_module_migrations = load_node_module(migrations_module)

            if migrations_module_name not in self.migrations_classes:
                # FIXME: this is important for nodes that changed node_class or module
                # this could probably be avoided if all migrations were loaded at startup,
                # not depending on node availability
                self.check_for_available_migration(node=msg, old_node=True)
            if migrations_module_name in self.migrations_classes:
                old_version = msg.version
                migrations_class = getattr(node_module_migrations, msg.node_class, None)
                try:
                    m = migrations_class(msg, modified_tree)
                    try:
                        new_msg, new_tree = m.migrate()
                        if old_version != new_msg.version:
                            if modified_tree != new_tree:
                                # the tree itself has been modified, so update it
                                modified_tree = new_tree
                            else:
                                modified_tree.nodes.remove(msg)
                                modified_tree.nodes.append(new_msg)
                            migration_performed = True
                    except MigrationException as e:
                        msg = f"Migration failed: {e}"
                        m.logwarn(msg)
                        return MigrateTreeResponse(error_message=msg)
                except TypeError as e:
                    msg = f"{msg.module}.{msg.node_class}: no migration available: {e}"
                    rospy.logerr(msg, logger_name="migration")
                    return MigrateTreeResponse(error_message=msg)

        return MigrateTreeResponse(
            migrated=migration_performed, success=True, tree=modified_tree
        )


class FakeNode:
    """Fake node for testing purposes."""

    def __init__(self, name: str):
        """Create a new fake node with a name.

        :param name: name of the fake node.
        :type name: str
        """
        self.name = name
        self.parent = None


class MigrationInfoCollection:
    """Collection of migration infos created on migration."""

    def __init__(self, valid=False):
        """Create a new collection of migration infos.

        :param valid: If all of the migrations were valid, defaults to False
        :type valid: bool, optional
        """
        self.info = []
        self.valid = valid


class MigrationInfo(object):
    """Information about a single migration with version info an a changelog."""

    def __init__(self, from_version: str, to_version: str, changelog: str):
        """Create a new migration information object describing one migration.

        :param from_version: Node version before the migration.
        :type from_version: str
        :param to_version: Node version after the migration.
        :type to_version: str
        :param changelog: _description_
        :type changelog: The changelog for the migration.
        """
        self.from_version = from_version
        self.to_version = to_version
        self.changelog = changelog
        self.function = None


class Migration:
    """Helpers for node migrations."""

    migration_info = {}

    class MigrationDecorator:
        """Decorator used to annotate migrations."""

        def __init__(self, from_version, to_version, changelog="no changelog provided"):
            """Create a new MigrationInformation from the Decorator."""
            self.module_name = None
            self.migration_info = MigrationInfo(
                from_version=from_version, to_version=to_version, changelog=changelog
            )

        def __call__(self, func, *args, **kwargs):
            """Register the migration with the collection of available migrations."""
            current_frame = inspect.currentframe()
            outer_frame = inspect.getouterframes(current_frame, context=0)[1]
            try:
                class_name = outer_frame[3]
                module_path = outer_frame[1]
                for path in sys.path:
                    if module_path.startswith(path):
                        self.module_name = (
                            module_path[len(path) :]
                            .replace(os.path.sep, ".")
                            .rstrip(".py")
                            .strip(".")
                            + "."
                            + class_name
                        )
            finally:
                # do not leak frame references
                del outer_frame
                del current_frame

            def inner_func(*args, **kwargs):
                # call self._change_version()
                args[0]._change_version(
                    version=self.migration_info.to_version,
                    changelog=self.migration_info.changelog,
                )
                return func(*args, **kwargs)

            self.migration_info.function = func.__name__
            Migration.migration_info.setdefault(
                self.module_name, MigrationInfoCollection()
            ).info.append(self.migration_info)
            # sort the list of migration functions by from_version
            Migration.migration_info[self.module_name].info.sort(
                key=lambda m: (m.from_version)
            )
            return inner_func

    def __init__(self, msg: Node, tree: Tree):
        """Create a new migration from a Node and a Tree.

        :param msg: Node instance to be migrated.
        :type msg: Node
        :param tree: Tree to migrate the node in.
        :type tree: Tree
        """
        self.msg = deepcopy(msg)
        self.tree = deepcopy(tree)

    def migrate(self):
        """Migrate the node within the tree."""
        self.loginfo("starting migration")
        self._do_migrate()
        self.loginfo("finished migration")
        return (self.msg, self.tree)

    def _do_migrate(self):
        module_name = f"{self.__module__}.{self.__class__.__name__}"
        if module_name not in Migration.migration_info:
            self.logwarn("please add a migration")
        elif not Migration.migration_info[module_name].valid:
            raise MigrationException("Migration not valid")
        else:
            # migrate away:
            for migration_info in Migration.migration_info[module_name].info:
                if self.msg.version != migration_info.from_version:
                    rospy.loginfo(
                        'currently at version "%s", skipping migration from "%s" to "%s"',
                        self.msg.version,
                        migration_info.from_version,
                        migration_info.to_version,
                        logger_name="migration",
                    )
                else:
                    rospy.loginfo(
                        'running migration...( current: "%s") "%s" to "%s"',
                        self.msg.version,
                        migration_info.from_version,
                        migration_info.to_version,
                        logger_name="migration",
                    )
                    # calling migration function
                    getattr(self, migration_info.function)()

    def change_node_class(self, new_node_class: str):
        """Change the class of the node in serialized format.

        :param new_node_class: The new class name for the node.
        :type new_node_class: str
        """
        self.msg.node_class = new_node_class

    def change_module(self, new_module: str):
        """Change the moule for the node class in serialized format.

        :param new_module: The new cpython module for the node.
        :type new_module: str
        """
        self.msg.module = new_module

    def _change_version(self, version: str, changelog="no changelog provided"):
        """Change the version field and logs the changes (if provided)."""
        self.msg.version = version

    def get_name(self) -> str:
        """Get the name of the node.

        :return: Name of the node.
        :rtype: str
        """
        return self.msg.name

    def get_option(self, key: str) -> Any:
        """Return an option form the node that is being migrated.

        :param key: The option key.
        :type key: str
        :raises MigrationException: If no option with this key exists.
        :return: The value of the option specified by the key.
        :rtype: Any
        """
        for option in self.msg.options:
            if option.key == key:
                return json_decode(option.serialized_value)
        raise MigrationException(f'option key "{key}" does not exists!')

    def add_option(self, key: str, data_type: Any, initial_value=None, static=False):
        """Add a new option to the node.

        :param key: The option key for the new option.
        :type key: str
        :param data_type: The data type for the new option.
        :type data_type: Any
        :param initial_value: The initial value of the type data_type, defaults to None
        :type initial_value: data_type, optional
        :param static: If the value can be changed or is static, defaults to False
        :type static: bool, optional
        :raises MigrationException: Raised if a option with the same key already exists.
        """
        for option in self.msg.options:
            if option.key == key:
                raise MigrationException(f'option key "{option.key}" already exists!')
        self.msg.options.append(
            self._create_node_data(
                key=key, data_type=data_type, initial_value=initial_value, static=static
            )
        )

    def add_input(self, key: str, data_type: Any):
        """Add a new input to the node.

        :param key: The key for the new input parameter.
        :type key: str
        :param data_type: The data type of the new input.
        :type data_type: Any
        :raises MigrationException: If a parameter with the same key already exists.
        """
        for node_input in self.msg.inputs:
            if node_input.key == key:
                raise MigrationException(
                    f'input key "{node_input.key}" already exists!'
                )
        self.msg.inputs.append(self._create_node_data(key=key, data_type=data_type))

    def add_output(self, key, data_type):
        """Add a new output to the node.

        :param key: The key for the new input parameter.
        :type key: str
        :param data_type: The data type of the new output.
        :type data_type: Any
        :raises MigrationException: If a parameter with the same key already exists.
        """
        for node_output in self.msg.outputs:
            if node_output.key == key:
                raise MigrationException(
                    f'output key "{node_output.key}" already exists!'
                )
        self.msg.outputs.append(self._create_node_data(key=key, data_type=data_type))

    def remove_option(self, key: str):
        """Remove a present option.

        :param key: Option to remove.
        :type key: str
        """
        # TODO: exception on removal error?
        self.msg.options = [option for option in self.msg.options if option.key != key]

    def rename_option(self, old_key: str, new_key: str):
        """Rename option to a new name.

        :param old_key: The old key for the option.
        :type old_key: str
        :param new_key: The new key for the option.
        :type new_key: str
        """
        # TODO: exception on rename error?
        for option in self.msg.options:
            if option.key == old_key:
                option.key = new_key
                break

    def change_input_type(self, key: str, data_type: Any):
        """Change input type for a existing inputs.

        :param key: Key for the input to use.
        :type key: str
        :param data_type: The new data type for the input.
        :type data_type: str
        """
        # TODO: exception on something really wrong
        for index, node_input in enumerate(self.msg.inputs):
            if node_input.key == key:
                node_data = self._create_node_data(key=key, data_type=data_type)
                node_data.serialized_value = "null"
                self.msg.inputs[index] = node_data

    def change_output_type(self, key, data_type):
        """Change output type for a existing outputs.

        :param key: Key for the input to use.
        :type key: str
        :param data_type: The new data type for the output.
        :type data_type: str
        """
        # TODO: exception on something really wrong
        for index, node_output in enumerate(self.msg.outputs):
            if node_output.key == key:
                node_data = self._create_node_data(key=key, data_type=data_type)
                node_data.serialized_value = "null"
                self.msg.outputs[index] = node_data

    def change_option_type(
        self,
        key,
        data_type,
        initial_value=None,
    ):
        """Change option type for a existing options.

        :param key: Key for the input to use.
        :type key: str
        :param data_type: The new data type for the output.
        :type data_type: str
        :param initial_value: The initial value of the type data_type, defaults to None
        :type initial_value: data_type, optional
        """
        # TODO: exception on something really wrong
        for index, node_option in enumerate(self.msg.options):
            if node_option.key == key:
                self.msg.options[index] = self._create_node_data(
                    key=key, data_type=data_type, initial_value=initial_value
                )

    def make_name_unique(self, name):
        """Make a unique node name."""
        node_names = []
        for node in self.tree.nodes:
            node_names.append(node.name)
        while name in node_names:
            name = increment_name(name)
        return name

    def get_parent_node(self):
        """Return the parent node of the node."""
        for node in self.tree.nodes:
            if self.msg.name in node.child_names:
                return node
        return None

    def replace_at_parent(self, new_node_name):
        """Move the node to a new parent."""
        parent = self.get_parent_node()
        child_index = -1
        for index, child in enumerate(parent.child_names):
            if child == self.msg.name:
                child_index = index
                break
        # TODO, check for errors
        parent.child_names[child_index] = new_node_name

    def add_node(self, node):
        """Add a node to the tree that is managed by the migration."""
        node.name = self.make_name_unique(node.name)
        self.tree.nodes.append(node.to_msg())
        return node.name

    def add_node_msg(self, node_msg):
        """Add a node from a msg that is managed by the migration."""
        node_msg.name = self.make_name_unique(node_msg.name)
        self.tree.nodes.append(node_msg)
        return node_msg.name

    def make_node(self, name):
        """Make a node with the given name."""
        return FakeNode(name=name)

    def _create_node_data(self, key, data_type, initial_value=None, static=False):
        if initial_value is None:
            if isinstance(data_type, OptionRef):
                available = False
                for option in self.msg.options:
                    if data_type.option_key == option.key:
                        data_type = json_decode(option.serialized_value)
                        available = True
                        break
                if not available:
                    raise MigrationException(
                        f'Option key "{data_type}" referenced by OptionRef does not exist!'
                    )
            initial_value = get_default_value(data_type)
        data = NodeData(data_type=data_type, initial_value=initial_value, static=static)
        return NodeDataMsg(
            key=key,
            serialized_value=data.get_serialized(),
            serialized_type=data.get_serialized_type(),
        )

    # Logging methods - these just use the ROS logging framework, but add the
    # name and type of the node so it's easier to trace errors.

    def logdebug(self, message):
        """Wrap :func:rospy.logdebug.

        Adds this node's name, module and class to the given message
        """
        rospy.logdebug(
            "%s (%s.%s): %s",
            self.msg.name,
            self.msg.module,
            self.msg.node_class,
            message,
            logger_name="migration",
        )

    def loginfo(self, message):
        """Wrap :func:rospy.loginfo.

        Adds this node's name, module and class to the given message
        """
        rospy.loginfo(
            "%s (%s.%s): %s",
            self.msg.name,
            self.msg.module,
            self.msg.node_class,
            message,
        )

    def logwarn(self, message):
        """Wrap :func:rospy.logwarn.

        Adds this node's name, module and class to the given message
        """
        rospy.logwarn(
            "%s (%s.%s): %s",
            self.msg.name,
            self.msg.module,
            self.msg.node_class,
            message,
        )

    def logerr(self, message):
        """Wrap :func:rospy.logerr.

        Adds this node's name, module and class to the given message
        """
        rospy.logerr(
            "%s (%s.%s): %s",
            self.msg.name,
            self.msg.module,
            self.msg.node_class,
            message,
        )

    def logfatal(self, message):
        """Wrap for :func:rospy.logfatal.

        Adds this node's name, module and class to the given message
        """
        rospy.logfatal(
            "%s (%s.%s): %s",
            self.msg.name,
            self.msg.module,
            self.msg.node_class,
            message,
        )


migration = Migration.MigrationDecorator
