from collections import OrderedDict
from copy import deepcopy
from functools import wraps
import importlib
import inspect
try:  # pragma: no cover
    from itertools import izip
except ImportError:  # pragma: no cover
    izip = zip
import os
import sys
import traceback
import yaml

import genpy
import rospkg
import rospy

from ros_bt_py_msgs.msg import NodeData as NodeDataMsg, Tree
from ros_bt_py_msgs.srv import GetAvailableNodesRequest, MigrateTreeRequest, MigrateTreeResponse

from ros_bt_py.node import increment_name, load_node_module
from ros_bt_py.node_config import OptionRef
from ros_bt_py.node_data import NodeData

from ros_bt_py.exceptions import MigrationException

from ros_bt_py.ros_helpers import LoggerLevel
from ros_bt_py.helpers import get_default_value, json_encode, json_decode


def load_migration_module(package_name):
    """Import the named module at run-time.
    """
    try:
        return importlib.import_module(package_name)
    except (ImportError, ValueError) as e:
        return None


class MigrationManager(object):
    """Checks the loaded node modules for missing version tags, incorrect migrations, etc.
    """
    def __init__(self, tree_manager):
        rospy.loginfo('initializing MigrationManager')
        self.rospack = rospkg.RosPack()
        self.migrations_classes = {}
        self.tree_manager = tree_manager
        response = self.tree_manager.get_available_nodes(request=GetAvailableNodesRequest())
        available_nodes = response.available_nodes
        # sort the list by module then name:
        available_nodes.sort(key=lambda node: (node.module, node.node_class))
        for node in available_nodes:
            self.check_for_available_migration(node)

    def check_for_available_migration(self, node, old_node=False):
        # check if a migration for this node exists
        index = node.module.rindex('.')
        migrations_module = node.module[:index] + '.migrations' + node.module[index:]
        migrations_module_name = migrations_module + '.' + node.node_class
        node_module_migrations = load_migration_module(migrations_module)
        module_name = node.module + '.' + node.node_class

        if node_module_migrations is None:
            if node.version == '':
                rospy.logwarn(
                    '%s has no version information, please add a version '
                    'and a migration to that version!' % (
                        module_name))
                Migration.migration_info.setdefault(
                    migrations_module_name, MigrationInfoCollection(valid=True))
            else:
                rospy.logerr(
                    '%s has a version (%s) but no migration, '
                    'please add the missing migration "%s"!' % (
                        module_name, node.version, migrations_module))
        else:
            # check if a migrations class is available
            migrations_class = getattr(node_module_migrations, node.node_class, None)
            if migrations_class is None and not old_node:
                if node.version == '':
                    rospy.logwarn(
                        '%s has no version information, please add a version '
                        'and a migration to that version!' % (
                            module_name))
                else:
                    rospy.logwarn(
                        '%s has a version (%s) but no migration, '
                        'please add the missing migration "%s"!' % (
                            module_name, node.version, migrations_module))
            else:
                try:
                    m = migrations_class(node, Tree())
                    self.migrations_classes[migrations_module_name] = m
                    if (not m.migration_info
                            or migrations_module_name not in m.migration_info):
                        rospy.logwarn(
                            '%s has a migration class (%s) but no migration functions, '
                            'please fix this!' % (
                                module_name,
                                migrations_module_name))
                    else:
                        module_migration_info = m.migration_info[migrations_module_name].info
                        valid = True
                        # check if the first migration is not from "" to some version number
                        if module_migration_info[0].from_version != '':
                            rospy.logwarn(
                                '%s has no migration from "" to the first version, '
                                'consider adding one. The first migration is "%s" => "%s".' % (
                                    module_name,
                                    module_migration_info[0].from_version,
                                    module_migration_info[0].to_version)
                            )
                            valid = False
                        else:
                            if node.version == '' and not old_node:
                                rospy.logwarn(
                                    '%s has no version information, but a migration "%s" '
                                    'to a first version ("%s" => "%s") please add a version '
                                    'and a migration to that version!' % (
                                        module_name,
                                        module_migration_info[0].function,
                                        module_migration_info[0].from_version,
                                        module_migration_info[0].to_version)
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
                                    'ARE IDENTICAL! This is a bug, please fix it!' % (
                                        module_name,
                                        migration_info.function,
                                        migration_info.from_version,
                                        migration_info.to_version)
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
                                    '%s has a broken migration! '
                                    '"%s" => "%s" is NOT AVAILABLE! '
                                    'This is a bug, please fix it!' % (
                                        module_name,
                                        current_version,
                                        first)
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
                                    'This is a bug, please fix it!' % (
                                        module_name,
                                        module_migration_info[-1].function,
                                        module_migration_info[-1].from_version,
                                        module_migration_info[-1].to_version,
                                        node.version)
                                )
                                valid = False
                        m.migration_info[migrations_module_name].valid = valid
                except TypeError as e:
                    rospy.logerr(traceback.format_exc())
                    rospy.logwarn('%s: no migration available' % (
                        module_name))

    def check_node_versions(self, request):
        load_response = self.tree_manager.load_tree_from_file(request)

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
                    'version in tree: "%s", loaded: "%s"', msg.version, c._node_config.version)

                if msg.version != c._node_config.version:
                    perform_migration = True

        return MigrateTreeResponse(migrated=perform_migration, success=True)

    def migrate_tree(self, request):
        load_response = self.tree_manager.load_tree_from_file(request)

        if not load_response.success:
            return load_response

        tree = load_response.tree

        migration_performed = False
        modified_tree = deepcopy(tree)

        for msg in tree.nodes:
            index = msg.module.rindex('.')
            migrations_module = msg.module[:index] + '.migrations' + msg.module[index:]
            migrations_module_name = migrations_module + '.' + msg.node_class

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
                        msg = 'Migration failed: %s' % e
                        m.logwarn(msg)
                        return MigrateTreeResponse(error_message=msg)
                except TypeError as e:
                    msg = '%s.%s: no migration available: %s' % (msg.module, msg.node_class, e)
                    rospy.logerr(msg)
                    return MigrateTreeResponse(error_message=msg)

        return MigrateTreeResponse(migrated=migration_performed, success=True, tree=modified_tree)


class FakeNode(object):
    def __init__(self, name):
        self.name = name
        self.parent = None


class MigrationInfoCollection(object):
    def __init__(self, valid=False):
        self.info = []
        self.valid = valid


class MigrationInfo(object):
    def __init__(self, from_version, to_version, changelog):
        self.from_version = from_version
        self.to_version = to_version
        self.changelog = changelog
        self.function = None


class Migration(object):
    """Helpers for node migrations
    """
    migration_info = {}

    class MigrationDecorator(object):
        def __init__(self, from_version, to_version, changelog='no changelog provided'):
            self.module_name = None
            self.migration_info = MigrationInfo(
                from_version=from_version, to_version=to_version, changelog=changelog)

        def __call__(self, func, *args, **kwargs):
            current_frame = inspect.currentframe()
            outer_frame = inspect.getouterframes(current_frame, context=0)[1]
            try:
                class_name = outer_frame[3]
                module_path = outer_frame[1]
                for path in sys.path:
                    if module_path.startswith(path):
                        self.module_name = module_path[len(path):].replace(
                            os.path.sep, '.').rstrip('.py').strip('.') + '.' + class_name
            finally:
                # do not leak frame references
                del outer_frame
                del current_frame

            def inner_func(*args, **kwargs):
                # call self._change_version()
                args[0]._change_version(
                    version=self.migration_info.to_version,
                    changelog=self.migration_info.changelog)
                return func(*args, **kwargs)
            self.migration_info.function = func.__name__
            Migration.migration_info.setdefault(
                self.module_name, MigrationInfoCollection()).info.append(self.migration_info)
            # sort the list of migration functions by from_version
            Migration.migration_info[self.module_name].info.sort(
                key=lambda m: (m.from_version))
            return inner_func

    def __init__(self, msg, tree):
        self.msg = deepcopy(msg)
        self.tree = deepcopy(tree)

    def migrate(self):
        self.loginfo('starting migration')
        self._do_migrate()
        self.loginfo('finished migration')
        return (self.msg, self.tree)

    def _do_migrate(self):
        module_name = self.__module__ + '.' + self.__class__.__name__
        if module_name not in Migration.migration_info:
            self.logwarn('please add a migration')
        elif not Migration.migration_info[module_name].valid:
            raise MigrationException('Migration not valid')
        else:
            # migrate away:
            for migration_info in Migration.migration_info[module_name].info:
                if self.msg.version != migration_info.from_version:
                    rospy.logwarn(
                        'currently at version "%s", skipping migration from "%s" to "%s"',
                        self.msg.version,
                        migration_info.from_version, migration_info.to_version)
                else:
                    rospy.loginfo('running migration...( current: "%s") "%s" to "%s"',
                                  self.msg.version,
                                  migration_info.from_version, migration_info.to_version)
                    # calling migration function
                    getattr(self, migration_info.function)()

    def change_node_class(self, new_node_class):
        self.msg.node_class = new_node_class

    def change_module(self, new_module):
        self.msg.module = new_module

    def _change_version(self, version, changelog='no changelog provided'):
        """Changes the version field and logs the changes (if provided)
        """
        self.msg.version = version

    def get_name(self):
        return self.msg.name

    def get_option(self, key):
        for option in self.msg.options:
            if option.key == key:
                return json_decode(option.serialized_value)
        raise MigrationException('option key "%s" does not exists!' % key)

    def add_option(self, key, data_type, initial_value=None, static=False):
        for option in self.msg.options:
            if option.key == key:
                raise MigrationException('option key "%s" already exists!' % option.key)
        self.msg.options.append(
            self._create_node_data(
                key=key,
                data_type=data_type,
                initial_value=initial_value,
                static=static))

    def add_input(self, key, data_type):
        for node_input in self.msg.inputs:
            if node_input.key == key:
                raise MigrationException('input key "%s" already exists!' % node_input.key)
        self.msg.inputs.append(
            self._create_node_data(
                key=key,
                data_type=data_type))

    def add_output(self, key, data_type):
        for node_output in self.msg.outputs:
            if node_output.key == key:
                raise MigrationException('output key "%s" already exists!' % node_output.key)
        self.msg.outputs.append(
            self._create_node_data(
                key=key,
                data_type=data_type))

    def remove_option(self, key):
        # TODO: exception on removal error?
        self.msg.options = [option for option in self.msg.options if option.key != key]

    def rename_option(self, old_key, new_key):
        # TODO: exception on rename error?
        for option in self.msg.options:
            if option.key == old_key:
                option.key = new_key
                break

    def make_name_unique(self, name):
        node_names = []
        for node in self.tree.nodes:
            node_names.append(node.name)
        while name in node_names:
            name = increment_name(name)
        return name

    def get_parent_node(self):
        for node in self.tree.nodes:
            if self.msg.name in node.child_names:
                return node
        return None

    def replace_at_parent(self, new_node_name):
        parent = self.get_parent_node()
        child_index = -1
        for index, child in enumerate(parent.child_names):
            if child == self.msg.name:
                child_index = index
                break
        # TODO, check for errors
        parent.child_names[child_index] = new_node_name

    def add_node(self, node):
        node.name = self.make_name_unique(node.name)
        self.tree.nodes.append(node.to_msg())
        return node.name

    def add_node_msg(self, node_msg):
        node_msg.name = self.make_name_unique(node_msg.name)
        self.tree.nodes.append(node_msg)
        return node_msg.name

    def make_node(self, name):
        """Makes a node with the given name
        """
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
                        'Option key "%s" referenced by OptionRef does not exist!' % (data_type))
            initial_value = get_default_value(data_type)
        data = NodeData(data_type=data_type,
                        initial_value=initial_value,
                        static=static)
        return NodeDataMsg(key=key,
                           serialized_value=data.get_serialized(),
                           serialized_type=data.get_serialized_type())

    # Logging methods - these just use the ROS logging framework, but add the
    # name and type of the node so it's easier to trace errors.

    def logdebug(self, message):
        """Wrapper for :func:rospy.logdebug

        Adds this node's name, module and class to the given message"""
        rospy.logdebug('%s (%s.%s): %s',
                       self.msg.name,
                       self.msg.module,
                       self.msg.node_class,
                       message)

    def loginfo(self, message):
        """Wrapper for :func:rospy.loginfo

        Adds this node's name, module and class to the given message"""
        rospy.loginfo('%s (%s.%s): %s',
                      self.msg.name,
                      self.msg.module,
                      self.msg.node_class,
                      message)

    def logwarn(self, message):
        """Wrapper for :func:rospy.logwarn

        Adds this node's name, module and class to the given message"""
        rospy.logwarn('%s (%s.%s): %s',
                      self.msg.name,
                      self.msg.module,
                      self.msg.node_class,
                      message)

    def logerr(self, message):
        """Wrapper for :func:rospy.logerr

        Adds this node's name, module and class to the given message"""
        rospy.logerr('%s (%s.%s): %s',
                     self.msg.name,
                     self.msg.module,
                     self.msg.node_class,
                     message)

    def logfatal(self, message):
        """Wrapper for :func:rospy.logfatal

        Adds this node's name, module and class to the given message"""
        rospy.logfatal('%s (%s.%s): %s',
                       self.msg.name,
                       self.msg.module,
                       self.msg.node_class,
                       message)


migration = Migration.MigrationDecorator
