import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg, Tree
from ros_bt_py.exceptions import BehaviorTreeException, MigrationException
from ros_bt_py.testing_nodes.migrations.migrations_test_nodes import (
    NodeWithoutMigrationFunction,
    NodeWithoutMigrationToFirstVersion,
    NodeWithoutVersionAndWithFirstMigration,
    NodeWithBrokenMigrationPath,
    NodeWithIdenticalVersions,
)

from ros_bt_py.testing_nodes.migrations_test_nodes import (
    NodeWithoutMigrationFunction as NodeWithoutMigrationFunctionNode,
    NodeWithoutMigrationToFirstVersion as NodeWithoutMigrationToFirstVersionNode,
    NodeWithoutVersionAndWithFirstMigration as NodeWithoutVersionAndWithFirstMigrationNode,
    NodeWithBrokenMigrationPath as NodeWithBrokenMigrationPathNode,
    NodeWithIdenticalVersions as NodeWithIdenticalVersionsNode,
)


class TestMigrationsOfMigrationsTestNodes(unittest.TestCase):
    def testMigrationNodes(self):
        tree = Tree()
        node = NodeWithoutMigrationToFirstVersionNode()
        migration = NodeWithoutMigrationToFirstVersion(node.to_msg(), tree)

        migration.first_migration()

        node = NodeWithoutVersionAndWithFirstMigrationNode()
        migration = NodeWithoutVersionAndWithFirstMigration(node.to_msg(), tree)

        migration.first_migration()

        node = NodeWithBrokenMigrationPathNode()
        migration = NodeWithBrokenMigrationPath(node.to_msg(), tree)

        migration.first_migration()
        migration.broken_migration()

        node = NodeWithIdenticalVersionsNode()
        migration = NodeWithIdenticalVersions(node.to_msg(), tree)

        migration.first_migration()
        migration.broken_migration()

        node = NodeWithoutMigrationFunctionNode()
        migration = NodeWithoutMigrationFunction(node.to_msg(), tree)
        migration.migrate()
