import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.testing_nodes.migrations_test_nodes import (
    NodeWithoutVersion, NodeWithVersionButWithoutMigration, NodeWithoutMigrationFunction,
    NodeWithoutMigrationToFirstVersion, NodeWithoutVersionAndWithFirstMigration,
    NodeWithBrokenMigrationPath, NodeWithIdenticalVersions, NodeWithWorkingMigrations,
    NodeWithWorkingMigrationsThatChangesTree, NodeWithGetOptionException,
    NodeWithAddOptionException, NodeWithAddInputException, NodeWithAddOutputException,
    NodeWithOptionrefException
)

from ros_bt_py.testing_nodes.migrations_test_nodes_without_migrations import (
    NodeWithoutVersionAndWithoutMigrationsModule,
    NodeWithVersionAndWithoutMigrationsModule
)


class TestMigrationsTestNodes(unittest.TestCase):
    def testTestNodes(self):
        # do the "standard" tests
        list_of_nodes = [
            NodeWithoutVersion(), NodeWithVersionButWithoutMigration(),
            NodeWithoutMigrationFunction(), NodeWithoutMigrationToFirstVersion(),
            NodeWithoutVersionAndWithFirstMigration(), NodeWithBrokenMigrationPath(),
            NodeWithIdenticalVersions(),
            NodeWithWorkingMigrations({'valid_option': 42,
                                       'type_option': int}),
            NodeWithWorkingMigrationsThatChangesTree(), NodeWithGetOptionException(),
            NodeWithAddOptionException({'valid_option': 42}), NodeWithAddInputException(),
            NodeWithAddOutputException(), NodeWithOptionrefException(),
            NodeWithoutVersionAndWithoutMigrationsModule(),
            NodeWithVersionAndWithoutMigrationsModule()
        ]
        for node in list_of_nodes:
            node.setup()
            node.tick()
            self.assertEqual(node.state, NodeMsg.SUCCEEDED)

            node.reset()
            self.assertEqual(node.state, NodeMsg.IDLE)

            node.untick()
            self.assertEqual(node.state, NodeMsg.IDLE)

            node.shutdown()
            self.assertEqual(node.state, NodeMsg.SHUTDOWN)
