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
