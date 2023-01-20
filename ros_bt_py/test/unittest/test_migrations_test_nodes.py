#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022 FZI Forschungszentrum Informatik
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
#    * Neither the name of the {copyright_holder} nor the names of its
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
#  -------- END LICENSE BLOCK --------
import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.testing_nodes.migrations_test_nodes import (
    NodeWithoutVersion,
    NodeWithVersionButWithoutMigration,
    NodeWithoutMigrationFunction,
    NodeWithoutMigrationToFirstVersion,
    NodeWithoutVersionAndWithFirstMigration,
    NodeWithBrokenMigrationPath,
    NodeWithIdenticalVersions,
    NodeWithWorkingMigrations,
    NodeWithWorkingMigrationsChangeType,
    NodeWithWorkingMigrationsThatChangesTree,
    NodeWithGetOptionException,
    NodeWithAddOptionException,
    NodeWithAddInputException,
    NodeWithAddOutputException,
    NodeWithOptionrefException,
)

from ros_bt_py.testing_nodes.migrations_test_nodes_without_migrations import (
    NodeWithoutVersionAndWithoutMigrationsModule,
    NodeWithVersionAndWithoutMigrationsModule,
)


class TestMigrationsTestNodes(unittest.TestCase):
    def testTestNodes(self):
        # do the "standard" tests
        list_of_nodes = [
            NodeWithoutVersion(),
            NodeWithVersionButWithoutMigration(),
            NodeWithoutMigrationFunction(),
            NodeWithoutMigrationToFirstVersion(),
            NodeWithoutVersionAndWithFirstMigration(),
            NodeWithBrokenMigrationPath(),
            NodeWithIdenticalVersions(),
            NodeWithWorkingMigrations({"valid_option": 42, "type_option": int}),
            NodeWithWorkingMigrationsThatChangesTree(),
            NodeWithGetOptionException(),
            NodeWithAddOptionException({"valid_option": 42}),
            NodeWithAddInputException(),
            NodeWithAddOutputException(),
            NodeWithOptionrefException(),
            NodeWithoutVersionAndWithoutMigrationsModule(),
            NodeWithVersionAndWithoutMigrationsModule(),
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

    def testTestNodeWithInput(self):
        node = NodeWithWorkingMigrationsChangeType({"change_type": "hi"})
        node.setup()
        node.inputs["change_type"] = "hi"
        node.tick()
        self.assertEqual(node.state, NodeMsg.SUCCEEDED)

        node.reset()
        self.assertEqual(node.state, NodeMsg.IDLE)

        node.untick()
        self.assertEqual(node.state, NodeMsg.IDLE)

        node.shutdown()
        self.assertEqual(node.state, NodeMsg.SHUTDOWN)
