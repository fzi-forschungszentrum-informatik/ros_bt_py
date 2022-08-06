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

try:
    import unittest.mock as mock
except ImportError:
    import mock

import sys
import time

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import Tree
from ros_bt_py_msgs.srv import (
    MigrateTreeRequest,
    GetAvailableNodesRequest,
    MigrateTreeResponse,
)

from ros_bt_py.testing_nodes import migrations_test_nodes

from ros_bt_py.migration import MigrationManager
from ros_bt_py.tree_manager import TreeManager


class TestMigrationManager(unittest.TestCase):
    def testSimpleMigration(self):
        tree_manager = TreeManager()
        request = GetAvailableNodesRequest(
            node_modules=[
                "ros_bt_py.testing_nodes.migrations_test_nodes",
                "ros_bt_py.testing_nodes.migrations_test_nodes_without_migrations",
            ]
        )

        response = get_available_nodes(request)
        self.assertTrue(response.success)

        migration_manager = MigrationManager(tree_manager=tree_manager)

        tree = Tree()
        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_nodewithworkingmigrations.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_nodewithworkingmigrations_change_type.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        node = migrations_test_nodes.NodeWithWorkingMigrationsChangeType(
            {"change_type": "hi"}
        )
        node.shutdown()
        node_msg = node.to_msg()
        self.assertEqual(migrate_reply.tree.nodes[0], node_msg)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_nodewithworkingmigrations2.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migration_nodewithbrokenmigrationpath.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # package, but file does not exist
        tree.path = "package://ros_bt_py/etc/trees/notareal.file"
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # file does not exist
        tree.path = "/notareal.file"
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # file does not exist
        tree.path = "file://"
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/etc/trees/two_trees.yaml"
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/etc/trees/empty.yaml"
        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # test get/add exceptions
        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_nodewithgetoptionexception.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_nodewithaddoptionexception.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_nodewithaddinputexception.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_nodewithaddoutputexception.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_nodewithoptionrefexception.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

    def testNodesMigrations(self):
        tree_manager = TreeManager()
        request = GetAvailableNodesRequest(
            node_modules=[
                "ros_bt_py.nodes.action",
                "ros_bt_py.nodes.constant",
                "ros_bt_py.nodes.sequence",
            ]
        )

        response = get_available_nodes(request)
        self.assertTrue(response.success)

        migration_manager = MigrationManager(tree_manager=tree_manager)

        tree = Tree()
        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_action.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_service.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_serviceinput.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_waitforservice.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_compare.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_decorators.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_decorators_repeat_if_fail.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_fallback.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_file.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_fileinput.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_yamlinput.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_yamllistinput.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_yamllistoption.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_yamldictinput.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_format.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_getters.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_io.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_list_getlistelementoption.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_list_insertinlist.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_list_isinlist.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_list_iteratelist.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_list_listlength.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_log.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_log_error.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_log_warn.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_log_err.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_mock_nodes.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_parallel_if_remote.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_parallel.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_passthrough_node.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_random_number.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_remote_slot.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_ros_param.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_sequence.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_setters.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_shovable.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_subtree.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_topic.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_wait.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        # ros_nodes
        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_fields_to_message.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_message_to_fields.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_message_from_const_dict.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/"
            "migrations_message_from_dict.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_enum.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_maths_convert.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_maths_unary.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

        tree.path = (
            "package://ros_bt_py/test/testdata/trees/" "migrations_maths_binary.yaml"
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

    @mock.patch("ros_bt_py.migration.getattr")
    def testMigrateTreeError(self, mock_getattr):
        tree_manager = TreeManager()
        request = GetAvailableNodesRequest(node_modules=["ros_bt_py.nodes.constant"])

        response = get_available_nodes(request)
        self.assertTrue(response.success)

        migration_manager = MigrationManager(tree_manager=tree_manager)

        tree = Tree(
            nodes=[NodeMsg(module="ros_bt_py.nodes.constant", node_class="Constant")]
        )
        migrate_request = MigrateTreeRequest(tree=tree)

        migration_manager.tree_manager.load_tree_from_file = mock.MagicMock()
        migration_manager.tree_manager.load_tree_from_file.return_value = (
            MigrateTreeResponse(success=Tree, tree=tree)
        )

        mock_getattr.return_value = None

        migrate_reply = migration_manager.migrate_tree(migrate_request)
        self.assertFalse(migrate_reply.success)

    def testCheckNodeVersions(self):
        tree_manager = TreeManager()
        request = GetAvailableNodesRequest(
            node_modules=[
                "ros_bt_py.nodes.action",
                "ros_bt_py.nodes.constant",
                "ros_bt_py.nodes.sequence",
            ]
        )

        response = get_available_nodes(request)
        self.assertTrue(response.success)

        migration_manager = MigrationManager(tree_manager=tree_manager)

        tree = Tree()
        # package, but file does not exist
        tree.path = "package://ros_bt_py/etc/trees/notareal.file"
        migrate_request = MigrateTreeRequest(tree=tree)
        migrate_reply = check_node_versions(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # file does not exist
        tree.path = "/notareal.file"
        migrate_request = MigrateTreeRequest(tree=tree)
        migrate_reply = check_node_versions(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        # file does not exist
        tree.path = "file://"
        migrate_request = MigrateTreeRequest(tree=tree)
        migrate_reply = check_node_versions(migrate_request)
        self.assertFalse(migrate_reply.success)
        self.assertFalse(migrate_reply.migrated)

        tree.path = "package://ros_bt_py/test/testdata/trees/" "migrations_action.yaml"
        migrate_request = MigrateTreeRequest(tree=tree)

        migrate_reply = check_node_versions(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

    def testCheckNodeVersionsNone(self):
        tree_manager = TreeManager()
        request = GetAvailableNodesRequest(node_modules=["ros_bt_py.nodes.constant"])

        response = get_available_nodes(request)
        self.assertTrue(response.success)

        migration_manager = MigrationManager(tree_manager=tree_manager)

        tree = Tree(nodes=[NodeMsg(module="ros_bt_py.does_not_exist", node_class="")])
        migrate_request = MigrateTreeRequest(tree=tree)

        migration_manager.tree_manager.load_tree_from_file = mock.MagicMock()
        migration_manager.tree_manager.load_tree_from_file.return_value = (
            MigrateTreeResponse(success=Tree, tree=tree)
        )

        migrate_reply = check_node_versions(migrate_request)
        self.assertTrue(migrate_reply.success)
        self.assertTrue(migrate_reply.migrated)

    @mock.patch("ros_bt_py.migration.izip")
    def testCheckForAvailableMigration(self, mock_izip):
        tree_manager = TreeManager()
        request = GetAvailableNodesRequest(node_modules=["ros_bt_py.nodes.constant"])

        response = get_available_nodes(request)
        self.assertTrue(response.success)

        mock_izip.side_effect = TypeError()

        migration_manager = MigrationManager(tree_manager=tree_manager)
        self.assertIsNotNone(migration_manager)
