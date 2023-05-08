#!/usr/bin/env python
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


from copy import deepcopy
from threading import Lock
import unittest

try:
    import unittest.mock as mock
except ImportError:
    import mock

import rospy

from ros_bt_py_msgs.msg import Tree
from ros_bt_py_msgs.srv import GetMessageFieldsRequest, GetPackageStructureRequest
from ros_bt_py_msgs.srv import (
    SaveTreeRequest,
    LoadTreeRequest,
    ControlTreeExecutionRequest,
)

from ros_bt_py.package_manager import PackageManager
from ros_bt_py.tree_manager import TreeManager

PKG = "ros_bt_py"


class TestPackageManager(unittest.TestCase):
    def setUp(self):
        self.message_list_msg = None
        self.packages_list_msg = None

        class Publisher(object):
            def __init__(self, callback):
                self.callback = callback

            def publish(self, data):
                self.callback(data)

        def set_message_list_msg(msg):
            self.message_list_msg = msg

        def set_packages_list_msg(msg):
            self.packages_list_msg = msg

        message_list_cb = Publisher(callback=set_message_list_msg)
        packages_list_cb = Publisher(callback=set_packages_list_msg)

        self.package_manager = PackageManager(
            publish_message_list_callback=message_list_cb,
            publish_packages_list_callback=packages_list_cb,
        )

    def testPackageManagerWithoutCallbacks(self):
        package_manager = PackageManager(
            publish_message_list_callback=None, publish_packages_list_callback=None
        )
        package_manager.publish_message_list()
        package_manager.publish_packages_list()

    def testPackageManagerWithCallbacks(self):
        self.package_manager.publish_message_list()
        self.package_manager.publish_packages_list()

    def testGetMessageFields(self):
        request = GetMessageFieldsRequest(
            message_type="ros_bt_py_msgs/Tree", service=False
        )
        response = self.package_manager.get_message_fields(request=request)

        self.assertTrue(response.success)
        self.assertEqual(len(response.field_names), 8)

        request = GetMessageFieldsRequest(
            message_type="ros_bt_py_msgs/LoadTreeRequest", service=True
        )
        response = self.package_manager.get_message_fields(request=request)

        self.assertTrue(response.success)
        self.assertEqual(len(response.field_names), 10)

        request = GetMessageFieldsRequest(
            message_type="ros_bt_py_msgs/ThisMessageDoesNotExist", service=False
        )
        response = self.package_manager.get_message_fields(request=request)

        self.assertFalse(response.success)

    def testGetMessageConstFields(self):
        request = GetMessageFieldsRequest(
            message_type="ros_bt_py_msgs/Tree", service=False
        )
        response = self.package_manager.get_message_constant_fields_handler(
            request=request
        )

        self.assertTrue(response.success)
        self.assertEqual(len(response.field_names), 7)

        request = GetMessageFieldsRequest(
            message_type="ros_bt_py_msgs/LoadTreeRequest", service=True
        )
        response = self.package_manager.get_message_constant_fields_handler(
            request=request
        )

        self.assertFalse(response.success)
        self.assertEqual(len(response.field_names), 0)

        request = GetMessageFieldsRequest(
            message_type="ros_bt_py_msgs/ThisMessageDoesNotExist", service=False
        )
        response = self.package_manager.get_message_constant_fields_handler(
            request=request
        )

        self.assertFalse(response.success)

    def testGetPackageStructure(self):
        request = GetPackageStructureRequest(
            package="ros_bt_py_msgs", show_hidden=False
        )
        response = self.package_manager.get_package_structure(request=request)

        self.assertTrue(response.success)

        request = GetPackageStructureRequest(package="ros_bt_py_msgs", show_hidden=True)
        response = self.package_manager.get_package_structure(request=request)

        self.assertTrue(response.success)

        request = GetPackageStructureRequest(
            package="ros_bt_py_msgs_does_not_exist", show_hidden=False
        )
        response = self.package_manager.get_package_structure(request=request)

        self.assertFalse(response.success)

    def testSaveTree(self):
        request = SaveTreeRequest(
            filename="test/testdata/save_data/existing_files/existing_file",
            package="ros_bt_py_does_not_exists",
            tree=Tree(),
            allow_overwrite=False,
            allow_rename=False,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertFalse(response.success)

        request = SaveTreeRequest(
            filename="../test",
            package="ros_bt_py",
            tree=Tree(),
            allow_overwrite=False,
            allow_rename=False,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertFalse(response.success)

        request = SaveTreeRequest(
            filename="test/testdata/save_data/existing_files/existing_file",
            package="ros_bt_py",
            tree=Tree(),
            allow_overwrite=False,
            allow_rename=False,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertFalse(response.success)

        request = SaveTreeRequest(
            filename="test/testdata/save_data/existing_files",
            package="ros_bt_py",
            tree=Tree(),
            allow_overwrite=False,
            allow_rename=False,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertFalse(response.success)

        request = SaveTreeRequest(
            filename="test/testdata/save_data/existing_files/existing_file",
            package="ros_bt_py",
            tree=Tree(),
            allow_overwrite=True,
            allow_rename=False,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertTrue(response.success)

        request = SaveTreeRequest(
            filename="test/testdata/save_data/generated_files/generated_file",
            package="ros_bt_py",
            tree=Tree(),
            allow_overwrite=True,
            allow_rename=False,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertTrue(response.success)

        request = SaveTreeRequest(
            filename="test/testdata/save_data/generated_files/generated_file",
            package="ros_bt_py",
            tree=Tree(),
            allow_overwrite=False,
            allow_rename=True,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertTrue(response.success)

    def testSaveAndReloadTree(self):
        manager = TreeManager()
        load_request = LoadTreeRequest(
            tree=Tree(path="package://ros_bt_py/test/testdata/trees/load_save.yaml"),
            permissive=True,
        )
        response = manager.load_tree(load_request)

        self.assertTrue(response.success)

        tree_loaded = deepcopy(manager.to_msg())

        manager.tick(once=True)

        tree_ticked = deepcopy(manager.to_msg())

        self.assertNotEqual(tree_loaded, tree_ticked)

        execution_request = ControlTreeExecutionRequest(
            command=ControlTreeExecutionRequest.SHUTDOWN
        )
        self.assertTrue(manager.control_execution(execution_request).success)

        tree_shutdown = deepcopy(manager.to_msg())

        request = SaveTreeRequest(
            filename="test/testdata/save_data/generated_files/generated_file",
            package="ros_bt_py",
            tree=tree_shutdown,
            allow_overwrite=True,
            allow_rename=False,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertTrue(response.success)

        load_request = LoadTreeRequest(
            tree=Tree(
                path="package://ros_bt_py/test/testdata/save_data/generated_files/generated_file"
            ),
            permissive=True,
        )
        response = manager.load_tree(load_request)

        self.assertTrue(response.success)

        tree_loaded_again = deepcopy(manager.to_msg())
        tree_loaded_again.name = "load_save.yaml"

        self.assertEqual(tree_loaded, tree_loaded_again)

    def testSaveTreeWithMockedRename(self):
        def side_effect(value):
            return value

        self.package_manager.make_filepath_unique = mock.MagicMock(
            side_effect=side_effect
        )
        request = SaveTreeRequest(
            filename="test/testdata/save_data/existing_files/existing_file",
            package="ros_bt_py",
            tree=Tree(),
            allow_overwrite=False,
            allow_rename=True,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertFalse(response.success)

    @mock.patch("ros_bt_py.package_manager.os.makedirs")
    def testSaveTreeWithMockedMakeDirs(self, mock_makedirs):
        mock_makedirs.side_effect = OSError()

        request = SaveTreeRequest(
            filename="test/testdata/save_data/existing_files/new_folder/new_file",
            package="ros_bt_py",
            tree=Tree(),
            allow_overwrite=False,
            allow_rename=False,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertFalse(response.success)

    @mock.patch("ros_bt_py.package_manager.os.path.isfile")
    def testSaveTreeWithMockedIsFile(self, mock_isfile):
        mock_isfile.side_effect = IOError()

        request = SaveTreeRequest(
            filename="test/testdata/save_data/existing_files/new_file",
            package="ros_bt_py",
            tree=Tree(),
            allow_overwrite=False,
            allow_rename=False,
        )

        response = self.package_manager.save_tree(request=request)

        self.assertFalse(response.success)


if __name__ == "__main__":
    rospy.init_node("test_package_manager")
    import rostest

    rostest.rosrun(PKG, "test_package_manager", TestPackageManager)
