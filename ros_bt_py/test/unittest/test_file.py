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
from ros_bt_py.nodes.file import YamlListOption, YamlListInput, YamlDictInput


class BasicFileInputTest(unittest.TestCase):
    def make_file_input(self):
        return YamlListInput()

    def testFileLoadNotAvailable(self):
        path = "file://"
        file_node = self.make_file_input()
        file_node.setup()
        file_node.inputs["file_path"] = path
        self.assertEqual(NodeMsg.FAILED, file_node.tick())
        self.assertEqual(file_node.outputs["load_success"], False)

    def testFileLoadMalformedPath(self):
        path = "malformed://ros_bt_py/etc/data/greetings.yaml"
        file_node = self.make_file_input()
        file_node.setup()
        file_node.inputs["file_path"] = path
        self.assertEqual(NodeMsg.FAILED, file_node.tick())
        self.assertEqual(file_node.outputs["load_success"], False)

    def testFileLoadMalformedContent(self):
        path = "package://ros_bt_py/test/testdata/data/file_malformed.yaml"
        file_node = self.make_file_input()
        file_node.setup()
        file_node.inputs["file_path"] = path
        self.assertEqual(file_node.state, NodeMsg.IDLE)

        self.assertEqual(file_node.tick(), NodeMsg.FAILED)
        self.assertEqual(file_node.outputs["content"], None)

    def testFileLoadMalformedList(self):
        path = "package://ros_bt_py/test/testdata/data/file_malformed_list.yaml"
        file_node = self.make_file_input()
        file_node.setup()
        file_node.inputs["file_path"] = path
        self.assertEqual(file_node.state, NodeMsg.IDLE)

        self.assertEqual(file_node.tick(), NodeMsg.FAILED)
        self.assertEqual(file_node.outputs["content"], None)

    def testFileInvalidYaml(self):
        path = "package://ros_bt_py/test/testdata/data/file_invalid_yaml.yaml"
        file_node = self.make_file_input()
        file_node.setup()
        file_node.inputs["file_path"] = path
        self.assertEqual(file_node.state, NodeMsg.IDLE)

        self.assertEqual(file_node.tick(), NodeMsg.FAILED)
        self.assertEqual(file_node.outputs["content"], None)


class TestYamlListOption(unittest.TestCase):
    def testFileLoad(self):
        path = "package://ros_bt_py/test/testdata/data/file_greetings.yaml"
        file_node = YamlListOption(
            options={
                "file_path": path,
            }
        )
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)

        self.assertEqual(file_node.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(file_node.untick(), NodeMsg.IDLE)
        self.assertEqual(file_node.reset(), NodeMsg.IDLE)
        self.assertEqual(file_node.shutdown(), NodeMsg.SHUTDOWN)

    def testFileLoadMalformedPath(self):
        file_node = YamlListOption(
            options={
                "file_path": "malformed://ros_bt_py/etc/data/greetings.yaml",
            }
        )
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)
        self.assertEqual(file_node.tick(), NodeMsg.FAILED)
        self.assertEqual(file_node.outputs["content"], None)

    def testFileLoadNotAvailable(self):
        file_node = YamlListOption(
            options={
                "file_path": "file://",
            }
        )
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)
        self.assertEqual(file_node.tick(), NodeMsg.FAILED)
        self.assertEqual(file_node.outputs["content"], None)

    def testFileLoadMalformedContent(self):
        path = "package://ros_bt_py/test/testdata/data/file_malformed.yaml"
        file_node = YamlListOption(
            options={
                "file_path": path,
            }
        )
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)

        self.assertEqual(file_node.tick(), NodeMsg.FAILED)

    def testFileLoadListObject(self):
        path = "package://ros_bt_py/test/testdata/data/file_object_list.yaml"
        file_node = YamlListOption(
            options={
                "file_path": path,
            }
        )
        self.assertEqual(file_node.state, NodeMsg.UNINITIALIZED)
        file_node.setup()
        self.assertEqual(file_node.state, NodeMsg.IDLE)
        self.assertEqual(file_node.tick(), NodeMsg.SUCCESS)

        self.assertIsInstance(file_node.outputs["content"][0], dict)
        self.assertEqual(
            list(file_node.outputs["content"][0].keys()), ["not_basestring"]
        )


class TestYamlListInput(BasicFileInputTest, unittest.TestCase):
    def make_file_input(self):
        return YamlListInput()

    def testFileLoad(self):
        path = "package://ros_bt_py/test/testdata/data/file_greetings.yaml"
        file_node = self.make_file_input()
        file_node.setup()

        file_node.inputs["file_path"] = path
        self.assertTrue(file_node.inputs.is_updated("file_path"))
        self.assertEqual(NodeMsg.SUCCEEDED, file_node.tick())

        self.assertEqual(file_node.outputs["load_success"], True)
        self.assertEqual(file_node.outputs["line_count"], 4)
        self.assertEqual(file_node.outputs["content"][2], "Hola")

        self.assertEqual(file_node.untick(), NodeMsg.IDLE)
        self.assertEqual(file_node.reset(), NodeMsg.IDLE)
        self.assertEqual(file_node.shutdown(), NodeMsg.SHUTDOWN)


class TestYamlDictInput(BasicFileInputTest, unittest.TestCase):
    def make_file_input(self):
        return YamlDictInput()

    def testFileLoad(self):
        path = "package://ros_bt_py/test/testdata/data/file_dict_greetings.yaml"
        file_node = YamlDictInput()
        file_node.setup()

        file_node.inputs["file_path"] = path
        self.assertTrue(file_node.inputs.is_updated("file_path"))
        self.assertEqual(NodeMsg.SUCCEEDED, file_node.tick())

        self.assertEqual(file_node.outputs["load_success"], True)
        self.assertEqual(file_node.outputs["content"]["french"], "bonjour")
        self.assertEqual(file_node.outputs["content"]["german"]["south"], "moin moin")

        self.assertEqual(file_node.untick(), NodeMsg.IDLE)
        self.assertEqual(file_node.reset(), NodeMsg.IDLE)
        self.assertEqual(file_node.shutdown(), NodeMsg.SHUTDOWN)
