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
from ros_bt_py.nodes.format import (
    FormatOptionNode,
    FormatInputNode,
    FormatOptionListNode,
    FormatInputListNode,
    StringConcatenation,
    GetFileExtension,
)

from ros_bt_py.exceptions import BehaviorTreeException


class TestFormatOptionNode(unittest.TestCase):
    def testSimpleFormatString(self):
        format_option = FormatOptionNode(
            {"format_string": "foo {first} {third!u} {third!l} {third!c}"}
        )
        format_option.inputs["dict"] = {
            "first": "bar",
            "second": "not_printed",
            "third": "ToTo",
        }
        self.assertEqual(format_option.state, NodeMsg.UNINITIALIZED)

        format_option.setup()

        self.assertEqual(format_option.state, NodeMsg.IDLE)
        self.assertEqual(format_option.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(
            format_option.outputs["formatted_string"], "foo bar TOTO toto Toto"
        )

        self.assertEqual(format_option.untick(), NodeMsg.IDLE)
        self.assertEqual(format_option.reset(), NodeMsg.IDLE)
        self.assertEqual(format_option.shutdown(), NodeMsg.SHUTDOWN)

    def testFormatStringWrongKey(self):
        format_option = FormatOptionNode({"format_string": "foo {missing}"})
        format_option.inputs["dict"] = {"first": "bar", "second": "not_printed"}
        self.assertEqual(format_option.state, NodeMsg.UNINITIALIZED)

        format_option.setup()

        self.assertEqual(format_option.state, NodeMsg.IDLE)
        self.assertEqual(format_option.tick(), NodeMsg.FAILED)


class TestFormatOptionListNode(unittest.TestCase):
    def testSimpleFormatListString(self):
        format_option = FormatOptionListNode(
            {"format_strings": ["foo {first}", "woot {first} {third!l}"]}
        )
        format_option.inputs["dict"] = {
            "first": "bar",
            "second": "not_printed",
            "third": "ToTo",
        }
        self.assertEqual(format_option.state, NodeMsg.UNINITIALIZED)

        format_option.setup()

        self.assertEqual(format_option.state, NodeMsg.IDLE)
        self.assertEqual(format_option.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(format_option.outputs["formatted_strings"][0], "foo bar")
        self.assertEqual(format_option.outputs["formatted_strings"][1], "woot bar toto")

        self.assertEqual(format_option.untick(), NodeMsg.IDLE)
        self.assertEqual(format_option.reset(), NodeMsg.IDLE)
        self.assertEqual(format_option.shutdown(), NodeMsg.SHUTDOWN)

    def testFormatStringWrongKey(self):
        format_option = FormatOptionListNode({"format_strings": ["foo {missing}"]})
        format_option.inputs["dict"] = {"first": "bar", "second": "not_printed"}
        self.assertEqual(format_option.state, NodeMsg.UNINITIALIZED)

        format_option.setup()

        self.assertEqual(format_option.state, NodeMsg.IDLE)
        self.assertEqual(format_option.tick(), NodeMsg.FAILED)


class TestFormatInputNode(unittest.TestCase):
    def testSimpleFormatString(self):
        format_input = FormatInputNode()
        format_input.inputs[
            "format_string"
        ] = "foo {first} {third!u} {third!l} {third!c}"
        format_input.inputs["dict"] = {
            "first": "bar",
            "second": "not_printed",
            "third": "ToTo",
        }
        self.assertEqual(format_input.state, NodeMsg.UNINITIALIZED)

        format_input.setup()

        self.assertEqual(format_input.state, NodeMsg.IDLE)
        self.assertEqual(format_input.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(
            format_input.outputs["formatted_string"], "foo bar TOTO toto Toto"
        )

        self.assertEqual(format_input.untick(), NodeMsg.IDLE)
        self.assertEqual(format_input.reset(), NodeMsg.IDLE)
        self.assertEqual(format_input.shutdown(), NodeMsg.SHUTDOWN)

    def testFormatStringWrongKey(self):
        format_input = FormatInputNode()
        format_input.inputs["format_string"] = "foo {missing}"
        format_input.inputs["dict"] = {"first": "bar", "second": "not_printed"}
        self.assertEqual(format_input.state, NodeMsg.UNINITIALIZED)

        format_input.setup()

        self.assertEqual(format_input.state, NodeMsg.IDLE)
        self.assertEqual(format_input.tick(), NodeMsg.FAILED)


class TestFormatInputListNode(unittest.TestCase):
    def testSimpleFormatString(self):
        format_input = FormatInputListNode()
        format_input.inputs["format_strings"] = [
            "foo {first}",
            "woot {first} {third!l}",
        ]
        format_input.inputs["dict"] = {
            "first": "bar",
            "second": "not_printed",
            "third": "ToTo",
        }
        self.assertEqual(format_input.state, NodeMsg.UNINITIALIZED)

        format_input.setup()

        self.assertEqual(format_input.state, NodeMsg.IDLE)
        self.assertEqual(format_input.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(format_input.outputs["formatted_strings"][0], "foo bar")
        self.assertEqual(format_input.outputs["formatted_strings"][1], "woot bar toto")

        self.assertEqual(format_input.untick(), NodeMsg.IDLE)
        self.assertEqual(format_input.reset(), NodeMsg.IDLE)
        self.assertEqual(format_input.shutdown(), NodeMsg.SHUTDOWN)

    def testFormatStringWrongKey(self):
        format_input = FormatInputListNode()
        format_input.inputs["format_strings"] = ["foo {missing}"]
        format_input.inputs["dict"] = {"first": "bar", "second": "not_printed"}
        self.assertEqual(format_input.state, NodeMsg.UNINITIALIZED)

        format_input.setup()

        self.assertEqual(format_input.state, NodeMsg.IDLE)
        self.assertEqual(format_input.tick(), NodeMsg.FAILED)


class TestStringConcatenation(unittest.TestCase):
    def testConcatenation(self):
        concat = StringConcatenation()
        concat.inputs["a"] = "toto"
        concat.inputs["b"] = "foobar"
        self.assertEqual(concat.state, NodeMsg.UNINITIALIZED)
        concat.setup()

        self.assertEqual(concat.state, NodeMsg.IDLE)
        self.assertEqual(concat.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(concat.outputs["formatted_string"], "totofoobar")

        self.assertEqual(concat.untick(), NodeMsg.IDLE)
        self.assertEqual(concat.reset(), NodeMsg.IDLE)
        self.assertEqual(concat.shutdown(), NodeMsg.SHUTDOWN)


class TestFileExtension(unittest.TestCase):
    def testFileExtension(self):
        fileExtension = GetFileExtension()
        fileExtension.inputs["path"] = "toto.yaml"
        self.assertEqual(fileExtension.state, NodeMsg.UNINITIALIZED)
        fileExtension.setup()

        self.assertEqual(fileExtension.state, NodeMsg.IDLE)
        self.assertEqual(fileExtension.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(fileExtension.outputs["extension"], ".yaml")
        self.assertEqual(fileExtension.outputs["filename"], "toto")

        fileExtension.inputs["path"] = "some/long/filepath/and.weird_filename.csv"
        self.assertEqual(fileExtension.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(fileExtension.outputs["extension"], ".csv")
        self.assertEqual(
            fileExtension.outputs["filename"], "some/long/filepath/and.weird_filename"
        )

        self.assertEqual(fileExtension.untick(), NodeMsg.IDLE)
        self.assertEqual(fileExtension.reset(), NodeMsg.IDLE)
        self.assertEqual(fileExtension.shutdown(), NodeMsg.SHUTDOWN)
