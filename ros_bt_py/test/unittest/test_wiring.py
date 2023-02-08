#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022-2023 FZI Forschungszentrum Informatik
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
from ros_bt_py_msgs.msg import NodeDataWiring
from ros_bt_py_msgs.msg import NodeDataLocation

from ros_bt_py.nodes.mock_nodes import MockLeaf
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.nodes.compare import Compare

from ros_bt_py.exceptions import BehaviorTreeException


class TestWiring(unittest.TestCase):
    def setUp(self):
        self.root = Sequence()

        self.first = MockLeaf(
            {
                "state_values": [NodeMsg.SUCCEEDED],
                "output_type": str,
                "output_values": ["Yay"],
            }
        )
        self.root.add_child(self.first)

        self.second = Compare({"compare_type": str})
        self.root.add_child(self.second)

        self.root.setup()

    def testWiringWithInvalidDataKind(self):
        with self.assertRaises(BehaviorTreeException):
            self.second.wire_data(
                NodeDataWiring(
                    source=NodeDataLocation(
                        node_name=self.first.name, data_kind="broken", data_key="out"
                    ),
                    target=NodeDataLocation(
                        node_name=self.second.name,
                        data_kind=NodeDataLocation.INPUT_DATA,
                        data_key="a",
                    ),
                )
            )
        with self.assertRaises(BehaviorTreeException):
            self.second.wire_data(
                NodeDataWiring(
                    source=NodeDataLocation(
                        node_name=self.first.name,
                        data_kind=NodeDataLocation.OUTPUT_DATA,
                        data_key="out",
                    ),
                    target=NodeDataLocation(
                        node_name=self.second.name, data_kind="broken", data_key="a"
                    ),
                )
            )

    def testWiringFromMissingNode(self):
        with self.assertRaises(BehaviorTreeException):
            self.second.wire_data(
                NodeDataWiring(
                    source=NodeDataLocation(
                        node_name="fake_node",
                        data_kind=NodeDataLocation.OUTPUT_DATA,
                        data_key="out",
                    ),
                    target=NodeDataLocation(
                        node_name=self.second.name,
                        data_kind=NodeDataLocation.INPUT_DATA,
                        data_key="a",
                    ),
                )
            )

    def testWiringFromMissingKey(self):
        with self.assertRaises(KeyError):
            # Target is valid, but source key should be 'out', not
            # 'output'
            self.second.wire_data(
                NodeDataWiring(
                    source=NodeDataLocation(
                        node_name=self.first.name,
                        data_kind=NodeDataLocation.OUTPUT_DATA,
                        data_key="output",
                    ),
                    target=NodeDataLocation(
                        node_name=self.second.name,
                        data_kind=NodeDataLocation.INPUT_DATA,
                        data_key="a",
                    ),
                )
            )

    def testWiringToMissingKey(self):
        with self.assertRaises(KeyError):
            # Source is valid, but target key should be 'a', not
            # 'input'
            self.second.wire_data(
                NodeDataWiring(
                    source=NodeDataLocation(
                        node_name=self.first.name,
                        data_kind=NodeDataLocation.OUTPUT_DATA,
                        data_key="out",
                    ),
                    target=NodeDataLocation(
                        node_name=self.second.name,
                        data_kind=NodeDataLocation.INPUT_DATA,
                        data_key="input",
                    ),
                )
            )

    def testWiringToOtherNode(self):
        with self.assertRaises(BehaviorTreeException):
            # Source is valid, but target is not the node we call
            # wire_data() on
            self.second.wire_data(
                NodeDataWiring(
                    source=NodeDataLocation(
                        node_name=self.first.name,
                        data_kind=NodeDataLocation.OUTPUT_DATA,
                        data_key="out",
                    ),
                    target=NodeDataLocation(
                        node_name="other",
                        data_kind=NodeDataLocation.INPUT_DATA,
                        data_key="a",
                    ),
                )
            )

    def testCorrectWiring(self):
        self.second.wire_data(
            NodeDataWiring(
                source=NodeDataLocation(
                    node_name=self.first.name,
                    data_kind=NodeDataLocation.OUTPUT_DATA,
                    data_key="out",
                ),
                target=NodeDataLocation(
                    node_name=self.second.name,
                    data_kind=NodeDataLocation.INPUT_DATA,
                    data_key="a",
                ),
            )
        )

        self.assertEqual(len(self.first.subscribers), 1)
        self.assertEqual(len(self.first.subscriptions), 0)
        self.assertEqual(len(self.second.subscribers), 0)
        self.assertEqual(len(self.second.subscriptions), 1)

    def testMultipleWirings(self):
        self.second.inputs["a"] = "asdf"
        self.second.wire_data(
            NodeDataWiring(
                source=NodeDataLocation(
                    node_name=self.first.name,
                    data_kind=NodeDataLocation.OUTPUT_DATA,
                    data_key="out",
                ),
                target=NodeDataLocation(
                    node_name=self.second.name,
                    data_kind=NodeDataLocation.INPUT_DATA,
                    data_key="a",
                ),
            )
        )

        self.second.wire_data(
            NodeDataWiring(
                source=NodeDataLocation(
                    node_name=self.first.name,
                    data_kind=NodeDataLocation.OUTPUT_DATA,
                    data_key="out",
                ),
                target=NodeDataLocation(
                    node_name=self.second.name,
                    data_kind=NodeDataLocation.INPUT_DATA,
                    data_key="b",
                ),
            )
        )

        self.assertEqual(len(self.first.subscribers), 2)
        self.assertEqual(len(self.first.subscriptions), 0)
        self.assertEqual(len(self.second.subscribers), 0)
        self.assertEqual(len(self.second.subscriptions), 2)

        self.root.tick()
