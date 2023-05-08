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
from std_msgs.msg import Duration, Header, Time

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.ros_nodes.enum import Enum, EnumFields
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.node_data import NodeData
from ros_bt_py.ros_helpers import EnumValue


class TestEnum(unittest.TestCase):
    def testNoROSMessage(self):
        self.assertRaises(
            BehaviorTreeException,
            Enum,
            {"ros_message_type": int, "constant_name": EnumValue("does_not_exist")},
        )

    def testTreeMessageWrongField(self):
        self.assertRaises(
            BehaviorTreeException,
            Enum,
            {"ros_message_type": Tree, "constant_name": EnumValue("does_not_exist")},
        )

    def testTreeMessage(self):
        enum = Enum({"ros_message_type": Tree, "constant_name": EnumValue("IDLE")})
        enum.setup()

        enum.tick()
        self.assertEqual(enum.state, NodeMsg.SUCCEEDED)
        self.assertEqual(enum.outputs["out"], "IDLE")

        enum.reset()
        self.assertEqual(enum.state, NodeMsg.IDLE)

        enum.untick()
        self.assertEqual(enum.state, NodeMsg.IDLE)

        enum.shutdown()
        self.assertEqual(enum.state, NodeMsg.SHUTDOWN)

    def testHeaderMessage(self):
        self.assertRaises(
            BehaviorTreeException,
            Enum,
            {"ros_message_type": Header, "constant_name": EnumValue("does_not_exist")},
        )


class TestEnumFields(unittest.TestCase):
    def testNoROSMessage(self):
        self.assertRaises(BehaviorTreeException, EnumFields, {"ros_message_type": int})

    def testTreeMessage(self):
        enum = EnumFields({"ros_message_type": Tree})
        enum.setup()

        enum.tick()
        self.assertEqual(enum.state, NodeMsg.SUCCEEDED)
        self.assertEqual(enum.outputs["IDLE"], "IDLE")
        self.assertEqual(enum.outputs["WAITING_FOR_TICK"], "WAITING_FOR_TICK")
        self.assertEqual(enum.outputs["ERROR"], "ERROR")

        enum.reset()
        self.assertEqual(enum.state, NodeMsg.IDLE)

        enum.untick()
        self.assertEqual(enum.state, NodeMsg.IDLE)

        enum.shutdown()
        self.assertEqual(enum.state, NodeMsg.SHUTDOWN)

    def testHeaderMessage(self):
        self.assertRaises(
            BehaviorTreeException, EnumFields, {"ros_message_type": Header}
        )
