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
from time import sleep, time

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.nodes.wait import Wait, WaitInput


class TestWait(unittest.TestCase):
    def testPositive(self):
        wait = Wait({"seconds_to_wait": 1})
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.RUNNING)
        # should be enough time to complete waiting
        sleep(2)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)
        self.assertEqual(wait.untick(), NodeMsg.IDLE)
        self.assertEqual(wait.reset(), NodeMsg.IDLE)
        self.assertEqual(wait.shutdown(), NodeMsg.SHUTDOWN)

    def testZero(self):
        wait = Wait({"seconds_to_wait": 0})
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)

    def testNegative(self):
        wait = Wait({"seconds_to_wait": -1})
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)


class TestWaitInput(unittest.TestCase):
    def testPositive(self):
        wait = WaitInput()
        wait.inputs["seconds_to_wait"] = 1
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.RUNNING)
        # should be enough time to complete waiting
        sleep(2)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)
        self.assertEqual(wait.untick(), NodeMsg.IDLE)
        self.assertEqual(wait.reset(), NodeMsg.IDLE)
        self.assertEqual(wait.shutdown(), NodeMsg.SHUTDOWN)

    def testZero(self):
        wait = WaitInput()
        wait.inputs["seconds_to_wait"] = 0
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)

    def testNegative(self):
        wait = WaitInput()
        wait.inputs["seconds_to_wait"] = -1
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)
