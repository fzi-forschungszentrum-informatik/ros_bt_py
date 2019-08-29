import unittest
from time import sleep, time

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.nodes.wait import Wait, WaitInput


class TestWait(unittest.TestCase):
    def testPositive(self):
        wait = Wait({'seconds_to_wait': 1})
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
        wait = Wait({'seconds_to_wait': 0})
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)

    def testNegative(self):
        wait = Wait({'seconds_to_wait': -1})
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)


class TestWaitInput(unittest.TestCase):
    def testPositive(self):
        wait = WaitInput()
        wait.inputs['seconds_to_wait'] = 1
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
        wait.inputs['seconds_to_wait'] = 0
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)

    def testNegative(self):
        wait = WaitInput()
        wait.inputs['seconds_to_wait'] = -1
        self.assertEqual(wait.state, NodeMsg.UNINITIALIZED)
        wait.setup()
        self.assertEqual(wait.state, NodeMsg.IDLE)
        self.assertEqual(wait.tick(), NodeMsg.SUCCESS)
