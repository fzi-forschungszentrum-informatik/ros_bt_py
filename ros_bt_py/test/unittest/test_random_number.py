import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.nodes.random_number import RandomInt, RandomIntInputs

from ros_bt_py.exceptions import BehaviorTreeException


class TestRandomInt(unittest.TestCase):
    def testEqualNumbers(self):
        random_int = RandomInt({'min': 0,
                                'max': 0})
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        self.assertRaises(BehaviorTreeException, random_int.setup)

    def testMinGreaterMax(self):
        random_int = RandomInt({'min': 1,
                                'max': 0})
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        self.assertRaises(BehaviorTreeException, random_int.setup)

    def testMaxGreaterMin(self):
        random_int = RandomInt({'min': 0,
                                'max': 1})
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        random_int.setup()

        self.assertEqual(random_int.state, NodeMsg.IDLE)
        self.assertEqual(random_int.tick(), NodeMsg.SUCCEEDED)
        self.assertAlmostEqual(random_int.outputs['random_number'], 0, delta=1)

        self.assertEqual(random_int.untick(), NodeMsg.IDLE)
        self.assertEqual(random_int.reset(), NodeMsg.IDLE)
        self.assertEqual(random_int.shutdown(), NodeMsg.SHUTDOWN)


class TestRandomIntInputs(unittest.TestCase):
    def testEqualNumbers(self):
        random_int = RandomIntInputs()
        random_int.inputs['min'] = 0
        random_int.inputs['max'] = 0

        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        self.assertRaises(BehaviorTreeException, random_int.setup)

    def testMinGreaterMax(self):
        random_int = RandomIntInputs()
        random_int.inputs['min'] = 1
        random_int.inputs['max'] = 0
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        self.assertRaises(BehaviorTreeException, random_int.setup)

    def testMaxGreaterMin(self):
        random_int = RandomIntInputs()
        random_int.inputs['min'] = 0
        random_int.inputs['max'] = 1
        self.assertEqual(random_int.state, NodeMsg.UNINITIALIZED)
        random_int.setup()

        self.assertEqual(random_int.state, NodeMsg.IDLE)
        self.assertEqual(random_int.tick(), NodeMsg.SUCCEEDED)
        self.assertAlmostEqual(random_int.outputs['random_number'], 0, delta=1)

        self.assertEqual(random_int.untick(), NodeMsg.IDLE)
        self.assertEqual(random_int.reset(), NodeMsg.IDLE)
        self.assertEqual(random_int.shutdown(), NodeMsg.SHUTDOWN)
