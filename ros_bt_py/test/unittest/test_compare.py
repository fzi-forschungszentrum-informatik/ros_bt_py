import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.compare import Compare, CompareNewOnly, CompareConstant
from ros_bt_py.nodes.compare import ALessThanB, LessThanConstant


class TestCompare(unittest.TestCase):
    def setUp(self):
        self.compare = Compare({'compare_type': int})
        self.compare.setup()
        self.compare.inputs['a'] = 42
        self.compare.inputs['b'] = 42

    def testResult(self):
        # Both inputs are set, we should have an output
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        # Another tick, same result
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.inputs['b'] = 41

        # 'in' changed, so we get a different result, FAILED
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

    def testReset(self):
        # Start as before, 42 == 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.reset()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)
        # Neither of the inputs have updated, so ticking fails
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

        # If we update the inputs, the result changes to SUCCEEDED
        self.compare.inputs['a'] = 42
        self.compare.inputs['b'] = 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

    def testUntick(self):
        self.compare.untick()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)

    def testShutdown(self):
        self.compare.shutdown()
        self.assertEqual(self.compare.state, NodeMsg.SHUTDOWN)


class TestCompareNewOnly(unittest.TestCase):
    def setUp(self):
        self.compare = CompareNewOnly({'compare_type': int})
        self.compare.setup()
        self.compare.inputs['a'] = 42
        self.compare.inputs['b'] = 42

    def testResult(self):
        # Both inputs are set, we should have an output
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        # Another tick, inputs were not updated => RUNNING
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.RUNNING)

        self.compare.inputs['b'] = 41
        # 'in' changed, so we get a different result, FAILED
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

    def testReset(self):
        # Start as before, 42 == 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.reset()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)
        # Neither of the inputs have updated, so the node stays
        # RUNNING
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.RUNNING)

        # If we update the inputs, the result changes to SUCCEEDED
        self.compare.inputs['a'] = 42
        self.compare.inputs['b'] = 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

    def testUntick(self):
        self.compare.untick()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)

    def testShutdown(self):
        self.compare.shutdown()
        self.assertEqual(self.compare.state, NodeMsg.SHUTDOWN)


class TestCompareConstant(unittest.TestCase):
    def setUp(self):
        self.compare = CompareConstant({'compare_type': int,
                                        'expected': 5})
        self.compare.setup()
        self.compare.inputs['in'] = 5

    def testResult(self):
        # Both inputs are set, we should have an output
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        # Another tick, same result
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.inputs['in'] = 41

        # 'in' changed, so we get a different result, FAILED
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

    def testReset(self):
        # Start as before, 42 == 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

        self.compare.reset()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)
        # The input hasn't updated yet, so ticking fails
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.FAILED)

        # If we update the input, the result changes to SUCCEEDED
        self.compare.inputs['in'] = 5
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)

    def testUntick(self):
        self.compare.untick()
        self.assertEqual(self.compare.state, NodeMsg.IDLE)

    def testShutdown(self):
        self.compare.shutdown()
        self.assertEqual(self.compare.state, NodeMsg.SHUTDOWN)


class TestLessThan(unittest.TestCase):
    def testALessThanB(self):
        less_than = ALessThanB()
        less_than.setup()

        with self.assertRaises(ValueError):
            less_than.tick()

        less_than.inputs['a'] = 42
        less_than.inputs['b'] = 42

        self.assertEqual(less_than.tick(), NodeMsg.FAILED)

        less_than.inputs['a'] = 100
        self.assertEqual(less_than.tick(), NodeMsg.FAILED)

        less_than.inputs['a'] = 1
        self.assertEqual(less_than.tick(), NodeMsg.SUCCEEDED)

        less_than.reset()
        self.assertEqual(less_than.state, NodeMsg.IDLE)
        # Neither of the inputs have updated, so ticking fails
        less_than.tick()
        self.assertEqual(less_than.state, NodeMsg.FAILED)

        # If we update the inputs, the result changes to SUCCEEDED
        less_than.inputs['a'] = 1
        less_than.inputs['b'] = 42
        less_than.tick()
        self.assertEqual(less_than.state, NodeMsg.SUCCEEDED)

        less_than.untick()
        self.assertEqual(less_than.state, NodeMsg.IDLE)

        less_than.shutdown()
        self.assertEqual(less_than.state, NodeMsg.SHUTDOWN)

    def testLessThanConstant(self):
        less_than = LessThanConstant({'target': 42})
        less_than.setup()

        with self.assertRaises(ValueError):
            less_than.tick()

        less_than.inputs['a'] = 42

        self.assertEqual(less_than.tick(), NodeMsg.FAILED)

        less_than.inputs['a'] = 100
        self.assertEqual(less_than.tick(), NodeMsg.FAILED)

        less_than.inputs['a'] = 1
        self.assertEqual(less_than.tick(), NodeMsg.SUCCEEDED)

        less_than.reset()
        self.assertEqual(less_than.state, NodeMsg.IDLE)
        # Neither of the inputs have updated, so ticking fails
        less_than.tick()
        self.assertEqual(less_than.state, NodeMsg.FAILED)

        # If we update the inputs, the result changes to SUCCEEDED
        less_than.inputs['a'] = 1
        less_than.tick()
        self.assertEqual(less_than.state, NodeMsg.SUCCEEDED)

        less_than.untick()
        self.assertEqual(less_than.state, NodeMsg.IDLE)

        less_than.shutdown()
        self.assertEqual(less_than.state, NodeMsg.SHUTDOWN)
