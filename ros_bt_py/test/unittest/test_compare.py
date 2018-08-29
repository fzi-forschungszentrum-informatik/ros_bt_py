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
        with self.assertRaises(ValueError):
            self.compare.tick()

        # If we update the inputs, the result changes to SUCCEEDED
        self.compare.inputs['a'] = 42
        self.compare.inputs['b'] = 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)


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
        # Neither of the inputs have updated, so ticking fails
        with self.assertRaises(ValueError):
            self.compare.tick()

        # If we update the inputs, the result changes to SUCCEEDED
        self.compare.inputs['a'] = 42
        self.compare.inputs['b'] = 42
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)


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
        with self.assertRaises(ValueError):
            self.compare.tick()

        # If we update the input, the result changes to SUCCEEDED
        self.compare.inputs['in'] = 5
        self.compare.tick()
        self.assertEqual(self.compare.state, NodeMsg.SUCCEEDED)


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
