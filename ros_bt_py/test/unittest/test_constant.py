import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.constant import Constant


class TestConstant(unittest.TestCase):
    def setUp(self):
        self.constant = Constant({'constant_type': int,
                                  'constant_value': 42})
        self.constant.setup()

    def testConstant(self):
        self.constant.tick()
        self.assertEqual(self.constant.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.constant.outputs['constant'], 42)

        self.constant.tick()
        self.assertEqual(self.constant.state, NodeMsg.SUCCEEDED)
        self.assertEqual(self.constant.outputs['constant'], 42)

        self.constant.reset()
        self.assertEqual(self.constant.state, NodeMsg.IDLE)

        self.constant.untick()
        self.assertEqual(self.constant.state, NodeMsg.IDLE)

        self.constant.shutdown()
        self.assertEqual(self.constant.state, NodeMsg.SHUTDOWN)
