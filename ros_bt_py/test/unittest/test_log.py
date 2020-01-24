import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.nodes.log import Log

from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.ros_helpers import LoggerLevel


class TestLog(unittest.TestCase):
    def testDebug(self):
        log = Log({'logger_level': LoggerLevel(logger_level='debug')})
        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)

    def testInfo(self):
        log = Log({'logger_level': LoggerLevel(logger_level='info')})
        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)

    def testWarning(self):
        log = Log({'logger_level': LoggerLevel(logger_level='warning')})
        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)

    def testError(self):
        log = Log({'logger_level': LoggerLevel(logger_level='error')})
        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)

    def testFatal(self):
        log = Log({'logger_level': LoggerLevel(logger_level='fatal')})
        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)
