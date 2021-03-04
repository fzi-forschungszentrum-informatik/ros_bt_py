import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.nodes.log import Log

from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.ros_helpers import LoggerLevel


class TestLog(unittest.TestCase):
    def logdebug(self, msg):
        self.msg_debug = msg

    def loginfo(self, msg):
        self.msg_info = msg

    def logwarn(self, msg):
        self.msg_warn = msg

    def logerr(self, msg):
        self.msg_err = msg

    def logfatal(self, msg):
        self.msg_fatal = msg

    def setUp(self):
        self.msg_debug = None
        self.msg_info = None
        self.msg_warn = None
        self.msg_err = None
        self.msg_fatal = None

    def testDebug(self):
        log = Log({'logger_level': LoggerLevel(logger_level='debug'),
                   'log_type': str})

        log.logdebug = self.logdebug
        log.loginfo = self.loginfo
        log.logwarn = self.logwarn
        log.logerr = self.logerr
        log.logfatal = self.logfatal

        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertEqual(self.msg_debug, 'hello there')
        self.assertNotEqual(self.msg_info, 'hello there')
        self.assertNotEqual(self.msg_warn, 'hello there')
        self.assertNotEqual(self.msg_err, 'hello there')
        self.assertNotEqual(self.msg_fatal, 'hello there')

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)

    def testInfo(self):
        log = Log({'logger_level': LoggerLevel(logger_level='info'),
                   'log_type': str})

        log.logdebug = self.logdebug
        log.loginfo = self.loginfo
        log.logwarn = self.logwarn
        log.logerr = self.logerr
        log.logfatal = self.logfatal

        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertNotEqual(self.msg_debug, 'hello there')
        self.assertEqual(self.msg_info, 'hello there')
        self.assertNotEqual(self.msg_warn, 'hello there')
        self.assertNotEqual(self.msg_err, 'hello there')
        self.assertNotEqual(self.msg_fatal, 'hello there')

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)

    def testWarning(self):
        log = Log({'logger_level': LoggerLevel(logger_level='warning'),
                   'log_type': str})

        log.logdebug = self.logdebug
        log.loginfo = self.loginfo
        log.logwarn = self.logwarn
        log.logerr = self.logerr
        log.logfatal = self.logfatal

        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertNotEqual(self.msg_debug, 'hello there')
        self.assertNotEqual(self.msg_info, 'hello there')
        self.assertEqual(self.msg_warn, 'hello there')
        self.assertNotEqual(self.msg_err, 'hello there')
        self.assertNotEqual(self.msg_fatal, 'hello there')

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)

    def testError(self):
        log = Log({'logger_level': LoggerLevel(logger_level='error'),
                   'log_type': str})

        log.logdebug = self.logdebug
        log.loginfo = self.loginfo
        log.logwarn = self.logwarn
        log.logerr = self.logerr
        log.logfatal = self.logfatal

        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertNotEqual(self.msg_debug, 'hello there')
        self.assertNotEqual(self.msg_info, 'hello there')
        self.assertNotEqual(self.msg_warn, 'hello there')
        self.assertEqual(self.msg_err, 'hello there')
        self.assertNotEqual(self.msg_fatal, 'hello there')

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)

    def testFatal(self):
        log = Log({'logger_level': LoggerLevel(logger_level='fatal'),
                   'log_type': str})

        log.logdebug = self.logdebug
        log.loginfo = self.loginfo
        log.logwarn = self.logwarn
        log.logerr = self.logerr
        log.logfatal = self.logfatal

        log.inputs['in'] = 'hello there'
        self.assertEqual(log.state, NodeMsg.UNINITIALIZED)

        log.setup()

        self.assertEqual(log.state, NodeMsg.IDLE)
        self.assertEqual(log.tick(), NodeMsg.SUCCEEDED)

        self.assertNotEqual(self.msg_debug, 'hello there')
        self.assertNotEqual(self.msg_info, 'hello there')
        self.assertNotEqual(self.msg_warn, 'hello there')
        self.assertNotEqual(self.msg_err, 'hello there')
        self.assertEqual(self.msg_fatal, 'hello there')

        self.assertEqual(log.untick(), NodeMsg.IDLE)
        self.assertEqual(log.reset(), NodeMsg.IDLE)
        self.assertEqual(log.shutdown(), NodeMsg.SHUTDOWN)
