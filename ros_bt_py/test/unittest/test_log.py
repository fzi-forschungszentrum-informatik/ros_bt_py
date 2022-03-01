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
