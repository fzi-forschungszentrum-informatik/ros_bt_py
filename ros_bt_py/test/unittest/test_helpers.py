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
import sys
try:
    import unittest.mock as mock
except ImportError:
    import mock

import logging
import rospy

from ros_bt_py.helpers import rospy_log_level_to_logging_log_level, get_default_value
from ros_bt_py.helpers import json_encode, json_decode
from ros_bt_py.ros_helpers import LoggerLevel, EnumValue


class TestHelpers(unittest.TestCase):
    def testLogLevelMapping(self):
        self.assertEqual(
            rospy_log_level_to_logging_log_level(rospy.DEBUG),
            logging.DEBUG)

        self.assertEqual(
            rospy_log_level_to_logging_log_level(rospy.INFO),
            logging.INFO)

        self.assertEqual(
            rospy_log_level_to_logging_log_level(rospy.WARN),
            logging.WARN)

        self.assertEqual(
            rospy_log_level_to_logging_log_level(rospy.ERROR),
            logging.ERROR)

        self.assertEqual(
            rospy_log_level_to_logging_log_level(rospy.FATAL),
            logging.FATAL)

    def testGetDefaultValue(self):
        self.assertEqual(get_default_value(LoggerLevel).logger_level, 'info')

        self.assertEqual(get_default_value(EnumValue).enum_value, '')

    def testJsonEncode(self):
        self.assertEqual(json_encode(int), '{"py/type": "__builtin__.int"}')

    def testJsonDecode(self):
        if sys.version_info.major == 2:
            self.assertEqual(json_decode('{"py/type": "__builtin__.int"}'), int)
        else:
            self.assertEqual(json_decode('{"py/type": "builtins.int"}'), int)

    def testJsonDecode23(self):
        with mock.patch.object(sys, 'version_info') as version_info:
            version_info.major = 2
            self.assertEqual(json_decode('{}'), {})
            version_info.major = 3
            self.assertEqual(json_decode('{}'), {})
