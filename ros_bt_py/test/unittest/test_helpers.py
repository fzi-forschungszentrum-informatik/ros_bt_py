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
