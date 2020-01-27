import unittest

import logging
import rospy

from ros_bt_py.helpers import rospy_log_level_to_logging_log_level


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
