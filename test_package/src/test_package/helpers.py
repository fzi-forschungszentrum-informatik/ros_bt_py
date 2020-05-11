import logging
import rospy


try:  # pragma: no cover
    basestring
except NameError:  # pragma: no cover
    basestring = str

try:  # pragma: no cover
    unicode
except NameError:  # pragma: no cover
    unicode = str

try:  # pragma: no cover
    long
except NameError:  # pragma: no cover
    long = int


def rospy_log_level_to_logging_log_level(rospy_level):
    if rospy_level == rospy.DEBUG:
        return logging.DEBUG
    if rospy_level == rospy.INFO:
        return logging.INFO
    if rospy_level == rospy.WARN:
        return logging.WARN
    if rospy_level == rospy.ERROR:
        return logging.ERROR
    if rospy_level == rospy.FATAL:
        return logging.FATAL


def get_default_value(data_type, ros=False):
    if data_type is type:
        return int
    elif data_type is int or data_type is long:
        return 0
    elif data_type is str or data_type is basestring or data_type is unicode:
        return 'foo'
    elif data_type is float:
        return 1.2
    elif data_type is bool:
        return False
    elif data_type is list:
        return []
    elif data_type is dict:
        return {}
    elif ros:
        return data_type()
    else:
        return {}
