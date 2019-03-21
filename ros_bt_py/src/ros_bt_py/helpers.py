import logging
import rospy
import functools


def loglevel_is(level):
    """Determine the current logging level of the default ROS logger

    Useful to guard log statements that would incur a performance
    penalty if they ran when the log isn't published.

    For easier use, this translates the `rospy` logger levels into
    `logging` levels.

    :param int level:

    The logger level to compare against. Since lower levels are more
    verbose, any level *below* `level` also returns `True`.

    """
    logging_level = logging.getLogger('rosout').getEffectiveLevel()
    return logging_level <= rospy_log_level_to_logging_log_level(level)


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


# handling nested objects,
# see https://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-objects
def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)
