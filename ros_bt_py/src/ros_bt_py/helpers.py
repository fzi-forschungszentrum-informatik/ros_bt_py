import logging
import rospy
import functools
from collections import OrderedDict
from ros_bt_py.ros_helpers import LoggerLevel

from ros_bt_py_msgs.srv import FixYamlResponse

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


def fix_yaml(request):
    response = FixYamlResponse()

    tree_yaml = request.broken_yaml

    last_index = 0

    index = 0
    search_string = 'child_names: - '
    replace_string = 'child_names:'
    search_len = len(search_string)
    replace_len = len(replace_string)
    while index < len(tree_yaml):
        index = tree_yaml.find(search_string, index)
        if index == -1:
            break

        # find the last linebreak and count number of spaces until child_names:
        linebreak_index = tree_yaml.rfind('\n', last_index, index)

        indent = index - linebreak_index - 1 + 2

        # now replace the search_string with the proper linebreak
        tree_yaml = tree_yaml[:index + replace_len] + '\n' + \
            tree_yaml[index + replace_len + 1:]

        # now check all newlines until they are not "\n- " any more
        newline_index = index + replace_len

        # update for next check
        index = index + search_len
        last_index = index

        # rospy.logerr("newline: %s" % tree_yaml[newline_index:])
        while newline_index < len(tree_yaml):
            # skip "\n"
            newline_index = newline_index + 1
            # check if the line starts with "- "
            if tree_yaml[newline_index:newline_index + 2] == "- ":
                # fix it by adding the correct indent:
                tree_yaml = tree_yaml[:newline_index] + ' ' * indent + \
                    tree_yaml[newline_index:]
            else:
                # found no compatible newline
                break
            # find next "\n"
            newline_index = tree_yaml.find('\n', newline_index + indent + 2)

    response.success = True
    response.fixed_yaml = tree_yaml

    return response


# handling nested objects,
# see https://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-objects
def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)


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
    elif data_type is OrderedDict:
        return OrderedDict()
    elif data_type is LoggerLevel:
        return LoggerLevel()
    elif ros:
        return data_type()
    else:
        return {}
