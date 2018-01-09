import rospy


class NodeData(object):
    """
    Represents a piece of data (input, output or option) held by a Node

    """
    def __init__(self, data_type, initial_value=None, static=False):
        self._value = None
        self._static = static

        # Relax type checking for string types
        if data_type is str or data_type is unicode:
            self._data_type = basestring
        else:
            self._data_type = data_type

        # use set here to ensure initial_value is the right type
        self.set(initial_value)
        # reset updated to False (set() set it to True)
        self.updated = False

    def __repl__(self):
        return '{!s} ({}) [{}]'.format(self._value,
                                       self._data_type.__name__,
                                       ('#' if self.updated else ' '))

    def set(self, new_value):
        if self._static:
            raise Exception('Trying to write to a static NodeData object')
        if not isinstance(new_value, self._data_type) and new_value is not None:
            raise TypeError('Expected data to be of type {}, got {} instead'.format(
                self._data_type.__name__, type(new_value).__name__))
        self._value = new_value
        self.updated = True

    def get(self):
        """Get the current value

        Will log a warning if the value has not been updated since the
        last call of `reset_updated` (unless this piece of data is
        marked static, in which case we do not expect updates).
        """
        if not self._static and not self.updated:
            rospy.logwarn('Reading non-updated value!')
        return self._value

    def reset_updated(self):
        self.updated = False
