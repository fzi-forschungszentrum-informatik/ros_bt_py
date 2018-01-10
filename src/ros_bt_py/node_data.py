import rospy


class NodeData(object):
    """Represents a piece of data (input, output or option) held by a Node

    Each NodeData object is typed and will only accept new data (via its
    :meth:NodeData.set method) if it is of the correct type.

    NodeData can also be static, in which case it will never accept any
    update, i.e. it will be stuck with the initial value.
    """
    def __init__(self, data_type, initial_value=None, static=False):
        if initial_value is None and static:
            raise ValueError('Static NodeData must have an initial value!')

        self.updated = False
        self._value = None
        # Set this to False initially so we can call set() below
        self._static = False

        # Relax type checking for string types
        if data_type is str or data_type is unicode:
            self._data_type = basestring
        else:
            self._data_type = data_type

        # use set here to ensure initial_value is the right type
        # this also sets updated to True
        if initial_value is not None:
            self.set(initial_value)
        # set _static to the supplied value
        self._static = static

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


class NodeDataMap(object):
    """Custom container class that hides :meth:NodeData.get and :meth:NodeData.set

    Acts like a regular dict, with three caveats:

    1. All keys must be strings

    2. All values must be :class:NodeData objects

    3. It is not possible to delete or overwrite values once they've
    been added to the map. This is because the setters for values might
    be used as callbacks elsewhere, leading to unexpected breakage!
    """
    def __init__(self):
        self._map = {}

    def add(self, key, value):
        """
        :param basestring key: The key for the new data object
        :param NodeData value: The value of the new data object

        :raises: TypeError, KeyError
        """
        if not isinstance(key, basestring):
            raise TypeError('Key must be a string!')
        if not isinstance(value, NodeData):
            raise TypeError('Value must be a NodeData object!')
        if key in self._map:
            raise KeyError('Key %s is already taken!' % key)
        self._map[key] = value

    def is_updated(self, key):
        """Check whether the data at the given key has been updated since the `updated` property was last reset

        :param basestring key: Key for the data object whose updated status we want to check

        :rtype: bool
        """
        return self._map[key].updated

    def reset_updated(self):
        """Reset the `updated` property of all data in this map.
        """
        for key in self._map:
            self._map[key].reset_updated()

    def get_callback(self, key):
        """Return the setter function for the :class:NodeData object at the given key.

        The usual reason to use this is wanting to wire inputs and
        outputs (see :meth:Node.subscribe and :meth:Node._wire_input).

        :raises: KeyError
        """
        if key not in self._map:
            raise KeyError('No member named %s' % key)
        return self._map[key].set

    def __len__(self):
        return len(self._map)

    def __getitem__(self, key):
        if key not in self._map:
            raise KeyError('No member named %s' % key)
        return self._map[key].get()

    def __setitem__(self, key, value):
        if key not in self._map:
            raise KeyError('No member named %s' % key)
        self._map[key].set(value)

    def __iter__(self):
        return self._map.__iter__()

    def __contains__(self, key):
        return key in self._map
