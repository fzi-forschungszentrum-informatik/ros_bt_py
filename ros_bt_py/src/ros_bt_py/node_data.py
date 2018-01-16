import rospy


def from_string(data_type, string_value, static=False):
    return NodeData(data_type=data_type,
                    initial_value=data_type(string_value),
                    static=static)


class NodeData(object):
    """Represents a piece of data (input, output or option) held by a Node

    Each NodeData object is typed and will only accept new data (via its
    :meth:NodeData.set method) if it is of the correct type.

    NodeData can also be static, in which case it will only accept one
    update (the initial value, if not empty, counts as an update!)
    """
    def __init__(self, data_type, initial_value=None, static=False):
        self.updated = False
        self._value = None
        self._static = static

        # Relax type checking for string types
        if data_type is str or data_type is unicode:
            self._data_type = basestring
        else:
            self._data_type = data_type

        # use set here to ensure initial_value is the right type
        # this also sets updated to True
        if initial_value is not None:
            self.set(initial_value)

    def __repl__(self):
        return '{!s} ({}) [{}]'.format(self._value,
                                       self._data_type.__name__,
                                       ('#' if self.updated else ' '))

    def set(self, new_value):
        """Set a new value

        After calling this successfully, :py:attribute:: updated is `True`.
        :param new_value:
        Must be either `None` or an instance of type :py:attribute:: Node._data_type

        :raises: Exception, TypeError
        """
        if self._static and self.updated:
            raise Exception('Trying to overwrite data in static NodeData object')
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
        if not self.updated:
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
    def __init__(self, name='data'):
        self._name = name
        self._map = {}
        self._callbacks = {}

    def subscribe(self, key, callback, path=''):
        """Subscribe to changes in the value at `key`.

        :param str key: the key you want to subscribe to

        :param callback: a callback that takes a parameter of
        the same type as the value at `key`. This will be called when
        :meth:NodeDatamap.handle_subscriptions is called, if a change
        was made to the value since the last call of :meth:NodeDataMap.reset_updated

        :param str path: the tree path of the subscriber. Only used for logging.

        :raises KeyError: if `key` is not a valid key
        """

        if key not in self._map:
            raise KeyError('%s is not a key of %s'
                           % (key, self._name))
        if key not in self._callbacks:
            self._callbacks[key] = []
        self._callbacks[key].append((callback, path))

    def handle_subscriptions(self):
        """Execute the callbacks registered by :meth:NodeDataMap.subscribe:

        Only executes a callback if the corresponding piece of data has
        been updated since the last call of
        :meth:NodeDataMap.reset_updated.
        """
        for key in self._map:
            if self.is_updated(key):
                if key in self._callbacks:
                    for callback, path in self._callbacks[key]:
                        rospy.logdebug('Forwarding value %s to subscriber %s',
                                       key, path)
                        callback(self[key])

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