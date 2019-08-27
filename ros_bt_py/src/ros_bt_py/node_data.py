import jsonpickle
import rospy

from ros_bt_py.helpers import loglevel_is


def from_string(data_type, string_value, static=False):
    return NodeData(data_type=data_type,
                    initial_value=data_type(string_value),
                    static=static)


class NodeData(object):
    """Represents a piece of data (input, output or option) held by a Node

    Each `NodeData` object is typed and will only accept new data (via its
    :meth:`set` method) if it is of the correct type.

    `NodeData` can also be static, in which case it will only accept one
    update (the initial value, if not empty, counts as an update!)
    """
    def __init__(self, data_type, initial_value=None, static=False):
        self.updated = False
        self._value = None
        self._serialized_value = jsonpickle.encode(None)
        self._static = static

        # Relax type checking for string types
        if data_type is str or data_type is unicode:
            self.data_type = basestring
        else:
            self.data_type = data_type

        self._serialized_type = jsonpickle.encode(self.data_type)

        # use set here to ensure initial_value is the right type
        # this also sets updated to True
        if initial_value is not None:
            self.set(initial_value)

    def __repr__(self):
        return '{!s} ({}) [{}]'.format(self._value,
                                       self.data_type.__name__,
                                       ('#' if self.updated else ' '))

    def __eq__(self, other):
        return (self.updated == other.updated and
                self._static == other._static and
                self._value == other._value and
                self.data_type == other.data_type)

    def __ne__(self, other):
        return not self == other

    def takes(self, new_value):
        """Check whether `new_value` has a type that matches this NodeData object"""
        if self._static and self.updated:
            return False

        if isinstance(new_value, self.data_type) or new_value is None:
            return True

        # Special case for int and float
        if self.data_type == float and isinstance(new_value, int):
            return True

        return False

    def set(self, new_value):
        """Set a new value

        After calling this successfully, :py:attribute:: updated is `True`.
        :param new_value:
        Must be either `None` or an instance of type :py:attribute:: NodeData.data_type

        :raises: Exception, TypeError
        """
        if self._static and self.updated:
            raise Exception('Trying to overwrite data in static NodeData object')
        if not isinstance(new_value, self.data_type) and new_value is not None:
            # Silently convert ints to float
            if self.data_type == float and isinstance(new_value, int):
                new_value = float(new_value)
            else:
                if type(new_value) == dict and "py/type" in new_value:
                    raise TypeError(
                        ('Expected data to be of type {}, got {} instead. '
                         'Looks like failed jsonpickle decode, does type {} exist?').format(
                             self.data_type.__name__,
                             type(new_value).__name__,
                             new_value['py/type']))
                raise TypeError('Expected data to be of type {}, got {} instead'.format(
                    self.data_type.__name__, type(new_value).__name__))
        if self._serialized_value is not None and new_value != self._value:
            self._serialized_value = jsonpickle.encode(new_value)
        self._value = new_value
        self.updated = True

    def get(self):
        """Get the current value

        Will log a warning if the value has not been updated since the
        last call of :meth:`reset_updated` (unless this piece of data is
        marked static, in which case we do not expect updates).
        """
        # if not self.updated:
        #     rospy.loginfo('Reading non-updated value!')
        return self._value

    def get_serialized(self):
        if self._serialized_value is None:
            self._serialized_value = jsonpickle.encode(self._value)
        return self._serialized_value

    def get_serialized_type(self):
        return self._serialized_type

    def set_updated(self):
        self.updated = True

    def reset_updated(self):
        self.updated = False


class NodeDataMap(object):
    """Custom container class that hides :meth:`NodeData.get` and :meth:`NodeData.set`

    Acts like a regular dict, with three caveats:

    1. All keys must be strings

    2. All values must be :class:NodeData objects

    3. It is not possible to delete or overwrite values once they've
    been added to the map. This is because the setters for values might
    be used as callbacks elsewhere, leading to unexpected breakage!
    """
    def __init__(self, name='data'):
        self.name = name
        self._map = {}
        self.callbacks = {}

    def subscribe(self, key, callback, subscriber_name=''):
        """Subscribe to changes in the value at `key`.

        :param str key: the key you want to subscribe to

        :param callback:
          a callback that takes a parameter of
          the same type as the value at `key`. This will be called when
          :meth:`handle_subscriptions` is called, if a change
          was made to the value since the last call of :meth:`reset_updated`

        :param str subscriber_name: the name of the subscriber node. Only used for logging.

        :raises KeyError: if `key` is not a valid key
        """
        if key not in self._map:
            raise KeyError('%s is not a key of %s'
                           % (key, self.name))
        if key not in self.callbacks:
            self.callbacks[key] = []
        if any([subscriber_name == name for _, name in self.callbacks[key]]):
            # Do nothing if we already have a callback with the given
            # name for this key
            return
        self.callbacks[key].append((callback, subscriber_name))

    def unsubscribe(self, key, callback=None):
        """Remove the given subscriber callback from `key`'s subscriber list.

        :param str key: The key that `callback` is currently subscribed to
        :param callback: The callback to remove - if `None`, remove all callbacks

        :raises KeyError: if `key` is not a valid key
        """
        if key not in self._map:
            raise KeyError('%s is not a key of %s'
                           % (key, self.name))
        if key not in self.callbacks:
            self.callbacks[key] = []
            rospy.loginfo('Trying to remove callback for key with no callbacks at all (%s[%s])',
                          self.name,
                          key)
            return

        if callback:
            index = None
            rospy.loginfo(str(self.callbacks))
            for i, value in enumerate(self.callbacks[key]):
                if value[0] == callback:
                    index = i
            if index is not None:
                callback_name = self.callbacks[key].pop(index)[1]
                rospy.loginfo('Removed callback "%s" of key %s[%s]',
                              callback_name,
                              self.name, key)
            else:
                rospy.loginfo('Trying to remove unknown callback for key %s[%s]',
                              self.name,
                              key)
        else:
            rospy.loginfo('Removing all callbacks for key %s[%s]',
                          self.name,
                          key)
            self.callbacks[key] = []

    def handle_subscriptions(self):
        """Execute the callbacks registered by :meth:`subscribe`:

        Only executes a callback if the corresponding piece of data has
        been updated since the last call of
        :meth:`reset_updated`.
        """
        debugging = loglevel_is(rospy.DEBUG)
        for key in self._map:
            if self.is_updated(key):
                if key in self.callbacks:
                    for callback, subscriber_name in self.callbacks[key]:
                        if debugging:
                            rospy.logdebug('Forwarding value %s of key %s to subscriber %s',
                                           str(self[key]), key, subscriber_name)
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
        """Check whether the data at the given key has been updated since the `updated` property
        was last reset

        :param basestring key: Key for the data object whose updated status we want to check

        :rtype: bool
        """
        return self._map[key].updated

    def set_updated(self, key):
        """Set the `updated` property for the given datum"""
        if key in self._map:
            self._map[key].set_updated()
        else:
            raise KeyError('No member named %s' % key)

    def reset_updated(self):
        """Reset the `updated` property of all data in this map.
        """
        for key in self._map:
            self._map[key].reset_updated()

    def get_callback(self, key):
        """Return the setter function for the :class:NodeData object at the given key.

        The usual reason to use this is wanting to wire inputs and
        outputs (see :meth:`Node.subscribe and :meth:`Node._wire_input`).

        :raises: KeyError
        """
        if key not in self._map:
            raise KeyError('No member named %s' % key)
        return self._map[key].set

    def get_serialized(self, key):
        """
        Return the jsonpickle'd value of the NodeData object at `key`
        """
        if key not in self._map:
            raise KeyError('No member named %s' % key)
        return self._map[key].get_serialized()

    def get_serialized_type(self, key):
        """
        Return the jsonpickle'd type of the NodeData object at `key`
        """
        if key not in self._map:
            raise KeyError('No member named %s' % key)
        return self._map[key].get_serialized_type()

    def get_type(self, key):
        """
        Return the type of the NodeData object at `key`
        """
        if key not in self._map:
            raise KeyError('No member named %s' % key)
        return self._map[key].data_type

    def compatible(self, key, new_val):
        """Check if `new_val` can be put into the :class:`NodeData` at `key`"""
        return self._map[key].takes(new_val)

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

    def __eq__(self, other):
        return (self.name == other.name and
                len(self) == len(other) and
                all([key in other for key in self]) and
                all([other[key] == self[key] for key in self]) and
                len(self.callbacks) == len(other.callbacks) and
                all([key in other.callbacks for key in self.callbacks]) and
                all([other.callbacks[key] == self.callbacks[key] for key in self.callbacks]))

    def __ne__(self, other):
        return not self == other

    def __repr__(self):
        return 'NodeDataMap(name=%r), data:%r, callbacks:%r' % (
            self.name, self._map, self.callbacks)
