import rospy

from ros_bt_py.node_data import NodeData, NodeDataMap
from ros_bt_py.node_config import OptionRef


def define_bt_node(node_config):
    """Provide information about this Node's interface

    Every class that derives, directly or indirectly, from :class:Node,
    must be decorated with this!

    :param NodeConfig node_config:

    This describes your Node's interface. All inputs, outputs and
    options defined here are automatically registered with your
    class. You should not need to register anything manually!
    """
    def inner_dec(node_class):
        for base in node_class.__bases__:
            if hasattr(base, 'node_config') and base.node_config:
                node_config.extend(base.node_config)
        node_class.node_config = node_config
        return node_class
    return inner_dec


class Node(object):
    """Base class for Behavior Tree nodes

    Each node has a set of inputs, outputs and options. At every tick
    (usually somewhere between 10 and 30 times a second),
    :meth:Node.step is called with the appropriate data.

    Nodes in a behavior Tree can be roughly divided into four classes:

    Leaf Nodes
      These do not have any children and can take one of two forms:
      *Predicates* and *Behaviors*. *Predicates* check a condition and instantly
      return `SUCCEEDED` or `FAILED`. *Behaviors* are more involved and may
      return `RUNNING`, but should be interruptible (see :meth:Node.untick).

    Inner Nodes
      These too come in two flavors: *Combiners* and *Decorators*. *Combiners*
      have multiple children and decide which of those children to run (and in
      what fashion) based on some criteria. *Decorators* however have only a
      single child and work with that child's result - for instance, a *Decorator*
      could invert `FAILED` into `SUCCEEDED`.

      """
    class States(object):
        UNINITIALIZED = 'UNINITIALIZED'
        IDLE = 'IDLE'
        RUNNING = 'RUNNING'
        SUCCEEDED = 'SUCCEEDED'
        FAILED = 'FAILED'
        BROKEN = 'BROKEN'
        PAUSED = 'PAUSED'

    node_config = None

    def __init__(self, options=None):
        """Prepare class members

        After this finishes, the Node is *not* ready to run. You still
        need to do your own initialization in :meth:Node.do_setup.

        :param dict options:

        Map from option names to option values. Use these for
        configuring your node, do not provide a custom `__init()__`
        method!
        """
        self.name = type(self).__name__

        if not self.node_config:
            raise ValueError('Missing node_config, cannot initialize!')

        self.options = NodeDataMap()
        self._register_node_data(source_map=self.node_config.options,
                                 target_map=self.options,
                                 map_name='options',
                                 allow_ref=False)

        # Set options from constructor parameter
        unset_option_keys = []
        for key in self.options:
            if options is None or key not in options:
                unset_option_keys.append(key)
            else:
                self.options[key] = options[key]
        if unset_option_keys:
            raise ValueError('Missing options: %s'
                             % str(unset_option_keys))

        self.inputs = NodeDataMap()
        self._register_node_data(source_map=self.node_config.inputs,
                                 target_map=self.inputs,
                                 map_name='inputs',
                                 allow_ref=True)

        self.outputs = NodeDataMap()
        self._register_node_data(source_map=self.node_config.outputs,
                                 target_map=self.outputs,
                                 map_name='outputs',
                                 allow_ref=True)

        self.state = self.setup()

    def setup(self):
        """Prepare the node to be ticked for the first time.

        This is called after all the input, output and option values
        have been registered (and in the case of options, populated), so
        you can use those values in your implementation of
        :meth:Node.do_setup

        :returns: One of the values in :class:Node.States - should be `IDLE`
        """
        return self.do_setup()

    def do_setup(self):
        """Use this to do custom node setup.

        Note that this will be called once, when the tree is first
        started, and before the first call of :meth:Node.tick.

        :rtype basestring:
        :return:
        This method must return one of the constants in :class:Node.State
        """
        raise NotImplementedError('Trying to setup a node with no do_setup() method')

    def handle_inputs(self):
        """Execute the callbacks registered by :meth:Node._wire_input

        But only if an input has been updated since the last tick.

        :raises: ValueError
        If any input is unset
        """
        for input_name in self.inputs:
            if not self.inputs.is_updated(input_name):
                rospy.logwarn('Running tick() with stale data!')
            if self.inputs[input_name] is None:
                raise ValueError('Trying to tick a node with an unset input!')
        self.inputs.handle_subscriptions()

    def handle_outputs(self):
        """Execute the callbacks registered by :meth:Node.subscribe:

        But only if the output has changed during this tick (see where
        the :meth:NodeDataMap.reset_updated is called in
        :meth:Node.tick)
        """
        self.outputs.handle_subscriptions()

    def tick(self):
        """This is called every tick (ticks happen at ~10-20Hz, usually.

        You should not need to override this method, but instead
        implement :meth:Node.do_tick in your own class.

        :returns:
        The state of the node after ticking - should be `SUCCEEDED`, `FAILED` or `RUNNING`.
        """
        if self.state is Node.States.UNINITIALIZED:
            raise Exception('Trying to tick uninitialized node!')

        unset_options = []
        for option_name in self.options:
            if not self.options.is_updated(option_name):
                unset_options.append(option_name)
        if unset_options:
            msg = 'Trying to tick node with unset options: %s' % str(unset_options)
            self.logerr(msg)
            raise Exception(msg)
        self.options.handle_subscriptions()

        # Outputs are updated in the tick. To catch that, we need to reset here.
        self.outputs.reset_updated()

        # Inputs can override options!
        self.handle_inputs()
        # Inputs are updated by other nodes' outputs, i.e. some time after we
        # use them here. In some cases, inputs might be connected to child
        # outputs (or even our own), which is why we reset before calling step()
        self.inputs.reset_updated()

        self.state = self.step()
        self.handle_outputs()

        return self.state

    def do_tick(self):
        """
        Every Node class must override this.

        This method should **NOT** block, ever, and return one of the
        constants from `Node.Status`.

        :returns:
        One of the constants in :class:Node.States
        """
        msg = 'Ticking a node without a do_tick function!'
        self.logerr(msg)
        raise NotImplementedError(msg)

    def untick(self):
        """Calling this method signals to a node that it should stop any background tasks.

        A new tick has started and this node has **not** been ticked.
        The node's state should be `IDLE` after calling this.

        The node's outputs' `updated` flags are also reset!

        A class inheriting from :class:Node should override :meth:Node.do_untick instead of this!
        """
        if self.state is Node.States.UNINITIALIZED:
            raise Exception('Trying to untick uninitialized node!')
        self.state = self.do_untick()
        if self.state != Node.States.IDLE and self.state != Node.States.PAUSED:
            self.logwarn('untick() did not result in IDLE state, but %s' % self.state)

        self.outputs.reset_updated()

    def do_untick(self):
        """This is called by :meth:Node.untick - override it!

        After executing this method, your node should:

        1. Be in the IDLE or PAUSED state, unless an error happened
        2. Not execute any of its behavior in the background
        3. Be ready to resume on the next call of :meth:Node.tick / :meth:Node.step
        """
        msg = 'Unticking a node without untick function!'
        self.logerr(msg)
        raise NotImplementedError(msg)

    def reset(self):
        """Use this to reset a node completely

        Whereas :meth:Node.untick / :meth:Node.do_untick only pauses
        execution, ready to be resumed, :meth:Node.reset means returning
        to the same state the node was in right after construction.
        """
        if self.state is Node.States.UNINITIALIZED:
            raise Exception('Trying to reset uninitialized node!')
        self.state = self.do_reset()
        if self.state != Node.States.UNINITIALIZED:
            self.logerr('untick() did not result in UNINITIALIZED state, but %s'
                        % self.state)

    def do_reset(self):
        """
        This is called to reset the node to its initial state.

        After executing this method, your node should:

        1. Be in the IDLE state
        2. Not be doing anything in the background
        3. On the next tick, behave as if it has just been created

        :returns:
        The new state of the node (should be IDLE unless an error happened)
        """
        msg = 'Trying to reset a node without do_reset function'
        self.logerr(msg)
        raise NotImplementedError(msg)

    def validate(self):
        """You must also override this.

        Called after constructing and wiring up a node to check that
        everything is in proper working order.

        Errors to check for:

        1. if any child input hasn't been wired, that's a
           **fatal** error
        2. if a child input has been wired to an output of a child that
           may execute after it (or never), that's a **warning**
        3. any unconnected outputs are logged for possible **debugging**
        """
        pass

    def _register_node_data(self, source_map, target_map, map_name, allow_ref):
        """Register a number of typed :class:NodeData in the given map

        :param dict(str, type) source_map: a dictionary mapping from data keys to data types,
        i.e. ``{ 'a_string' : str, 'an_int' : int }``

        :param NodeDataMap target_map:
        The :class:NodeDataMap to add :class:NodeData values to

        :param str map_name:
        The name of `target_map`, used for logging

        :param bool allow_ref:
        Decides whether :class:OptionRef is accepted as a type. If True, the
        option keys referenced by any :class:OptionRef objects must
        exist and be populated!

        :raises: KeyError, ValueError
        If any of the keys in `source_map` already exist in `target_map`,
        raise KeyError.
        If an OptionRef value is passed, but `allow_ref` is `False`, raise ValueError.
        If an OptionRef references an option value that has not been set or
        does not exist, raise KeyError.
        If an OptionRef references an option value that does not hold a `type`,
        raise ValueError.
        """
        for key, data_type in source_map.iteritems():
            if key in target_map:
                raise KeyError('Duplicate output name: %s' % key)

            if isinstance(data_type, OptionRef):
                if not allow_ref:
                    raise ValueError('OptionRef not allowed while adding to %s' % map_name)
                if data_type.option_key not in self.options:
                    raise KeyError("OptionRef for %s key '%s' references invalid option key '%s'"
                                   % (map_name, key, data_type.option_key))
                if not self.options.is_updated(data_type.option_key):
                    raise ValueError("OptionRef for %s key '%s' references unwritten "
                                     "option key '%s'"
                                     % (map_name, key, data_type.option_key))
                if not isinstance(self.options[data_type.option_key], type):
                    raise ValueError("OptionRef for %s key '%s' references option key '%s' "
                                     "that does not contain a type!"
                                     % (map_name, key, data_type.option_key))
                target_map.add(key, NodeData(data_type=self.options[data_type.option_key]))
            else:
                target_map.add(key, NodeData(data_type=data_type))

    # Logging methods - these just use the ROS logging framework, but add the
    # name and type of the node so it's easier to trace errors.

    def logdebug(self, message):
        """Wrapper for :func:rospy.logdebug

        Adds this node's name and type to the given message"""
        rospy.logdebug('%s (%s): %s', self.name, type(self).__name__, message)

    def loginfo(self, message):
        """Wrapper for :func:rospy.loginfo

        Adds this node's name and type to the given message"""
        rospy.loginfo('%s (%s): %s', self.name, type(self).__name__, message)

    def logwarn(self, message):
        """Wrapper for :func:rospy.logwarn

        Adds this node's name and type to the given message"""
        rospy.logwarn('%s (%s): %s', self.name, type(self).__name__, message)

    def logerr(self, message):
        """Wrapper for :func:rospy.logerr

        Adds this node's name and type to the given message"""
        rospy.logerr('%s (%s): %s', self.name, type(self).__name__, message)

    def logfatal(self, message):
        """Wrapper for :func:rospy.logfatal

        Adds this node's name and type to the given message"""
        rospy.logfatal('%s (%s): %s', self.name, type(self).__name__, message)
