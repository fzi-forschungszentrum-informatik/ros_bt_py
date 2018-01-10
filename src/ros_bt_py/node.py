import rospy

from ros_bt_py.node_data import NodeData, NodeDataMap


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
        UNINITIALIZED = -1
        IDLE = 0
        RUNNING = 1
        SUCCEEDED = 2
        FAILED = 3
        BROKEN = 4

    def __init__(self):
        """Prepare class members

        After this finishes, the Node is *not* ready to run. You still
        need to register inputs, outputs and potentially options and set
        the Node's state to :const:`Node.States.IDLE`
        """
        self.inputs = NodeDataMap()
        self.outputs = NodeDataMap()
        self.options = NodeDataMap()
        self.subscriptions = {}
        self.input_callbacks = {}
        self.output_callbacks = {}
        self.state = Node.States.UNINITIALIZED

    def subscribe(self, output_name, callback):
        """Subscribe to changes in the named output.

        :param str output_name: identifier of the output you want to
        subscribe to

        :param callback: a callback that takes a parameter of
        the same type as that output. This will be called after each
        tick, if a change was made to the output

        :raises KeyError: if `output_name` is not a valid output
        """
        if output_name not in self.outputs:
            raise KeyError('%s is not an output of %s'
                           % (output_name, type(self).__name__))
        if output_name not in self.output_callbacks:
            self.output_callbacks[output_name] = []
        self.output_callbacks[output_name].append(callback)

    def _wire_input(self, input_name, callback):
        """Wire an input to be passed to some callback.

        This is a lot like `subscribe`, but for input instead of output
        data. Input callbacks are called before the :meth:Node.step function in
        :meth:Node.tick, so any child nodes get the data they need.
        """
        if input_name not in self.inputs:
            raise KeyError('%s is not an input of %s'
                           % (input_name, type(self).__name__))
        if input_name not in self.input_callbacks:
            self.input_callbacks[input_name] = []
        self.input_callbacks[input_name].append(callback)

    def handle_inputs(self):
        """Execute the callbacks registered by :meth:Node._wire_input

        But only if an input has been updated since the last tick.

        :raises: ValueError
        If any input is unset
        """
        for input_name in self.inputs:
            if self.inputs.is_updated(input_name):
                if input_name in self.input_callbacks:
                    for callback in self.input_callbacks[input_name]:
                        callback(self.inputs[input_name])
            else:
                rospy.logwarn('Running tick() with stale data!')
            if self.inputs[input_name] is None:
                raise ValueError('Trying to tick a node with an unset input!')

    def handle_outputs(self):
        """Execute the callbacks registered by :meth:Node.subscribe:

        But only if the output has changed during this tick (see where
        the :meth:NodeDataMap.reset_updated is called in
        :meth:Node.tick)
        """
        for output_name in self.outputs:
            if self.outputs.is_updated(output_name):
                if output_name in self.output_callbacks:
                    for callback in self.output_callbacks[output_name]:
                        callback(self.outputs[output_name])

    def tick(self):
        """This is called every tick (ticks happen at ~10-20Hz, usually.

        You should not need to override this method, but instead
        implement :meth:Node.step in your own class.

        :returns:
        The state of the node after ticking - should be `SUCCEEDED`, `FAILED` or `RUNNING`.
        """
        if self.state is Node.States.UNINITIALIZED:
            raise Exception('Trying to tick uninitialized node!')

        # Outputs are updated in the tick. To catch that, we need to reset here.
        self.outputs.reset_updated()

        self.handle_inputs()

        self.state = self.step()

        self.handle_outputs()

        self.inputs.reset_updated()

    def step(self):
        """
        Every Node class must override this.

        This method should **NOT** block, ever, and return one of the
        constants from `Node.Status`.

        :returns:
        One of the constants in :class:Node.States
        """
        raise NotImplementedError('Ticking a node without a step function!')

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

    def _register_inputs(self, input_map):
        """Register a number of typed inputs for this Node

        :param dict(str, type) input_map: a dictionary mapping from input names to input types,
        i.e. ``{ 'string_input' : str, 'int_input' : int }``
        """
        for input_name, data_type in input_map.iteritems():
            if input_name in self.inputs:
                raise ValueError('Duplicate input name: %s' % input_name)
            self.inputs.add(input_name, NodeData(data_type=data_type))

    def _register_outputs(self, output_map):
        """Register a number of typed outputs for this Node

        :param dict(str, type) output_map: a dictionary mapping from input names to output types,
        i.e. ``{ 'string_output' : str, 'int_output' : int }``
        """
        for output_name, data_type in output_map.iteritems():
            if output_name in self.outputs:
                raise ValueError('Duplicate output name: %s' % output_name)
            self.outputs.add(output_name, NodeData(data_type=data_type))

    def _register_options(self, option_map):
        """Register a number of options and their values for this Node.

        Options are static, meaning they cannot be changed after
        registering them, and their types are inferred automatically
        from the values.

        :param dict(str, tuple) option_map:
        a dictionary mapping from option names to the corresponding
        values i.e. ``{ 'string_option' : 'Hello World!', 'int_option' : 42 }``
        """
        for option_name, option_value in option_map.iteritems():
            if option_name in self.options:
                raise ValueError('Duplicate option name: %s' % option_name)
            self.options.add(option_name, NodeData(data_type=type(option_value),
                                                   initial_value=option_value,
                                                   static=True))

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
