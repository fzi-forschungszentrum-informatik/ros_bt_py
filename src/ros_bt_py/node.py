import rospy


class Node(object):
    IDLE = 0
    RUNNING = 1
    SUCCEEDED = 2
    FAILED = 3
    BROKEN = 4

    def __init__(self):
        self.inputs = {}
        self.outputs = {}
        self.options = {}
        self.subscriptions = {}
        self.input_callbacks = {}
        self.output_callbacks = {}

    def subscribe(self, output_name, callback):
        """Subscribe to changes in the named output.

        :type output_name: str
        :param output_name: identifier of the output you want to
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

    def __wire_input(self, input_name, callback):
        """Wire an input to be passed to some callback.

        This is a lot like `subscribe`, but for input instead of output
        data. Input callbacks are called before the step function in
        `tick`, so any child nodes get the data they need.
        """
        if input_name not in self.inputs:
            raise KeyError('%s is not an input of %s'
                           % (input_name, type(self).__name__))
        if input_name not in self.input_callbacks:
            self.input_callbacks[input_name] = []
        self.input_callbacks[input_name].append(callback)

    def tick(self):
        for output_name in self.outputs:
            self.outputs[output_name].reset_updated()

        for input_name in self.inputs:
            if self.inputs[input_name].updated:
                if input_name in self.input_callbacks:
                    for callback in self.input_callbacks[input_name]:
                        callback(self.inputs[input_name].get())
            else:
                rospy.logwarn('Running tick() with stale data!')
            if self.inputs[input_name].get() is None:
                raise Exception('Trying to tick a node with an unset input!')

        self.step()

        for output_name in self.outputs:
            if self.outputs[output_name].updated:
                if output_name in self.output_callbacks:
                    for callback in self.output_callbacks[output_name]:
                        callback(self.outputs[output_name].get())

        for input_name in self.inputs:
            self.inputs[input_name].reset_updated()

    def step(self):
        """
        Every node class must override this.

        This method should NOT block, ever, and return one of the
        constants above.
        """
        raise NotImplementedError('Ticking a node without a step function!')

    def validate(self):
        """You must also override this.

        Called after constructing and wiring up a node to check that
        everything is in proper working order.

        Errors to check for:

        1. if any child input hasn't been assigned a getter, that's a
           **fatal** error
        2. if a child input has been assigned a getter from a child that
           may execute after it (or never), that's a **warning**
        3. any unconnected outputs are logged for possible **debugging**
        """
        pass

    def set(self, new_value):
        if not isinstance(new_value, self._data_type):
            raise TypeError('Expected data to be of type {}, got {} instead'.format(
                self._data_type.__name__, type(new_value).__name__))
        self._value = new_value
        self.updated = True

    def get(self):
        if not self.updated:
            rospy.logwarn('Reading non-updated value!')
