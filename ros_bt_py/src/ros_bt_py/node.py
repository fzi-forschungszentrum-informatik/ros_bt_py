from contextlib import contextmanager

import importlib
import jsonpickle

import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

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
        if node_class.__module__ not in Node.node_classes:
            Node.node_classes[node_class.__module__] = {
                node_class.__name__: node_class
                }
        else:
            Node.node_classes[node_class.__module__][node_class.__name__] = node_class
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
    @contextmanager
    def dummy_report_tick(self):
        self.loginfo('Ticking without debug manager')
        yield

    node_classes = {}
    node_config = None

    def __init__(self, options=None, debug_manager=None):
        """Prepare class members

        After this finishes, the Node is *not* ready to run. You still
        need to do your own initialization in :meth:Node.do_setup.

        :param dict options:

        Map from option names to option values. Use these for
        configuring your node, do not provide a custom `__init()__`
        method!

        :param ros_bt_py.debug_manager.DebugManager debug_manager:

        """
        self.name = type(self).__name__
        self.state = NodeMsg.UNINITIALIZED
        self.children = []

        self.debug_manager = debug_manager

        if not self.node_config:
            raise ValueError('Missing node_config, cannot initialize!')

        self.options = NodeDataMap(name='options')
        self._register_node_data(source_map=self.node_config.options,
                                 target_map=self.options,
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

        self.inputs = NodeDataMap(name='inputs')
        self._register_node_data(source_map=self.node_config.inputs,
                                 target_map=self.inputs,
                                 allow_ref=True)

        self.outputs = NodeDataMap(name='outputs')
        self._register_node_data(source_map=self.node_config.outputs,
                                 target_map=self.outputs,
                                 allow_ref=True)

        # Don't setup automatically - nodes should be available as pure data
        # containers before the user decides to call setup() themselves!
        # self.state = self.setup()

    def setup(self):
        """Prepare the node to be ticked for the first time.

        This is called after all the input, output and option values
        have been registered (and in the case of options, populated), so
        you can use those values in your implementation of
        :meth:Node.do_setup

        Sets the state of the node to whatever :meth:Node.do_setup
        returned.
        """
        self.state = self.do_setup()

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
        report_tick = self.dummy_report_tick()
        if self.debug_manager:
            report_tick = self.debug_manager.report_tick(self)

        with report_tick:
            if self.state is NodeMsg.UNINITIALIZED:
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

            self.state = self.do_tick()
            self.handle_outputs()

            return self.state

    def do_tick(self):
        """
        Every Node class must override this.

        This method should **NOT** block, ever, and return one of the
        constants from `Node.Status`.

        :returns:
        One of the constants in :class:ros_bt_py_msgs.msg.Node
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
        if self.state is NodeMsg.UNINITIALIZED:
            raise Exception('Trying to untick uninitialized node!')
        self.state = self.do_untick()
        if self.state != NodeMsg.IDLE and self.state != NodeMsg.PAUSED:
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
        if self.state is NodeMsg.UNINITIALIZED:
            raise Exception('Trying to reset uninitialized node!')
        self.state = self.do_reset()
        if self.state != NodeMsg.UNINITIALIZED:
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

    def add_child(self, child, at_index=None):
        """Add a child to this node at the given index

        :raises: Exception, KeyError

        `Exception` if the number of children after the add operation
        would exceed the maximum number of children, `KeyError` if a
        child of the same name already exists
        """
        if (self.node_config.max_children is not None and
                len(self.children) == self.node_config.max_children):
            error_msg = ('Trying to add child when maximum number of '
                         'children (%d) is already present'
                         % self.node_config.max_children)
            self.logerr(error_msg)
            raise Exception(error_msg)

        if child.name in (child.name for child in self.children):
            raise KeyError('Already have a child with name "%s"' % child.name)
        if at_index is None:
            at_index = len(self.children)
        # Use array slicing to efficiently insert child at the correct position
        # (the value we assign needs to be a list for this to work)
        self.children[at_index:at_index] = [child]

    def move_child(self, child_name, new_index):
        """
        Move the child with the name `child_name` to `new_index`

        :raises: IndexError if `new_index` is invalid
        """
        old_index = (child.name for child in self.children).index(child_name)
        self.children[old_index], self.children[new_index] = \
            self.children[new_index], self.children[old_index]

    def remove_child(self, child_name):
        """Remove the child with the given name and return it.

        :param basestring child_name: The name of the child to remove

        :rtype: Node
        :returns: The child that was just removed
        """
        child_index = (child.name for child in self.children).index(child_name)
        tmp = self.children[child_index]
        del self.children[child_index]
        return tmp

    def _register_node_data(self, source_map, target_map, allow_ref):
        """Register a number of typed :class:NodeData in the given map

        :param dict(str, type) source_map: a dictionary mapping from data keys to data types,
        i.e. ``{ 'a_string' : str, 'an_int' : int }``

        :param NodeDataMap target_map:
        The :class:NodeDataMap to add :class:NodeData values to

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
                    raise ValueError('OptionRef not allowed while adding to %s' % target_map.name)
                if data_type.option_key not in self.options:
                    raise KeyError("OptionRef for %s key '%s' references invalid option key '%s'"
                                   % (target_map.name, key, data_type.option_key))
                if not self.options.is_updated(data_type.option_key):
                    raise ValueError("OptionRef for %s key '%s' references unwritten "
                                     "option key '%s'"
                                     % (target_map.name, key, data_type.option_key))
                if not isinstance(self.options[data_type.option_key], type):
                    raise ValueError("OptionRef for %s key '%s' references option key '%s' "
                                     "that does not contain a type!"
                                     % (target_map.name, key, data_type.option_key))
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

    @classmethod
    def from_msg(cls, msg):
        """Construct a Node from the given ROS message.

        This will try to import the requested node class, instantiate it
        and populate its `name`, `options`, `input` and `output` members
        from the ROS message.

        :param ros_bt_py_msgs.msg.Node msg:

        A ROS message describing a node class. The node class must be
        available in the current environment (but does not need to be
        imported before calling this).

        :returns:

        An instance of the class named by `msg`, populated with the
        values from `msg`.

        Note that this does *not* include the node's state. Any node
        created by this will be in state UNININITIALIZED.
        """
        if (msg.module not in cls.node_classes or
                msg.node_class not in cls.node_classes[msg.module]):
            # If the node class was not available, try to load it
            load_node_module(msg.module)

        # If loading didn't work, abort
        if (msg.module not in cls.node_classes or
                msg.node_class not in cls.node_classes[msg.module]):
            rospy.logerr('Failed to instantiate node from message - node class '
                         'not available. Original message:\n%s', str(msg))
            return None

        node_class = cls.node_classes[msg.module][msg.node_class]

        # Populate options dict
        options_dict = {}
        for option in msg.options:
            options_dict[option.key] = jsonpickle.decode(option.value_serialized)

        # Instantiate node - this shouldn't do anything yet, since we don't
        # call setup()
        node_instance = node_class(options=options_dict)

        # Set name from ROS message
        if msg.name:
            node_instance.name = msg.name

        # Set inputs
        for input_msg in msg.current_inputs:
            node_instance.inputs[input_msg.key] = jsonpickle.decode(
                input_msg.value_serialized)

        # Set outputs
        for output_msg in msg.current_outputs:
            node_instance.outputs[output_msg.key] = jsonpickle.decode(
                output_msg.value_serialized)

        return node_instance

def load_node_module(package_name):
    """Import the named module at run-time.

    If the module contains any (properly decorated) node classes,
    they will be registered and available to load via the other
    commands in this class.
    """
    try:
        return importlib.import_module(package_name)
    except ImportError:
        return None
