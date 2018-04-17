from contextlib import contextmanager

import importlib
import jsonpickle
import re

import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import NodeData as NodeDataMsg
from ros_bt_py_msgs.msg import NodeDataLocation

from ros_bt_py.exceptions import BehaviorTreeException, NodeStateError, NodeConfigError
from ros_bt_py.node_data import NodeData, NodeDataMap
from ros_bt_py.node_config import NodeConfig, OptionRef


def define_bt_node(node_config):
    """Provide information about this Node's interface

    Every class that derives, directly or indirectly, from :class:`Node`,
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
    :meth:`tick` is called with the appropriate data.

    Nodes in a behavior Tree can be roughly divided into two classes,
    with two sub-classes each:

    Leaf Nodes
      These do not have any children and can take one of two forms:
      *Predicates* and *Behaviors*. *Predicates* check a condition and instantly
      return `SUCCEEDED` or `FAILED`. *Behaviors* are more involved and may
      return `RUNNING`, but should be interruptible (see :meth:`untick`).

    Inner Nodes
      These too come in two flavors: *Combiners* and *Decorators*. *Combiners*
      have multiple children and decide which of those children to run (and in
      what fashion) based on some criteria. *Decorators* however have only a
      single child and work with that child's result - for instance, a *Decorator*
      could invert `FAILED` into `SUCCEEDED`.
    """
    @contextmanager
    def _dummy_report_tick(self):
        self.loginfo('Ticking without debug manager')
        yield

    node_classes = {}
    node_config = None

    def __init__(self, options=None, debug_manager=None, name=None):
        """Prepare class members

        After this finishes, the Node is *not* ready to run. You still
        need to do your own initialization in :meth:`_do_setup`.

        :param dict options:

        Map from option names to option values. Use these for
        configuring your node, do not provide a custom `__init()__`
        method!

        :param ros_bt_py.debug_manager.DebugManager debug_manager:

        :param str name:

        Name of the node - defaults to None, in which case the node name
        is set to the name of its class.

        :raises: NodeConfigError

        If anything is wrong with the node configuration defined via
        :function:`ros_bt_py.node.define_bt_node`
        """
        if name is not None:
            self.name = name
        else:
            self.name = type(self).__name__
        # Only used to make finding the root of the tree easier
        self.parent_name = ''
        self.state = NodeMsg.UNINITIALIZED
        self.children = []

        self.debug_manager = debug_manager

        if not self.node_config:
            raise NodeConfigError('Missing node_config, cannot initialize!')

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

    def setup(self):
        """Prepare the node to be ticked for the first time.

        This is called after all the input, output and option values
        have been registered (and in the case of options, populated), so
        you can use those values in your implementation of
        :meth:`_do_setup`

        Sets the state of the node to whatever :meth:`_do_setup`
        returned.

        :raises: BehaviorTreeException if called when the node is not UNINITIALIZED or SHUTDOWN
        """
        if self.state != NodeMsg.UNINITIALIZED and self.state != NodeMsg.SHUTDOWN:
            raise BehaviorTreeException(
                'Calling setup() is only allowed in states %s and %s, but node %s is in state %s'
                % (NodeMsg.UNINITIALIZED, NodeMsg.SHUTDOWN, self.name, self.state))
        self.state = self._do_setup()
        self._setup_called = True

    def _do_setup(self):
        """Use this to do custom node setup.

        Note that this will be called once, when the tree is first
        started, before the first call of :meth:`tick`.

        :rtype: basestring
        :returns:
          This method must return one of the constants in :class:`ros_bt_py_msgs.msg.Node`
        """

        msg = ('Trying to setup a node of type %s without _do_setup function!'
               % self.__class__.__name__)
        rospy.logerr(msg)
        raise NotImplementedError(msg)

    def _handle_inputs(self):
        """Execute the callbacks registered by :meth:`_wire_input`

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

    def _handle_outputs(self):
        """Execute the callbacks registered by :meth:`NodeDataMap.subscribe`:

        But only if the output has changed during this tick (see where
        the :meth:`NodeDataMap.reset_updated` is called in
        :meth:`tick`)
        """
        self.outputs.handle_subscriptions()

    def tick(self):
        """This is called every tick (ticks happen at ~10-20Hz, usually.

        You should not need to override this method, but instead
        implement :meth:`_do_tick` in your own class.

        :returns:
          The state of the node after ticking - should be `SUCCEEDED`, `FAILED` or `RUNNING`.

        :raises: BehaviorTreeException if a tick is impossible / not allowed
        """
        report_tick = self._dummy_report_tick()
        if self.debug_manager:
            report_tick = self.debug_manager.report_tick(self)

        with report_tick:
            if self.state is NodeMsg.UNINITIALIZED:
                raise BehaviorTreeException('Trying to tick uninitialized node!')

            unset_options = []
            for option_name in self.options:
                if not self.options.is_updated(option_name):
                    unset_options.append(option_name)
            if unset_options:
                msg = 'Trying to tick node with unset options: %s' % str(unset_options)
                self.logerr(msg)
                raise BehaviorTreeException(msg)
            self.options.handle_subscriptions()

            # Outputs are updated in the tick. To catch that, we need to reset here.
            self.outputs.reset_updated()

            # Inputs can override options!
            self._handle_inputs()
            # Inputs are updated by other nodes' outputs, i.e. some time after we
            # use them here. In some cases, inputs might be connected to child
            # outputs (or even our own), which is why we reset before calling step()
            self.inputs.reset_updated()

            self.state = self._do_tick()
            self.raise_if_in_invalid_state(allowed_states=[NodeMsg.RUNNING,
                                                           NodeMsg.SUCCEEDED,
                                                           NodeMsg.FAILED],
                                           action_name='tick()')
            self._handle_outputs()

            return self.state

    def raise_if_in_invalid_state(self, allowed_states, action_name):
        """Raise an error if `self.state` is not in `allowed_states`"""
        if self.state not in allowed_states:
            raise NodeStateError('Node %s (%s) was in invalid state %s after action %s. '
                                 'Allowed states: %s' % (
                                     self.name,
                                     type(self).__name__,
                                     self.state,
                                     action_name,
                                     str(allowed_states)))

    def _do_tick(self):
        """
        Every Node class must override this.

        This method should **NOT** block, ever, and return one of the
        constants from `Node.Status`.

        :returns:
          One of the constants in :class:`ros_bt_py_msgs.msg.Node`
        """
        msg = ('Ticking a node of type %s without _do_tick function!'
               % self.__class__.__name__)
        rospy.logerr(msg)
        raise NotImplementedError(msg)

    def untick(self):
        """Calling this method signals to a node that it should stop any background tasks.

        A new tick has started and this node has **not** been ticked.
        The node's state should be `IDLE` after calling this.

        The node's outputs' `updated` flags are also reset!

        A class inheriting from :class:`Node` should override :meth:`_do_untick` instead of this!

        :raises: BehaviorTreeException
          When trying to untick a node that has not been initialized yet.
        """
        if self.state is NodeMsg.UNINITIALIZED:
            raise BehaviorTreeException('Trying to untick uninitialized node!')
        self.state = self._do_untick()
        self.raise_if_in_invalid_state(allowed_states=[NodeMsg.IDLE,
                                                       NodeMsg.PAUSED],
                                       action_name='untick()')

        self.outputs.reset_updated()

    def _do_untick(self):
        """This is called by :meth:`untick` - override it!

        After executing this method, your node should:

        1. Be in the IDLE or PAUSED state, unless an error happened
        2. Not execute any of its behavior in the background
        3. Be ready to resume on the next call of :meth:`tick`
        """
        msg = ('Unticking a node of type %s without _do_untick function!'
               % self.__class__.__name__)
        self.logerr(msg)
        raise NotImplementedError(msg)

    def reset(self):
        """Use this to reset a node completely

        Whereas :meth:`untick` / :meth:`_do_untick` only pauses
        execution, ready to be resumed, :meth:`reset` means returning
        to the same state the node was in right after calling :meth:`setup`

        :raises: BehaviorTreeException
          When trying to reset a node that hasn't been initialized yet
        """
        if self.state is NodeMsg.UNINITIALIZED:
            raise BehaviorTreeException('Trying to reset uninitialized node!')
        self.state = self._do_reset()
        self.raise_if_in_invalid_state(allowed_states=[NodeMsg.IDLE],
                                       action_name='reset()')

    def _do_reset(self):
        """
        This is called to reset the node to its initial state.

        After executing this method, your node should:

        1. Be in the IDLE state
        2. Not be doing anything in the background
        3. On the next tick, behave as if it has just been created

        :returns:
          The new state of the node (should be IDLE unless an error happened)
        """
        msg = ('Resetting a node of type %s without _do_reset function!'
               % self.__class__.__name__)
        self.logerr(msg)
        raise NotImplementedError(msg)

    def shutdown(self):
        """Should be called before deleting a node.

        This method just calls :meth:`_do_shutdown`, which any
        subclass must override.

        This gives the node a chance to clean up any resources it might
        be holding before getting deleted.
        """
        if self.state != NodeMsg.SHUTDOWN:
            self._do_shutdown()
            self.state = NodeMsg.SHUTDOWN
        else:
            self.logwarn('Shutdown called twice')

        unshutdown_children = ['%s (%s), state: %s' % (child.name,
                                                       type(child).__name__,
                                                       child.state)
                               for child in self.children
                               if child.state != NodeMsg.SHUTDOWN]
        if unshutdown_children:
            self.logwarn('Not all children are shut down after calling shutdown(). '
                         'List of not-shutdown children and states:\n' +
                         '\n'.join(unshutdown_children))

    def _do_shutdown(self):
        """This is called before destroying the node.

        Implement this in your node class and release any resources you
        might be holding (file pointers, ROS topic subscriptions etc.)
        """
        msg = ('Shutting down a node of type %s without _do_shutdown function!'
               % self.__class__.__name__)
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

        :raises: BehaviorTreeException, KeyError

        `BehaviorTreeException` if the number of children after the add operation
        would exceed the maximum number of children, `KeyError` if a
        child of the same name already exists
        """
        if (self.node_config.max_children is not None and
                len(self.children) == self.node_config.max_children):
            error_msg = ('Trying to add child when maximum number of '
                         'children (%d) is already present'
                         % self.node_config.max_children)
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

        if child.name in (child.name for child in self.children):
            raise KeyError('Already have a child with name "%s"' % child.name)
        if at_index is None:
            at_index = len(self.children)
        # Use array slicing to efficiently insert child at the correct position
        # (the value we assign needs to be a list for this to work)
        self.children[at_index:at_index] = [child]
        child.parent_name = self.name

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
        :raises: KeyError if no child with that name exists
        """
        try:
            child_index = [child.name for child in self.children].index(child_name)
        except ValueError:
            raise KeyError('Node %s has no child named "%s"' % (self.name, child_name))
        tmp = self.children[child_index]
        del self.children[child_index]
        return tmp

    def _register_node_data(self, source_map, target_map, allow_ref):
        """Register a number of typed :class:`NodeData` in the given map

        :param dict(str, type) source_map: a dictionary mapping from data keys to data types,
        i.e. ``{ 'a_string' : str, 'an_int' : int }``

        :param NodeDataMap target_map:
        The :class:`NodeDataMap` to add :class:`NodeData` values to

        :param bool allow_ref:
        Decides whether :class:`OptionRef` is accepted as a type. If True, the
        option keys referenced by any :class:`OptionRef` objects must
        exist and be populated!

        :raises: NodeConfigError in any of the following cases:
          * If any of the keys in `source_map` already exist in `target_map`
          * If an OptionRef value is passed, but `allow_ref` is `False`
          * If an OptionRef references an option value that has not been set or
            does not exist
          * If an OptionRef references an option value that does not hold a `type`
        """
        for key, data_type in source_map.iteritems():
            if key in target_map:
                raise NodeConfigError('Duplicate output name: %s' % key)

            if isinstance(data_type, OptionRef):
                if not allow_ref:
                    raise NodeConfigError('OptionRef not allowed while adding to %s' %
                                          target_map.name)
                if data_type.option_key not in self.options:
                    raise NodeConfigError('OptionRef for %s key "%s" references invalid '
                                          'option key "%s"' %
                                          (target_map.name, key, data_type.option_key))
                if not self.options.is_updated(data_type.option_key):
                    raise NodeConfigError('OptionRef for %s key "%s" references unwritten '
                                          'option key "%s"' %
                                          (target_map.name, key, data_type.option_key))
                if not isinstance(self.options[data_type.option_key], type):
                    raise NodeConfigError('OptionRef for %s key "%s" references option key '
                                          '"%s" that does not contain a type!' %
                                          (target_map.name, key, data_type.option_key))
                target_map.add(key, NodeData(data_type=self.options[data_type.option_key]))
            else:
                target_map.add(key, NodeData(data_type=data_type))

    def __repr__(self):
        return '%s(options=%r), parent_name:%r, state:%r, inputs:%r, outputs:%r, children:%r' % (
            type(self).__name__,
            {key: self.options[key] for key in self.options},
            self.parent_name,
            self.state,
            self.inputs,
            self.outputs,
            self.children)

    def __eq__(self, other):
        return (self.name == other.name and
                self.parent_name == other.parent_name and
                self.state == other.state and
                type(self).__module__ == type(other).__module__ and
                type(self).__name__ == type(other).__name__ and
                self.options == other.options and
                self.inputs == other.inputs and
                self.outputs == other.outputs and
                self.children == other.children)

    def __ne__(self, other):
        return not self == other

    def get_data_map(self, data_kind):
        """Return one of our NodeDataMaps by string name

        :param basestring data_kind:
          One of the constants in :class:`ros_bt_py_msgs.msg.NodeDataLocation`

        :rtype: NodeDataMap
        """
        if data_kind == NodeDataLocation.INPUT_DATA:
            return self.inputs
        if data_kind == NodeDataLocation.OUTPUT_DATA:
            return self.outputs
        if data_kind == NodeDataLocation.OPTION_DATA:
            return self.options

        raise KeyError('%s is not a valid value to pass to Node.get_data_map()!' % data_kind)

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

        :raises:

        BehaviorTreeException if
        node cannot be instantiated.
        """
        if (msg.module not in cls.node_classes or
                msg.node_class not in cls.node_classes[msg.module]):
            # If the node class was not available, try to load it
            load_node_module(msg.module)

        # If loading didn't work, abort
        if (msg.module not in cls.node_classes or
                msg.node_class not in cls.node_classes[msg.module]):
            raise BehaviorTreeException(
                'Failed to instantiate node from message - node class '
                'not available. Original message:\n%s' % str(msg))

        node_class = cls.node_classes[msg.module][msg.node_class]

        # Populate options dict
        options_dict = {}
        try:
            for option in msg.options:
                options_dict[option.key] = jsonpickle.decode(option.serialized_value)
        except ValueError, e:
            raise BehaviorTreeException('Failed to instantiate node from message: %s' %
                                        str(e))

        # Instantiate node - this shouldn't do anything yet, since we don't
        # call setup()
        node_instance = node_class(options=options_dict)

        # Set name from ROS message
        if msg.name:
            node_instance.name = msg.name

        # Set inputs
        try:
            for input_msg in msg.inputs:
                node_instance.inputs[input_msg.key] = jsonpickle.decode(
                    input_msg.serialized_value)
        except ValueError, e:
            raise BehaviorTreeException('Failed to instantiate node from message: %s' %
                                        str(e))

        # Set outputs
        try:
            for output_msg in msg.outputs:
                node_instance.outputs[output_msg.key] = jsonpickle.decode(
                    output_msg.serialized_value)
        except ValueError, e:
            raise BehaviorTreeException('Failed to instantiate node from message: %s' %
                                        str(e))

        return node_instance

    def to_msg(self):
        """Populate a ROS message with the information from this Node

        Round-tripping the result through :meth:`Node.from_msg` should
        yield a working node object, with the caveat that state will not
        be preserved.

        :rtype: ros_bt_py_msgs.msg.Node
        :returns:

        A ROS message that describes the node.
        """
        node_type = type(self)
        return NodeMsg(is_subtree=False,
                       module=node_type.__module__,
                       node_class=node_type.__name__,
                       name=self.name,
                       child_names=[child.name for child in self.children],
                       options=[NodeDataMsg(key=key,
                                            serialized_value=jsonpickle.encode(
                                                self.options[key]))
                                for key in self.options],
                       inputs=[NodeDataMsg(key=key,
                                           serialized_value=jsonpickle.encode(
                                               self.inputs[key]))
                               for key in self.inputs],
                       outputs=[NodeDataMsg(key=key,
                                            serialized_value=jsonpickle.encode(
                                                self.outputs[key]))
                                for key in self.outputs],
                       state=self.state)


def load_node_module(package_name):
    """Import the named module at run-time.

    If the module contains any (properly decorated) node classes,
    they will be registered and available to load via the other
    commands in this class.
    """
    try:
        return importlib.import_module(package_name)
    except (ImportError, ValueError):
        return None


def increment_name(name):
    """If `name` does not already end in a number, add "_2" to it.

    Otherwise, increase the number after the underscore.
    """
    match = re.search('_([0-9]+)$', name)
    prev_number = 1
    if match:
        prev_number = int(match.group(1))
        # remove the entire _$number part from the name
        name = name[:len(name) - len(match.group(0))]

    name += '_%d' % (prev_number + 1)
    return name


def is_valid_node_message(msg, node_dict=None):
    """Check whether a message contains a valid message definition.

    Tries to instantiate the node using :meth:`Node.from_msg` and
    returns a tuple `(success, error_message)`.

    :param ros_bt_py_msgs.msg.Node node: The message to check

    :param dict node_dict:

    If given, check missing parent errors against this dictionary. If
    not given, those errors are ignored since of course there's no
    parent.
    """
    try:
        Node.from_msg(msg, node_dict)
        return (True, '')
    except KeyError, e:
        if node_dict is None:
            return (True, '')
        else:
            return (False, str(e))
    except BehaviorTreeException, e:
        return (False, str(e))


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=1))
class Decorator(Node):
    """Base class for Decorator nodes.

    Decorators have exactly one child and somehow modify that child's
    output. Subclasses can add inputs, outputs and options, but never
    change `max_children`.
    """
    pass


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=0))
class Leaf(Node):
    """Base class for leaf nodes in the tree.

    Leaf nodes have no children. Subclasses can define inputs, outputs
    and options, but never change `max_children`.
    """
    pass


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=None))
class FlowControl(Node):
    """Base class for flow control nodes

    Flow control nodes (mostly Sequence, Fallback and their derivatives)
    can have an unlimited number of children and each have a unique set
    of rules for when to tick which of their children.
    """
    pass
