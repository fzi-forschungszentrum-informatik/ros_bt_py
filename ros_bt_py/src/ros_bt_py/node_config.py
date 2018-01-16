class OptionRef(object):
    """Marks an input or output type as dependent on an option value.

    Can be used instead of an actual type in the maps passed to a
    :class:Nodefonfig
    """
    def __init__(self, option_key):
        self.option_key = option_key


class NodeConfig(object):
    def __init__(self, options, inputs, outputs, max_children):
        """Describes the interface of a :class:ros_bt_py.node.Node

        :param dict(str, type) options

        Map from option names to their types. Note that unlike `inputs`
        and `outputs`, option types can **not** use :class:OptionRef !

        :param dict(str, type) inputs:

        Map from input names to their types, or an :class:OptionRef
        object that points to the option key to take the type from.

        :param dict(str, type) outputs:

        Map from output names to their types, or an :class:OptionRef
        object that points to the option key to take the type from.

        :type max_children: int or None
        :param max_children:

        The maximum number of children this node type is allowed to
        have.  If this is None, the node type supports any amount of
        children.
        """
        self.inputs = inputs
        self.outputs = outputs
        self.options = options
        self.max_children = max_children

    def extend(self, other):
        """Extend the input, output and option dicts with values from `other`

        :raises: KeyError, ValueError
        If any of the dicts in `other` contains keys that already exist,
        raise `KeyError`. If `max_children` has a value different from ours,
        raise `ValueError`.
        """
        if self.max_children != other.max_children:
            raise ValueError('Mismatch in max_children: %s vs %s'
                             % (self.max_children, other.max_children))
        duplicate_inputs = []
        for key in other.inputs:
            if key in self.inputs:
                duplicate_inputs.append(key)
                continue
            self.inputs[key] = other.inputs[key]
        duplicate_outputs = []
        for key in other.outputs:
            if key in self.outputs:
                duplicate_outputs.append(key)
                continue
            self.outputs[key] = other.outputs[key]
        duplicate_options = []
        for key in other.options:
            if key in self.options:
                duplicate_options.append(key)
                continue
            self.options[key] = other.options[key]

        if duplicate_inputs or duplicate_outputs or duplicate_options:
            msg = 'Duplicate keys: '
            keys_strings = []
            if duplicate_inputs:
                keys_strings.append('inputs: %s' % str(duplicate_inputs))
            if duplicate_outputs:
                keys_strings.append('outputs: %s' % str(duplicate_outputs))
            if duplicate_options:
                keys_strings.append('options: %s' % str(duplicate_options))
            msg += ', '.join(keys_strings)
            raise KeyError(msg)