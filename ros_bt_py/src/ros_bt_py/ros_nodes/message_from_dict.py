from rosbridge_library.internal.message_conversion import populate_instance
from rosbridge_library.internal.message_conversion import InvalidMessageException,\
    NonexistentFieldException, FieldTypeMismatchException
from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'message_type': type},
    inputs={'dict': dict},
    outputs={'message': OptionRef('message_type')},
    max_children=0))
class MessageFromDict(Leaf):
    """Fill a ROS message with the values from `dict`"""
    def _do_setup(self):
        pass

    def _do_tick(self):
        if self.inputs.is_updated('dict'):
            message = self.options['message_type']()
            try:
                populate_instance(self.inputs['dict'], message)
                self.outputs['message'] = message
            except (InvalidMessageException,
                    NonexistentFieldException,
                    FieldTypeMismatchException) as ex:
                self.logerr('Error populating message of type %s: %s' %
                            (self.options['message_type'].__name__, str(ex)))
                return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version='0.9.0',
    options={'message_type': type,
             'dict': dict},
    inputs={},
    outputs={'message': OptionRef('message_type')},
    max_children=0,
    option_wirings=[{'source': 'message_type', 'target': 'dict'}]))
class MessageFromConstDict(Leaf):
    """Fill a ROS message with the values from `dict`"""
    def _do_setup(self):
        pass

    def _do_tick(self):
        message = self.options['message_type']()
        try:
            populate_instance(self.options['dict'], message)
            self.outputs['message'] = message
        except (InvalidMessageException,
                NonexistentFieldException,
                FieldTypeMismatchException) as ex:
            self.logerr('Error populating message of type %s: %s' %
                        (self.options['message_type'].__name__, str(ex)))
            return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
