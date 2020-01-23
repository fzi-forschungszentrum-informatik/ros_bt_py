from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.ros_helpers import LoggerLevel


@define_bt_node(NodeConfig(
    version='1.0.0',
    options={'logger_level': LoggerLevel},
    inputs={'in': str},
    outputs={},
    max_children=0))
class Log(Leaf):
    """Logs the input string to the console with the provided logger_level

    """
    def _do_setup(self):
        logger_level = self.options['logger_level']
        self.log = self.loginfo

        if logger_level.logger_level == 'debug':
            self.log = self.logdebug
        elif logger_level.logger_level == 'info':
            self.log = self.loginfo
        elif logger_level.logger_level == 'warning':
            self.log = self.logwarn
        elif logger_level.logger_level == 'error':
            self.log = self.logerr
        elif logger_level.logger_level == 'fatal':
            self.log = self.logfatal

        return NodeMsg.IDLE

    def _do_tick(self):
        self.log(self.inputs['in'])
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE
