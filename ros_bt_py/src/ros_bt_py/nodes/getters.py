from threading import Lock
import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={'list_type': type,
             'index': int,
             'succeed_on_stale_data': bool},
    inputs={'list': list},
    outputs={'item': OptionRef('list_type')},
    max_children=1))
class GetListItem(Decorator):
    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        # Tick child (if any) so it can produce its output before we process it
        for child in self.children:
            child.tick()

        if self.inputs.is_updated('list'):
            try:
                self.outputs['item'] = self.inputs['list'][self.options['index']]
                return NodeMsg.SUCCEEDED
            except IndexError:
                self.logerr('List index %d out of bound for list %s'
                            % (self.options['index'], self.inputs['list']))
                return NodeMsg.FAILED
        else:
            if self.outputs['item'] is None:
                return NodeMsg.RUNNING
            if self.options['succeed_on_stale_data']:
                return NodeMsg.SUCCEEDED
            else:
                self.logerr('No new data since last tick!')
                return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['item'] = None
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
