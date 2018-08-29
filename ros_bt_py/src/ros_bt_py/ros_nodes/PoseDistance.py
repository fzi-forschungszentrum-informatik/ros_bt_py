import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={},
    inputs={'list': list},
    outputs={'item': OptionRef('list_type')},
    max_children=1))
class GetListItem(Decorator):
    """Extracts the item at the given `index` from `list`

    The option parameter `succeed_on_stale_data` determines whether
    the node returns SUCCEEDED or RUNNING if `list` hasn't been
    updated since the last tick.

    """
    def _do_setup(self):
        for child in self.children:
            child.setup()
            # We have a child, so set list to an empty list. We're avoiding an
            # error this way because we know what we're doing, don't use this
            # gratuitously!
            self.inputs['list'] = []
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
            if self.options['succeed_on_stale_data']:
                return NodeMsg.SUCCEEDED
            else:
                self.loginfo('No new data since last tick!')
                return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs['item'] = None
        self.outputs.reset_updated()
        self.inputs['list'] = None
        self._do_setup()
        self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
