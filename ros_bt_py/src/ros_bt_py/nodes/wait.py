import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    options={'seconds_to_wait': int},
    inputs={},
    outputs={},
    max_children=0))
class Wait(Leaf):
    """Returns "RUNNING" until at least the specified amount of seconds are elapsed
    from the first received tick.
    This is naturally not extremely precise because it depends on the tick interval
    """
    def _do_setup(self):
        self.first_tick = True
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.first_tick:
            self.start_time = rospy.get_time()
            self.end_time = self.start_time + float(self.options['seconds_to_wait'])
            self.first_tick = False
        if rospy.get_time() > self.end_time:
            return NodeMsg.SUCCESS
        else:
            return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.first_tick = True
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    # Uncomment this if your node provides a utility calculation
    #
    # def _do_calculate_utility(self):
    #     pass


@define_bt_node(NodeConfig(
    options={},
    inputs={'seconds_to_wait': int},
    outputs={},
    max_children=0))
class WaitInput(Leaf):
    """Returns "RUNNING" until at least the specified amount of seconds are elapsed
    from the first received tick.
    This is naturally not extremely precise because it depends on the tick interval
    """
    def _do_setup(self):
        self.first_tick = True
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.first_tick:
            self.start_time = rospy.get_time()
            self.end_time = self.start_time + float(self.inputs['seconds_to_wait'])
            self.first_tick = False
        if rospy.get_time() > self.end_time:
            return NodeMsg.SUCCESS
        else:
            return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.first_tick = True
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    # Uncomment this if your node provides a utility calculation
    #
    # def _do_calculate_utility(self):
    #     pass
