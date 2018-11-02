import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import FlowControl, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

from ros_bt_py.nodes.shovable import Shovable

@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=None))
class ParallelIfRemote(FlowControl):
    """Act like a Sequence if children are run locally, like a Parallel if remotely

    This can be used to model tasks that a single robot must execute
    sequentially, but a group of robots can do in a distributed
    fashion.

    """
    def _do_setup(self):
        for child in self.children:
            child.setup()

    def _do_tick(self):
        # This ensures a tick with no children will succeed
        child_state = NodeMsg.SUCCEEDED

        remote_running = False
        for child in self.children:
            child_state = child.tick()
            # If the child succeeded, tick the next
            if child_state == NodeMsg.SUCCEEDED:
                continue
            # If the child is running, but is a Shovable and executing
            # remotely, also tick the next child
            elif child_state == NodeMsg.RUNNING:
                if isinstance(child, Shovable):
                    if child.outputs['running_remotely']:
                        remote_running = True
                        continue
            # In other cases, stop ticking children and return
            break

        if remote_running:
            return NodeMsg.RUNNING
        return child_state

    def _do_shutdown(self):
        for child in self.children:
            child.shutdown()

    def _do_reset(self):
        for child in self.children:
            child.reset()

        return NodeMsg.IDLE

    def _do_untick(self):
        for child in self.children:
            child.untick()

        return NodeMsg.IDLE

    # Uncomment this if your node provides a utility calculation
    #
    # def _do_calculate_utility(self):
    #     pass

