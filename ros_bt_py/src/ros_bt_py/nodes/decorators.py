import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=1))
class IgnoreFailure(Decorator):
    """Return SUCCEEDED regardless of whether the child actually succeeded

    RUNNING is forwarded

    """
    def _do_setup(self):
        for child in self.children:
            child.setup()

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result == NodeMsg.FAILED:
                return NodeMsg.SUCCEEDED
            return result

        # Succeed if we have no children
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return NodeMsg.IDLE

    def _do_untick(self):
        for child in self.children:
            child.untick()

    # Decorator's default utility calculation works here
    #
    # def _do_calculate_utility(self):
    #     pass


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=1))
class UntilSuccess(Decorator):
    """Return RUNNING until the child node returns SUCCEEDED

    This means the child node will be retried until it succeeds or the
    tree moves on to a different branch. Failure means a restart and
    will be translated into RUNNING!

    """
    def _do_setup(self):
        for child in self.children:
            child.setup()

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result == NodeMsg.FAILED:
                return NodeMsg.RUNNING
            return result

        # Succeed if we have no children
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return NodeMsg.IDLE

    def _do_untick(self):
        for child in self.children:
            child.untick()

    # Decorator's default utility calculation works here
    #
    # def _do_calculate_utility(self):
    #     pass


@define_bt_node(NodeConfig(
    options={'num_retries': int},
    inputs={},
    outputs={},
    max_children=1))
class Retry(Decorator):
    """Retry the child `num_retries` times

    Retry, here, means ignoring a FAILED result from the child,
    reporting it as RUNNING instead and resetting it.

    """
    def _do_setup(self):
        self._retry_count = 0
        for child in self.children:
            child.setup()

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result == NodeMsg.FAILED:
                if self._retry_count < self.options['num_retries']:
                    self._retry_count += 1
                    child.reset()
                    return NodeMsg.RUNNING
                else:
                    self._retry_count = 0
                    return NodeMsg.FAILED
            return result

        # Succeed if we have no children
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()

    def _do_reset(self):
        self._retry_count = 0
        for child in self.children:
            return child.reset()
        return NodeMsg.IDLE

    def _do_untick(self):
        for child in self.children:
            child.untick()

    # Decorator's default utility calculation works here
    #
    # def _do_calculate_utility(self):
    #     pass
