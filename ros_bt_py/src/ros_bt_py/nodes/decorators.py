import rospy

from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

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
            return child.untick()
        return NodeMsg.IDLE

    # Decorator's default utility calculation works here
    #
    # def _do_calculate_utility(self):
    #     pass


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=1))
class IgnoreSuccess(Decorator):
    """Return FAILURE regardless of whether the child actually failed

    RUNNING is forwarded

    """
    def _do_setup(self):
        for child in self.children:
            child.setup()

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result == NodeMsg.SUCCEEDED:
                return NodeMsg.FAILED
            return result

        # Fails if we have no children
        return NodeMsg.FAILED

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()

    def _do_reset(self):
        for child in self.children:
            return child.reset()
        return NodeMsg.IDLE

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return NodeMsg.IDLE


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
            return child.untick()
        return NodeMsg.IDLE

    # Decorator's default utility calculation works here
    #
    # def _do_calculate_utility(self):
    #     pass


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=1))
class Inverter(Decorator):
    """Inverts the result of the child.
    Return SUCCEEDED if the child returned FAILED,
    return FAILED if the child returned SUCCEEDED.

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
            elif result == NodeMsg.SUCCEEDED:
                return NodeMsg.FAILED
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
            return child.untick()
        return NodeMsg.IDLE

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
            return child.untick()
        return NodeMsg.IDLE

    # Decorator's default utility calculation works here
    #
    # def _do_calculate_utility(self):
    #     pass


@define_bt_node(NodeConfig(
    options={'num_repeats': int},
    inputs={},
    outputs={},
    max_children=1))
class Repeat(Decorator):
    """Repeat the child `num_repeat` times

    Repeat, here, means counting the number of times the child SUCCEEDED,
    if the number of repeats is not yet reached, the child will be resetted.
    Returns RUNNING while the number of repeats is not yet reached,
    returns FAILED when the child fails.

    """
    def _do_setup(self):
        self._repeat_count = 0
        for child in self.children:
            child.setup()

    def _do_tick(self):
        for child in self.children:
            result = child.tick()
            if result == NodeMsg.FAILED:
                return NodeMsg.FAILED
            elif result == NodeMsg.SUCCEEDED:
                if self._repeat_count < self.options['num_repeats']:
                    self._repeat_count += 1
                    child.reset()
                    return NodeMsg.RUNNING
                else:
                    return NodeMsg.SUCCEEDED
            return result

        # Succeed if we have no children
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        for child in self.children:
            return child.shutdown()

    def _do_reset(self):
        self._repeat_count = 0
        for child in self.children:
            return child.reset()
        return NodeMsg.IDLE

    def _do_untick(self):
        for child in self.children:
            return child.untick()
        return NodeMsg.IDLE

    # Decorator's default utility calculation works here
    #
    # def _do_calculate_utility(self):
    #     pass


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=1))
class Optional(Decorator):
    """Wraps a child that may not be able to execute

    A child that cannot execute will not be ticked. Instead, this
    decorator will always return SUCCEEDED.

    If the child *can* execute, the decorator will simply forward all
    events.

    """
    def _do_setup(self):
        self.execute_child = False
        for child in self.children:
            if child.calculate_utility().can_execute:
                child.setup()
                self.execute_child = True

    def _do_tick(self):
        if self.execute_child:
            return self.children[0].tick()
        else:
            return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        if self.execute_child:
            self.children[0].shutdown()

    def _do_reset(self):
        if self.execute_child:
            self.children[0].reset()
        else:
            return NodeMsg.IDLE

    def _do_untick(self):
        if self.execute_child:
            self.children[0].untick()
        else:
            return NodeMsg.IDLE

    def _do_calculate_utility(self):
        for child in self.children:
            bounds = child.calculate_utility()
            if bounds.can_execute:
                return bounds

        # If the child can't execute, return a UtilityBounds object
        # that can execute, but does not have any bounds set (that is,
        # if another executor can actually execute our child, it is
        # pretty much guaranteed to have a better Utility score than
        # us)
        return UtilityBounds(can_execute=True)
