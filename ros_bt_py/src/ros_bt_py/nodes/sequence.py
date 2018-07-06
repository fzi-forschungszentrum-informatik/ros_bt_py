from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py_msgs.msg import UtilityBounds

from ros_bt_py.node import FlowControl, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=None))
class Sequence(FlowControl):
    """Flow control node that succeeds when all children succeed.

    At every call of :meth:`tick`, it ticks all of its children in
    order until one of three things happens:

    1. A node returns FAILED:

       In this case, the sequence also returns FAILED, and calls
       :meth:`ros_bt_py.node.Node.untick` on all remaining children.

    2. A node returns RUNNING:

       Just as with FAILED, the sequence will also return RUNNING and call
       :meth:`ros_bt_py.node.Node.untick` on all remaining children.

    3. All nodes return SUCCEEDED:

       The Sequence will also return SUCCEEDED.

    *Special case:*

    If a Sequence has no children, its :meth:`tick` method will always
    return FAILED.
    """
    def _do_setup(self):
        for child in self.children:
            child.setup()

    def _do_tick(self):
        if not self.children:
            self.logwarn('Ticking without children. Is this really what you want?')
            return NodeMsg.FAILED
        # Tick children until one returns FAILED or RUNNING
        for index, child in enumerate(self.children):
            result = child.tick()
            if result != NodeMsg.SUCCEEDED:
                # untick all children after the one that hasn't succeeded
                for untick_child in self.children[index + 1:]:
                    untick_child.untick()
                return result
        # If all children succeeded, we too succeed
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        for child in self.children:
            child.untick()
        return NodeMsg.IDLE

    def _do_reset(self):
        for child in self.children:
            child.reset()
        return NodeMsg.IDLE

    def _do_shutdown(self):
        for child in self.children:
            child.shutdown()

    def _do_calculate_utility(self):
        bounds = UtilityBounds(has_lower_bound_success=False,
                               has_upper_bound_success=False,
                               has_lower_bound_failure=False,
                               has_upper_bound_failure=False)
        if self.children:
            # These are the direct inverse of the Fallback's boundaries - check
            # the detailed description in fallback.py if you're interested in
            # why this is the solution.
            have_bounds = True
            success_bounds = [UtilityBounds(has_lower_bound_success=False,
                                           lower_bound_success=0,
                                           has_upper_bound_success=False,
                                           upper_bound_success=0)
                                  for _ in range(len(self.children))]
            for index, child_bounds in enumerate((child.calculate_utility()
                                                  for child in self.children)):
                # We can only provide an estimate if all children have an estimate
                # TODO(nberg): Maybe relax this?
                have_bounds = (have_bounds
                                   and child_bounds.has_lower_bound_success
                                   and child_bounds.has_upper_bound_success
                                   and child_bounds.has_lower_bound_failure
                                   and child_bounds.has_upper_bound_failure)
                if index == 0:
                    bounds.has_lower_bound_success = child_bounds.has_lower_bound_success
                    bounds.lower_bound_success = child_bounds.lower_bound_success
                    bounds.has_upper_bound_success = child_bounds.has_upper_bound_success
                    bounds.upper_bound_success = child_bounds.upper_bound_success

                failure_bounds[index].lower_bound_failure += child_bounds.lower_bound_failure
                failure_bounds[index].upper_bound_failure += child_bounds.upper_bound_failure
                # Range returns an empty range if the first parameter is larger
                # than the second, so no bounds checking necessary
                for i in range(index+1, len(failure_bounds)):
                    failure_bounds[i].lower_bound_failure += child_bounds.lower_bound_success
                    failure_bounds[i].upper_bound_failure += child_bounds.upper_bound_success

            # Select the minimum and maximum values to get the final bounds
            bounds.lower_bound_failure = min((x.lower_bound_failure for x in failure_bounds))
            bounds.upper_bound_failure = min((x.upper_bound_failure for x in failure_bounds))

            # Check if we actually have bounds
            bounds.has_lower_bound_success = have_bounds
            bounds.has_upper_bound_success = have_bounds
            bounds.has_lower_bound_failure = have_bounds
            bounds.has_upper_bound_failure = have_bounds

        return bounds
