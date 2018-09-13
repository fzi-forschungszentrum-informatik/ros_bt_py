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

       In this case, the Sequence also returns FAILED, and calls
       :meth:`ros_bt_py.node.Node.untick` on all remaining children.

    2. A node returns RUNNING:

       Just as with FAILED, the Sequence will also return RUNNING, but
       not call :meth:`ros_bt_py.node.Node.untick` on the remaining
       children until the RUNNING node produces a result. This
       prevents thrashing when there's multiple nodes that take a tick
       or two to produce a result.

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

        # If we've previously succeeded or failed, reset all children
        # so we get a clean run
        if self.state in [NodeMsg.SUCCEEDED, NodeMsg.FAILED]:
            for child in self.children:
                child.reset()

        # Tick children until one returns FAILED or RUNNING
        result = NodeMsg.FAILED
        for index, child in enumerate(self.children):
            result = child.tick()
            if result != NodeMsg.SUCCEEDED:
                # For all states other than RUNNING...
                if result != NodeMsg.RUNNING:
                    # ...untick all children after the one that hasn't
                    # succeeded
                    for untick_child in self.children[index + 1:]:
                        untick_child.untick()
                break

        return result

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
        bounds = UtilityBounds(has_lower_bound_success=True,
                               lower_bound_success=0.0,
                               has_upper_bound_success=True,
                               upper_bound_success=0.0,
                               has_lower_bound_failure=False,
                               has_upper_bound_failure=False)
        if self.children:
            # These are the direct inverse of the Fallback's boundaries - check
            # the detailed description in fallback.py if you're interested in
            # why this is the solution.
            have_bounds = True
            failure_bounds = [UtilityBounds(has_lower_bound_failure=False,
                                            lower_bound_failure=0,
                                            has_upper_bound_failure=False,
                                            upper_bound_failure=0)
                              for _ in self.children]
            for index, child_bounds in enumerate((child.calculate_utility()
                                                  for child in self.children)):
                # We can only provide an estimate if all children have an estimate
                # TODO(nberg): Maybe relax this?
                have_bounds = (have_bounds and
                               child_bounds.has_lower_bound_success and
                               child_bounds.has_upper_bound_success and
                               child_bounds.has_lower_bound_failure and
                               child_bounds.has_upper_bound_failure)

                bounds.has_lower_bound_success &= child_bounds.has_lower_bound_success
                bounds.lower_bound_success += child_bounds.lower_bound_success
                bounds.has_upper_bound_success &= child_bounds.has_upper_bound_success
                bounds.upper_bound_success += child_bounds.upper_bound_success

                failure_bounds[index].lower_bound_failure += child_bounds.lower_bound_failure
                failure_bounds[index].upper_bound_failure += child_bounds.upper_bound_failure
                # Range returns an empty range if the first parameter is larger
                # than the second, so no bounds checking necessary
                for i in range(index+1, len(failure_bounds)):
                    failure_bounds[i].lower_bound_failure += child_bounds.lower_bound_success
                    failure_bounds[i].upper_bound_failure += child_bounds.upper_bound_success

            # Select the minimum and maximum values to get the final bounds
            bounds.lower_bound_failure = min((x.lower_bound_failure for x in failure_bounds))
            bounds.upper_bound_failure = max((x.upper_bound_failure for x in failure_bounds))

            # Check if we actually have bounds
            bounds.has_lower_bound_success = have_bounds
            bounds.has_upper_bound_success = have_bounds
            bounds.has_lower_bound_failure = have_bounds
            bounds.has_upper_bound_failure = have_bounds

        return bounds


@define_bt_node(NodeConfig(
    options={},
    inputs={},
    outputs={},
    max_children=None))
class MemorySequence(FlowControl):
    """Flow control node that succeeds when all children succeed and has a memory.

    At every call of :meth:`tick`, it ticks all of its children in
    order until one of three things happens:

    1. A node returns FAILED:

       In this case, the Sequence also returns FAILED, and calls
       :meth:`ros_bt_py.node.Node.untick` on all remaining children.

    2. A node returns RUNNING:

       Just as with FAILED, the Sequence will also return RUNNING, but
       not call :meth:`ros_bt_py.node.Node.untick` on the remaining
       children until the RUNNING node produces a result. This
       prevents thrashing when there's multiple nodes that take a tick
       or two to produce a result.

    3. All nodes return SUCCEEDED:

       The Sequence will also return SUCCEEDED.


    The *Memory* part of the node means that after a child returns
    RUNNING, the execution will start at that same child on the next
    tick. This means that changes in the previous nodes' outcomes will
    not influence the execution of later children until either a) this
    node receives an `untick()` or b) the sequence returns SUCCEEDED
    or FAILED as described above.

    *Special case:*

    If a Sequence has no children, its :meth:`tick` method will always
    return FAILED.

    """
    def _do_setup(self):
        self.last_running_child = 0
        for child in self.children:
            child.setup()

    def _do_tick(self):
        if not self.children:
            self.logwarn('Ticking without children. Is this really what you want?')
            return NodeMsg.FAILED

        # If we've previously succeeded or failed, reset all children
        # so we get a clean run
        if self.state in [NodeMsg.SUCCEEDED, NodeMsg.FAILED]:
            for child in self.children:
                child.reset()

        # Tick children until one returns FAILED or RUNNING
        for index, child in enumerate(self.children):
            if index < self.last_running_child:
                continue
            result = child.tick()

            if result != NodeMsg.SUCCEEDED:
                if result == NodeMsg.RUNNING:
                    self.last_running_child = index
                else:
                    # For all states other than RUNNING, untick all
                    # children after the one that hasn't succeeded
                    for untick_child in self.children[index + 1:]:
                        untick_child.untick()
                return result
        # If all children succeeded, we too succeed
        self.last_running_child = 0
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        for child in self.children:
            child.untick()
        self.last_running_child = 0
        return NodeMsg.IDLE

    def _do_reset(self):
        for child in self.children:
            child.reset()
        self.last_running_child = 0
        return NodeMsg.IDLE

    def _do_shutdown(self):
        for child in self.children:
            child.shutdown()
        self.last_running_child = 0

    def _do_calculate_utility(self):
        bounds = UtilityBounds(has_lower_bound_success=True,
                               lower_bound_success=0.0,
                               has_upper_bound_success=True,
                               upper_bound_success=0.0,
                               has_lower_bound_failure=False,
                               has_upper_bound_failure=False)
        if self.children:
            # These are the direct inverse of the Fallback's boundaries - check
            # the detailed description in fallback.py if you're interested in
            # why this is the solution.
            have_bounds = True
            failure_bounds = [UtilityBounds(has_lower_bound_failure=False,
                                            lower_bound_failure=0,
                                            has_upper_bound_failure=False,
                                            upper_bound_failure=0)
                              for _ in self.children]
            for index, child_bounds in enumerate((child.calculate_utility()
                                                  for child in self.children)):
                # We can only provide an estimate if all children have an estimate
                # TODO(nberg): Maybe relax this?
                have_bounds = (have_bounds and
                               child_bounds.has_lower_bound_success and
                               child_bounds.has_upper_bound_success and
                               child_bounds.has_lower_bound_failure and
                               child_bounds.has_upper_bound_failure)

                bounds.has_lower_bound_success &= child_bounds.has_lower_bound_success
                bounds.lower_bound_success += child_bounds.lower_bound_success
                bounds.has_upper_bound_success &= child_bounds.has_upper_bound_success
                bounds.upper_bound_success += child_bounds.upper_bound_success

                failure_bounds[index].lower_bound_failure += child_bounds.lower_bound_failure
                failure_bounds[index].upper_bound_failure += child_bounds.upper_bound_failure
                # Range returns an empty range if the first parameter is larger
                # than the second, so no bounds checking necessary
                for i in range(index+1, len(failure_bounds)):
                    failure_bounds[i].lower_bound_failure += child_bounds.lower_bound_success
                    failure_bounds[i].upper_bound_failure += child_bounds.upper_bound_success

            # Select the minimum and maximum values to get the final bounds
            bounds.lower_bound_failure = min((x.lower_bound_failure for x in failure_bounds))
            bounds.upper_bound_failure = max((x.upper_bound_failure for x in failure_bounds))

            # Check if we actually have bounds
            bounds.has_lower_bound_success = have_bounds
            bounds.has_upper_bound_success = have_bounds
            bounds.has_lower_bound_failure = have_bounds
            bounds.has_upper_bound_failure = have_bounds

        return bounds
