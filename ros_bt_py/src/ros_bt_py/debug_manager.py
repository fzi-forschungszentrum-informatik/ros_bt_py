from contextlib import contextmanager
import inspect
import math
from sys import getrecursionlimit
from threading import Event
import time

import rospy

from ros_bt_py_msgs.msg import DebugInfo, Node, TickTime, TreeLocation


class DebugManager(object):
    def __init__(self, target_tick_frequency_hz=20.0, window_size=None, tree_name=None):
        # TODO(nberg): Ensure this is set at least once on shutdown
        self.continue_event = Event()
        self.debug_info_msg = DebugInfo()
        self.debug_info_msg.tree_name = tree_name if tree_name else ''

        self.debug_info_msg.target_tick_time = rospy.Duration.from_sec(
            1.0 / target_tick_frequency_hz)

        if window_size is None:
            # Record samples for around one second by default (more if
            # tick_frequency < 1.0)
            window_size = int(math.ceil(target_tick_frequency_hz))

        # TODO(nberg): Check performance, maybe hold dict of TickTime objects
        # to reduce time spent searching
        self.debug_info_msg.window_size = window_size
        # List of node names to break on
        self.breakpoints = []
        # lists of tick times, by node name
        self.tick_time_windows = {}
        # Only collect analytics if this is True
        self.debug = False
        # if True, wait for confirmation on every report call
        self.stepping = False

    def continue_debug(self):
        self.continue_event.set()

    @contextmanager
    def report_tick(self, node_instance):
        """A context manager that collects debug data from Node execution.

        It measures the time between the beginning and the end of the
        tick function (which includes the ticks of any children) and
        calculates a moving window average of execution times as well as
        a minimum and maximum value.

        Additionally, it provides pause functionality to enable stepping
        through a tree and adding break points.

        :param instance: The node that's executing
        """
        if (self.breakpoints and node_instance.name in self.breakpoints) or self.stepping:
            old_state = node_instance.state
            node_instance.state = Node.DEBUG_PRE_TICK
            self.wait_for_continue()
            node_instance.state = old_state
        if self.debug:
            start_time = time.clock()
            self.debug_info_msg.current_recursion_depth = len(inspect.stack())
            self.debug_info_msg.max_recursion_depth = getrecursionlimit()

        # Contextmanager'ed code is executed here
        yield

        if self.debug:
            end_time = time.clock()
            if node_instance.name not in self.tick_time_windows:
                self.tick_time_windows[node_instance.name] = []
            window = self.tick_time_windows[node_instance.name]
            current_window_size = len(window)
            if current_window_size == self.debug_info_msg.window_size:
                window = window[1:]
            window.append(end_time - start_time)

            average_duration = rospy.Duration.from_sec(
                sum(window) / (1.0 * len(window)))
            last_duration = rospy.Duration.from_sec(window[-1])

            # Find the TickTime message for the current node in debug_info_msg
            ticktime_msg_list = [x for x in self.debug_info_msg.node_tick_times
                                 if x.node_name == node_instance.name]
            # If there is one, take that
            if ticktime_msg_list:
                timing_msg = ticktime_msg_list[0]
            # If not, create one and add it to the list
            else:
                timing_msg = TickTime(node_name=node_instance.name)
                self.debug_info_msg.node_tick_times.append(timing_msg)

            # Either way, timing_msg is now a reference to the correct item
            # in debug_info_msg
            timing_msg.avg_tick_time = average_duration
            if ((timing_msg.min_tick_time.secs == 0 and timing_msg.min_tick_time.nsecs == 0) or
                    duration_greater(timing_msg.min_tick_time, last_duration)):
                timing_msg.min_tick_time = last_duration

            if duration_greater(last_duration, timing_msg.max_tick_time):
                timing_msg.max_tick_time = last_duration

            self.debug_info_msg.current_recursion_depth = len(inspect.stack())
            self.debug_info_msg.max_recursion_depth = getrecursionlimit()

        if (self.breakpoints and node_instance.name in self.breakpoints) or self.stepping:
            old_state = node_instance.state
            node_instance.state = Node.DEBUG_PRE_TICK
            self.wait_for_continue()
            node_instance.state = old_state
            # TODO(nberg): Really autoremove all breakpoints?
            if node_instance.name in self.breakpoints:
                self.breakpoints.remove(node_instance.name)

    def wait_for_continue(self):
        # Ensure that we're not picking up an extra continue request sent earlier
        self.continue_event.clear()
        self.continue_event.wait()
        self.continue_event.clear()


def duration_greater(first, second):
    """Helper to compare rospy.Duration to genpy.Duration"""
    return first.secs >= second.secs and first.nsecs > second.nsecs
