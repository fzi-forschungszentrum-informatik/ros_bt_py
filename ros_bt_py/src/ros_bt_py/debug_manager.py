from copy import deepcopy
from contextlib import contextmanager
import inspect
import math
from sys import getrecursionlimit
from threading import Event, Lock
import time

import rospy

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py_msgs.msg import DebugInfo, DebugSettings, Node, TickTime


class DebugManager(object):
    def __init__(self,
                 target_tick_frequency_hz=20.0,
                 window_size=None,
                 tree_name=None,
                 debug_info_publish_callback=None,
                 debug_settings_publish_callback=None):
        # TODO(nberg): Ensure this is set at least once on shutdown
        self.continue_event = Event()
        self._lock = Lock()

        if window_size is None:
            # Record samples for around one second by default (more if
            # tick_frequency < 1.0)
            window_size = int(math.ceil(target_tick_frequency_hz))

        self.publish_debug_info = debug_info_publish_callback
        self.publish_debug_settings = debug_settings_publish_callback

        self.subtrees = dict()

        # TODO(nberg): Check performance, maybe hold dict of TickTime objects
        # instead of just using the list in DebugInfo to reduce time spent
        # searching.
        with self._lock:
            self._debug_info_msg = DebugInfo()
            self._debug_info_msg.tree_name = tree_name if tree_name is not None else ''
            self._debug_info_msg.window_size = window_size

        # set_tick_frequency() uses _lock, so don't put it in the with block,
        # or we'll deadlock!
        self.set_tick_frequency(target_tick_frequency_hz)

        self._debug_settings_msg = DebugSettings(
            # List of node names to break on
            breakpoint_names=[],
            # Only collect analytics if this is True
            collect_performance_data=False,
            # Don't publish subtree states by default
            publish_subtrees=False,
            # if True, wait for a continue request before and after every tick
            single_step=False)

        # lists of tick times, by node name
        self.tick_time_windows = {}

    def set_tree_name(self, name):
        self._debug_info_msg.tree_name = name

    def set_tick_frequency(self, frequency_hz):
        with self._lock:
            self._debug_info_msg.target_tick_time = rospy.Duration.from_sec(
                1.0 / frequency_hz)

    def set_execution_mode(self, single_step, collect_performance_data, publish_subtrees):
        with self._lock:
            self._debug_settings_msg.single_step = single_step
            self._debug_settings_msg.collect_performance_data = collect_performance_data
            self._debug_settings_msg.publish_subtrees = publish_subtrees
        if self.publish_debug_settings:
            self.publish_debug_settings(self._debug_settings_msg)

    def modify_breakpoints(self, add=None, remove=None, remove_all=False):
        with self._lock:
            if remove_all:
                self._debug_settings_msg.breakpoint_names = []
            if remove:
                for bp in remove:
                    if bp in self._debug_settings_msg.breakpoint_names:
                        self._debug_settings_msg.breakpoint_names.remove(bp)
            if add:
                for bp in add:
                    if bp not in self._debug_settings_msg.breakpoint_names:
                        self._debug_settings_msg.breakpoint_names.append(bp)
            if self.publish_debug_settings:
                self.publish_debug_settings(self._debug_settings_msg)
            return self._debug_settings_msg.breakpoint_names

    def continue_debug(self):
        # TODO(nberg): Make sure event can't get set ahead of time if nobody is
        # waiting for it (should be fine, since wait_for_continue clears it
        # first)
        self.continue_event.set()

    def is_debugging(self):
        with self._lock:
            return (self._debug_settings_msg.breakpoint_names or
                    self._debug_settings_msg.single_step)

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
        if (self.is_debugging()):
            old_state = node_instance.state
            node_instance.state = Node.DEBUG_PRE_TICK
            self.wait_for_continue()
            node_instance.state = old_state
        if self._debug_settings_msg.collect_performance_data:
            start_time = time.time()
            with self._lock:
                self._debug_info_msg.current_recursion_depth = len(inspect.stack())
                self._debug_info_msg.max_recursion_depth = getrecursionlimit()

        # Contextmanager'ed code is executed here
        yield

        if self._debug_settings_msg.collect_performance_data:
            end_time = time.time()
            if node_instance.name not in self.tick_time_windows:
                self.tick_time_windows[node_instance.name] = []
            window = self.tick_time_windows[node_instance.name]
            current_window_size = len(window)
            with self._lock:
                if current_window_size == self._debug_info_msg.window_size:
                    window = window[1:]
            window.append(end_time - start_time)

            average_duration = rospy.Duration.from_sec(
                sum(window) / (1.0 * len(window)))
            last_duration = rospy.Duration.from_sec(window[-1])

            # Find the TickTime message for the current node in debug_info_msg
            with self._lock:
                ticktime_msg_list = [x for x in self._debug_info_msg.node_tick_times
                                     if x.node_name == node_instance.name]
            # If there is one, take that
            if ticktime_msg_list:
                timing_msg = ticktime_msg_list[0]
            # If not, create one and add it to the list
            else:
                timing_msg = TickTime(node_name=node_instance.name)
                with self._lock:
                    self._debug_info_msg.node_tick_times.append(timing_msg)

            # Either way, timing_msg is now a reference to the correct item
            # in debug_info_msg
            timing_msg.avg_tick_time = average_duration
            if ((timing_msg.min_tick_time.secs == 0 and timing_msg.min_tick_time.nsecs == 0) or
                    _duration_greater(timing_msg.min_tick_time, last_duration)):
                timing_msg.min_tick_time = last_duration

            if _duration_greater(last_duration, timing_msg.max_tick_time):
                timing_msg.max_tick_time = last_duration

            with self._lock:
                self._debug_info_msg.current_recursion_depth = len(inspect.stack())
                self._debug_info_msg.max_recursion_depth = getrecursionlimit()

        if (self.is_debugging()):
            self.wait_for_continue()
            old_state = node_instance.state
            node_instance.state = Node.DEBUG_POST_TICK
            self.wait_for_continue()
            node_instance.state = old_state
            # TODO(nberg): Really autoremove all breakpoints?
            if node_instance.name in self._debug_settings_msg.breakpoint_names:
                self._debug_settings_msg.breakpoint_names.remove(node_instance.name)
                if self.publish_debug_settings:
                    self.publish_debug_settings(self._debug_settings_msg)

    def wait_for_continue(self):
        # If we have a publish callback, publish debug info
        if self.publish_debug_info:
            self.publish_debug_info(self.get_debug_info_msg())
        # Ensure that we're not picking up an extra continue request sent earlier
        self.continue_event.clear()
        self.continue_event.wait()
        self.continue_event.clear()

    def get_debug_info_msg(self):
        with self._lock:
            return deepcopy(self._debug_info_msg)

    def add_subtree_info(self, node_name, subtree_msg):
        """Used by the :class:`ros_bt_py.nodes.Subtree` node to publish subtree states

        Serialization of subtrees (and calling this method) should
        only happen when the `publish_subtrees` option is set via
        `SetExecutionMode`.

        :param str node_name:

        The name of the subtree node. This will be prefixed to the
        subtree name to ensure it is unique.

        :raises: `ros_bt_py.exceptions.BehaviorTreeException`

        If this method is called when `publish_subtrees` is `False`.
        """
        subtree_name = '{}.{}'.format(node_name, subtree_msg.name)
        with self._lock:
            if not self._debug_settings_msg.publish_subtrees:
                raise BehaviorTreeException(
                    'Trying to add subtree info when subtree publishing is disabled!')
            self.subtrees[subtree_name] = subtree_msg
            self._debug_info_msg.subtree_states = [msg for msg in self.subtrees.iteritems()]

    def clear_subtrees(self):
        with self._lock:
            self.subtree_dict.clear()
            self._debug_info_msg.subtree_states = []

    def get_publish_subtrees(self):
        with self._lock:
            return self._debug_settings_msg.publish_subtrees


def _duration_greater(first, second):
    """Helper to compare rospy.Duration to genpy.Duration"""
    return first.secs >= second.secs and first.nsecs > second.nsecs
