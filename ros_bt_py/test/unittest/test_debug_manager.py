import inspect
import sys
from threading import Thread
import time
import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.nodes.passthrough_node import PassthroughNode


class TestDebugManager(unittest.TestCase):
    def setUp(self):
        self.manager = DebugManager(target_tick_frequency_hz=20.0)

    def testInit(self):
        self.assertEqual(self.manager.debug_info_msg.target_tick_time.secs, 0)
        # Should be 0.05 seconds -> 5 * 10^7 nanoseconds
        self.assertEqual(self.manager.debug_info_msg.target_tick_time.nsecs, 5e7)
        self.assertEqual(self.manager.debug_info_msg.window_size, 20)

    def testReport(self):
        self.manager = DebugManager(target_tick_frequency_hz=20.0)
        self.manager.debug = True

        node = PassthroughNode({'passthrough_type': int})
        node.setup()
        node.name = 'foo'

        starting_recursion_depth = len(inspect.stack())
        with self.manager.report_tick(node):
            time.sleep(0.01)

        self.assertEqual(self.manager.debug_info_msg.max_recursion_depth,
                         sys.getrecursionlimit())
        # Plus one for the context self.manager, and another one for the contextlib decorator
        self.assertEqual(self.manager.debug_info_msg.current_recursion_depth,
                         starting_recursion_depth + 2)
        self.assertEqual(self.manager.debug_info_msg.node_tick_times[0].node_name,
                         'foo')
        self.assertEqual(self.manager.debug_info_msg.node_tick_times[0].min_tick_time.nsecs,
                         self.manager.debug_info_msg.node_tick_times[0].max_tick_time.nsecs)

        with self.manager.report_tick(node):
            time.sleep(0.05)
        self.assertGreater(self.manager.debug_info_msg.node_tick_times[0].max_tick_time.nsecs,
                           self.manager.debug_info_msg.node_tick_times[0].min_tick_time.nsecs)

    def testStep(self):
        self.manager.stepping = True

        self.node = PassthroughNode({'passthrough_type': int},
                                    debug_manager=self.manager)
        self.node.setup()
        self.node.name = 'foo'
        self.node.inputs['in'] = 1

        def do_stuff():
            self.node.tick()

        test_thread = Thread(target=do_stuff)
        test_thread.start()
        time.sleep(0.05)

        # Thread should be blocked on first continue -> the output does not
        # have a value yet.
        self.assertEqual(self.node.outputs['out'], None)
        self.assertEqual(self.node.state, NodeMsg.DEBUG_PRE_TICK)
        self.assertTrue(test_thread.isAlive())
        self.manager.continue_debug()
        time.sleep(0.05)

        # Now out should be changed
        self.assertEqual(self.node.outputs['out'], 1)
        self.assertEqual(self.node.state, NodeMsg.DEBUG_POST_TICK)
        # Should still be waiting for second continue -> join won't work
        test_thread.join(0.01)
        self.assertTrue(test_thread.isAlive())

        self.manager.continue_debug()
        # This join should work
        test_thread.join(0.01)
        self.assertFalse(test_thread.isAlive())
