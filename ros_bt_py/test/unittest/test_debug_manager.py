import inspect
import sys
from threading import Thread
import time
import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.nodes.passthrough_node import PassthroughNode
from ros_bt_py.nodes.subtree import Subtree


class TestDebugManager(unittest.TestCase):
    def setUp(self):
        self.debug_settings_msg = None
        self.debug_info_msg = None

    def testReport(self):
        self.manager = DebugManager()
        self.manager._debug_settings_msg.collect_performance_data = True

        node = PassthroughNode(name='foo',
                               options={'passthrough_type': int})
        node.setup()

        starting_recursion_depth = len(inspect.stack())
        with self.manager.report_tick(node):
            time.sleep(0.01)

        self.assertEqual(self.manager.get_debug_info_msg().max_recursion_depth,
                         sys.getrecursionlimit())
        # Plus one for the context self.manager, and another one for the contextlib decorator
        self.assertEqual(self.manager.get_debug_info_msg().current_recursion_depth,
                         starting_recursion_depth + 2)

    def testStep(self):
        self.manager = DebugManager()
        self.manager._debug_settings_msg.single_step = True

        self.node = PassthroughNode(name='foo',
                                    options={'passthrough_type': int},
                                    debug_manager=self.manager)
        self.node.setup()
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
        self.assertEqual(self.node.state, NodeMsg.SUCCEEDED)
        # Should still be waiting for second continue -> join won't work
        test_thread.join(0.01)
        self.assertTrue(test_thread.isAlive())

        # Continue to DEBUG_POST_TICK - this is only to show in the tree where
        # the debug handler is - no other changes since the previous continue
        self.manager.continue_debug()
        time.sleep(0.05)
        self.assertEqual(self.node.state, NodeMsg.DEBUG_POST_TICK)
        self.manager.continue_debug()
        # This join should work
        test_thread.join(0.01)
        self.assertFalse(test_thread.isAlive())
        # The node state should be restored after the last continue
        self.assertEqual(self.node.state, NodeMsg.SUCCEEDED)

    def _publish_debug_settings_callback(self, msg):
        self.debug_settings_msg = msg

    def _publish_debug_info_callback(self, msg):
        self.debug_info_msg = msg

    def testPublishDebugSettings(self):
        self.assertIsNone(self.debug_settings_msg)
        manager = DebugManager(
            debug_settings_publish_callback=self._publish_debug_settings_callback)
        manager.set_execution_mode(
            single_step=False,
            collect_performance_data=False,
            publish_subtrees=False,
        )
        time.sleep(0.05)
        self.assertIsNotNone(self.debug_settings_msg)

    def testModifyBreakpoints(self):
        self.assertIsNone(self.debug_settings_msg)
        manager = DebugManager(
            debug_settings_publish_callback=self._publish_debug_settings_callback)
        self.assertEqual(manager.modify_breakpoints(), [])
        breakpoints = ["first", "second", "third", "fourth"]
        self.assertEqual(manager.modify_breakpoints(add=breakpoints), breakpoints)
        self.assertEqual(manager.modify_breakpoints(
            remove=["second", "third"]), ["first", "fourth"])
        self.assertEqual(manager.modify_breakpoints(remove_all=True), [])

    def testReportWithBreakpoint(self):
        self.assertIsNone(self.debug_settings_msg)
        self.assertIsNone(self.debug_info_msg)
        manager = DebugManager(
            debug_info_publish_callback=self._publish_debug_info_callback,
            debug_settings_publish_callback=self._publish_debug_settings_callback)

        self.node = PassthroughNode(name='foo',
                                    options={'passthrough_type': int},
                                    debug_manager=manager)
        self.node.setup()
        self.node.inputs['in'] = 1

        breakpoints = ['foo']
        self.assertEqual(manager.modify_breakpoints(add=breakpoints), breakpoints)
        time.sleep(0.01)
        self.assertIsNotNone(self.debug_settings_msg)
        self.assertEqual(self.debug_settings_msg.breakpoint_names, breakpoints)

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
        manager.continue_debug()
        time.sleep(0.05)

        # Now out should be changed
        self.assertEqual(self.node.outputs['out'], 1)
        self.assertEqual(self.node.state, NodeMsg.SUCCEEDED)
        # Should still be waiting for second continue -> join won't work
        test_thread.join(0.01)
        self.assertTrue(test_thread.isAlive())

        # Continue to DEBUG_POST_TICK - this is only to show in the tree where
        # the debug handler is - no other changes since the previous continue
        manager.continue_debug()
        time.sleep(0.05)
        self.assertEqual(self.node.state, NodeMsg.DEBUG_POST_TICK)
        manager.continue_debug()
        # This join should work
        test_thread.join(0.01)
        self.assertFalse(test_thread.isAlive())
        # The node state should be restored after the last continue
        self.assertEqual(self.node.state, NodeMsg.SUCCEEDED)

        self.assertEqual(self.debug_settings_msg.breakpoint_names, [])
        self.assertIsNotNone(self.debug_info_msg)

    def testAddSubtreeInfoWithoutPublishSubtrees(self):
        manager = DebugManager()
        mock_subtree_msg = NodeMsg()
        mock_subtree_msg.name = 'foo'
        self.assertRaises(
            BehaviorTreeException,
            manager.add_subtree_info,
            'bar',
            mock_subtree_msg)

    def testDebugManagerAndSubtree(self):
        self.assertIsNone(self.debug_settings_msg)
        self.assertIsNone(self.debug_info_msg)
        debug_manager = DebugManager(
            debug_info_publish_callback=self._publish_debug_info_callback,
            debug_settings_publish_callback=self._publish_debug_settings_callback)
        debug_manager.set_execution_mode(single_step=False,
                                         collect_performance_data=False, publish_subtrees=True)
        subtree_options = {
            'subtree_path': 'package://ros_bt_py/etc/trees/test.yaml',
            'use_io_nodes': False
        }
        subtree = Subtree(
            options=subtree_options,
            debug_manager=debug_manager,
        )
        self.assertEqual(subtree.state, NodeMsg.UNINITIALIZED)
        subtree.setup()
        self.assertEqual(subtree.state, NodeMsg.IDLE)
        self.assertEqual(
            subtree.debug_manager.subtrees['Subtree.Subtree.'], subtree.manager.to_msg())

        self.assertEqual(subtree.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(subtree.untick(), NodeMsg.IDLE)
        self.assertEqual(subtree.reset(), NodeMsg.IDLE)
        self.assertEqual(subtree.shutdown(), NodeMsg.SHUTDOWN)

        info_msg = debug_manager.get_debug_info_msg()
        self.assertIsNotNone(info_msg.subtree_states)
        self.assertNotEqual(info_msg.subtree_states, [])
        debug_manager.clear_subtrees()
        info_msg = debug_manager.get_debug_info_msg()
        self.assertEqual(info_msg.subtree_states, [])
