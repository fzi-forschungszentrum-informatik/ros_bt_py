import unittest

from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds
from ros_bt_py_msgs.msg import NodeDataWiring, NodeDataLocation

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.nodes.subtree import Subtree


class TestSubtree(unittest.TestCase):
    def setUp(self):
        self.subtree_options = {
            'subtree_path': 'package://ros_bt_py/etc/trees/test.yaml',
            'use_io_nodes': False
        }

    def testSubtreeLoad(self):
        subtree = Subtree(options=self.subtree_options)

        self.assertTrue(subtree.outputs['load_success'])
        subtree.setup()
        subtree.tick()

        self.assertEqual(subtree.state, NodeMsg.SUCCEEDED)
        self.assertIn('succeeder.out', subtree.outputs)
        self.assertEqual(subtree.outputs['succeeder.out'], 'Yay!')

        subtree.untick()
        self.assertEqual(subtree.state, NodeMsg.IDLE)

    def testSubtreeLoadError(self):
        path = 'package://ros_bt_py/test/testdata/trees/this_tree_does_not_exist.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': False
        })
        self.assertFalse(subtree.outputs['load_success'])

        self.assertRaises(BehaviorTreeException, subtree.setup)

        expected_bounds = UtilityBounds(can_execute=False,
                                        has_lower_bound_success=False,
                                        has_upper_bound_success=False,
                                        has_lower_bound_failure=False,
                                        has_upper_bound_failure=False)

        self.assertEqual(subtree.calculate_utility(), expected_bounds)

    def testSubtreeLoadErrorPublicOption(self):
        path = 'package://ros_bt_py/test/testdata/trees/subtree_constant_public_option.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': False
        })
        self.assertFalse(subtree.outputs['load_success'])

        self.assertRaises(BehaviorTreeException, subtree.setup)

        expected_bounds = UtilityBounds(can_execute=False,
                                        has_lower_bound_success=False,
                                        has_upper_bound_success=False,
                                        has_lower_bound_failure=False,
                                        has_upper_bound_failure=False)

        self.assertEqual(subtree.calculate_utility(), expected_bounds)

    def testSubtreeWithDebugManager(self):
        debug_manager = DebugManager()
        debug_manager.set_execution_mode(single_step=False,
                                         collect_performance_data=False, publish_subtrees=True,
                                         collect_node_diagnostics=False)
        subtree = Subtree(
            options=self.subtree_options,
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

    def testSubtreeUtility(self):
        path = 'package://ros_bt_py/test/testdata/trees/subtree_mockutility.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': False
        })
        self.assertTrue(subtree.outputs['load_success'])

        self.assertEqual(subtree.state, NodeMsg.UNINITIALIZED)
        subtree.setup()
        self.assertEqual(subtree.state, NodeMsg.IDLE)

        expected_bounds = UtilityBounds(can_execute=True,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)

        self.assertEqual(subtree.calculate_utility(), expected_bounds)

    def testSubtreeUntick(self):
        subtree = Subtree(options=self.subtree_options)

        self.assertTrue(subtree.outputs['load_success'])
        subtree.setup()
        subtree.tick()
        subtree.shutdown()
        self.assertEqual(subtree.state, NodeMsg.SHUTDOWN)
        subtree.setup()
        subtree.untick()

        self.assertTrue(subtree.state == NodeMsg.IDLE or subtree.state == NodeMsg.PAUSED,
                        'subtree node is neither IDLE nor PAUSED after unticking!')

    def testIOSubtree(self):
        self.subtree_options['subtree_path'] = 'package://ros_bt_py/etc/trees/io_test.yaml'
        subtree = Subtree(options=self.subtree_options)

        # Should have one input for the subtree's one public input
        self.assertEqual(len(subtree.inputs), 1)
        # And 3 outputs (load_success, load_error_msg, plus one for the public output)
        self.assertEqual(len(subtree.outputs), 3)

        self.assertTrue(subtree.outputs['load_success'], subtree.outputs['load_error_msg'])
        subtree.setup()

        self.assertRaises(ValueError, subtree.tick)

        subtree.inputs['passthrough.in'] = 'Hewwo'

        subtree.tick()
        self.assertEqual(subtree.state, NodeMsg.SUCCEEDED)
        self.assertEqual(subtree.outputs['passthrough.out'], 'Hewwo')

        subtree.inputs['passthrough.in'] = 'owo'

        subtree.tick()
        self.assertEqual(subtree.state, NodeMsg.SUCCEEDED)
        self.assertEqual(subtree.outputs['passthrough.out'], 'owo')

    def testIOSubtreeWithIONodesInput(self):
        path = 'package://ros_bt_py/test/testdata/trees/subtree_ioinputoption.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': True
        })
        self.assertTrue(subtree.outputs['load_success'])

        # Should have one input for the subtree's one IOInputNode input
        self.assertEqual(len(subtree.inputs), 1)
        # And 2 outputs (load_success, load_error_msg)
        self.assertEqual(len(subtree.outputs), 2)

    def testIOSubtreeWithIONodesInputAnd2Nodes(self):
        path = 'package://ros_bt_py/test/testdata/trees/subtree_ioinputoption_and_2_nodes.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': False
        })
        self.assertTrue(subtree.outputs['load_success'])

        # Should have 3 inputs (in, a, b)
        self.assertEqual(len(subtree.inputs), 3)
        # And 4 outputs (load_success, load_error_msg, out)
        self.assertEqual(len(subtree.outputs), 4)

    def testIOSubtreeWithIONodesInputAnd2NodesAndUseIONodes(self):
        path = 'package://ros_bt_py/test/testdata/trees/subtree_ioinputoption_and_2_nodes.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': True
        })
        self.assertTrue(subtree.outputs['load_success'])

        # Should have 1 input (in)
        self.assertEqual(len(subtree.inputs), 1)
        # And 2 outputs (load_success, load_error_msg)
        self.assertEqual(len(subtree.outputs), 2)

    def testIOSubtreeWithIONodesOutput(self):
        path = 'package://ros_bt_py/test/testdata/trees/subtree_iooutputoption.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': True
        })
        self.assertTrue(subtree.outputs['load_success'])

        # Should have 0 inputs
        self.assertEqual(len(subtree.inputs), 0)
        # And 3 outputs (load_success, load_error_msg, out)
        self.assertEqual(len(subtree.outputs), 3)

    def testIOSubtreeWithIONodesInputAndOutput(self):
        path = 'package://ros_bt_py/test/testdata/trees/subtree_ioinput_output_option.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': True
        })
        self.assertTrue(subtree.outputs['load_success'])

        # Should have 1 input
        self.assertEqual(len(subtree.inputs), 1)
        # And 3 outputs (load_success, load_error_msg, out)
        self.assertEqual(len(subtree.outputs), 3)

    def testSubtreeWithChangedInputs(self):
        path = 'package://ros_bt_py/test/testdata/trees/subtree_changed_inputs.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': False
        })
        self.assertTrue(subtree.outputs['load_success'])

        # Should have no inputs
        self.assertEqual(len(subtree.inputs), 0)
        # And 5 outputs (2xload_success, 2xload_error_msg, out)
        self.assertEqual(len(subtree.outputs), 5)

    def testSubtreeWithChangedOutputs(self):
        path = 'package://ros_bt_py/test/testdata/trees/subtree_changed_outputs.yaml'
        subtree = Subtree(options={
            'subtree_path': path,
            'use_io_nodes': False
        })
        self.assertTrue(subtree.outputs['load_success'])

        # Should have 2 inputs
        self.assertEqual(len(subtree.inputs), 2)
        # And 4 outputs (2xload_success, 2xload_error_msg)
        self.assertEqual(len(subtree.outputs), 4)
