#!/usr/bin/env python2.7
from threading import Lock
import unittest

try:
    import unittest.mock as mock
except ImportError:
    import mock

import rospy

from std_msgs.msg import Int32
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.ros_param import RosParamOption, RosParamInput

PKG = 'ros_bt_py'


class TestRosParamOption(unittest.TestCase):
    def testExistingParam(self):
        ros_param = RosParamOption(options={
            'param_name': '/param_int',
            'param_type': int,
        })
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs['default_value'] = 23
        self.assertEqual(ros_param.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_param.outputs['param'], 42)

        expected_bounds = UtilityBounds(can_execute=True,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)

        self.assertEqual(ros_param.calculate_utility(), expected_bounds)

        self.assertEqual(ros_param.untick(), NodeMsg.IDLE)
        self.assertEqual(ros_param.reset(), NodeMsg.IDLE)
        self.assertEqual(ros_param.shutdown(), NodeMsg.SHUTDOWN)

    def testMissingParam(self):
        ros_param = RosParamOption(options={
            'param_name': '/param_missing',
            'param_type': int,
        })
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs['default_value'] = 23
        self.assertEqual(ros_param.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_param.outputs['param'], 23)

        expected_bounds = UtilityBounds(can_execute=False,
                                        has_lower_bound_success=False,
                                        has_upper_bound_success=False,
                                        has_lower_bound_failure=False,
                                        has_upper_bound_failure=False)

        self.assertEqual(ros_param.calculate_utility(), expected_bounds)

    def testWrongParamType(self):
        ros_param = RosParamOption(options={
            'param_name': '/param_int',
            'param_type': str,
        })
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs['default_value'] = 'default'
        self.assertEqual(ros_param.tick(), NodeMsg.FAILED)

    def testROSException(self):
        ros_param = RosParamOption(options={
            'param_name': '/param_missing',
            'param_type': int,
        })

        expected_bounds = UtilityBounds()
        with mock.patch('rospy.get_param_names') as mocked_get_param_names:
            mocked_get_param_names.side_effect = rospy.ROSException()
            self.assertEqual(ros_param.calculate_utility(), expected_bounds)


class TestRosParamInput(unittest.TestCase):
    def testExistingParam(self):
        ros_param = RosParamInput(options={
            'param_type': int,
            'default_value': 0
        })
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs['param_name'] = '/param_int'
        self.assertEqual(ros_param.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_param.outputs['param'], 42)

        self.assertEqual(ros_param.untick(), NodeMsg.IDLE)
        self.assertEqual(ros_param.reset(), NodeMsg.IDLE)
        self.assertEqual(ros_param.shutdown(), NodeMsg.SHUTDOWN)

    def testMissingParam(self):
        ros_param = RosParamInput(options={
            'param_type': int,
            'default_value': 0
        })

        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs['param_name'] = '/param_missing'
        self.assertEqual(ros_param.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_param.outputs['param'], 0)

        expected_bounds = UtilityBounds()

        self.assertEqual(ros_param.calculate_utility(), expected_bounds)

    def testWrongParamType(self):
        ros_param = RosParamInput(options={
            'default_value': 'toto',
            'param_type': str,
        })
        self.assertEqual(ros_param.state, NodeMsg.UNINITIALIZED)
        ros_param.setup()
        self.assertEqual(ros_param.state, NodeMsg.IDLE)

        ros_param.inputs['param_name'] = 'param_int'
        self.assertEqual(ros_param.tick(), NodeMsg.FAILED)


if __name__ == '__main__':
    rospy.init_node('test_ros_param')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_ros_param')
    rostest.rosrun(PKG, 'test_ros_param', TestRosParamOption,
                   sysargs=sys.argv + ['--cov'])
    rostest.rosrun(PKG, 'test_ros_param', TestRosParamInput,
                   sysargs=sys.argv + ['--cov'])
