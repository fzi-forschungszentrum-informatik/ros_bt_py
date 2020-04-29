#!/usr/bin/env python
from threading import Lock
import unittest

try:
    import unittest.mock as mock
except ImportError:
    import mock

import rospy

from std_msgs.msg import Header
from ros_bt_py_msgs.msg import Node as NodeMsg, UtilityBounds

from ros_bt_py.node_config import NodeConfig
from ros_bt_py.nodes.ros_header import GetStdHeader

PKG = 'ros_bt_py'


class TestGetStdHeader(unittest.TestCase):
    def testHeader(self):
        ros_header = GetStdHeader(options={
            'header_type': Header,
        })
        self.assertEqual(ros_header.state, NodeMsg.UNINITIALIZED)
        ros_header.setup()
        self.assertEqual(ros_header.state, NodeMsg.IDLE)

        self.assertEqual(ros_header.tick(), NodeMsg.SUCCEEDED)
        self.assertEqual(ros_header.outputs['header'], ros_header.header)

        expected_bounds = UtilityBounds(can_execute=True,
                                        has_lower_bound_success=True,
                                        has_upper_bound_success=True,
                                        has_lower_bound_failure=True,
                                        has_upper_bound_failure=True)

        self.assertEqual(ros_header.calculate_utility(), expected_bounds)

        self.assertEqual(ros_header.untick(), NodeMsg.IDLE)
        self.assertEqual(ros_header.reset(), NodeMsg.IDLE)
        self.assertEqual(ros_header.shutdown(), NodeMsg.SHUTDOWN)

    def testBrokenHeader(self):
        ros_header = GetStdHeader(options={
            'header_type': int,
        })
        self.assertEqual(ros_header.state, NodeMsg.UNINITIALIZED)
        ros_header.setup()
        self.assertEqual(ros_header.state, NodeMsg.IDLE)

        self.assertEqual(ros_header.tick(), NodeMsg.FAILED)


if __name__ == '__main__':
    rospy.init_node('test_ros_header')
    import rostest
    import sys
    import os
    os.environ['COVERAGE_FILE'] = '%s.%s.coverage' % (PKG, 'test_ros_header')
    rostest.rosrun(PKG, 'test_ros_header', TestGetStdHeader,
                   sysargs=sys.argv + ['--cov'])
