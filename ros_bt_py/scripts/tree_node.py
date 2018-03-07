#! /usr/bin/env python2.7

import rospy

from ros_bt_py_msgs.msg import Tree, DebugInfo
from ros_bt_py_msgs.srv import AddNode, ControlTreeExecution, RemoveNode, WireNodeData
from ros_bt_py.tree_manager import TreeManager
from ros_bt_py.debug_manager import DebugManager


class TreeNode(object):
    def __init__(self):
        node_module_names = rospy.get_param('~node_modules', default=[])
        if isinstance(node_module_names, basestring):
            if ',' in node_module_names:
                # Try to parse comma-separated list of modules
                node_module_names = [module.strip() for module in node_module_names.split(',')]
            else:
                # try to parse whitespace-separated list of modules
                node_module_names = node_module_names.split()
        if not isinstance(node_module_names, list):
            raise TypeError('node_modules must be a list, but is a %s' %
                            type(node_module_names.__name__))

        self.tree_pub = rospy.Publisher('tree', Tree, latch=True, queue_size=1)
        self.debug_info_pub = rospy.Publisher('debug_info', DebugInfo, latch=True, queue_size=1)

        self.debug_manager = DebugManager(
            target_tick_frequency_hz=rospy.get_param('~target_tick_frequency_hz'))
        self.tree_manager = TreeManager(module_list=node_module_names,
                                        debug_manager=self.debug_manager,
                                        publish_tree_callback=self.tree_pub.publish,
                                        publish_debug_info_callback=self.debug_info_pub.publish)

        self.add_node_service = rospy.Service('add_node',
                                              AddNode,
                                              self.tree_manager.add_node)
        self.remove_node_service = rospy.Service('remove_node',
                                                 RemoveNode,
                                                 self.tree_manager.remove_node)
        self.wire_data_service = rospy.Service('wire_data',
                                               WireNodeData,
                                               self.tree_manager.wire_data)
        self.unwire_data_service = rospy.Service('unwire_data',
                                                 WireNodeData,
                                                 self.tree_manager.unwire_data)
        self.modify_breakpoints_service = rospy.Service('modify_breakpoints',
                                                        ModifyBreakpoints,
                                                        self.tree_manager.modify_breakpoints)

        self.control_tree_execution_service = rospy.Service('control_tree_execution',
                                                            ControlTreeExecution,
                                                            self.tree_manager.control_execution)

if __name__ == '__main__':
    rospy.init_node('tree_node')

    _ = TreeNode()
    rospy.spin()
