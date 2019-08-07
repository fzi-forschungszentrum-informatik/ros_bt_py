#! /usr/bin/env python2.7
import rospy

from ros_bt_py_msgs.msg import Messages, Packages

from ros_bt_py_msgs.msg import Tree, DebugInfo, DebugSettings
from ros_bt_py_msgs.srv import AddNode, AddNodeAtIndex, ControlTreeExecution, ModifyBreakpoints, RemoveNode, \
     WireNodeData, GetAvailableNodes, SetExecutionMode, SetOptions, Continue, LoadTree, \
     MoveNode, ReplaceNode, GetSubtree, ClearTree, MorphNode, GenerateSubtree, SaveTree
from ros_bt_py_msgs.srv import ControlTreeExecutionRequest, GetMessageFields, GetPackageStructure
from ros_bt_py.tree_manager import TreeManager, get_success, get_error_message
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.package_manager import PackageManager


# FIXME !!!! !!!!! !!!!! move the "ListOfPackages" message type to ros_bt_py !!!!
from ros_ta_msgs.msg import ListOfPackages


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

        show_traceback_on_exception = rospy.get_param('~show_traceback_on_exception', default=False)

        self.tree_pub = rospy.Publisher('~tree', Tree, latch=True, queue_size=1)
        self.debug_info_pub = rospy.Publisher('~debug/debug_info', DebugInfo, latch=True, queue_size=1)
        self.debug_settings_pub = rospy.Publisher(
            '~debug/debug_settings',
            DebugSettings,
            latch=True,
            queue_size=1)

        self.debug_manager = DebugManager()
        self.tree_manager = TreeManager(
            module_list=node_module_names,
            debug_manager=self.debug_manager,
            publish_tree_callback=self.tree_pub.publish,
            publish_debug_info_callback=self.debug_info_pub.publish,
            publish_debug_settings_callback=self.debug_settings_pub.publish,
            show_traceback_on_exception=show_traceback_on_exception)

        self.add_node_service = rospy.Service('~add_node',
                                              AddNode,
                                              self.tree_manager.add_node)
        self.add_node_at_index_service = rospy.Service('~add_node_at_index',
                                                       AddNodeAtIndex,
                                                       self.tree_manager.add_node_at_index)
        self.remove_node_service = rospy.Service('~remove_node',
                                                 RemoveNode,
                                                 self.tree_manager.remove_node)
        self.morph_node_service = rospy.Service('~morph_node',
                                                MorphNode,
                                                self.tree_manager.morph_node)
        self.wire_data_service = rospy.Service('~wire_data',
                                               WireNodeData,
                                               self.tree_manager.wire_data)
        self.unwire_data_service = rospy.Service('~unwire_data',
                                                 WireNodeData,
                                                 self.tree_manager.unwire_data)
        self.modify_breakpoints_service = rospy.Service('~debug/modify_breakpoints',
                                                        ModifyBreakpoints,
                                                        self.tree_manager.modify_breakpoints)

        self.control_tree_execution_service = rospy.Service('~control_tree_execution',
                                                            ControlTreeExecution,
                                                            self.tree_manager.control_execution)

        self.get_available_nodes_service = rospy.Service('~get_available_nodes',
                                                         GetAvailableNodes,
                                                         self.tree_manager.get_available_nodes)

        self.get_subtree_service = rospy.Service('~get_subtree',
                                                 GetSubtree,
                                                 self.tree_manager.get_subtree)

        self.generate_subtree_service = rospy.Service('~generate_subtree',
                                                      GenerateSubtree,
                                                      self.tree_manager.generate_subtree)

        self.set_execution_mode_service = rospy.Service('~debug/set_execution_mode',
                                                        SetExecutionMode,
                                                        self.tree_manager.set_execution_mode)

        self.set_options_service = rospy.Service('~set_options',
                                                 SetOptions,
                                                 self.tree_manager.set_options)

        self.move_node_servcice = rospy.Service('~move_node',
                                                MoveNode,
                                                self.tree_manager.move_node)

        self.replace_node_servcice = rospy.Service('~replace_node',
                                                   ReplaceNode,
                                                   self.tree_manager.replace_node)

        self.continue_servcice = rospy.Service('~debug/continue',
                                               Continue,
                                               self.tree_manager.debug_step)

        self.load_tree_service = rospy.Service('~load_tree',
                                               LoadTree,
                                               self.tree_manager.load_tree)

        self.clear_service = rospy.Service('~clear',
                                           ClearTree,
                                           self.tree_manager.clear)

        self.message_list_pub = rospy.Publisher('~messages', Messages, latch=True, queue_size=1)
        self.packages_list_pub = rospy.Publisher('~packages', Packages, latch=True, queue_size=1)

        self.package_manager = PackageManager(publish_message_list_callback=self.message_list_pub,
                                              publish_packages_list_callback=self.packages_list_pub)

        self.get_message_fields_service = rospy.Service('~get_message_fields',
                                                        GetMessageFields,
                                                        self.package_manager.get_message_fields)

        self.get_package_structure_service = rospy.Service('~get_package_structure',
                                                            GetPackageStructure,
                                                            self.package_manager.get_package_structure)

        self.save_tree_service = rospy.Service('~save_tree',
                                               SaveTree,
                                               self.package_manager.save_tree)

    def shutdown(self):
        if self.tree_manager.get_state() not in [Tree.IDLE, Tree.EDITABLE, Tree.ERROR]:
            rospy.loginfo('Shutting down Behavior Tree')
            response = self.tree_manager.control_execution(ControlTreeExecutionRequest(
                command=ControlTreeExecutionRequest.SHUTDOWN))
            if not get_success(response):
                rospy.logerr('Failed to shut down Behavior Tree: %s',
                             get_error_message(response))


if __name__ == '__main__':
    rospy.init_node('tree_node')

    node = TreeNode()
    rospy.spin()
    node.shutdown()
