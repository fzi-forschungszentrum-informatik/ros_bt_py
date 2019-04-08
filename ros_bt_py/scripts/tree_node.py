#! /usr/bin/env python2.7
import os
import jsonpickle

import rospkg
import rospy
import roslib

from catkin.find_in_workspaces import find_in_workspaces

from ros_bt_py_msgs.msg import Message, Messages

from ros_bt_py_msgs.msg import Tree, DebugInfo, DebugSettings
from ros_bt_py_msgs.srv import AddNode, ControlTreeExecution, ModifyBreakpoints, RemoveNode, \
     WireNodeData, GetAvailableNodes, SetExecutionMode, SetOptions, Continue, LoadTree, \
     MoveNode, ReplaceNode, GetSubtree, ClearTree, MorphNode
from ros_bt_py_msgs.srv import ControlTreeExecutionRequest, GetMessageFields, GetMessageFieldsResponse
from ros_bt_py.tree_manager import TreeManager, get_success, get_error_message
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
            publish_debug_settings_callback=self.debug_settings_pub.publish)

        self.add_node_service = rospy.Service('~add_node',
                                              AddNode,
                                              self.tree_manager.add_node)
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

        self.get_message_fields_service = rospy.Service('~get_message_fields',
                                                        GetMessageFields,
                                                        self.get_message_fields)

        self.message_list_pub = rospy.Publisher('~messages', Messages, latch=True, queue_size=1)
        self.publish_message_list()

    def _get_package_paths(self, pkgname, rospack):
        _catkin_workspace_to_source_spaces = {}
        _catkin_source_path_to_packages = {}
        paths = []
        path = rospack.get_path(pkgname)
        paths.append(path)
        results = find_in_workspaces(search_dirs=['share'],
                                     project=pkgname,
                                     first_match_only=True,
                                     workspace_to_source_spaces=_catkin_workspace_to_source_spaces,
                                     source_path_to_packages=_catkin_source_path_to_packages)
        if results and results[0] != path:
            paths.append(results[0])
        return paths

    def publish_message_list(self):
        rospack = rospkg.RosPack()

        messages = []
        packages = rospack.list()

        actions = []
        for package in packages:
            for package_path in self._get_package_paths(package, rospack):
                path = package_path + "/msg"
                resources = []
                if os.path.isdir(path):
                    resources = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
                result = [x[:-len(".msg")] for x in resources]
                result.sort()
                for msg_type in result:
                    if msg_type[-6:] == "Action":
                        actions.append(msg_type)
                    append_msg = True
                    for action in actions:
                        if msg_type == action+"Feedback" or msg_type == action+"Goal" or msg_type == action+"Result":
                            append_msg = False
                    if append_msg:
                        messages.append(Message(msg=package+"/"+msg_type, service=False))
                path = package_path + "/srv"
                resources = []
                if os.path.isdir(path):
                    resources = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
                result = [x[:-len(".srv")] for x in resources]
                result.sort()
                for srv_type in result:
                    messages.append(Message(msg=package+"/"+srv_type, service=True))

        msg = Messages()
        msg.messages = messages
        self.message_list_pub.publish(msg)

    def get_message_fields(self, request):
        response = GetMessageFieldsResponse()
        try:
            message_class = None
            if request.service:
                message_class = roslib.message.get_service_class(request.message_type)
            else:
                message_class = roslib.message.get_message_class(request.message_type)
            for field in str(message_class()).split("\n"):
                response.field_names.append(field.split(":")[0].strip())
            response.fields = jsonpickle.encode(message_class())
            response.success = True
        except Exception as e:
            response.success = False
            response.error_message = "Could not get message fields for {}: {}".format(request.message_type, e)
        return response

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
