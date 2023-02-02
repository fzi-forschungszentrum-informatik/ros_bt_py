#!/usr/bin/env python
#  -------- BEGIN LICENSE BLOCK --------
# Copyright 2022-2023 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
"""Module containing the main node for a ros_bt_py instance running the BT."""

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from ros_bt_py_msgs.msg import (
    Tree,
    DebugInfo,
    DebugSettings,
    NodeDiagnostics,
    Messages,
    Packages,
)
from ros_bt_py_msgs.srv import (
    AddNode,
    AddNodeAtIndex,
    ControlTreeExecution,
    ModifyBreakpoints,
    RemoveNode,
    WireNodeData,
    GetAvailableNodes,
    SetExecutionMode,
    SetOptions,
    Continue,
    LoadTree,
    LoadTreeFromPath,
    MoveNode,
    ReplaceNode,
    GetSubtree,
    ClearTree,
    MorphNode,
    SaveTree,
    FixYaml,
    GetCapabilityInterfaces,
)
from ros_bt_py_msgs.srv import (
    LoadTreeRequest,
    ControlTreeExecutionRequest,
    GetMessageFields,
    GetPackageStructure,
    MigrateTree,
    GenerateSubtree,
    ReloadTree,
    ChangeTreeName,
)

from ros_bt_py.capability import (
    capability_node_class_from_capability_interface_callback,
)
from ros_bt_py.tree_manager import (
    TreeManager,
    get_success,
    get_error_message,
    get_available_nodes,
)
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.migration import MigrationManager, check_node_versions
from ros_bt_py.package_manager import PackageManager
from ros_bt_py.helpers import fix_yaml


class TreeNode(object):
    """ROS node running a single behavior tree."""

    def __init__(self):
        """Create a new Behavior Tree Node with all required publishers and subscribers."""
        rospy.loginfo("initializing tree node...")
        node_module_names = rospy.get_param("~node_modules", default=[])
        if isinstance(node_module_names, str):
            if "," in node_module_names:
                # Try to parse comma-separated list of modules
                node_module_names = [
                    module.strip() for module in node_module_names.split(",")
                ]
            else:
                # try to parse whitespace-separated list of modules
                node_module_names = node_module_names.split()
        if not isinstance(node_module_names, list):
            raise TypeError(
                f"node_modules must be a list, but is a {type(node_module_names.__name__)}"
            )

        show_traceback_on_exception = rospy.get_param(
            "~show_traceback_on_exception", default=False
        )
        load_default_tree = rospy.get_param("~load_default_tree", default=False)
        load_default_tree_permissive = rospy.get_param(
            "~load_default_tree_permissive", default=False
        )
        default_tree_path = rospy.get_param("~default_tree_path", default="")
        default_tree_tick_frequency_hz = rospy.get_param(
            "~default_tree_tick_frequency_hz", default=1
        )
        default_tree_diagnostics_frequency_hz = rospy.get_param(
            "~default_tree_diagnostics_frequency_hz", default=1
        )
        default_tree_control_command = rospy.get_param(
            "~default_tree_control_command", default=2
        )

        local_mc_prefix = f"{rospy.get_namespace()}/mission_control"

        get_capability_interfaces_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/capability_repository/capabilities/interfaces/get"
        )

        self.get_capability_interface_service = rospy.ServiceProxy(
            name=get_capability_interfaces_topic,
            service_class=GetCapabilityInterfaces,
            persistent=True,
        )

        interfaces_announcement_topic = rospy.resolve_name(
            f"{rospy.get_namespace()}/capability_repository/capabilities/interfaces"
        )

        self.capability_interface_subscription = rospy.Subscriber(
            name=interfaces_announcement_topic,
            data_class=std_msgs.msg.Time,
            callback=capability_node_class_from_capability_interface_callback,
            callback_args=(
                local_mc_prefix,
                self.get_capability_interface_service,
            ),
        )

        self.tree_pub = rospy.Publisher("~tree", Tree, latch=True, queue_size=1)
        self.debug_info_pub = rospy.Publisher(
            "~debug/debug_info", DebugInfo, latch=True, queue_size=1
        )
        self.debug_settings_pub = rospy.Publisher(
            "~debug/debug_settings", DebugSettings, latch=True, queue_size=1
        )
        self.node_diagnostics_pub = rospy.Publisher(
            "~debug/node_diagnostics", NodeDiagnostics, latch=True, queue_size=10
        )

        node_in_namespace = rospy.get_namespace().strip("/")
        namespace = rospy.get_namespace() if node_in_namespace else ""
        self.ros_diagnostics_pub = rospy.Publisher(
            f"/diagnostics/{namespace}", DiagnosticArray, queue_size=1
        )

        self.debug_manager = DebugManager()
        self.tree_manager = TreeManager(
            module_list=node_module_names,
            debug_manager=self.debug_manager,
            publish_tree_callback=self.tree_pub.publish,
            publish_debug_info_callback=self.debug_info_pub.publish,
            publish_debug_settings_callback=self.debug_settings_pub.publish,
            publish_node_diagnostics_callback=self.node_diagnostics_pub.publish,
            publish_diagnostic_callback=self.ros_diagnostics_pub.publish,
            diagnostics_frequency=default_tree_diagnostics_frequency_hz,
            show_traceback_on_exception=show_traceback_on_exception,
        )

        self.add_node_service = rospy.Service(
            "~add_node", AddNode, self.tree_manager.add_node
        )
        self.add_node_at_index_service = rospy.Service(
            "~add_node_at_index", AddNodeAtIndex, self.tree_manager.add_node_at_index
        )
        self.remove_node_service = rospy.Service(
            "~remove_node", RemoveNode, self.tree_manager.remove_node
        )
        self.morph_node_service = rospy.Service(
            "~morph_node", MorphNode, self.tree_manager.morph_node
        )
        self.wire_data_service = rospy.Service(
            "~wire_data", WireNodeData, self.tree_manager.wire_data
        )
        self.unwire_data_service = rospy.Service(
            "~unwire_data", WireNodeData, self.tree_manager.unwire_data
        )
        self.modify_breakpoints_service = rospy.Service(
            "~debug/modify_breakpoints",
            ModifyBreakpoints,
            self.tree_manager.modify_breakpoints,
        )

        self.control_tree_execution_service = rospy.Service(
            "~control_tree_execution",
            ControlTreeExecution,
            self.tree_manager.control_execution,
        )

        self.get_available_nodes_service = rospy.Service(
            "~get_available_nodes", GetAvailableNodes, get_available_nodes
        )

        self.get_subtree_service = rospy.Service(
            "~get_subtree", GetSubtree, self.tree_manager.get_subtree
        )

        self.generate_subtree_service = rospy.Service(
            "~generate_subtree", GenerateSubtree, self.tree_manager.generate_subtree
        )

        self.set_execution_mode_service = rospy.Service(
            "~debug/set_execution_mode",
            SetExecutionMode,
            self.tree_manager.set_execution_mode,
        )

        self.set_options_service = rospy.Service(
            "~set_options", SetOptions, self.tree_manager.set_options
        )

        self.move_node_servcice = rospy.Service(
            "~move_node", MoveNode, self.tree_manager.move_node
        )

        self.replace_node_servcice = rospy.Service(
            "~replace_node", ReplaceNode, self.tree_manager.replace_node
        )

        self.continue_servcice = rospy.Service(
            "~debug/continue", Continue, self.tree_manager.debug_step
        )

        self.load_tree_service = rospy.Service(
            "~load_tree", LoadTree, self.tree_manager.load_tree
        )

        self.load_tree_from_path_service = rospy.Service(
            "~load_tree_from_path",
            LoadTreeFromPath,
            self.tree_manager.load_tree_from_path,
        )

        self.clear_service = rospy.Service("~clear", ClearTree, self.tree_manager.clear)

        self.reload_service = rospy.Service(
            "~reload", ReloadTree, self.tree_manager.reload_tree
        )

        self.change_tree_name_service = rospy.Service(
            "~change_tree_name", ChangeTreeName, self.tree_manager.change_tree_name
        )

        self.fix_yaml_service = rospy.Service("~fix_yaml", FixYaml, fix_yaml)

        rospy.loginfo("initialized tree manager")

        if load_default_tree:
            rospy.logwarn(f"loading default tree: {default_tree_path}")
            tree = Tree(path=default_tree_path)
            load_tree_request = LoadTreeRequest(
                tree=tree, permissive=load_default_tree_permissive
            )
            load_tree_response = self.tree_manager.load_tree(load_tree_request)
            if not load_tree_response.success:
                rospy.logerr(
                    f"could not load default tree: {load_tree_response.error_message}"
                )
            else:
                control_tree_execution_request = ControlTreeExecutionRequest(
                    command=default_tree_control_command,
                    tick_frequency_hz=default_tree_tick_frequency_hz,
                )
                control_tree_execution_response = self.tree_manager.control_execution(
                    control_tree_execution_request
                )
                if not control_tree_execution_response.success:
                    rospy.logerr(
                        f"could not execute default tree: "
                        f"{control_tree_execution_response.error_message}"
                    )

        rospy.loginfo("initializing migration manger ...")
        self.migration_manager = MigrationManager(tree_manager=self.tree_manager)

        self.check_node_versions_service = rospy.Service(
            "~check_node_versions", MigrateTree, check_node_versions
        )

        self.migrate_tree_service = rospy.Service(
            "~migrate_tree", MigrateTree, self.migration_manager.migrate_tree
        )
        rospy.loginfo("initialized migration manager")

        rospy.loginfo("initializing package manager...")
        self.message_list_pub = rospy.Publisher(
            "~messages", Messages, latch=True, queue_size=1
        )
        self.packages_list_pub = rospy.Publisher(
            "~packages", Packages, latch=True, queue_size=1
        )

        self.package_manager = PackageManager(
            publish_message_list_callback=self.message_list_pub,
            publish_packages_list_callback=self.packages_list_pub,
        )

        self.package_manager.publish_packages_list()
        self.get_message_fields_service = rospy.Service(
            "~get_message_fields",
            GetMessageFields,
            self.package_manager.get_message_fields,
        )

        self.get_message_constant_fields_service = rospy.Service(
            "~get_message_constant_fields",
            GetMessageFields,
            self.package_manager.get_message_constant_fields_handler,
        )

        self.get_package_structure_service = rospy.Service(
            "~get_package_structure",
            GetPackageStructure,
            self.package_manager.get_package_structure,
        )

        self.save_tree_service = rospy.Service(
            "~save_tree", SaveTree, self.package_manager.save_tree
        )

        self.package_manager.publish_message_list()
        rospy.loginfo("initialized package manager")
        rospy.loginfo("initialized tree node")

    def shutdown(self):
        """Shut down tree node in a safe way."""
        if self.tree_manager.get_state() not in [Tree.IDLE, Tree.EDITABLE, Tree.ERROR]:
            rospy.loginfo("Shutting down Behavior Tree")
            response = self.tree_manager.control_execution(
                ControlTreeExecutionRequest(
                    command=ControlTreeExecutionRequest.SHUTDOWN
                )
            )
            if not get_success(response):
                rospy.logerr(
                    "Failed to shut down Behavior Tree: %s", get_error_message(response)
                )


if __name__ == "__main__":
    rospy.init_node("tree_node")

    node = TreeNode()
    rospy.spin()
    node.shutdown()
