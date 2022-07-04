cmake_minimum_required(VERSION 2.8.12)
project(ros_bt_py_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation)

## Generate messages in the 'msg' folder
add_message_files(
     FILES
     DebugInfo.msg
     DebugSettings.msg
     DocumentedNode.msg
     Message.msg
     Messages.msg
     Node.msg
     NodeData.msg
     NodeDataLocation.msg
     NodeDataWiring.msg
     NodeDiagnostics.msg
     Package.msg
     Packages.msg
     RemoteSlotState.msg
     Tree.msg
     TreeDataUpdate.msg
     UtilityBounds.msg
     CapabilityInterface.msg
     CapabilityImplementation.msg
     )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddNode.srv
  AddNodeAtIndex.srv
  ChangeTreeName.srv
  ClearTree.srv
  Continue.srv
  ControlTreeExecution.srv
  EvaluateUtility.srv
  FixYaml.srv
  GenerateSubtree.srv
  GetAvailableNodes.srv
  GetAvailableSubtrees.srv
  GetMessageFields.srv
  GetPackageStructure.srv
  GetSubtree.srv
  InsertNode.srv
  LoadTree.srv
  LoadTreeFromPath.srv
  MigrateTree.srv
  ModifyBreakpoints.srv
  MorphNode.srv
  MoveNode.srv
  ReloadTree.srv
  RemoveNode.srv
  ReplaceNode.srv
  SaveTree.srv
  SetExecutionMode.srv
  SetOptions.srv
  TestService.srv
  WireNodeData.srv
  capability_interface/GetCapabilityInterfaces.srv
  capability_interface/LoadCapabilityInterfaces.srv
  capability_interface/SaveCapabilityInterfaces.srv
  capability_interface/PutCapabilityInterfaces.srv
  capability_interface/UpdateCapabilityInterfaces.srv
  capability_implementation/DeleteCapabilityImplementation.srv
  capability_implementation/PutCapabilityImplementation.srv
  capability_implementation/GetCapabilityImplementations.srv
  )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  FindBestExecutor.action
  RunTree.action
  FindBestCapabilityImplementation.action
  ExecuteImplementation.action
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime
  )
