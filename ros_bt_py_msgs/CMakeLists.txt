cmake_minimum_required(VERSION 2.8.3)
project(ros_bt_py_msgs)

find_package(catkin REQUIRED COMPONENTS
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
     NodeDataLocation.msg
     NodeData.msg
     NodeDataWiring.msg
     NodeOptionWiring.msg
     Node.msg
     Package.msg
     Packages.msg
     RemoteSlotState.msg
     Tree.msg
     TreeDataUpdate.msg
     UtilityBounds.msg
     )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddNode.srv
  AddNodeAtIndex.srv
  ClearTree.srv
  Continue.srv
  ControlTreeExecution.srv
  EvaluateUtility.srv
  GetAvailableNodes.srv
  GetAvailableSubtrees.srv
  GetMessageFields.srv
  GetPackageStructure.srv
  GetSubtree.srv
  InsertNode.srv
  LoadTree.srv
  LoadTreeFromPath.srv
  ModifyBreakpoints.srv
  MorphNode.srv
  MoveNode.srv
  RemoveNode.srv
  ReplaceNode.srv
  SaveTree.srv
  SetExecutionMode.srv
  SetOptions.srv
  WireNodeData.srv
  )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  FindBestExecutor.action
  RunTree.action
  )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES actionlib_msgs geometry_msgs
   )

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs geometry_msgs
  )
