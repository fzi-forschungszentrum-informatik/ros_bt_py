cmake_minimum_required(VERSION 2.8.3)
project(ros_bt_py_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation)

## Generate messages in the 'msg' folder
add_message_files(
     FILES
     DebugInfo.msg
     DebugSettings.msg
     Node.msg
     NodeData.msg
     NodeDataLocation.msg
     NodeDataWiring.msg
     TickTime.msg
     Tree.msg
     TreeLocation.msg
     )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddNode.srv
  Continue.srv
  ControlTreeExecution.srv
  GetAvailableNodes.srv
  GetAvailableSubtrees.srv
  InsertNode.srv
  LoadTree.srv
  ModifyBreakpoints.srv
  MoveNode.srv
  RemoveNode.srv
  ReplaceNode.srv
  SetExecutionMode.srv
  SetOptions.srv
  WireNodeData.srv
  )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  RunTree.action
  )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES actionlib_msgs
   )

catkin_package(
  CATKIN_DEPENDS message_runtime
  )
