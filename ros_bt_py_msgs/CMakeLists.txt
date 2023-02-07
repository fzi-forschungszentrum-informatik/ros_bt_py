cmake_minimum_required(VERSION 3.0)
project(ros_bt_py_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation)

## Generate messages in the 'msg' folder
add_message_files(FILES
     AuctionMessage.msg
     CapabilityExecutionStatus.msg
     CapabilityImplementation.msg
     CapabilityInterface.msg
     CapabilityIOBridgeData.msg
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
     PingMsg.msg
     Precondition.msg
     RemoteCapabilitySlotStatus.msg
     RemoteSlotState.msg
     Tree.msg
     TreeDataUpdate.msg
     UtilityBounds.msg
     )

## Generate services in the 'srv' folder
add_service_files(FILES
  AddNode.srv
  AddNodeAtIndex.srv
  capabilities/CancelRemoteCapabilitySlot.srv
  capabilities/CheckPreconditionStatus.srv
  capabilities/DeleteCapabilityImplementation.srv
  capabilities/FindBestCapabilityExecutor.srv
  capabilities/GetAvailableRemoteCapabilitySlots.srv
  capabilities/GetCapabilityImplementations.srv
  capabilities/GetCapabilityInterfaces.srv
  capabilities/GetLocalBid.srv
  capabilities/LoadCapabilities.srv
  capabilities/PrepareLocalImplementation.srv
  capabilities/PutCapabilityImplementation.srv
  capabilities/PutCapabilityInterfaces.srv
  capabilities/RequestCapabilityExecution.srv
  capabilities/ReserveRemoteCapabilitySlot.srv
  capabilities/RunRemoteCapabilitySlot.srv
  capabilities/SaveCapabilities.srv
  capabilities/UpdateCapabilityInterfaces.srv
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
  SetSimulateTick.srv
  TestService.srv
  WireNodeData.srv
  )

## Generate actions in the 'action' folder
add_action_files(FILES
  ExecuteRemoteCapability.action
  RunTree.action
  FindBestExecutor.action
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime
  )
