#!/usr/bin/env python3
import rospy

from ros_bt_py_msgs.srv import LoadCapabilityInterfaces, SaveCapabilityInterfaces, SubmitCapabilityInterfaces, \
    GetCapabilityInterfaces, DeleteCapabilityInterfaces, \
    GetCapabilityImplementations, DeleteCapabilityImplementation, SubmitCapabilityImplementation, UpdateCapabilityImplementation

from ros_bt_py.capability_repository import CapabilityRepository


class CapabilityRepositoryNode(object):
    def __init__(self):
        rospy.loginfo("initializing capability repository node...")

        self.capability_repository = CapabilityRepository()

        self.load_capability_interfaces_service = rospy.Service('~/capabilities/interfaces/load',
                                                                LoadCapabilityInterfaces,
                                                                self.capability_repository.load_capability_interfaces)

        self.save_capability_interfaces_service = rospy.Service('~/capabilities/interfaces/save',
                                                                SaveCapabilityInterfaces,
                                                                self.capability_repository.save_capability_interfaces)
        self.submit_capability_interfaces_service = rospy.Service('~/capabilities/interfaces/submit',
                                                                  SubmitCapabilityInterfaces,
                                                                  self.capability_repository.submit_capability_interfaces)
        self.get_capability_interfaces_service = rospy.Service('~/capabilities/interfaces/get',
                                                               GetCapabilityInterfaces,
                                                               self.capability_repository.get_capability_interfaces)
        self.get_capability_interfaces_service = rospy.Service('~/capabilities/interfaces/delete',
                                                               DeleteCapabilityInterfaces,
                                                               self.capability_repository.delete_capability_interfaces)

        self.get_capability_implementation_service = rospy.Service('~/capabilities/implementations/get',
                                                                   GetCapabilityImplementations,
                                                                   self.capability_repository.get_capability_implementations)

        self.delete_capability_implementation_service = rospy.Service('~/capabilities/implementations/delete',
                                                                   DeleteCapabilityImplementation,
                                                                   self.capability_repository.delete_capability_implementation)

        self.submit_capability_implementation_service = rospy.Service('~/capabilities/implementations/submit',
                                                                   SubmitCapabilityImplementation,
                                                                   self.capability_repository.submit_capability_implementation)

        self.update_capability_implementation_service = rospy.Service('~/capabilities/implementations/update',
                                                                   UpdateCapabilityImplementation,
                                                                   self.capability_repository.update_capability_implementation)

        rospy.loginfo("initialized capability repository node")

    def shutdown(self):
        self.capability_repository.shutdown()


if __name__ == '__main__':
    rospy.init_node('capability_repository_node')

    node = CapabilityRepositoryNode()
    rospy.spin()
    node.shutdown()
