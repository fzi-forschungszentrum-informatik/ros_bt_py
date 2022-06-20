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

        self.load_capability_interfaces_service = rospy.Service('~load_capability_interfaces',
                                                                LoadCapabilityInterfaces,
                                                                self.capability_repository.load_capability_interfaces)

        self.save_capability_interfaces_service = rospy.Service('~save_capability_interfaces',
                                                                SaveCapabilityInterfaces,
                                                                self.capability_repository.save_capability_interfaces)
        self.submit_capability_interfaces_service = rospy.Service('~submit_capability_interfaces',
                                                                  SubmitCapabilityInterfaces,
                                                                  self.capability_repository.submit_capability_interfaces)
        self.get_capability_interfaces_service = rospy.Service('~get_capability_interfaces',
                                                               GetCapabilityInterfaces,
                                                               self.capability_repository.get_capability_interfaces)
        self.get_capability_interfaces_service = rospy.Service('~delete_capability_interfaces',
                                                               DeleteCapabilityInterfaces,
                                                               self.capability_repository.delete_capability_interfaces)

        self.get_capability_implementation_service = rospy.Service('~get_capability_implementations',
                                                                   GetCapabilityImplementations,
                                                                   self.capability_repository.get_capability_implementations)

        self.delete_capability_implementation_service = rospy.Service('~delete_capability_implementation',
                                                                   DeleteCapabilityImplementation,
                                                                   self.capability_repository.delete_capability_implementation)

        self.submit_capability_implementation_service = rospy.Service('~submit_capability_implementation',
                                                                   SubmitCapabilityImplementation,
                                                                   self.capability_repository.submit_capability_implementation)

        self.update_capability_implementation_service = rospy.Service('~update_capability_implementation',
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
