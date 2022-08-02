#!/usr/bin/env python3
"""
ROS Node running the the CapabilityRepository class.
"""
import rospy

from ros_bt_py.assignment_manager.local_auction_manager import LocalAssignmentManager
from ros_bt_py.mission_control import MissionControlNode

if __name__ == '__main__':
    rospy.init_node('capability_repository_node')

    LOCAL_CAPABILITY_TOPIC_PREFIX = rospy.get_param("robot_namespace", "/mission_control")
    GLOBAL_CAPABILITY_TOPIC_PREFIX = rospy.get_param("capabilities_topic", "/capabilities")

    CAPABILITY_REPOSITORY = MissionControlNode(
        LocalAssignmentManager,
        local_capability_topic_prefix=LOCAL_CAPABILITY_TOPIC_PREFIX,
        global_capability_topic_prefix=GLOBAL_CAPABILITY_TOPIC_PREFIX
    )
    rospy.spin()
    CAPABILITY_REPOSITORY.shutdown()
