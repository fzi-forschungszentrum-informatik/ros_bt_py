#!/usr/bin/env python3
#  -------- BEGIN LICENSE BLOCK --------
#  Copyright 2022 FZI Forschungszentrum Informatik
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#        contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#  -------- END LICENSE BLOCK --------
"""
Module containing a ros node for the AssignmentManager class.
"""
from typing import Type

import rospy
from ros_bt_py_msgs.srv import (
    AddAssignmentRequest, CancelAssignmentRequest, GetAssignmentRequestStatus,
    GetAssignmentRequestResult,
)

from ros_bt_py.assignment_manager.assignment_manager import AssignmentManager


class AssignmentManagerNode:
    """
    Node to run a AssignmentManager class implementation.

    This sets up the rospy services and maps them to the respective methods in the AssignmentManager.
    """
    def __init__(
            self,
            local_topic_prefix: str,
            global_assignment_msg_topic_prefix: str,
            assignment_manager: Type[AssignmentManager]
            ):
        """
        Creates a new AssignmentManagerNode.
        This method will directly setup rospy services.

        :param local_topic_prefix: The ros topic prefix used to communicate with local services like the mission
        control on the same robot.
        :param global_assignment_msg_topic_prefix: The ros topic used for communicating with other AssignmentManagers
        for example in an auction.
        :param assignment_manager: The assignment manager class implementation to use.
        """
        self.assignment_manager = assignment_manager(
            local_mission_control_topic=f'{local_topic_prefix}/mission_control',
            global_assignment_msg_topic_prefix=global_assignment_msg_topic_prefix
        )

        self.__add_assignment_request_service = rospy.Service(
            f'{local_topic_prefix}/assignment_manager/request',
            AddAssignmentRequest,
            self.assignment_manager.request_assignment
        )

        self.__cancel_assignment_request_service = rospy.Service(
            f'{local_topic_prefix}/assignment_manager/cancel',
            CancelAssignmentRequest,
            self.assignment_manager.cancel_assignment_request
        )

        self.__get_assignment_request_status_service = rospy.Service(
            f'{local_topic_prefix}/assignment_manager/status',
            GetAssignmentRequestStatus,
            self.assignment_manager.get_assignment_request_status
        )

        self.__get_assignment_request_result_service = rospy.Service(
            f'{local_topic_prefix}/assignment_manager/result',
            GetAssignmentRequestResult,
            self.assignment_manager.get_assignment_request_result
        )

    def shutdown(self):
        """
        Shuts down the rospy service.
        """
        self.__get_assignment_request_result_service.shutdown()
        self.__cancel_assignment_request_service.shutdown()
        self.__get_assignment_request_status_service.shutdown()
        self.__add_assignment_request_service.shutdown()

if __name__ == "__main__":
    try:
        LOCAL_TOPIC_PREFIX: str = rospy.get_param("local_topic_prefix", default="/robot1")
        ASSIGNMENT_TOPIC_PREFIX: str =\
            rospy.get_param("assignment_topic_prefix", default="/assignments")

    except rospy.ROSException as exc:
        rospy.logerr(f"Failed to retrieve parameters from parameter server: {exc}")
        raise exc
    ASSIGNMENT_MANAGER_NODE = AssignmentManagerNode(
        local_topic_prefix=LOCAL_TOPIC_PREFIX,
        global_assignment_msg_topic_prefix=ASSIGNMENT_TOPIC_PREFIX,
        assignment_manager=
    )
    rospy.spin()
    ASSIGNMENT_MANAGER_NODE.shutdown()
