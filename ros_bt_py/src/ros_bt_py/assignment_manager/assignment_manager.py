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
Module that provides the ABC for the AssignmentManagerNodes.
"""
import abc
from typing import Dict, Tuple

import actionlib
import rospy
import rosservice
from ros_bt_py_msgs.srv import (
    FindBestCapabilityExecutor, FindBestCapabilityExecutorRequest,
    FindBestCapabilityExecutorResponse, GetLocalBid, GetLocalBidRequest, GetLocalBidResponse,
)

from rospy import Service, ServiceProxy


class AssignmentManager(abc.ABC):
    """
    Abstract base class that implements the function callbacks used for a AssignmentManagerNode.

    This class provides services to find a valid assignment of a capability implementation to a
    capability interface.
    """

    def __init__(
            self,
            local_mission_control_topic: str,
            global_assignment_msg_topic_prefix: str,
    ):
        """
        Creates a new assignment manager instance.

        :param local_mission_control_topic: The topic to communicate with a local mission control node.
        :param global_assignment_msg_topic_prefix: The topic used to communicate with other assignment manager nodes
        that might run on other robots.
        """
        self.global_assignment_msg_topic_prefix: str = global_assignment_msg_topic_prefix

        self._find_best_capability_executor_service = Service(
            f'~find_best_executor',
            FindBestCapabilityExecutor,
            handler=self.find_best_capability_executor
        )

    @abc.abstractmethod
    def find_best_capability_executor(
            self,
            goal: FindBestCapabilityExecutorRequest
            ) -> FindBestCapabilityExecutorResponse:
        """

    @abc.abstractmethod
    def get_assignment_request_result(
            self,
            request: GetAssignmentRequestResultRequest
            ) -> GetAssignmentRequestResultResponse:
        """
        Returns the result of the assignment request.
         :param request: Contains the information about the assignment request.
        :return: The implementation name and the assigned mission controller.
        """


class SimpleAssignmentManager(AssignmentManager):
    def find_best_capability_executor(
            self,
            goal: FindBestCapabilityExecutorRequest
    ) -> FindBestCapabilityExecutorResponse:

        response = FindBestCapabilityExecutorResponse()
        bids: Dict[str, GetLocalBidResponse] = {}

        local_bid_services = rosservice.rosservice_find("ros_bt_py_msgs/GetLocalBid")
        rospy.logfatal(
            f"Available services: {local_bid_services}", logger_name="assignment_system"
        )
        for service_name in local_bid_services:
            service_proxy = ServiceProxy(
                service_name,
                GetLocalBid
            )
            service_proxy.wait_for_service(timeout=rospy.Duration.from_sec(2))
            bid_response: GetLocalBidResponse = service_proxy.call(
                GetLocalBidRequest(
                    interface=goal.capability
                )
            )
            if not bid_response.success:
                rospy.logwarn(
                    f"Failed to get bid from: {service_name}, {bid_response.error_message}",
                    logger_name="assignment_system")
                continue
            bids[service_name] = bid_response

        if len(bids) < 1:
            response.error_message = "No bids were present for this capability!"
            rospy.logerr(response.error_message, logger_name="assignment_system")
            response.success = False
            return response

        top_bid: Tuple[str, GetLocalBidResponse] = list(sorted(bids.items(), key=lambda item: item[1].bid))[0]

        top_service_name: str = rosservice.get_service_node(top_bid[0])
        top_service_response: GetLocalBidResponse = top_bid[1]

        rospy.loginfo(f"Executing  on {top_service_name} - {top_service_response}", logger_name="assignment_system")

        response.success = True
        response.executor_mission_control_topic = top_service_name
        response.execute_local = (top_service_name == goal.mission_control_name)
        response.implementation_name = top_service_response.implementation_name
        return response
