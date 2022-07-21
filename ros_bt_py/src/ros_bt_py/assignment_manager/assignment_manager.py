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

from ros_bt_py_msgs.srv import (
    AddAssignmentRequestRequest, AddAssignmentRequestResponse,
    CancelAssignmentRequestRequest, CancelAssignmentRequestResponse, GetAssignmentRequestStatusRequest,
    GetAssignmentRequestStatusResponse, GetAssignmentRequestResultRequest, GetAssignmentRequestResultResponse,
)


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
        self.local_mission_control_topic: str = local_mission_control_topic
        self.global_assignment_msg_topic_prefix: str = global_assignment_msg_topic_prefix

    @abc.abstractmethod
    def request_assignment(self, request: AddAssignmentRequestRequest) -> AddAssignmentRequestResponse:
        """
        Function to request a new assignment of the given capability interface to an implementation.

        :param request: The request containing the assignment request data.
        :return: The response to the request containing the auction id or error messages.
        """

    @abc.abstractmethod
    def cancel_assignment_request(self, request: CancelAssignmentRequestRequest) -> CancelAssignmentRequestResponse:
        """
        Cancels the running assignment request with the given id.
        This causes the get_assignment_result method to never produce an output and the id will become invalid.

        :param request: The request containing the unique assignment request id.
        :return: The status of the cancel request.
        """

    @abc.abstractmethod
    def get_assignment_request_status(
            self,
            request: GetAssignmentRequestStatusRequest
            ) -> GetAssignmentRequestStatusResponse:
        """
        Returns the status of the assignment request.
        :param request: Contains the information about the assignment request.
        :return: The current status or an error message.
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
